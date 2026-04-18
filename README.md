# InsightAT

**Open-source incremental SfM with GPU-accelerated feature extraction and matching.**

InsightAT 是一个开源的 Structure from Motion 软件。
核心的特征提取和匹配环节使用 GPU 加速（CUDA 或 GLSL），从照片到稀疏点云可通过 pipeline 一次完成。

> ⚠️ **v0.1 — Early Release**：核心 pipeline 可用，API 和数据格式可能变动。欢迎试用和反馈。

---

## ✨ 特性

- **GPU 加速的前端** — SIFT 提取和特征匹配在 GPU 上运行（支持 CUDA 和 GLSL 两种后端）
- **无 CUDA 也能跑** — GLSL 后端通过 EGL 无头渲染工作，不依赖 CUDA Toolkit，适配更多 GPU 环境
- **Docker 友好** — EGL 无头渲染无需 X11/显示器，容器和云服务器直接运行
- **端到端 Pipeline** — 从照片目录到稀疏重建，一条命令完成全流程
- **逐步可视化** — 增量式 SfM 每次注册/BA 后输出 Bundler 格式，`at_bundler_viewer` 可查看每一步的重建状态
- **CLI-First 架构** — 每个步骤都是独立的 C++ 命令行工具，可单独运行，方便集成到自动化流程
- **COLMAP 兼容** — 支持导出 COLMAP sparse model 格式，可衔接 dense reconstruction

### GPU 加速范围（v0.1）

| 步骤 | 后端 | 说明 |
|------|------|------|
| 特征提取 | **GPU**（CUDA 或 GLSL） | CUDA 更快；GLSL 不依赖 CUDA Toolkit |
| 特征匹配 | **GPU**（CUDA 或 GLSL） | 同上 |
| 几何验证（RANSAC） | **GPU** 或 PoseLib（CPU） | 可切换，默认 GPU |
| 图像检索 | CPU（VLAD / 序列） | VLAD 的 PCA 训练支持 GPU（cuBLAS + cuSOLVER） |
| Track 构建 | CPU | |
| 增量式 SfM（Resection + BA） | CPU（Ceres Solver / PoseLib） | Resection 支持 GPU 和 PoseLib 两种后端 |

> 长期目标是将更多环节迁移到 GPU，但 v0.1 优先保证正确性和可用性。

## 📸 效果展示

<!-- TODO: 替换为实际截图 -->
```
at_bundler_viewer 加载 Bundler（`bundle.out`）或 COLMAP text 稀疏模型（`sparse/0` 下的 `cameras.txt`、`images.txt`、`points3D.txt`）后的 3D 点云 + 相机位姿可视化
（截图待补充）
```

---

## 🚀 快速开始

### 1. 准备照片

将照片放在一个目录下（支持子目录自动分组）：
```
my_images
├── camera1/
│   ├── IMG_0001.jpg
│   ├── IMG_0002.jpg
│   └── ...
└── camera2/
    ├── IMG_0100.jpg
    └── ...
```

### 2. 运行 Pipeline

```bash
# 编译后，一条命令完成全流程
./build/isat_sfm -i /path/to/photos -w work/ -v
```

这会自动执行：创建项目 → 特征提取 → 图像检索 → 特征匹配 → 几何验证 → Track 构建 → 增量式 SfM。

**没有 CUDA？** 用 GLSL 后端：
```bash
./build/isat_sfm -i /path/to/photos -w work/ \
    --extract-backend glsl --match-backend glsl -v
```

### 3. 查看结果

```bash
# 内置 3D 查看器（点云 + 相机位姿）
./build/at_bundler_viewer 

# 或用 MeshLab
meshlab work/incremental_sfm/bundle.out
```

输出文件：
```
work/incremental_sfm/
├── poses.json      # 相机位姿（R, C）
├── bundle.out      # Bundler 格式（点云 + 相机，可直接可视化）
└── list.txt        # 对应的图像列表
```

> 开启调试，增量式 SfM 过程中每次 BA 迭代都会输出中间 bundle.out，可以用 `at_bundler_viewer` 观察重建的逐步演进。

---

## 📦 安装

### 从源码编译

**系统要求**：Ubuntu 22.04 / 24.04，NVIDIA GPU（OpenGL 3.3+；CUDA 可选但推荐）

**依赖安装**：
```bash
# 可选：CUDA Toolkit（≥11.8，推荐 12.x）— 不装也能用 GLSL 后端
# 参考 https://developer.nvidia.com/cuda-downloads

# 系统依赖
sudo apt install -y \
    cmake build-essential git python3 \
    libeigen3-dev libceres-dev libopencv-dev libgdal-dev \
    libgoogle-glog-dev libglew-dev libegl1-mesa-dev \
    qtbase5-dev
```

**编译**：
```bash
git clone https://github.com/your-org/InsightAT.git
cd InsightAT
# To control whether SiftGPU builds with the CUDA backend from the top-level
# CMake configure step, pass the `SIFTGPU_ENABLE_CUDA` cache variable.
# Enable CUDA backend:
cmake -S . -B build -DSIFTGPU_ENABLE_CUDA=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Disable CUDA backend (force GLSL/EGL-only build):
cmake -S . -B build -DSIFTGPU_ENABLE_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

编译产物在 `build/` 目录下，包括所有 CLI 工具和 `at_bundler_viewer`。

> 如果没有安装 CUDA Toolkit，CMake 会自动禁用 CUDA 后端，仅编译 GLSL 后端。

### Docker

```bash
docker build -t insightat .

# 运行（需要 nvidia-container-toolkit）
docker run --gpus all \
    -v /path/to/photos:/data/input \
    -v /path/to/output:/data/output \
    insightat \
    isat_sfm -i /data/input -w /data/output/work
```

---

## 🛠️ Pipeline 详解

### 完整运行

```bash
./build/isat_sfm -i <photos_dir> -w <work_dir>
```

### 分步运行

`isat_sfm` 支持 `--steps` 只跑部分步骤；项目与工作目录约定为 `<work>/project.iat` 与中间文件均在 `<work>/` 下：

```bash
# 只跑到匹配与几何（例如先检查 pair / geo）
./build/isat_sfm -i photos/ -w work/ --steps create,extract,match

# 从已有 match/geo 继续：tracks + 增量式 SfM
./build/isat_sfm -i photos/ -w work/ --steps tracks,incremental_sfm
```

更细粒度控制可直接调用各 `isat_*` 工具（见下表）。

### `isat_sfm` 主要参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-i` / `--input` | 照片根目录 | （必填） |
| `-w` / `--work-dir` | 工作目录（含 `project.iat`、特征、匹配、SfM 输出） | （必填） |
| `--steps` | 运行步骤 | `create,extract,match,tracks,incremental_sfm` |
| `--extract-backend` | 特征提取后端（cuda / glsl） | `cuda` |
| `--match-backend` | 匹配后端（cuda / glsl） | `cuda` |
| `--fix-intrinsics` | 固定相机内参（环拍/物体扫描推荐） | `false` |
| `-v` / `--verbose` | 详细日志 | `false` |

### CLI 工具

每个算法步骤都是独立的 C++ 命令行工具：

| 工具 | 功能 | 加速 |
|------|------|------|
| `isat_sfm` | **端到端 pipeline**（一条命令完成全流程） | — |
| `isat_extract` | 特征提取（SIFT） | GPU（CUDA / GLSL） |
| `isat_match` | 特征匹配 | GPU（CUDA / GLSL） |
| `isat_geo` | 几何验证 + 两视图位姿 | GPU 或 PoseLib |
| `isat_retrieve` | 图像检索（VLAD / 序列） | CPU |
| `isat_tracks` | Track 构建 | CPU |
| `isat_incremental_sfm` | 增量式 SfM（Resection + BA） | CPU（Ceres） |
| `isat_project` | 项目管理（创建/导入/导出） | — |
| `isat_camera_estimator` | 相机参数估计（EXIF + sensor_db） | — |
| `isat_train_vlad` | 训练 VLAD 编码器 | — |
| `at_bundler_viewer` | 3D 点云 + 相机位姿查看器 | OpenGL |

每个工具支持 `-h` 查看详细用法。

---

## 📊 输出格式

### Bundler（默认，可直接可视化）

增量式 SfM 输出 `bundle.out` + `list.txt`，可用 `at_bundler_viewer` 或 MeshLab 查看；COLMAP 结果可打开对应 `sparse/0` 目录对比。

### COLMAP 兼容导出

支持导出 COLMAP sparse model 格式（`cameras.txt`, `images.txt`, `points3D.txt`），
可用于 COLMAP dense reconstruction 或 OpenMVS。

### IDC 中间格式

InsightAT 使用 IDC（Insight Data Container）二进制格式存储中间数据（`.isat_feat`, `.isat_match`, `.isat_tracks`）。
IDC 自描述（JSON 头 + 二进制体），8 字节对齐，详见 [doc/design/](doc/design/)。

---

## 🆚 与 COLMAP 的定位区别

InsightAT 不是 COLMAP 的替代品。COLMAP 是成熟的通用 SfM + MVS 系统，功能全面。
InsightAT 是一个轻量的增量式 SfM 工具，可以和 COLMAP 配合使用。

| | InsightAT v0.1 | COLMAP |
|---|---|---|
| 特征提取/匹配 | GPU（CUDA 或 GLSL） | GPU |
| SfM 核心 | CPU（Ceres + PoseLib） | CPU |
| 无 CUDA 运行 | ✅（GLSL 后端） | ❌（需要 CUDA） |
| 过程可视化 | 每步输出 Bundler，实时查看 | 完成后查看 |
| Dense Reconstruction | ❌（导出到 COLMAP/OpenMVS） | ✅ 内置 |
| 成熟度 | 早期版本 | 生产就绪 |
| 平台 | Linux | Linux / macOS / Windows |

典型工作流：InsightAT 稀疏重建 → 导出 COLMAP 格式 → COLMAP dense reconstruction。

**批量评测**：与 COLMAP 对比、ETH3D 数据整理与推荐命令顺序见 [benchmarks/README.md](benchmarks/README.md)。

---

## 🗺️ Roadmap

- [x] **v0.1** — 增量式 SfM，GPU 前端（CUDA + GLSL），`isat_sfm` 一键 pipeline，CLI 工具链
- [ ] **v0.2** — PopSIFT 替换 SiftGPU；更多 benchmark
- [ ] **v0.3** — 预编译包（.deb / Windows installer）；航拍场景优化（GNSS 先验、航线检索）
- [ ] **v0.4** — Cluster + Merge + Global BA（大规模数据）
- [ ] **v1.0** — 两层 SfM（粗到精）；更多 GPU 加速环节；生产就绪

---

## 📖 设计文档

- [设计文档索引](doc/design/index.md) — 架构、数据模型、坐标系、序列化、UI
- [SfM 算法设计](doc/design/01_algorithm_sfm_philosophy.md) — 两层 SfM 设计思想
- [CLI I/O 规范](doc/design/05_cli_io_conventions.md) — 工具输入输出约定
- [开发笔记](doc/dev-notes/) — 实现过程记录

---

## 📄 License

InsightAT is licensed under the [GNU General Public License v3.0](LICENSE).

**Third-party notice**: The bundled SiftGPU library (third_party/SiftGPU) is
Copyright © 2007 University of North Carolina at Chapel Hill and is licensed
for educational, research and non-profit purposes only. See
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) for details.

> A future release will replace SiftGPU with [PopSIFT](https://github.com/alicevision/popsift)
> (BSD) to enable a more permissive license.

---

## 🤝 Contributing

欢迎提 Issue 和 PR。开发前请阅读[编码规范](doc/design/02_coding_style.md)。
