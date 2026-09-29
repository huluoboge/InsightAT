# InsightAT 技术现状

**InsightAT：Simple Automated Aerial Triangulation**

[简体中文](TECHNICAL_STATUS.md) | [English](TECHNICAL_STATUS_EN.md)

| 项目 | 内容 |
|------|------|
| 软件名称 | InsightAT（Aerial Triangulation，空三） |
| 版本 | `0.2.5`（仓库根 `VERSION`） |
| 代码基线 | `main` 分支，提交 `305bbd1` |
| 报告日期 | 2026-09-28 |
| 许可 | MIT License，Copyright (c) 2026 Yang Hu |
| 引用 | DOI [10.5281/zenodo.20042104](https://doi.org/10.5281/zenodo.20042104) |

> 本报告只记录截至上述基线**代码实际在做的事**：命令行工具、真实默认参数、输出文件、算法实现与已跑出的基准数据。
> 未实现的能力统一列在第 10 节，不混入正文。设计稿一律以现有实现为准；历史文档已整体归档，见 `docs/archive/2026-09-28/README.md`。

---

## 执行摘要

- **系统形态**：CLI-first、文件驱动、单机稀疏重建流水线；`isat_sfm` 负责编排，算法阶段各自作为独立 `isat_*` 进程运行。
- **输入契约**：项目先冻结为 `ATTask::InputSnapshot`，再由 `extract -t` / `intrinsics -t` 导出图像清单与内参，保证一次重建的输入可复现。
- **GPU 边界**：特征提取、级联哈希匹配、两视图几何默认走 GPU；BA 当前仍是 CPU Ceres，线性求解器可在可用时使用 `CUDA_SPARSE`。
- **已验证收益**：ETH3D 13 场景端到端总耗时由 v0.1 的 761.3 s 降至 v0.2 的 523.1 s；GPU 几何退化模型求解器的 IPI 路径相对 Jacobi 提速 34–76×。
- **当前主要边界**：没有簇划分、Sim3 合并、位姿图优化或跨机调度；CRS、GNSS、IMU、GCP 尚不进入默认求解链路。

---

## 1. 系统概览

InsightAT 是一套 **C++17 + CUDA**、**CLI 优先（CLI-first）** 的摄影测量空三（Aerial Triangulation，即 SfM）系统。

- **输入**：一个图像目录，配合 EXIF 与内置相机传感器库（`data/config/camera_sensor_database.txt`）估计内参。
- **输出**：增量式稀疏重建——相机位姿（`poses.json`、Bundler `bundle.out`）、稀疏点云、COLMAP 兼容 `sparse/0`；可选去畸变图像 + COLMAP 稀疏模型，供 3DGS / MVS 使用。
- **形态**：默认构建为纯 CLI（`isat_*`）；产品界面是 Electron 应用，见第 8 节。
- **平台**：Ubuntu 22.04 与 Windows，CUDA 12.8；分发形式为 AppImage、`.deb`、Docker 镜像、Windows zip。

一条命令跑完：

```bash
isat_sfm -i /data/images -w /data/work
# 成果目录：/data/work/incremental_sfm/
```

---

## 2. 系统架构与代码组织

### 2.1 实际运行形态

当前系统的真实形态是 **CLI-first、文件驱动、单机单进程串联**。它不是一个由 UI 持有全局内存状态的应用，而是一组通过磁盘产物衔接的阶段式工具链：

| 层 | 主要位置 | 当前职责 |
|----|----------|----------|
| 产品界面 | `sfm-gui/`、`sfm-viewer/` | Electron 界面负责项目/任务操作、调用 CLI、跟踪日志和查看 COLMAP 稀疏成果 |
| 流水线驱动器 | `src/cli/isat_sfm.cpp` | 按顺序启动同目录的 `isat_*` 子进程、记录阶段耗时与日志；本身不实现算法 |
| 阶段 CLI | `src/cli/` | 提取、候选对发现、匹配、几何验证、轨迹构建、种子评估、增量 SfM、去畸变 |
| 算法模块 | `src/algorithm/modules/` | 不依赖 Qt，也不依赖 `src/database/`；通过最小内参结构和普通 C++ 容器工作 |
| 容器与导出 | `src/algorithm/io/`、`src/algorithm/export/` | IDC 读写、EXIF/geopack、COLMAP 稀疏导出 |
| 项目数据层 | `src/database/` | `Project` / `ImageGroup` / `ATTask` / `CameraModel` 等类型与 Cereal 序列化；由项目 CLI 使用 |
| 遗留 UI | `src/ui/`、`src/render/`、`src/tools/at_bundler_viewer/` | Qt 5.15 旧实现，默认不构建，不是产品路线 |

早期文档曾把它描述成 `Project → AT Task → Output` 三层应用。按当前代码核对，三层设计的落地程度并不相同：

| 早期设计层 | 当前实际状态 |
|------------|--------------|
| Project | 项目、图像组、相机与测量类型已经存在，由 `isat_project` / `isat_camera_estimator` 使用；CRS 只是元数据，GNSS / IMU / GCP 不进入默认求解 |
| AT Task | `InputSnapshot` 是实际输入契约；任务树通过 `prev_task_id` 展示，但从父任务播种位姿尚未实现 |
| Output | `poses.json`、Bundler 与 COLMAP 稀疏成果已落盘；按目标 CRS 或 OPK / YPR 约定导出尚未实现 |

### 2.2 进程、文件与身份契约

- **一阶段一进程**：驱动器调用同级目录下的 `isat_*` 可执行文件；正常阶段失败即中止流水线，可选阶段可按策略降级为告警。
- **stdout / stderr 分离**：stdout 只承载机器可读的 `ISAT_EVENT` NDJSON，进度、日志和诊断统一走 stderr。
- **文件是阶段间唯一接口**：IDC 保存二进制特征、匹配与轨迹，JSON 保存项目清单、配对关系和配置；没有阶段间共享内存数据库。
- **身份就是数组下标**：求解器用 `image_index ∈ [0, num_images)`，相机由 `image_to_camera_index[]` 选择；同一次任务的所有阶段必须消费同一份 `images_all.json`。
- **断点续跑**：`--existing-task` 复用已导出的 `images_all.json`；Electron 流程通过 `create-at-task → extract -t <id> → isat_sfm --existing-task` 复用任务快照。

### 2.3 代码结构

```text
InsightAT/
├── CMakeLists.txt      # C++17；默认 CLI-only；SiftGPU / CUDA 开关
├── VERSION             # 0.2.5
├── src/
│   ├── cli/            # 全部 isat_* 命令行工具
│   ├── algorithm/
│   │   ├── io/         # IDC 读写、geopack、EXIF
│   │   ├── export/     # COLMAP 导出
│   │   └── modules/    # camera / extraction / retrieval / matching /
│   │                   # cpu_cascade_hash / gpu_cascade_hash / geometry / sfm
│   ├── database/       # Project / ATTask / ImageGroup / 相机模型 + Cereal 序列化
│   ├── render/         # 遗留 OpenGL 视图（默认不构建）
│   ├── ui/             # 遗留 Qt 5.15 主窗口（默认不构建）
│   ├── tools/          # at_bundler_viewer（遗留 Qt，默认不构建）
│   └── util/
├── third_party/        # popsift、SiftGPU、PoseLib、cereal、nlohmann、nanoflann、ImageIO…
├── benchmarks/         # ETH3D 数据准备 + COLMAP / InsightAT 批跑与对比
├── docs/               # 主页 index.html + 图片 + 归档
├── packaging/          # linux / docker / appimage / deb / windows / legacy
├── sfm-gui/            # Electron 外壳（驱动 CLI 流水线）
└── sfm-viewer/         # Electron + Three.js 的 COLMAP 稀疏成果查看器
```

### 2.4 技术栈与编译默认值

技术栈：

| 层次 | 选型 |
|------|------|
| 语言 | C++17（`CMAKE_CXX_EXTENSIONS OFF`） |
| 数学 | Eigen3；Ceres Solver（BA） |
| 视觉 | OpenCV（只用 `calib3d` 等必要模块） |
| 特征 | PopSift（默认）、SiftGPU（`--use-sift-gpu`） |
| GPU | CUDA 12.8；自定义 CUDA kernel；EGL + OpenGL 4.3 Compute Shader |
| 日志 | glog（输出到 stderr） |
| 序列化 | Cereal + nlohmann/json |
| 构建 | CMake ≥ 3.16；Windows 走 vcpkg |

CMake 选项默认值：

| 选项 | 默认 | 说明 |
|------|------|------|
| `INSIGHTAT_BUILD_QT_UI` | `OFF` | 遗留 Qt GUI 与 `at_bundler_viewer`、`render`，默认不参与构建 |
| `INSIGHTAT_BUILD_GUI_ONLY` | `OFF` | 只构建 Qt GUI 目标、跳过 CUDA / 算法组件 |
| `INSIGHTAT_BUILD_RENDER_TESTS` | `OFF` | |
| `INSIGHTAT_ENABLE_SIFTGPU` | `ON` | |
| `SIFTGPU_ENABLE_CUDA` | 检测到 CUDA toolkit 时为 `ON`，否则 `OFF` | |

后端默认值在编译期决定：找到 CUDA toolkit 时，`isat_sfm` 默认 `extract=cuda / match=cuda / geo=cuda`，匹配实现为 `cascade-gpu`、提取器为 PopSift；否则回退 `glsl / cpu / gpu` + SiftGPU + `cascade`。

**SiftGPU 与 CUDA 12.x 的兼容问题已解决**：提交 `679a1ae`（`fix(siftgpu): port COLMAP texture-object API for CUDA 12.8`）用 `cudaTextureObject` 替换了被新版驱动移除的 texture-reference 绑定，同时保持 CUDA 11.8 可用。

### 2.5 硬件与并行模型

| 维度 | 当前实现 |
|------|----------|
| 单机 / 单卡 | 一次流水线使用本机一个 GPU 设备；没有多 GPU 或跨机调度 |
| GPU 阶段 | PopSift 提取、级联哈希匹配、F / E / H RANSAC 默认使用 CUDA；GLSL / EGL 路径作为无 CUDA 或显式选择时的回退 |
| CPU 阶段 | 增量 SfM 主循环、轨迹状态管理、Ceres BA 求解 |
| 线程 | 阶段内 I/O 使用 `--io-threads`，BA 使用 `--ba-threads`，部分 SfM 循环使用 OpenMP |
| 并发限制 | GL / EGL 几何路径持有全局上下文和静态 SSBO，不是线程安全；并行通常落在多个进程或批次上 |

---

## 3. 端到端流水线

`isat_sfm` 是驱动器：自身不实现算法，而是以子进程方式调度同目录的兄弟 CLI，结束时打印每阶段耗时表并写 `sfm_timing.json`。

默认阶段集合：

```text
create, extract, match, tracks, seed_eval, incremental_sfm
```

另有可选阶段 `undistort`；也可用 `-s/--steps` 指定子集，配合 `--existing-task` 在既有任务上续跑。

下图是 7 个阶段各自调用的子进程与产出文件（由 `scripts/gen_pipeline_diagrams.py` 生成，基线 `305bbd1`）：

![isat_sfm CLI 流水线：阶段 / 子进程 / 产物](images/pipeline/isat_sfm_pipeline.svg)

### 3.1 各阶段

**create** —— 「项目 → 任务快照 → 输入清单」三步，按顺序调用：

1. `isat_project create -p <work>/project.iat`
2. `add-group` × N、`add-images` × N：把 `-i` 下的图像目录分组导入
3. `isat_camera_estimator -p project.iat -a`：基于 EXIF 的逐组内参估计（`--max-sample` 默认 5 张），结果写回项目文件
4. `isat_project create-at-task -p project.iat`：把当前项目（图像组 + 内参 + 测量 + 输入 CRS）冻结为 `AT_0`
5. `isat_project extract -p project.iat -t 0 -o images_all.json -a`：**只从该任务快照**导出 `<work>/images_all.json`

**extract** —— 调用 `isat_extract`，先做一次全分辨率提取，再追加一次用于候选对发现的低分辨率提取：

| 用途 | 参数 | 值 |
|------|------|-----|
| 全分辨率 | `--nfeatures` | 10000 |
| 全分辨率 | `--threshold` | 0.0067（可被 `--sift-threshold` 覆盖） |
| 全分辨率 | `--octaves` / `--levels` | `-1`（自动）/ `3` |
| 全分辨率 | `--image-max-dim` | 3200 |
| 全分辨率 | `--norm` | `l1root` |
| 全分辨率 | NMS | 默认启用网格 NMS（`--no-grid` 关闭） |
| 候选对发现级 | `--nfeatures-retrieval` / `--resize-retrieval` | 1500 / 1024 px |

`isat_extract` 独立运行时的 `--image-max-dim` 默认是 **6000**；3200 是 `isat_sfm` 传入的值（`isat_sfm --help` 里仍写 “default: 6000”，是未更新的字符串）。

**match** —— 候选对生成 + 特征匹配 + 几何验证：

- 默认 `--match-impl cascade-gpu`（CUDA 级联哈希匹配），另有 `cascade`（CPU）与 `siftgpu` 类路径。
- **候选对不是由 VLAD / GPS 检索生成。** 默认路径是 `isat_retrieval_match` 的 Wu / VisualSFM 风格 retrieval-by-matching：先生成全部 `C(n,2)` 图像对，在小图低分辨率 SIFT 特征上做穷举匹配，再用 F RANSAC（16 px、最少 6 内点）验证；随后统计每张图的有效邻居数，对邻居少于 5 的图像补入穷举对。这里衡量的是低分辨率匹配支持，而不是全局描述子相似度。
- 图像数不超过 `--auto-exhaustive-max-images`（默认 60）时，`isat_sfm` 直接跳过 `isat_retrieval_match`，生成全部图像对并按常规全分辨率匹配执行。
- 几何验证默认 `--geo-backend cuda` → 调用 `isat_geo_cuda`（纯 CUDA 的 F/E/H RANSAC → E 分解 → 全量三角化）；该二进制缺失时降级到 `gpu`（`isat_geo --backend gpu-gl`，EGL + OpenGL），再退到 `poselib`（CPU）。
- `--geo-min-inliers` 默认 10；`--geo-thresh-f` 默认 16.0 px（Sampson 误差）。
- `--focal-from-geo`（`auto` | `always` | `never`，默认 `auto`）：几何验证后若判定内参先验不可靠（EXIF 走 fallback，或参数疑似 `f35=35` 兜底），调用 `isat_focal_from_geo` 由视图图 F 矩阵估计 `fx` 并回写 `images_all.json`；耗时单独计一行。`always` 下该步失败即中止，`auto` / `never` 下仅告警或跳过。

该阶段的分支全貌（候选对来源、匹配实现、几何后端回退链、焦距回写）：

![match 阶段决策图](images/pipeline/isat_sfm_match_detail.svg)

**tracks** —— `isat_tracks` 把「验证对 + `.isat_match` + `.isat_geo` + 图像清单」融合为 `<work>/tracks/tracks.isat_tracks`（IDC，内嵌 `view_graph_pairs`），默认 `--min-track-length 2`。

**seed_eval** —— `isat_seed_eval` 对四种初始对选择策略（`balanced` / `wide_baseline` / `support_first` / `conservative`）做短窗口（`--seed-eval-max-images` 默认 6，实际调用 `isat_incremental_sfm`）评估，输出 `seed_eval_all/{report.json,best_seed.json,report_plot.png}`。`incremental_sfm` 阶段读取 `best_seed.json`，把胜出策略的 `init_min_inliers`、`init_max_forward_motion`、`init_min_angle_deg`、`init_min_median_angle_deg`、`resection_min_inliers` 回填给 `isat_incremental_sfm`；不可用时退回默认值并告警。

**incremental_sfm** —— `isat_incremental_sfm`，核心求解阶段（见第 6 节），输出到 `<work>/incremental_sfm/`：

- `poses.json` —— 每张已注册图像的位姿
- `bundler/bundle.out` —— Bundler 格式
- `colmap/sparse/0` —— COLMAP 兼容稀疏模型
- `tracks.isat_tracks` —— 更新后的轨迹存储

`--output-interval-sfm` 会额外在 `<work>/sfm_interval/iter_NNNN/` 写每轮迭代快照（`bundle.out` + `list.txt`）。

**undistort（可选）** —— `--undistort` 触发 `isat_undistort`，基于 `tracks.isat_tracks` 与 `poses.json` 输出去畸变图像 + COLMAP 稀疏模型（PINHOLE，`%08d` 命名），作为 3DGS / MVS 输入；`--binary` 控制写 `.bin`。

### 3.2 工作目录布局

代码里把这套约定称为 “scheme B”：配对 JSON 收进 `match/`，轨迹收进 `tracks/`，Bundler 导出收进 `incremental_sfm/bundler/`。

```text
<work>/
├── project.iat                     # 项目文件
├── images_all.json                 # 输入清单（focal-from-geo 会回写 fx）
├── camera_estimate_meta.json       # 内参估计来源元数据
├── feat/                           # 全分辨率特征 .isat_feat
├── feat_retrieval/                 # 候选对发现用的低分辨率特征
├── match/
│   ├── pairs_retrieve.json         # 候选对（穷举或 retrieval-by-matching）
│   ├── pairs_matched.json          # 匹配对
│   └── *.isat_match
├── geo/                            # *.isat_geo + pairs.json（验证对）
├── tracks/tracks.isat_tracks
├── seed_eval_all/                  # report.json / best_seed.json / report_plot.png
├── retrieval_match_work/
├── incremental_sfm/
│   ├── poses.json
│   ├── tracks.isat_tracks
│   ├── bundler/bundle.out
│   └── colmap/sparse/0
├── sfm_interval/                   # 可选：每轮迭代快照
├── logs/run_<时间戳>/              # console.log / detail.log / events.ndjson
└── sfm_timing.json                 # 各阶段耗时（同时以 ISAT_EVENT 打到 stdout）
```

`--no-log-file` 关闭日志文件输出。

---

## 4. 项目、任务快照与输入契约

`isat_project` 的子命令：`create`、`add-group`、`add-images`、`set-camera`、`set-cs`、`inspect`、`ls`、`create-at-task`、`delete-at-task`、`extract`、`intrinsics`。

**任务快照是当前流水线的输入契约。** `create-at-task` 把项目冻结为 `ATTask`；`extract -t <task-id>` 与 `intrinsics -t <task-id>` **只从任务快照**读数据，导出 `images_all.json` 与逐图相机内参。因此估内参发生在快照之前，此后对项目的改动不会影响这次重建。

- `--parent-task-id` 写入 `initialization.prev_task_id`，Electron GUI 据此渲染任务树。
- `intrinsics` 支持 `-a/--all` 多相机模式，输出 `schema: multi_camera_v1`，以 `group_id` 为键；`extract -a` 在清单里嵌入 `camera_id=group_id`。
- `set-cs --type local|enu|epsg|wkt` 把坐标参考系写进项目元数据；新建项目默认 `local`。

`--existing-task` 模式跳过 `create`，直接复用 `<work>/images_all.json`。Electron GUI 的续跑流程即此模式：`create-at-task` → `extract -t <id>` → `isat_sfm --existing-task`。

SfM 侧真正读取的项目信息很薄：**图像清单 + 逐图相机内参**，两者都由任务快照导出。项目数据层（`src/database/database_types.h`）另有 `CoordinateSystem`、`InputPose`、`Measurement`、`ATTask`、`ImageGroup`、`CameraModel`、`CameraRig` 等类型，用 Cereal 做版本化（反）序列化，且这一层不含 Qt，保证无头环境可读写。

---

## 5. 数据格式

### 5.1 IDC（InsightAT Data Container）

贯穿全流水线的二进制容器：二进制体 + 可读 JSON 头，自描述、版本化、8 字节对齐。

```text
┌──────────────┬──────────────┬──────────────┬──────────────────┬──────────────┬─────────┐
│ Magic "ISAT" │ version u32  │ json_size u64│ JSON 描述符(UTF-8)│ padding 0-7B │ payload │
│     4 B      │     4 B      │     8 B      │      变长        │  对齐到 8B   │  blob   │
└──────────────┴──────────────┴──────────────┴──────────────────┴──────────────┴─────────┘

header_size    = 4 + 4 + 8 + json_size
padding        = (8 - (header_size % 8)) % 8
payload_offset = header_size + padding          // 必为 8 的倍数
```

选 8 字节对齐是为了 SIMD 访存、GPU 上传、跨架构（ARM64 / x86_64）一致，以及 `mmap` 后直接按 `float*` 访问的良定义性。

JSON 描述符中每个 blob 必须给出 `name`、`dtype`、`shape`、`offset`、`size`；其中 **`dtype` 缺失会让下游崩溃**。

| 产物 | blob | dtype / shape |
|------|------|---------------|
| 特征提取 | `keypoints` | `float32` / `[N, 4]`（x, y, scale, orientation） |
| 特征提取 | `descriptors` | `uint8` 或 `float32` / `[N, D]` |
| 特征匹配 | `indices` | `uint16` / `[N, 2]` |
| 特征匹配 | `coords_pixel` | `float32` / `[N, 4]`（`x1,y1,x2,y2`） |
| 特征匹配 | `distances` | `float32` / `[N]` |

有意存**像素坐标而非归一化坐标**：F 矩阵直接用像素坐标估计，E 矩阵在调用时再做 `K⁻¹` 归一化。索引用 `uint16` 的前提是单图特征数 < 65536。

work 目录中出现的扩展名：`.isat_feat`、`.isat_match`、`.isat_geo`、`.isat_tracks`。读写实现见 `src/algorithm/io/idc_reader.*` 与 `idc_writer.*`；`IDCReader` 用 O(1) 的名字索引，但必须完整保留原始 JSON 描述符字段。

### 5.2 轨迹存储 `.isat_tracks`

`src/algorithm/io/track_store_idc.cpp` 中定义的 schema 版本：

| 版本 | 内容 |
|------|------|
| `1.0` | 基础轨迹 |
| `1.1` | 内嵌 `view_graph_pairs` |
| `1.2` | 由 SfM 流水线写出的轨迹 |
| `1.3` | 再内嵌 pose + 内参 blob |

### 5.3 身份与索引

图像身份就是 `images_all.json` 中 `images[]` 的数组下标 `0..num_images()-1`，全程不带外部 ID。仓库里没有 `IdMapping` 之类的稠密化步骤（`src/cli/project_loader.h` 明确写着 “no IdMapping”），因为输入本来就是稠密的。`poses.json` 同样以 `image_index` 指代图像，并在 `image_to_camera_index` 中给出相机下标。

---

## 6. 关键算法

### 6.1 候选对发现：现有检索模块与默认路径

仓库里确实实现了 VLAD、PCA 白化和 GPS 空间检索，但它们**不在 `isat_sfm` 的默认流水线中**：

| 模块 | 已实现能力 | 实际使用位置 |
|------|------------|--------------|
| `vlad_encoding` / `vlad_retrieval` | VLAD 全局描述子编码与 top-k 相似检索 | `isat_retrieve --strategy vlad`；需要 `isat_train_vlad` 生成的码本。默认流水线不调用 |
| `pca_whitening` / `pca_whitening_cuda` | VLAD 向量降维白化 | `isat_retrieve` 的 VLAD 路径；默认流水线不调用 |
| `spatial_retrieval` | 基于 GNSS 位置 / 姿态的邻域检索 | `isat_retrieve --strategy gps`；默认流水线不调用 |
| `retrieval_types` | `ImageInfo`、`ImagePair`、`RetrievalOptions` 等类型 | 服务于独立的 `isat_retrieve` 工具 |

默认阶段 `match` 使用的是 **retrieval-by-matching**，不是向量检索：

1. 生成全部 `C(n,2)` 图像对作为候选；
2. 在 `feat_retrieval/` 的低分辨率 SIFT 特征上做穷举匹配，默认 matcher 为 `cascade-gpu`；
3. 用 F RANSAC 做几何验证（16 px、最少 6 内点）；
4. 从 F 通过对构建邻居图；邻居数少于 5 的图像补入穷举对；
5. 输出 `match/pairs_retrieve.json`。

这与 Wu / VisualSFM 的 retrieval-by-matching 思路一致：**先用小图匹配筛出有关联的图像对，再进入全分辨率匹配**。仓库中的 VLAD / PCA / GPS 模块属于旁路工具，不应描述为当前默认候选对算法。图像数少于 60 时，`isat_sfm` 甚至跳过这层低分辨率筛选，直接穷举全分辨率匹配。

### 6.2 级联哈希匹配（Cascade Hashing）

v0.2.0 引入、v0.2.1 起成为默认，CPU 与 CUDA 两套实现。核心是用哈希分桶把描述子匹配从 O(N₁·N₂) 暴力搜索降到桶内比较。

CPU 实现（`src/algorithm/modules/cpu_cascade_hash/cpu_cascade_hash.h`）为缓存局部性做了 SoA 布局：

```cpp
struct ImageFeatures {
  std::vector<std::array<uint64_t, 2>> compressed_hashes;  // 每个描述子的 128-bit 哈希
  std::vector<uint16_t> bucket_ids_flat;                   // 描述子 × bucket_groups 的桶号
  std::vector<int> bucket_counts;                          // (group, bucket) → 桶长度
  std::vector<int> bucket_offsets;                         // (group, bucket) → 起始偏移
  std::vector<int> bucket_indices;                         // 按桶连续的描述子下标
};
```

默认参数：`hash_bits=128`、`bucket_groups=6`、`bucket_bits=8`、`candidate_top_min/max=6/10`、`min_match_list_len=16`、`ratio_test=0.8`、`mutual_best=true`、`use_bucket_secondary_hash=true`。

GPU 版 `GpuCascadeHashBlockMatcher` 以**图像块**为单位（`add_image` → `finalize` → `match_pairs`），由 `--cascade-gpu-image-block-size`（默认 1000）、`--cascade-gpu-sample-images`（默认 256，用于估计全局平均描述子）与 `--cascade-gpu-min-output-matches`（默认 16）控制显存与输出规模。

### 6.3 几何验证（F / E / H RANSAC）

`src/algorithm/modules/geometry/` 提供两视图几何模型估计，支持 F / E / H，两条 GPU 路径：纯 CUDA（`cuda_geo_ransac.cu`）与 EGL + OpenGL 4.3 Compute Shader（`gpu_geo_ransac.cpp`）。

已确认的实现细节：

- `gpu_ransac_F` 与 `gpu_ransac_E` 的最小样本都是 **8 点**（归一化 8 点法），误差度量是平方 Sampson 距离；E 需要调用方预先乘 `K⁻¹`。两条路径都**不做内点重精化**。
- `isat_sfm` 默认的 `--geo-backend cuda` 即 `isat_geo_cuda`，走的正是上述 8 点 E。
- 独立的 `isat_geo` 默认后端是 `poselib`（5 点法）。
- EGL / GL 路径**不是线程安全的**（全局 EGL 上下文 + 静态 SSBO，见 `gpu_geo_ransac.cpp` 与 `gpu_geo_ransac.h` 的注释），并发调用需外部加锁。
- EGL 路径自动枚举设备并优先选择 NVIDIA，无需手动设置 `__NV_PRIME_RENDER_OFFLOAD`。
- 退化模型求解器可切换：`null_vector` 提供 Jacobi 与 IPI（Inverse Power Iteration）两种。

`src/algorithm/modules/geometry/design.md` 记录了一组实测（GTX 1060 6GB，N=2048，workgroup=32，50 次均值）：

| n | 模型 | Jacobi (ms) | IPI (ms) | 加速比 |
|---:|:---:|---:|---:|---:|
| 100 | H | 46.2 | 0.61 | 75.7× |
| 100 | F | 46.2 | 0.73 | 63.3× |
| 300 | E | 46.4 | 0.86 | 53.9× |
| 500 | H | 46.6 | 0.92 | 50.7× |
| 1000 | H | 46.6 | 1.26 | 37.0× |
| 1000 | E | 46.9 | 1.36 | 34.5× |

两点已定位的结论：

1. Jacobi 耗时几乎与点数无关（≈ 46 ms），瓶颈是 **GPU 寄存器溢出**：`null_vector` 内部约 234 个动态索引 `float` 数组（B[81]+V[81]+A[72]），GLSL 编译器无法全部映射到寄存器，被迫溢出到 Local Memory。用 `GL_TIME_ELAPSED` Timer Query 确认 dispatch 本身耗时 44 ms、`glMemoryBarrier` 仅 0.013 ms，所以不是同步开销。
2. IPI 在 `n=100~1000` 上快 34–76× 且结果正确：`B = AᵀA` 加正则 `μ = trace(B)/1000` 使 `B_μ` 正定 → 原地 Cholesky → 6 轮逆迭代。

### 6.4 轨迹构建（TrackStore）

`src/algorithm/modules/sfm/track_store.h`：

- **SoA 布局**：`xyz[3*cap]`、`flags[cap]`，观测以扁平结构存储并带 `obs_track_id`。
- **纯索引身份**：见 5.3 节。
- **逻辑删除**：只改标志位，不搬移数组。轨迹位 `kAlive`、`kNeedsRetriangulation`、`kHasTriangulated`、`kSkipFromBA`；观测位 `kAlive`、`kRestorable`。
- **反向索引** `image_index → 观测下标列表`，使「删除某图上的外点观测」这类操作是 O(obs_in_image)。
- **可恢复观测**：因重投影误差（MAD 阈值）被删除的观测带 `kRestorable`，当相机内参显著变化（如早期 BA 的焦距漂移）时可由 `restore_observations_from_cameras` 重新评估恢复；因几何原因（深度 ≤ 0、三角角、PnP 外点）删除的观测不带该标志，永不自动恢复。

### 6.5 增量 SfM

算法入口是 `run_incremental_sfm_pipeline`（`src/algorithm/modules/sfm/incremental_sfm_pipeline.cpp`），选项由 `isat_incremental_sfm` 组装：

![增量 SfM 内部流程](images/pipeline/isat_sfm_process.svg)

**初始化**：载入 `tracks.isat_tracks`（内含 view graph，缺失时由 `pairs.json` + `geo/` 重建）；`run_initial_pair_loop` 按得分枚举初始对，门限为 50 条 MAD 后轨迹、100 个 E-RANSAC 内点、`|tz|/‖t‖ < 0.95`、最小夹角 2.0°、最小中位夹角 30.0°、BA RMSE ≤ 10 px，搜索上限 100 × 50；第一个初始对图像 `im0` 定义为世界原点，所有 global BA 都固定它。

**主循环**（每次迭代注册 1 张新图，直到没有候选）：

| 步骤 | 实际门限 / 行为 |
|------|-----------------|
| ① 选择候选 | 可见性金字塔覆盖率（0.02，6 层）排序 + 3D-2D ≥ 30，每迭代最多 40 个候选，带分数缓存 |
| ② Resection | 候选 dry-run 最多 8 个，`min_inliers 30`、`min_inlier_ratio 0.10`（大场景 0.15），期望 50 / 0.20；PnP RANSAC 4 px，取最优者提交 |
| ③ 三角化 | 新注册相机三角化新轨迹，`commit_reproj_px 16.0`，夹角 0.5°–120° |
| ④ BA 调度 | `n<41` 每次注册都跑 global BA；`41≤n<100` 用线性间隔 `ceil(5+0.12n)`；`n≥100` 每迭代 local BA（`kBatchNeighbor`，k=8）+ 周期 global `ceil(22+0.06n)` |
| ⑤ BA 外点剔除 | Huber 稳健核 + MAD 迭代剔除，`threshold_px 4.0`、`mad_k 2.5`、Huber δ 0.5–3.0 px、夹角 0.5°–120°、深度 ≤ 200× 场景中位，最多 10 轮 |
| ⑥ 重三角化 | local BA 后 `kNewImages`；`kPendingOnly` 每 3 次迭代；`kFullScan` 每 10 次迭代 |
| ⑦ 观测恢复 | global BA 后若某相机 `|Δfx/fx| > 0.02`，重估 `kRestorable` 观测并以 4 px 门限恢复 |
| ⑧ 快照判定 | `--debug-dir` + `--debug-interval` 写 `sfm_interval/iter_NNNN/`；无候选连续 2 次触发 global BA + `kFullScan` 救援 |

内参按**每个相机自身**的已注册图像数分相位解锁：`n<3` 全部固定 → `≥3` 放开 fx + k1 → `≥10` 加 k2 → `≥50` 全部放开（`--fix-intrinsics` 可全程固定）。收尾做一次 `kPendingOnly` 重三角化与最终 global BA，之后不再做 `kFullScan`。

### 6.6 光束法平差（BA）

相机与观测模型：

```text
xu = (u - cx)/fx ; yu = (v - cy)/fy
r² = xu² + yu²
dx = xu·(1 + k1·r² + k2·r⁴ + k3·r⁶) + tang_x
dy = yu·(1 + k1·r² + k2·r⁴ + k3·r⁶) + tang_y
u  = fx·dx + cx ;  v = sigma·fx·dy + cy
```

- **畸变模型**：Brown–Conrady 五参数，切向按 Bentley 约定（`tang_x = 2·p2·xu·yu + p1·(r² + 2·xu²)`，`tang_y = 2·p1·xu·yu + p2·(r² + 2·yu²)`）。
- **sigma 参数化**：`fy = sigma · fx`；`sigma` 固定为 1 时退化为单焦距模型。
- **观测权重**：像素域标准差 `std_sigma_obs_px`，由特征尺度映射（`sigma_feat < 2 → 1.0`，`< 4 → 1.2`，`< 8 → 1.4`，否则 `1.6`）。v0.2.1 起统一为显式像素域观测标准差，增量 SfM 与 resection 路径共用。
- **稳健核**：Huber，δ 默认 4.0 px，可由残差自适应估计（`compute_huber_delta`）。
- **正则与先验**：Tikhonov 正则（`tikhonov_lambda`）、焦距先验权重（`focal_prior_weight`）、相机间距离弱先验（`BACameraDistancePrior`，在固定锚点后约束基线方向的尺度漂移）。
- **求解器选择与回退链**：小问题 `DENSE_SCHUR`；大问题 `SPARSE_SCHUR`，稀疏后端优先级 `CUDA_SPARSE (cuDSS/cuSPARSE) → SUITE_SPARSE (CHOLMOD) → EIGEN_SPARSE`，都不可用时回退 `ITERATIVE_SCHUR + JACOBI`；另有交替 BA（`run_alternating_ba`）作兜底。
- **可调项**（`BASolverOverrides`）：`gradient_tolerance`、`function_tolerance`、`parameter_tolerance`、`dense_schur_max_variable_cams`（默认 30，DENSE↔SPARSE 阈值）、`max_num_iterations`、`huber_loss_delta`、`tikhonov_lambda`、`num_threads`；流水线层为 `isat_incremental_sfm --ba-threads`。
- **位姿表示**：四元数 + 相机中心 `[qx,qy,qz,qw,Cx,Cy,Cz]`；内部角度单位为弧度。

---

## 7. 工程契约

### 7.1 CLI I/O

| 通道 | 约定 |
|------|------|
| 退出码 | `0` 成功；非 0 失败，且失败时 stdout 不输出“成功载荷” |
| stderr | 日志、提示、警告、错误、进度（`PROGRESS: 0.35`）；glog 默认也走 stderr |
| stdout | 仅机器可读输出 |

机器可读行是 NDJSON 风格、带固定前缀的单行紧凑 JSON：

```text
ISAT_EVENT {"type":"project.create","ok":true,"data":{"project_path":"demo.iat","uuid":"..."}}
ISAT_EVENT {"type":"project.add_group","ok":false,"error":"project file not found"}
```

前缀 `ISAT_EVENT `（含尾随空格）用于抵御第三方库误写 stdout。字段约定：`type`、`ok`、`data`，失败时附 `error`。

日志级别优先级（高 → 低）：`--log-level` > `-q` > `-v` > 默认 `warn`；`error/warn/info` 映射 glog `minloglevel`，`debug` 额外打开 `VLOG(1)`。

### 7.2 分层依赖规则

- `src/algorithm/`：**禁止 Qt 头文件与链接**；使用 `std::string` / STL / Eigen；**不依赖 `src/database/`**。内参与畸变由最小类型 `insight::camera::Intrinsics`（`fx, fy, cx, cy, width, height, k1, k2, k3, p1, p2`）描述；项目数据先由 `isat_project extract` / `intrinsics` 从任务快照导成 JSON，再传入求解器；GUI 也只调用这些 CLI。
- `src/database/`：**禁止 Qt**；类型必须能在无头环境（反）序列化。

这条规则的实际收益：算法层可独立编译、可被单元测试直接驱动、可在无显示设备的环境中运行。

### 7.3 测试与 CI

- 与模块同目录的单元测试：`test_ba_analytic`、`test_track_ray_lambda_ceres`、`test_track_store_state_cache`、`test_incremental_triangulation`、`test_pnp_resection`、`test_sfm_diag2`、`test_seed_eval_common` 等。
- 几何模块为 CUDA kernel 提供 CPU 参考实现做对比验证（`test_cuda_geo_ransac.cpp`）。
- CI：`.github/workflows/linux-build-ubuntu22.yml`（Ubuntu 22.04 AppImage/deb）、`linux-build-ubuntu24.yml`（Ubuntu 24.04 AppImage/deb）、`windows-build.yml`（`windows-2022` zip）、`electron-gui.yml`（Electron 界面）。`release: published`（含 pre-release；Draft 不会触发）或 `workflow_dispatch` + `tag` 会上传到 GitHub Release。Windows 侧经 vcpkg 装配依赖（`vcpkg.json`：ceres[lapack,schur,suitesparse]、eigen3、glog、gflags、glew、egl、gdal、nlohmann-json、opencv4[calib3d,jpeg,png,thread,tiff]）。
- 性能改动要求用 ETH3D 基准回归：注册数不退步、RMSE 差 ≤ 0.01 px。

### 7.4 打包与可复现

| 路径 | 作用 |
|------|------|
| `packaging/linux/build.sh` | 本地 cmake 构建，产物 `./build/isat_*` |
| `packaging/docker-build.sh` + `packaging/Dockerfile` | 发布镜像：容器内自建 Ceres + cuDSS，产出 AppImage 与 deb |
| `packaging/appimage/build.sh` | AppImage 打包 |
| `packaging/deb/package.sh` | Debian 包 |
| `packaging/windows/package.ps1` | Windows zip 暂存（CI 产出） |
| `packaging/legacy/qt-gui.sh` | 遗留 Qt GUI 打包脚本 |

Docker 发布镜像在容器内**自建 Ceres + cuDSS**，避免宿主机 Ceres 与 CUDA 版本耦合；本地开发脚本则优先复用 `~/.local/ceres-cuda128`，否则退回 apt `libceres-dev`（`INSIGHTAT_USE_SYSTEM_CERES=1` 可强制）。这一「本地宽松 / 发布严格」的双轨策略记录在 `packaging/README.md`。

---

## 8. 界面

- **`sfm-gui/`** —— Electron 外壳，驱动 CLI 流水线：建项目、估内参、`create-at-task`、`extract -t <id>`、`isat_sfm --existing-task`，管理任务与阶段续跑，并把 `prev_task_id` 渲染成任务树。
- **`sfm-viewer/`** —— Electron + Three.js 的 COLMAP 稀疏成果查看器。
- **`src/ui/`、`src/render/`、`src/main.cpp`、`src/tools/at_bundler_viewer/`** —— 遗留 Qt 5.15 + OpenGL 实现，默认不参与构建（`INSIGHTAT_BUILD_QT_UI=OFF`），仅在核查旧实现时打开。Qt 已不是产品界面路线。

---

## 9. 性能与基准

### 9.1 ETH3D 对比（v0.1 / v0.2 / COLMAP）

ETH3D 训练子集 13 个场景批跑，全部 `code=0`：

| scene | COLMAP SfM (s) | ISAT v0.1 wall (s) | ISAT v0.2 wall (s) | v0.2 / v0.1 |
| --- | ---: | ---: | ---: | ---: |
| courtyard | 117.1 | 60.6 | 44.3 | 0.73 |
| delivery_area | 129.1 | 72.7 | 57.0 | 0.78 |
| electro | 104.0 | 70.6 | 40.7 | 0.58 |
| facade | 331.7 | 249.6 | 129.7 | 0.52 |
| kicker | 76.5 | 32.0 | 27.1 | 0.85 |
| meadow | 24.8 | 7.9 | 9.8 | 1.24 |
| office | 42.2 | 22.5 | 23.4 | 1.04 |
| pipes | 24.0 | 10.6 | 8.3 | 0.78 |
| playground | 85.6 | 51.0 | 37.9 | 0.74 |
| relief | 89.9 | 55.2 | 35.6 | 0.64 |
| relief_2 | 87.8 | 58.1 | 40.7 | 0.70 |
| terrace | 49.5 | 25.4 | 21.3 | 0.84 |
| terrains | 102.5 | 45.1 | 47.3 | 1.05 |
| **Σ** | **1264.7** | **761.3** | **523.1** | **0.69** |

引用该表时必须同时给出这些口径：

- 参考硬件为 **NVIDIA GTX 1060 6GB**（较老的消费级卡，对大幅面图像 / 密集 SIFT 金字塔的显存很敏感）；所有时间与机器强相关。
- COLMAP 列是 `elapsed_sfm_s`（特征 + 匹配 + mapper），不含 `BIN→TXT` 导出（约 0.3–1.5 s/场景）；InsightAT 列是端到端 wall（含特征 / 匹配 / BA）。**两列分段口径并不完全一致，只作量级参考。**
- `n_points3d` 的统计方式不同，**点数不能当质量分数直接对比**。
- COLMAP 用自带 SIFT（CUDA 构建时常为 CUDA 加速），与 PopSift / SiftGPU 不是同一实现；`--use-sift-gpu` 可用于「同为 SiftGPU 类实现」的对照。
- 图中除时间 / 点数外还含 GT 对齐误差：按图像基名匹配两侧模型，用 Umeyama 相似变换拟合「参考相机中心 → 估计相机中心」，报告 RMSE / 中位 / 最大误差与尺度。

无 CUDA 或需要纯 CPU 复现时，可用无头 GLSL 路径（`--extract-backend glsl --match-backend glsl` 等）。批跑与绘图入口：`benchmarks/sfm_compare/run_colmap_batch.py`、`run_insightat_batch.py`、`compare_dataset_batch.py`、`plot_eth3d_benchmark.py`。

### 9.2 几何验证

见 6.3 节：IPI 相对 Jacobi 在 `n=100~1000` 上提速 34–76×，把几何验证压到 **1 ms 量级**（GTX 1060 6GB，N=2048，50 次均值）。

### 9.3 全 CUDA 增量 SfM（设计目标，尚未接入）

当前代码中已经存在若干 CUDA SfM 基础文件（`src/algorithm/modules/sfm/cuda/cuda_triangulation.*`、`cuda_resection.*`、`cuda_reproj.*`），但它们没有被 `sfm_module` 的 CMake 源列表编译，也没有被 `incremental_sfm_pipeline` 调用；实际编译和接入的是 `gpu_twoview_sfm_cuda.cu` 这条两视图路径。当前 BA 仍由 CPU Ceres 承担，所谓“全 CUDA 增量 SfM”是设计目标，不是现有能力。

归档设计稿给出的核心方案是：

- **不在 GLSL 中完成完整 BA**：EGL / GLSL / SSBO 适合“上传、处理、下载”的单次批处理；BA 的 LM 迭代需要把残差、Hessian、线性求解和参数更新整环留在 GPU 内，状态必须跨迭代驻留。
- **不做 iSAM2 式增量 BA**：航空摄影并非严格时序帧流，下一个 resection batch 可能来自另一条航带，树形因子图会带来较高 fill-in。
- **采用持久 GPU Hessian 的增量 rank-update**：`H_new = H_old + J_newᵀ W J_new`，只累加新相机贡献；逐点 3×3 逆按需维护。
- **持久化 GPU 状态**：位姿、内参、轨迹 XYZ、轨迹 / 观测标志位、观测 SoA 与 CSR 索引常驻显存，避免每轮 PCIe 往返。
- **混合精度**：Hessian 累加使用 FP32 + Kahan 求和，Schur 消元与求解使用 FP64，参数更新采用 FP64 增量；每若干轮重建一次完整 Hessian，抑制长期浮点漂移。

设计稿针对“1000 图 × 500K 轨迹 × 平均 8 观测”给出的显存预算约 **640 MB**，其中 dense Schur 块占约 392 MB；预计 RTX 3090 在 5000 图规模约需 4–5 GB，超过 10000 图时应把 Schur 改为稀疏存储。

| 阶段 | 当前 CPU 估计 | Phase 1 估计（非 BA 全 CUDA） | Phase 2 估计（CUDA BA） |
|------|---------------:|-------------------------------:|-------------------------:|
| 全量三角化 | ~8 min | ~20 s | ~8 s |
| 外点剔除（5 轮） | ~3 min | ~0.5 s | ~0.5 s |
| Resection（100 图） | ~2 min | ~15 s | ~15 s |
| Local BA（每批） | ~30 s | ~30 s | ~3 s |
| 定期 Global BA | ~5 min | ~5 min | ~30 s |
| **1000 图完整重建** | **~45 min** | **~12 min** | **~3 min** |

> 上表是设计文档中的**预估**，不是当前实测。Phase 1 / Phase 2 均未落地；在接入前，报告中的性能结论仍以第 9.1、9.2 节的实测数据为准。

---

## 10. 现状边界与路线图

### 10.1 当前不存在的部分

以下能力在 `305bbd1` 的代码中**没有实现**，不应作为现有功能引用：

| 项目 | 现状 |
|------|------|
| 大任务量 / 分布式规模 | 无簇划分、无合并 / Sim3 对齐、无位姿图优化、无跨机调度；规模上限未验证 |
| 从父任务播种位姿 | `ATTask::Initialization::initial_poses` 没有写入方；`--parent-task-id` 当前只写 `prev_task_id` 供任务树展示 |
| CRS 参与重建 | `CoordinateSystem`（local / enu / epsg / wkt）只存在于项目元数据；重建求解不做坐标变换，也不做高程基准转换 |
| GNSS / IMU / GCP 约束求解 | 模型层有 `Measurement`；`spatial_retrieval` 只服务 `isat_retrieve`；默认流水线不把它们作为约束 |
| 词汇树检索 | 仓库内无 vocab-tree 实现；默认流水线也不使用 VLAD，而是 `isat_retrieval_match` 的低分辨率穷举匹配 + F 验证；VLAD 仅保留在独立的 `isat_retrieve` 工具中 |
| SQLite 后端 | 无 |
| E 矩阵 5 点法与内点重精化 | 几何 GPU 路径用 8 点法且不做重精化 |
| 全 CUDA 增量 SfM | 仅有未接入的 CUDA 基础文件；没有 `GpuSfMState`、持久 Hessian 增量 BA 或 CUDA pipeline 开关 |

需要同时注意的实现局限：GL 几何路径非线程安全（见 6.3）、基准硬件较老且口径不同（见 9.1）。

### 10.2 后续路线

以下内容来自归档设计稿，只说明方向，不代表已经承诺或排期：

**并行混合 SfM。** 目标是把数万张航拍图按 500–1000 张切簇，簇内并行增量重建，再用 Sim3 对齐和可选位姿图优化合并，最后做第一轮全局 BA。第二级再基于第一级位姿做全分辨率引导匹配与高精度相对几何，并完成第二轮全局 BA。当前实际运行的是“单簇、无 merge”的退化形态。

**全 CUDA 增量 SfM。** 从 `GpuSfMState` 状态骨架开始，依次迁移外点剔除、三角化、PnP、局部 BA 与全局 BA；先保持 Ceres 作 ground truth，再逐步替换 BA 内核。设计细节与预算见第 9.3 节。

**云端与分布式。** “一阶段一容器、共享文件系统或对象存储交换产物”具备可编排的技术基础，但仓库不包含调度器、队列、重试或跨机产物服务；单机仍是当前唯一实现。

**其他工程项。** 词汇树检索与查询缓存；Agisoft 风格 XML、行业 POS 等补充导出；面向超大块匹配的可选 SQLite 数据后端；学习式匹配器与自动策略选择。

---

## 11. 结论

InsightAT 的主要工程价值在于把“算法可替换”落实成了可运行的边界：

1. **阶段契约清楚。** CLI-first、自描述文件、进程级隔离，加上算法层不依赖 Qt 和 `src/database/`，使提取器、匹配器、几何后端和 BA 求解器可以在各自接口后替换，而不必重写整条流水线。
2. **已有可验证的性能改进。** v0.2 相对 v0.1 在 ETH3D 13 场景上的端到端墙钟总耗时下降约 31%（761.3 s → 523.1 s）；GPU 几何求解器通过用 Cholesky 逆迭代替换 Jacobi，在测试范围内获得 34–76× 的单点加速。
3. **当前瓶颈是规模，不是产品入口。** 任务快照工作流与 Electron 界面已经可以驱动单机重建；尚未解决的是大任务的分簇、合并、位姿图与跨机调度。
4. **边界必须和结果一起使用。** GL 几何路径非线程安全，GPU E 矩阵仍用 8 点法且无内点重精化，大规模上限未验证，ETH3D 数据也存在硬件老、时间口径不一致的问题。这些限制已经写进正文，而不是被略过。

---

## 附录 A · CLI 工具清单

| 工具 | 职责 |
|------|------|
| `isat_sfm` | 端到端流水线驱动器 |
| `isat_project` | 项目与任务管理、输入清单导出 |
| `isat_camera_estimator` | 基于 EXIF 的逐组相机内参估计 |
| `isat_calibrate` | 焦距标定聚合：汇总两视图焦距估计做全局一维优化，输出 `K.json`（离线辅助，需外部两视图目录） |
| `isat_extract` | SIFT 特征提取（PopSift / SiftGPU，全分辨率与候选对发现级） |
| `isat_retrieve` | 独立图像对检索工具（exhaustive / sequential / GPS / VLAD）；默认 `isat_sfm` 不调用 |
| `isat_train_vlad` | 为独立 `isat_retrieve --strategy vlad` 训练 VLAD 码本；默认流水线不调用 |
| `isat_retrieval_match` | 默认候选对路径：小图低分辨率 SIFT 穷举匹配 + F 验证 + 邻居补边 |
| `isat_match` | 特征匹配（`--match-backend cuda/glsl`） |
| `isat_cpu_cascade_hashing_match` | CPU 级联哈希匹配 |
| `isat_gpu_cascade_hashing_match` | CUDA 级联哈希匹配 |
| `isat_geo` | 两视图几何验证（默认 `poselib`，可选 `gpu-gl`） |
| `isat_geo_cuda` | 纯 CUDA 几何流水线（F+E+H RANSAC、E 分解、全量三角化） |
| `isat_focal_from_geo` | 由视图图 F 矩阵估计相机焦距 `fx` |
| `isat_tracks` | 由匹配 + 几何构建轨迹 IDC |
| `isat_seed_eval` | 多策略种子对评估 |
| `isat_incremental_sfm` | 增量 SfM + BA 求解 |
| `isat_undistort` | 去畸变图像 + COLMAP 稀疏（3DGS / MVS 输入） |

（`isat_tools` 只存在于 AppImage 内，不是源码构建目标。）

---

## 附录 B · 关键默认参数速查

| 阶段 | 参数 | 默认值 |
|------|------|--------|
| 流水线 | 默认阶段 | `create,extract,match,tracks,seed_eval,incremental_sfm` |
| 流水线 | `--extract-backend` / `--match-backend` / `--geo-backend` | 有 CUDA 时 `cuda` / `cuda` / `cuda` |
| 流水线 | `--match-impl` | `cascade-gpu` |
| 流水线 | `--focal-from-geo` | `auto` |
| 流水线 | `--image-max-dim` | 3200 |
| 流水线 | `--sift-threshold` | 0.0067 |
| 流水线 | `--auto-exhaustive-max-images` | 60 |
| 流水线 | `--seed-eval-max-images` | 6 |
| 流水线 | `--cascade-gpu-image-block-size` / `--cascade-gpu-sample-images` | 1000 / 256 |
| 流水线 | `--cascade-gpu-min-output-matches` / `--retrieval-min-output-matches` | 16 / 16 |
| 提取 | `--nfeatures` / `--nfeatures-retrieval` | 10000 / 1500 |
| 提取 | `--resize-retrieval` | 1024 px |
| 提取 | `--image-max-dim`（独立运行 `isat_extract`） | 6000 |
| 匹配 | 比率检验 / 互近邻 | 0.8 / true |
| 级联哈希 | `hash_bits` / `bucket_groups` / `bucket_bits` | 128 / 6 / 8 |
| 几何 | `--geo-min-inliers` / `--geo-thresh-f` | 10 / 16.0 px |
| 轨迹 | `--min-track-length` | 2 |
| BA | Huber δ / DENSE↔SPARSE 阈值 | 4.0 px / 30 相机 |
| BA | 稀疏求解优先级 | `CUDA_SPARSE → SUITE_SPARSE → EIGEN_SPARSE`，回退 `ITERATIVE_SCHUR + JACOBI` |
| 坐标 | 内部角度单位 | 弧度 |

---

## 附录 C · 文档去向

| 内容 | 位置 |
|------|------|
| 项目主页 | `docs/index.html` |
| 本报告 | `docs/TECHNICAL_STATUS.md` |
| 历史文档（旧设计稿、开发笔记、早期报告等） | `docs/archive/2026-09-28/` |
| 归档说明 | `docs/archive/2026-09-28/README.md` |
| 随代码维护的模块设计 | `src/algorithm/modules/matching/DESIGN.md`、`src/algorithm/modules/geometry/design.md`、`src/algorithm/modules/extraction/DISTRIBUTION_COMPARISON.md` |
| 基准说明与脚本 | `benchmarks/README.md`、`benchmarks/sfm_compare/`、`docs/images/benchmarks/` |
| 流水线图（SVG）与生成脚本 | `docs/images/pipeline/`、`scripts/gen_pipeline_diagrams.py` |

引用格式：

```bibtex
@software{hu2026insightat,
  author = {Hu, Yang},
  title = {InsightAT: Simple Automated Aerial Triangulation},
  year = {2026},
  doi = {10.5281/zenodo.20042104},
  url = {https://github.com/huluoboge/InsightAT}
}
```

---

**基线：** `main` @ `305bbd1`　·　**报告日期：** 2026-09-28
