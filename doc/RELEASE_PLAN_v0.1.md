# InsightAT v0.1.0 发布计划

> 目标：发布一个可用的增量式 SfM 工具，面向航拍/倾斜摄影场景。
> 定位：GPU 加速、一键运行、开箱即用。

---

## 里程碑总览

| # | 里程碑 | 内容 | 预估工作量 | 状态 |
|---|--------|------|-----------|------|
| M1 | README 重写 | 面向用户的 README，Quick Start，效果展示 | 1 天 | 🔲 |
| M2 | LICENSE + 第三方声明 | GPL-3.0 + THIRD_PARTY_LICENSES.md | 0.5 天 | 🔲 |
| M3 | 安装文档 + Docker | 编译指南、依赖列表、Dockerfile 完善 | 1 天 | 🔲 |
| M4 | Pipeline 打磨 | 端到端跑通验证、错误处理、默认参数优化 | 2 天 | 🔲 |
| M5 | 可视化集成 | at_bundler_viewer 文档、迭代可视化说明 | 0.5 天 | 🔲 |
| M6 | COLMAP 导出验证 | 确保导出格式可被 COLMAP/OpenMVS 读取 | 1 天 | 🔲 |
| M7 | 示例数据 + Benchmark | 公开数据集跑通、贴结果（注册率、耗时） | 1-2 天 | 🔲 |
| M8 | 打包发布 | GitHub Release、.deb 包、tag v0.1.0 | 1 天 | 🔲 |

---

## M1: README 重写（最高优先级）

**原则**：README 是给用户看的，不是给开发者看的。设计文档移到 doc/ 链接。

**结构**：
```
# InsightAT
一句话介绍 + badge（build status, license）

## 特性（v0.1 实际能做的）
## 效果展示（截图/GIF）
## 快速开始（3 步跑通）
## 安装
  - Docker（推荐）
  - Ubuntu .deb
  - 从源码编译
## Pipeline 用法
  - 一键模式
  - 分步模式
  - 参数说明
## 输出格式
  - Bundler（at_bundler_viewer 可视化）
  - COLMAP 兼容导出
## 与 COLMAP 的区别
## Roadmap
## 设计文档（链接到 doc/design/）
## License
```

**差异化卖点（README 里要突出的）**：
1. **GPU 全链路加速** — 提取/匹配/RANSAC 全在 GPU，比 COLMAP 快
2. **一键运行** — `python -m src.pipeline.sfm -i photos/ -p out.iat -w work/` 搞定
3. **实时可视化** — 每次 BA 迭代输出 Bundler，at_bundler_viewer 实时查看
4. **航拍优化** — GNSS 先验检索、sensor_db 自动识别、航拍默认参数
5. **CLI-First** — 每个步骤可单独运行，方便集成到自动化流程

---

## M2: LICENSE + 第三方声明

**方案**：GPL-3.0（因为 SiftGPU 的 non-profit 限制比 GPL 更严格，GPL 是安全的上层选择）

**需要的文件**：
- `LICENSE` — GPL-3.0 全文
- `THIRD_PARTY_LICENSES.md` — 列出所有第三方库及其许可证：
  - SiftGPU: UNC Chapel Hill（educational/research/non-profit only）⚠️
  - PoseLib: BSD 3-Clause
  - DBow3: BSD-like
  - Ceres Solver: BSD 3-Clause
  - Eigen: MPL2
  - OpenCV: Apache 2.0
  - GDAL: MIT/X
  - stlplus3: BSD
  - cereal: BSD 3-Clause
  - nlohmann/json: MIT

**README 里注明**：
> InsightAT is licensed under GPL-3.0. Note: the bundled SiftGPU library
> is restricted to educational, research and non-profit use. A future
> release will replace SiftGPU with PopSIFT (BSD) to remove this restriction.

**后续路线**：
- 替换 SiftGPU → PopSIFT（BSD）后，可改为 Apache-2.0 或 MIT

---

## M3: 安装文档 + Docker

**编译依赖清单**（Ubuntu 22.04/24.04）：
```
CUDA Toolkit ≥11.8（推荐 12.x）
CMake ≥3.16
GCC ≥9 / Clang ≥12
Qt5 或 Qt6（UI 部分）
Eigen3, Ceres Solver ≥2.0, OpenCV ≥3.0, GDAL, glog, GLEW, EGL
```

**Dockerfile 完善**：
- 基于 nvidia/cuda:12.x-devel-ubuntu22.04
- 安装所有依赖
- 编译 InsightAT
- 入口点：pipeline 脚本
- 用法：`docker run --gpus all -v /data:/data insightat sfm -i /data/photos -p /data/out.iat -w /data/work`

**从源码编译**：
```bash
git clone --recursive https://github.com/xxx/InsightAT.git
cd InsightAT
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

---

## M4: Pipeline 打磨

**验证清单**：
- [ ] 端到端跑通：photos → extract → match → tracks → incremental_sfm → poses.json + bundle.out
- [ ] 错误信息友好：缺参数、路径不存在、CUDA 不可用时给出清晰提示
- [ ] 默认参数合理：航拍场景不需要调参就能出结果
- [ ] 进度输出：每个步骤有进度信息（stderr）
- [ ] 中断恢复：--steps 支持从中间步骤继续

---

## M5: 可视化集成

**已有能力**：
- 每次 BA 迭代输出 bundle.out
- at_bundler_viewer 可加载 bundle.out 查看 3D 点云 + 相机位姿

**需要补的**：
- [ ] README 里说明如何用 at_bundler_viewer 查看结果
- [ ] 截图/GIF 展示可视化效果
- [ ] pipeline 结束后打印 "用 at_bundler_viewer 查看结果" 的提示

---

## M6: COLMAP 导出验证

**目标**：用户可以把 InsightAT 的结果导入 COLMAP 做 dense reconstruction。

- [ ] 验证 ColmapExporter 输出的 cameras.txt / images.txt / points3D.txt 格式正确
- [ ] 用 COLMAP 的 `image_undistorter` + `patch_match_stereo` 验证能跑通
- [ ] 或者至少验证 COLMAP GUI 能加载 sparse model

---

## M7: 示例数据 + Benchmark

**公开数据集选择**（选一个小的）：
- ETH3D 的某个小场景
- 或者自己拍一组 50-100 张的航拍照片

**需要展示的指标**：
- 注册率（registered / total images）
- 总耗时（GPU: RTX 3090 或类似）
- 重投影误差（BA 后的 RMSE）
- 与 COLMAP 的简单对比（可选，有就更好）

---

## M8: 打包发布

- [ ] GitHub Release + tag v0.1.0
- [ ] Release Notes（基于 CHANGELOG）
- [ ] .deb 包（CPack 或手动打包，Ubuntu 22.04/24.04）
- [ ] Docker 镜像推送到 Docker Hub 或 GitHub Container Registry
- [ ] Windows 标注 "planned for v0.2"

---

## 后续版本路线（v0.2+）

| 版本 | 重点 |
|------|------|
| v0.2 | C++ pipeline 二进制 `isat_sfm`（无需 Python，跨平台）；PopSIFT 替换 SiftGPU |
| v0.3 | 预编译包（.deb / Windows installer）；更多 benchmark 和精度对比 |
| v0.4 | Cluster + Merge + Global BA（大规模数据支持） |
| v1.0 | 两层 SfM（粗到精）；全链路 GPU；生产就绪 |
