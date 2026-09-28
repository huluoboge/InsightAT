# Changelog

This is a user-facing changelog for quick scanning.
For detailed historical implementation notes, see the
[archived development changelog](docs/archive/2026-09-28/dev-notes/CHANGELOG.md).
For the current implemented baseline, see the
[English technical status](docs/TECHNICAL_STATUS_EN.md) and
[Chinese technical status](docs/TECHNICAL_STATUS.md).

## 0.2.5 - 2026-09-29

### 中文（要点）

- **新增 Electron 产品界面**：`sfm-gui/` 提供项目/任务工作流、阶段续跑与重建、日志和进度跟踪；`sfm-viewer/` 提供 COLMAP 稀疏成果浏览。打包覆盖 AppImage、deb 和 Windows Electron。
- **新增稳定的焦距估计**：`isat_focal_from_geo` 与 `isat_sfm --focal-from-geo auto` 由视图图 F 矩阵估计 `fx` 并回写输入清单，在 EXIF 先验不可靠或出现 `f35=35` 兜底时减少错误内参对重建的影响。
- **SfM 更稳定**：Resection 采用 trial-and-pick，避免早期位姿翻转；结合种子评估、渐进内参解锁、可恢复观测与稳健外点剔除，提升弱纹理和困难初始对场景的收敛稳定性。
- **重建链路增强**：新增可选去畸变图像 + COLMAP 稀疏导出，支持点云 RGB，改进工作目录布局、日志、进度事件和跨阶段续跑。
- **平台与后端**：适配 CUDA 12.8；SiftGPU 通过 `cudaTextureObject` 适配 CUDA 12.x，同时保持 CUDA 11.8 兼容。

### English (highlights)

- **Added the Electron product UI:** `sfm-gui/` provides project/task workflows, stage continuation and rebuild controls, log/progress tracking, and packaged AppImage / deb / Windows Electron builds. `sfm-viewer/` provides a COLMAP sparse-result viewer.
- **Added stable focal estimation:** `isat_focal_from_geo` and `isat_sfm --focal-from-geo auto` estimate `fx` from view-graph F matrices and write it back to the input manifest, reducing the impact of unreliable EXIF priors or `f35=35` fallback intrinsics.
- **More stable SfM:** trial-and-pick resection avoids early pose flips. Combined with seed evaluation, progressive intrinsics unlock, restorable observations, and robust outlier rejection, this improves convergence on weak-texture scenes and difficult initial pairs.
- **Reconstruction workflow improvements:** optional undistortion plus COLMAP sparse export, point RGB support, cleaner work-directory layout, better logging/progress events, and cross-stage continuation.
- **Platform and backends:** CUDA 12.8 support. SiftGPU is ported to CUDA 12.x through `cudaTextureObject` while remaining CUDA 11.8 compatible.

## 0.2.4 - 2026-05-19

### 中文（要点）

- 新增 `isat_seed_eval` 初始像对策略评估框架，并接入 `isat_sfm` 自动选择初始化门限。
- 稳定化 initial pair 与 resection，减少困难数据集上的早期重建失败。
- 大规模场景优化：BA 角度外点剔除最高约 8x 加速，full-scan 重三角化最高约 6x 加速。
- 修复未三角化 track 的 epoch gate，使新注册图像带来的可三角化机会能在 full-scan 中被及时利用。

### English (highlights)

- Added the `isat_seed_eval` initial-pair strategy framework and wired `isat_sfm` to propagate the best initialization gates.
- Stabilized initial-pair selection and resection for difficult datasets.
- Large-scene performance: BA angle-based outlier rejection up to ~8x faster and full-scan retriangulation up to ~6x faster.
- Fixed the epoch gate for non-triangulated tracks so newly triangulable tracks are retried during full scans.

Release notes: [`docs/archive/2026-09-28/develop/release/v0.2.4.md`](docs/archive/2026-09-28/develop/release/v0.2.4.md).

## 0.2.3 - 2026-05-17

### 中文（要点）

- 改进初始像对选择，使初始化过程在困难数据集上更稳定。

### English (highlights)

- Improved initial-pair selection for more stable initialization on difficult datasets.

## 0.2.2 - 2026-05-16

### 中文（要点）

- 修复了多项 BUG：IDC 序列化问题、GPU 内存泄漏、几何验证边界条件等。
- 大幅优化 I/O 性能：IDC 文件读写并行化、特征点批量加载、异步磁盘写入。
- 整体效率提升：GPU 调度优化、内存重用策略、特征匹配缓存改进。
- 改进 CLI 工具的错误提示和日志输出。
- 增强 Windows CUDA 12.8 构建稳定性。

### English (highlights)

- Fixed critical bugs: IDC serialization issues, GPU memory leaks, edge cases in geometric verification.
- Major I/O performance improvements: parallelized IDC file I/O, batch feature loading, asynchronous disk writes.
- Overall efficiency gains: GPU scheduling optimization, memory reuse strategies, enhanced feature matching cache.
- Improved CLI error messages and logging verbosity.
- Enhanced Windows CUDA 12.8 build stability.

## 0.2.1-rc.1 - 2026-05-08

### English (pre-release highlights)

- This pre-release supersedes `v0.2.0`, which was found to have major issues shortly after release.
- `v0.2.1` is intended to be both faster and more reliable than `v0.2.0` in practical SfM runs.
- The SfM pipeline now distinguishes candidate pairs, matched pairs, and geometry-verified pairs explicitly, reducing mismatch between matching outputs and geometry inputs.
- `isat_match`, `isat_cpu_cascade_hashing_match`, and `isat_gpu_cascade_hashing_match` can now emit a matched-pairs JSON for downstream stages.
- `isat_retrieval_match` and `isat_sfm` now consume that explicit matched-pairs output instead of relying on implicit directory scans or candidate-pair assumptions.
- `isat_sfm` now exposes `--sift-threshold` for full-resolution extraction and a separate retrieval-stage minimum output threshold.
- Logging was improved to print the candidate / matched / verified pair JSON paths directly for easier debugging.
- BA and PoseLib tuning were updated with larger iteration budgets and revised observation weighting based on pixel-domain standard deviations.

Pre-release notes: [`docs/archive/2026-09-28/dev-notes/release-v0.2.1.md`](docs/archive/2026-09-28/dev-notes/release-v0.2.1.md).

## 0.2.0 - 2026-05-06

### 中文（要点）

- 支持 `CUDA 12.8` 构建：默认移除/不启用 `SiftGPU`（上游未适配 CUDA 12）；需要时可在 `CUDA 11.8` 环境单独构建 `SiftGPU`。
- 支持 Ceres `CUDA_SPARSE`（cuDSS / cuSPARSE）求解：全局 BA 更快。
- 支持 `cpu_cascade_hash` / `gpu_cascade_hash`：默认使用 `gpu_cascade_hash`，匹配更快。
- 修复 `at_bundler_viewer` 的部分渲染/加载问题。
- 整体效率优化（包含几何验证/IDC 写入/GPU cascade 调度等路径）。

### English (highlights)

- Added a `CUDA 12.8` build path. By default `SiftGPU` is disabled (upstream has no CUDA 12 support). You can still build it on `CUDA 11.8` if needed.
- Enabled Ceres `CUDA_SPARSE` (cuDSS / cuSPARSE) for faster sparse BA.
- Added `cpu_cascade_hash` / `gpu_cascade_hash` matchers; default is `gpu_cascade_hash`.
- Fixed issues in `at_bundler_viewer` rendering/loading.
- Overall performance improvements across geometry verification / IDC writing / GPU cascade scheduling.

Release notes: [`docs/archive/2026-09-28/dev-notes/release-v0.2.0.md`](docs/archive/2026-09-28/dev-notes/release-v0.2.0.md).
