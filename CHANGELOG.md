# Changelog

This is a user-facing changelog for quick scanning.
For detailed implementation notes, see [`doc/dev-notes/CHANGELOG.md`](doc/dev-notes/CHANGELOG.md).

## Unreleased

See [`doc/dev-notes/CHANGELOG.md`](doc/dev-notes/CHANGELOG.md).

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

Pre-release notes: [`doc/dev-notes/release-v0.2.1.md`](doc/dev-notes/release-v0.2.1.md).

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

Release notes: [`doc/dev-notes/release-v0.2.0.md`](doc/dev-notes/release-v0.2.0.md).

