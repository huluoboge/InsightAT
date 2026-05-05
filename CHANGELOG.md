# Changelog

This is a user-facing changelog for quick scanning.
For detailed implementation notes, see [`doc/dev-notes/CHANGELOG.md`](doc/dev-notes/CHANGELOG.md).

## Unreleased

See [`doc/dev-notes/CHANGELOG.md`](doc/dev-notes/CHANGELOG.md).

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

