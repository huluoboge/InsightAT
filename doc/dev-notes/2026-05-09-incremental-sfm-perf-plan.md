# 2026-05-09 增量 SfM 性能优化总计划

**分支**：`feature/incremental-faster`

本文档是增量三维重建（Incremental SfM）全栈性能优化的设计与实施规划，覆盖理论分析、瓶颈定位、CPU 多线程、GPU 并行、以及 BA 架构三个层次的改进方案。目标整体加速 **10–50×**。

---

## 1. 现状架构与瓶颈分析

### 1.1 流水线各阶段时间占比（估算）

| 阶段 | 实现 | 并行度 | 估算占比 |
|------|------|--------|---------|
| 三角化（batch + full-scan） | DLT+GN，串行 per-track | ❌ 无 | ~35% |
| 外点剔除（5 passes × 全量 obs） | 串行 CPU，O(n_obs/pass) | ❌ 无 | ~20% |
| Global BA（Ceres+CHOLMOD） | CPU 多线程，Schur complement | ✅ CPU 线程 | ~30% |
| Local BA | 同上，subset | ✅ CPU 线程 | ~8% |
| Resection（PnP RANSAC） | CPU PoseLib（GPU path 存在但非默认） | ❌ 串行 | ~5% |
| 其他（I/O、retriangulation dispatch） | — | — | ~2% |

**关键结论**：三角化是最大单一热点。以 1000 张图、100K tracks、avg 6 obs/track、500 RANSAC 对估算，仅三角化就需 **~3 亿次算术操作**，全部串行执行。

### 1.2 瓶颈逐项分析

#### 三角化（`incremental_triangulation.cpp`）
- `run_batch_triangulation()` / `run_full_scan_triangulation()`：per-track 完全独立，每 track 做 DLT（4×4 JacobiSVD）+ Gauss-Newton 精化（finite-difference Jacobian，`O(5-10 iter)`）
- `undistort_point()` 在每次三角化内循环调用：fixed-point iteration 5–15 次，CPU bound
- 无任何线程并行，也无 SIMD

#### 外点剔除（`incremental_sfm_pipeline.cpp`）
- `reject_outliers_multiview()`：遍历所有 alive observations，对每条计算 reprojection error → 若 > MAD threshold，调用 `mark_observation_deleted_restorable()`
- 每轮 O(n_obs)，pipeline 中执行 5 轮
- 访问模式：随机跳转 obs → track_xyz → pose → camera_intrinsics，cache miss 密集

#### Bundle Adjustment（`bundle_adjustment_analytic.cpp`）
- Ceres 内部多线程，但 CHOLMOD 符号分解（symbolic factorization）是超线性的，单次随图像数二次以上增长
- `DENSE_SCHUR`（≤300 cameras）无法扩展到大场景
- `ITERATIVE_SCHUR + JACOBI`（>300 cameras）：PCG 收敛慢，无 GPU 加速（当前 Ceres 配置无 CUDA）

#### Undistortion 重复计算
- 三角化、外点剔除均重复计算 undistorted 坐标，无缓存
- `undistort_point()` 实现为 pixel-space fixed-point loop，~15 次迭代，每次 Brown-Conrady 评估

#### Full-scan Retriangulation 无增量性
- `run_full_scan_triangulation()` 每次扫描全部 track，即使大部分 track 几何未发生变化
- 无 epoch / dirty bit 机制跳过稳定 track

---

## 2. Phase 0 — CPU 多线程与计算复用（1–2 周）

### Step 0.1 — 并行化批量三角化

**文件**：`src/algorithm/modules/sfm/incremental_triangulation.cpp`

`run_batch_triangulation()` 和 `run_full_scan_triangulation()` 的 per-track 内循环天然独立（tracks 间无数据依赖）。

**方案**：
1. 将 `robust_triangulate_point_multiview()` 的结果写入临时 `std::vector<TriResult>`（独立 buffer，避免 TrackStore 并发写）
2. 用 `std::for_each(std::execution::par_unseq, ...)` 或 `tbb::parallel_for` 并行执行三角化
3. 主线程串行遍历 `TriResult` 应用写回（`set_track_xyz`, `mark_observation_deleted_restorable`）

**注意**：
- TrackStore 本身不线程安全，所有写操作必须在主线程串行完成
- Gauss-Newton 内部使用 `Eigen::LDLT`，完全 per-thread 局部，安全
- `undistort_point()` 由于 Step 0.2 引入缓存后变为只读，安全并行

**预期加速**：三角化阶段 **8–16×**（16 核 CPU）

### Step 0.2 — 未畸变坐标缓存（Undistortion Cache）

> **与全 CUDA 增量 pipeline 的关系**：下文描述的是 **CPU TrackStore 侧** 的预计算缓存，用于 CPU 三角化路径加速。`CudaSfMState` / `incremental_sfm_cuda_pipeline` 则以 **GPU 上整表、按内参版本按需重算** `d_obs_xn`/`d_obs_yn` 为准（BA 优化畸变后必须失效并重算），二者勿混读为「只做一次」。

**文件**：`src/algorithm/modules/sfm/track_store.h/.cpp`

**方案**：
TrackStore 新增两个 SoA 字段：
```cpp
std::vector<float> obs_u_undist_;   // 与 obs_u_ 同长
std::vector<float> obs_v_undist_;   // 与 obs_v_ 同长
uint32_t intrinsics_version_ = 0;   // 内参版本号，BA 后 bump
```

新增方法：
```cpp
void precompute_undistorted_obs(
    const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>& image_to_camera_index);
```

**触发时机**：
- 初始加载 TrackStore 后（首次预计算）
- Bundle Adjustment 完成后，若 `camera_delta > threshold`（参考现有 `restore_observations_from_cameras` 的 `significant_change` 判据）

三角化和外点剔除中直接读 `obs_u_undist_[oi]`，跳过 `undistort_point()`。

**预期加速**：三角化内循环 **2–4×**

### Step 0.3 — SIMD 批量 Brown-Conrady 畸变计算

**文件**：`src/algorithm/modules/camera/camera_utils.cpp/.h`

新增 AVX2 批量接口：
```cpp
void apply_distortion_batch(
    const float* u, const float* v, int n,
    const Intrinsics& K,
    float* ud, float* vd);
```

Brown-Conrady 多项式为：$r^2 = u^2 + v^2$，$u_d = (1+k_1 r^2+k_2 r^4+k_3 r^6)u + \ldots$，全为 fused multiply-add，AVX2 256-bit 寄存器一次处理 8 个 float，吞吐 8×。

供 `precompute_undistorted_obs()` 和外点剔除批量投影路径调用。

### Step 0.4 — 并行化 Resection 候选评估

**文件**：`src/algorithm/modules/sfm/resection_batch.cpp`

`run_batch_resection()` 遍历 top-K candidates（如 20 个），per-candidate 调用 `resection_single_image()`，相互独立。

**方案**：CPU PoseLib 路径下用 `std::async` / OpenMP `parallel for` 并行评估所有候选，收集结果后主线程选最优。

**注意**：GPU PnP（`gpu_ransac_pnp`）共享 EGL context，**不可多线程并行**，仍串行调用。

**预期加速**：Resection 阶段 **2–4×**

---

## 3. Phase 1 — GPU 批量加速（2–4 周）

### Step 1.1 — GPU 批量外点剔除（最高 ROI）

**新文件**：`src/algorithm/gpu/gpu_outlier_rejection.h/.cpp` + `gpu_outlier_rejection.glsl`

参照 `src/algorithm/gpu/gpu_twoview_sfm.cpp` 的 EGL + OpenGL Compute Shader 架构：

**Shader 设计**（`local_size_x = 64`）：
- 每 GPU 线程处理一个 observation
- 读入：pose（7 float：quat + C）、intrinsics（9 float）、3D point（3 float）、obs (u, v)
- 计算 Brown-Conrady distorted reprojection error
- 输出：1 byte flag（inlier=1 / outlier=0）

**SSBO 布局**：

| Binding | Mode | 内容 |
|---------|------|------|
| 0 | R | `float[7 × N_imgs]` poses [qx,qy,qz,qw,Cx,Cy,Cz] |
| 1 | R | `float[9 × N_cams]` intrinsics [fx,σ,cx,cy,k1,k2,k3,p1,p2] |
| 2 | R | `float[3 × N_tracks]` track_xyz |
| 3 | R | `struct{uint32 img_id, track_id; float u, v;}[M_obs]` |
| 4 | W | `uint8[M_obs]` result_flags |

CPU 端流程：上传 → dispatch(M_obs/64 workgroups) → readback → 串行调用 `mark_observation_deleted_restorable()` 或 `mark_observation_alive()`

替换 `reject_outliers_multiview()`、`reject_outliers_angle_multiview()`、`reject_outliers_depth()` 中的 CPU 循环。

**预期加速**：30M obs 外点剔除从 ~200ms → **~4ms**（**50×**）

### Step 1.2 — GPU N-View 批量 DLT 三角化

**新文件**：`src/algorithm/gpu/gpu_triangulation.h/.cpp` + `gpu_nview_triangulation.glsl`

**方案**：
- 以 CSR 格式组织 track-observation 数据上传 GPU
- **每 workgroup（64 线程）处理一个 track**：
  - Thread 0…K：分别处理一个 observation，计算 DLT 矩阵 $A$ 的 2 行，写入 shared memory
  - Warp reduce：在 shared memory 上累加 $A^T A$（4×4 symmetric matrix，10 distinct floats）
  - Thread 0：对 4×4 $A^T A$ 做幂迭代（power iteration，8 rounds），求最小特征向量
  - Dehomogenize → 写 `float[4]`（X, Y, Z, valid）到输出 SSBO
- N=2 的 pair 直接走现有 `gpu_twoview_sfm` shader

**预期加速**：full-scan triangulation **10–30×**

### Step 1.3 — GPU PnP 改为默认 Backend

**文件**：`src/algorithm/modules/sfm/resection.cpp`，`src/algorithm/tools/isat_incremental_sfm.cpp`

现状：`opts.resection.backend` 默认为 `kPoseLib`，GPU 路径 `kGpuRansac` 需手动指定。

**方案**：
- 在 pipeline 初始化时检测 `gpu_twoview_is_initialized()`，若为 true 则自动选择 `kGpuRansac`
- 增加质量 fallback：GPU PnP 返回的 `inlier_ratio < 0.1` 时，自动降级到 CPU PoseLib 重试
- `isat_incremental_sfm.cpp` 中 `opts.resection.backend` 注释说明自动选择逻辑

---

## 4. Phase 2 — BA 架构升级（4–8 周）

### Step 2.1 — CUDA Ceres for ITERATIVE_SCHUR

**文件**：`src/algorithm/modules/sfm/bundle_adjustment_analytic.cpp`，`CMakeLists.txt`

Ceres ≥2.2 支持 `solver_options.use_cuda = true`，底层调用 cuSPARSE 做 $J^T J$ sparse multiply、cuSOLVER 做不完全 Cholesky 预条件。

**方案**：
```cmake
option(WITH_CERES_CUDA "Enable CUDA acceleration in Ceres BA" OFF)
if(WITH_CERES_CUDA)
  target_compile_definitions(... PRIVATE ISAT_CERES_CUDA=1)
endif()
```
在 BA 中条件编译：
```cpp
#if ISAT_CERES_CUDA
  if (num_cameras > 300)
    solver_options.use_cuda = true;
#endif
```

**预期加速**：大场景（>300 cameras）Global BA **2–5×**

### Step 2.2 — Track Epoch 脏位系统（增量 Retriangulation）

**文件**：`src/algorithm/modules/sfm/track_store.h/.cpp`

**方案**：
TrackStore 新增：
```cpp
uint32_t global_pose_epoch_ = 0;
std::vector<uint32_t> track_last_triangulated_epoch_;  // per-track，与 track_xyz_ 同长
```

新增方法：
```cpp
// 在每次 BA 完成后调用，传入哪些 image 的 pose 发生了显著变化
void bump_pose_epoch(const std::vector<bool>& pose_changed_mask);
uint32_t current_epoch() const { return global_pose_epoch_; }
```

`run_full_scan_triangulation()` 跳过满足以下条件的 track：
- `track_has_triangulated(tid)` 为真
- `track_last_triangulated_epoch_[tid] == global_pose_epoch_`（所有观测相机 pose 均未变化）

**预期加速**：后期 full-scan retriangulation 减少 **80–90%**

### Step 2.3 — 分频 BA 策略（Pose-only 频繁 + Full BA 稀疏）

**文件**：`src/algorithm/modules/sfm/incremental_sfm_pipeline.cpp`，`bundle_adjustment_analytic.h`

**动机**：Full BA（poses + points + intrinsics）代价高，但许多中间迭代只需要稳住 poses，无须精化 points。

**方案**：
在 `BAOptions` 中增加：
```cpp
enum class BAMode { kFull, kPoseOnly };
BAMode mode = BAMode::kFull;
```

Pose-only 模式下：对所有 `track_xyz_` blocks 调用 `problem.SetParameterBlockConstant(pt_ptr)`，Ceres 矩阵规模从 $O(n_{cameras} + n_{tracks})$ 降为 $O(n_{cameras})$。

调度策略（在 `run_global_ba_schedule()` 中实现）：
- 每 3 次 local BA 周期 → 触发一次 pose-only global BA
- 每 10 次 local BA 周期（或 `n_registered` 增长 ≥ 50）→ 触发一次 full global BA

**预期加速**：中期场景整体 BA 时间 **2×**

### Step 2.4 — Grid-NMS 并行化与缓存

**文件**：`src/algorithm/modules/sfm/incremental_sfm_pipeline.cpp`（`select_ba_subset_grid_nms()`）

**方案**：
- Per-image grid 计算相互独立，用 `#pragma omp parallel for` 并行
- 增加 per-image NMS 结果缓存 `std::vector<std::vector<int>> nms_cache_`，在 BA 写回 track XYZ 时按 dirty image set invalidate（只重算变化图像的 grid）

---

## 5. Phase 3 — 增量 BA 与超大场景（8+ 周，按需）

### Step 3.1 — iLBA（增量 Schur Complement 更新）

新增相机时，标准做法是重新组装完整 Hessian $H = J^T J$，$O(n_{cams}^2 \cdot n_{common\_pts})$。

增量 BA 维护显式 Schur complement $S = J_c J_p^{-1} J_c^T$，新增相机时只更新 $S$ 的差量行列。参考 iSAM2 / Square Root SAM 框架。

**文件**：新增 `src/algorithm/modules/sfm/incremental_ba.h/.cpp`

**预期加速**：增量局部 BA **5–20×**

### Step 3.2 — Keyframe 选择 + 滑动窗口 Global BA

在 pipeline 中引入 `KeyframeSelector`：

判据（满足任一即为 keyframe）：
- 与上一 keyframe 共视 3D 点 < 50%
- median parallax > 10°
- 新三角化 track 数 > 200

Global BA 只覆盖 keyframes，non-keyframe 通过 pose graph 约束（基于共视相对位姿）附加到优化。

### Step 3.3 — 分布式 BA（>5000 张图）

将场景按空间 kd-tree 划分为 M 个 cluster，每个 cluster 独立 local BA（M 个线程并行），cluster boundary 相机用 ADMM consensus 约束，交替求解直至收敛。

---

## 6. 数值正确性保证

所有优化必须通过以下验证：

| 测试 | 命令 | 判据 |
|------|------|------|
| 三角化单元测试 | `./test_incremental_triangulation` | 全通过 |
| 几何 RANSAC 测试 | `./test_geo_ransac` | 全通过 |
| SfM 诊断（小场景） | `./test_sfm_diag2` | 全通过 |
| SfM 诊断（完整） | `./test_sfm_diagnosis` | 全通过 |
| 端到端对比 | ETH3D benchmark（或 scripts/ 下比对脚本） | registered count 不退步，RMSE 差 ≤ 0.01px |

---

## 7. 实施优先级与风险

| Step | 预期收益 | 实施风险 | 优先级 |
|------|---------|---------|--------|
| 0.1 并行三角化 | **8–16×** tri | 低（无 GPU 依赖，TrackStore 只要两阶段写回） | ⭐⭐⭐⭐⭐ |
| 0.2 Undistortion Cache | **2–4×** tri 内循环 | 低（纯 SoA 字段追加） | ⭐⭐⭐⭐⭐ |
| 1.1 GPU 外点剔除 | **~50×** outlier rej | 中（新 GLSL shader，SSBO 数据序列化） | ⭐⭐⭐⭐ |
| 0.3 并行 Resection | **2–4×** resection | 低 | ⭐⭐⭐ |
| 0.4 SIMD 畸变 | **2–4×** batch distortion | 低（AVX2，fallback 已有） | ⭐⭐⭐ |
| 2.2 Track Epoch | **后期 90%** full-scan 跳过 | 低（纯 bookkeeping） | ⭐⭐⭐⭐ |
| 2.3 分频 BA | **2×** 中期 BA | 中（调度逻辑，需验证收敛） | ⭐⭐⭐ |
| 1.2 GPU DLT | **10–30×** full-scan tri | 高（CSR 数据重组，warp reduce 调试） | ⭐⭐⭐ |
| 2.1 CUDA Ceres | **2–5×** large BA | 高（外部 build 依赖，Ceres ≥2.2） | ⭐⭐ |
| 1.3 GPU PnP 默认 | 小增量 | 低 | ⭐⭐ |
| 3.x 增量 BA / 分布式 | **5–20×** BA | 极高（全新架构） | ⭐（按需） |

**建议起始顺序**：Step 0.2 → Step 0.1 → Step 2.2 → Step 1.1 → Step 0.3

---

## 8. 关键文件索引

| 文件 | 涉及 Step |
|------|----------|
| `src/algorithm/modules/sfm/incremental_triangulation.cpp/h` | 0.1 |
| `src/algorithm/modules/sfm/track_store.h/cpp` | 0.2, 2.2 |
| `src/algorithm/modules/camera/camera_utils.cpp/h` | 0.4 |
| `src/algorithm/modules/sfm/resection_batch.cpp` | 0.3, 1.3 |
| `src/algorithm/gpu/gpu_twoview_sfm.cpp/h` | 1.1, 1.2（参考模板） |
| `src/algorithm/gpu/gpu_outlier_rejection.*`（新建） | 1.1 |
| `src/algorithm/gpu/gpu_triangulation.*`（新建） | 1.2 |
| `src/algorithm/modules/sfm/bundle_adjustment_analytic.cpp/h` | 2.1, 2.3 |
| `src/algorithm/modules/sfm/incremental_sfm_pipeline.cpp` | 2.3, 2.4 |
| `CMakeLists.txt` | 2.1（WITH_CERES_CUDA flag） |
