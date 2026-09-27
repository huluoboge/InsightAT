# 2026-05-09 增量 SfM 全面 CUDA 化架构设计

**分支**：`feature/incremental-faster`  
**背景**：在 [2026-05-09-incremental-sfm-perf-plan.md](2026-05-09-incremental-sfm-perf-plan.md) 的基础上，进一步讨论是否值得放弃现有 OpenGL Compute Shader 方案，全面迁移到 CUDA，以彻底消除 CPU-GPU 数据传输瓶颈并支持 GPU 原生 BA。

---

## 1. 现有 GPU 基础设施的根本性限制

当前实现（`gpu_twoview_sfm.cpp`, `gpu_geo_ransac.cpp`）使用 **EGL + OpenGL 4.3 Compute Shader + SSBO** 架构。这套方案在 `isat_geo` / `isat_twoview` 等 per-pair 工具中工作良好，因为那些工具天然是"上传-处理-下载"的流式模式。

但对于**增量 SfM** 这种迭代求解问题，该架构有根本性限制：

```
每次 dispatch 调用链：
  CPU (pack data)
    → PCIe → GPU SSBO upload     ← 每次不可避免
    → GPU compute dispatch
    → GPU→CPU readback            ← 每次不可避免
  CPU (unpack, make decision)
    → next iteration ...
```

BA 的 LM 迭代需要在 GPU 上：
1. 计算残差
2. 组装 Hessian
3. 求解线性系统（更新 Δx）
4. 更新参数（poses, intrinsics, 3D points）
5. 重算残差（接受/拒绝 LM step）
6. 回到步骤 1

步骤 1-6 全部在同一 GPU 内存上操作，中间不需要任何 CPU 介入。OpenGL 架构无法做到这一点：每个 dispatch 是独立调用，SSBO 的生命周期与 C++ 函数调用挂钩，无法跨 LM 迭代持久持有。

**结论**：要让 BA（以及三角化、外点剔除）的中间状态全程留在 GPU VRAM，必须换用 CUDA。

---

## 2. iLBA 对非序列航拍 SfM 的适用性分析

### iSAM2 风格 iLBA 的前提不满足

iSAM2 / Square Root SAM 式增量 BA 的设计假设：
- 数据按**时序**到达，有明确的时间顺序  
- Factor graph 保持近似**树形**结构（fill-in 可控）
- 相邻帧共视大量相同 3D 点（warm-start 自然成立）

航拍非序列 SfM 的实际情形：
- 下一个 resection batch 可能是对面航带，与上一批共视点仅有 30%–50%
- 每次 BA 后的连通性模式不可预测，Bayes tree fill-in 成本高
- 没有滑动窗口：任意图像可以与任意其他图像相连
- Global BA 每次涉及完全不同的变量集合

iSAM2 的核心收益来自于 Bayes tree 的**局部重排**（只更新受新 measurement 影响的子树），而 SfM 全局优化的 normal equations 是稠密的（每个 3D 点连接多个相机），这个结构让 iSAM2 的稀疏性优势大打折扣。

### 但 GPU Schur Complement 数据驻留给你等效收益

我们真正需要的不是 iSAM2 的因子图增量更新，而是：

**目标**：避免每次 Global BA 重新从零组装 $H = J^T W J$，利用已有的 $H_{old}$ 做增量更新。

当新增 $k$ 张相机（$k \ll N_{total}$）时：

$$H_{new} = H_{old} + J_{new}^T W_{new} J_{new}$$

其中 $J_{new}$ 只包含与新相机相关的残差块。在 GPU 上把 $H_{old}$ 保留在 VRAM，只 atomic-add 新相机的贡献，代价是 $O(k \times n_{new\_obs})$ 而非 $O(N \times M_{all\_obs})$。

更重要的是：**Schur complement 的 $H_{pp}^{-1}$（per-point 3×3 逆）可以增量维护**。已有 3D 点的 $H_{pp}^{-1}$ 在相机不变时不变；只有被新相机观测到的 3D 点需要更新 $H_{pp}$。

这不是 iSAM2，而是更简单的**增量 rank-update on persistent GPU Hessian**，完全适合非序列 SfM。

---

## 3. 全 CUDA 架构设计

### 3.1 GPU 持久状态（GpuSfMState）

Pipeline 启动时一次性分配，整个增量 SfM 过程不释放：

```cpp
// src/algorithm/gpu/gpu_sfm_state.h
struct GpuSfMState {
  // ── 相机与内参 ─────────────────────────────────────────────────────────
  double* d_poses;        // [N_imgs × 7] float64: [qx,qy,qz,qw, Cx,Cy,Cz]
  double* d_intrinsics;   // [N_cams × 9] float64: [fx,σ,cx,cy,k1,k2,k3,p1,p2]
  bool*   d_registered;   // [N_imgs]     bool

  // ── 3D 点 ──────────────────────────────────────────────────────────────
  float*   d_track_xyz;   // [T × 3]  float32 (对三角化和外点剔除精度足够)
  uint8_t* d_track_flags; // [T]      kAlive | kHasXYZ | kSkipBA | kNeedsRetri

  // ── 观测（Observation）────────────────────────────────────────────────
  float*    d_obs_u;      // [M]
  float*    d_obs_v;      // [M]
  uint32_t* d_obs_img;    // [M]
  uint32_t* d_obs_track;  // [M]
  uint32_t* d_obs_cam;    // [M]  (= image_to_camera_index[img])
  uint8_t*  d_obs_flags;  // [M]  kAlive | kRestorable

  // ── CSR 索引（观测的两个方向索引）────────────────────────────────────
  int* d_img_obs_ptr;    // [N_imgs + 1]  CSR row pointers (per-image)
  int* d_img_obs_idx;    // [M]           CSR column indices
  int* d_track_obs_ptr;  // [T + 1]       CSR row pointers (per-track)
  int* d_track_obs_idx;  // [M]           CSR column indices

  // ── BA Hessian（增量维护）────────────────────────────────────────────
  // H_cc: [N_imgs × 49] 每个相机 7×7 diagonal block (float64)
  // H_ii: [N_cams × 81] 每个内参 9×9 diagonal block (float64)  
  // H_pp: [T × 9]       每个 3D 点 3×3 block，及其逆 (float64)
  // H_cp: sparse CSR    相机-点 cross terms (float64)
  double* d_H_cc;
  double* d_H_ii;
  double* d_H_pp;
  double* d_inv_H_pp;

  // ── LM 求解中间量（per BA call 复用）────────────────────────────────
  double* d_g_c;   // [N_imgs × 7]   camera gradient
  double* d_g_i;   // [N_cams × 9]   intrinsics gradient
  double* d_g_p;   // [T × 3]        point gradient
  double* d_S;     // [N_imgs × N_imgs × 49] Schur complement (dense or sparse CSR)
  double* d_rhs;   // Schur RHS

  // ── 尺寸记录 ──────────────────────────────────────────────────────────
  int n_images, n_cameras, n_tracks, n_obs;
  int cap_images, cap_cameras, cap_tracks, cap_obs;
};
```

所有 SfM 算子（三角化、外点剔除、BA）都接受 `GpuSfMState*`，直接操作这块内存，**不进行任何 cudaMemcpy**。

### 3.2 CUDA 核心 Kernel 设计

#### Kernel 1：批量三角化（N-view DLT）

```cuda
// 每个 block 处理一个 track
// block_dim = 64 (max obs per track for shared memory)
__global__ void kernel_triangulate_nview(
    const int*     track_obs_ptr,   // CSR row pointers
    const int*     track_obs_idx,   // CSR column indices
    const float*   d_obs_u_undist,  // 预畸变校正坐标（见 3.3）
    const float*   d_obs_v_undist,
    const uint32_t* d_obs_img,
    const double*  d_poses,         // [7×N] 
    const double*  d_intrinsics,    // [9×C]
    const uint32_t* d_obs_cam,
    float*         d_track_xyz,     // OUTPUT
    uint8_t*       d_track_flags,
    int            n_tracks
);
// 每个 block 内：
//   - Thread 0..K: 各取一个 obs，从 d_obs_u_undist 读归一化坐标，计算 DLT A 的 2 行
//   - __shared__ float ATA[10];  // 4×4 对称阵的10个上三角元素
//   - warp reduce 累加 ATA
//   - Thread 0: power iteration (8 rounds) on ATA → 最小特征向量
//   - Thread 0: dehomogenize → 写 d_track_xyz，更新 d_track_flags
```

#### Kernel 2：批量外点剔除（完整畸变模型）

```cuda
// 每个 thread 处理一个 observation，embarrasingly parallel
__global__ void kernel_outlier_rejection(
    const float*   d_obs_u, d_obs_v,      // 原始像素坐标
    const uint32_t* d_obs_img,
    const uint32_t* d_obs_track,
    const uint32_t* d_obs_cam,
    const double*  d_poses,               // [7×N_imgs]
    const double*  d_intrinsics,          // [9×N_cams]
    const float*   d_track_xyz,           // [3×T]
    uint8_t*       d_obs_flags,           // R/W: kAlive, kRestorable
    float          threshold_px,          // 如 8.0f (MAD 自适应在 CPU 计算，一次 memcpy)
    int            n_obs
);
// 每个 thread：
//   1. 读 (pose, K, 3D_point, u, v)
//   2. Brown-Conrady 投影  (FP32 完全够)
//   3. |reproj_error| > threshold → 清 kAlive 置 kRestorable
```

#### Kernel 3：BA Jacobian 组装（FP32 accumulate）

```cuda
// 每个 thread 计算一个 observation 的 2×(7+9+3) Jacobian，
// 并 atomicAdd 到对应的 H blocks 和 g 向量
__global__ void kernel_ba_assemble_H(
    // inputs: 同上，加上 d_H_cc/ii/pp/cp 和 d_g_c/i/p
    // 使用解析 Jacobian（复用 bundle_adjustment_analytic.cpp 中的公式，翻译为 CUDA）
    // 关键优化：
    //   - 先写 thread-local accumulator（寄存器），
    //     batch 内 __syncwarp() 后再 warpReduce → 再 atomicAdd
    //   - 避免 per-obs atomicAdd（M × 49 次 atomic 成为 M/32 次 warpReduce + M/32 次 atomic）
);
```

#### Kernel 4：Schur Complement 计算（FP64）

```cuda
// Step 1: 计算 inv(H_pp) — 每个 thread 处理一个 3D 点
__global__ void kernel_invert_hpp(
    const double* d_H_pp,  // [T × 9]
    double*       d_inv_Hpp // [T × 9]
);

// Step 2: Schur RHS g_c_bar = g_c - H_cp * inv(H_pp) * g_p
//         Schur matrix S = H_cc - H_cp * inv(H_pp) * H_cp^T
// 用 cuSPARSE 的 SpMM 完成稀疏矩阵乘法
```

#### Kernel 5：参数更新

```cuda
// 每个 thread 更新一个 pose（或 intrinsics、3D point）
__global__ void kernel_update_params(
    double* d_poses, const double* d_delta_c,  // pose 更新
    double* d_intrinsics, const double* d_delta_i,
    float*  d_track_xyz, const double* d_delta_p,
    // Manifold: quaternion normalization inline
);
```

### 3.3 Undistortion 派生字段 on GPU（按需、版本化）

在 `GpuSfMState` / `CudaSfMState` 中持有：
```cpp
float* d_obs_u_undist;  // [M] 由当前 d_intrinsics 派生的去畸变归一化坐标
float* d_obs_v_undist;  // [M]
uint32_t intrinsics_version;  // BA 每写回一次内参可递增；与缓存代数比较
```

**语义**：这是**派生缓存**，不是静态预处理。BA 会优化 fx、cy、畸变等，只要内参与生成该缓存时不一致，就必须在 GPU 上重算（全表 `kernel_precompute_undistorted` / `undistort_all` 一次通常仍可接受）。

```cuda
__global__ void kernel_precompute_undistorted(
    const float* d_obs_u, d_obs_v,
    const uint32_t* d_obs_cam,
    const double* d_intrinsics,
    float* d_obs_u_undist, d_obs_v_undist,
    int n_obs
);
// 每个 thread: 一个 obs → fixed-point undistort（FP32，迭代次数可参数化）
```

触发：pipeline 启动后首次、以及**每次 BA 更新内参之后**（或按多项系数阈值判定变化）。三角化 / resection 内循环只读与当前 K 一致的缓存，不在内核里重复做 undistort。

### 3.4 全 GPU BA 循环（消除 LM 迭代内的 PCIe 传输）

```
GPU 上的完整 LM iteration（无 CPU 介入）:
┌─────────────────────────────────────────────────────────┐
│  kernel_ba_assemble_H (FP32 Jacobian, FP32 accumulate)  │
│  kernel_invert_hpp (FP64)                               │
│  cuSPARSE: S = H_cc - E * inv(H_pp) * E^T (FP64)       │
│  cuSOLVER csrlsvchol: solve S*Δc = g_c_bar (FP64)       │
│  kernel_backsubstitute: Δp = inv(H_pp)*(g_p - E^T Δc)  │
│  kernel_update_params: x += Δx (FP64 update, FP32 store)│
│  kernel_compute_cost: Σ ρ(r^T W r)  → d_cost[1]        │
└─────────────────────────────────────────────────────────┘
         ↑ 仅在此处一次微小 cudaMemcpy(d_cost, h_cost, 8B)
         CPU 决定是否接受 LM step（δ cost / λ 调整）
         ↓ 若拒绝：kernel_revert_params（从 d_params_backup 恢复）
```

全程只有 **1 个 8-byte cudaMemcpy**（cost value），其余全在 VRAM 内完成。与 Ceres CPU 的 LM 迭代相比，每次迭代省去了：
- Jacobian → Ceres problem 的 CPU 组装
- CHOLMOD 调用的 CPU 算法开销
- 结果写回 TrackStore 的 CPU 访存

---

## 4. 两阶段迁移路线图

### Phase 1 — 非 BA 部分全 CUDA 化（4–6 周）

**目标**：保留 Ceres 做 BA，但三角化、外点剔除、resection 全部迁移到 CUDA，且引入 `GpuSfMState` 持久 GPU 内存，消除这些操作的 PCIe 传输。

**Step 1.1 — 引入 CUDA 构建支持**

`CMakeLists.txt` 增加：
```cmake
option(WITH_CUDA "Build CUDA-accelerated SfM kernels" OFF)
if(WITH_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_ARCHITECTURES "75;86;89") # Turing, Ampere, Ada
endif()
```

新目录 `src/algorithm/cuda/`，CMakeLists 实现 `WITH_CUDA` 条件编译，保证无 CUDA 环境时全部 fallback 到现有 CPU 实现。

**Step 1.2 — GpuSfMState 生命周期管理**

新文件 `src/algorithm/cuda/gpu_sfm_state.h/.cu`：
- `gpu_sfm_state_init(N_imgs, N_cams, T_tracks, M_obs)` — 一次 cudaMalloc
- `gpu_sfm_state_upload_project(project)` — 初次上传 project 数据到 GPU
- `gpu_sfm_state_reindex_csr()` — obs 添加/删除后重建 CSR（仅在 batch 边界调用）
- `gpu_sfm_state_shutdown()` — cudaFree

**Step 1.3 — CUDA 三角化 kernels**

新文件 `src/algorithm/cuda/cuda_triangulation.h/.cu`：
- `cuda_triangulate_batch(state, track_ids[], n)` — 只三角化指定 track 集合
- `cuda_triangulate_full_scan(state)` — 复用 epoch 系统跳过稳定 track
- 内部调用 `kernel_precompute_undistorted` + `kernel_triangulate_nview`

**Step 1.4 — CUDA 外点剔除 kernels**

新文件 `src/algorithm/cuda/cuda_outlier_rejection.h/.cu`：
- `cuda_reject_outliers_reproj(state, threshold_px)` — 替换 `reject_outliers_multiview()`
- MAD 自适应阈值：先 `kernel_compute_reproj_errors` → `std::nth_element` on GPU（cub::DeviceSelect）→ MAD → 再 kernel 标记
- `cuda_reject_outliers_angle(state)` — 替换 `reject_outliers_angle_multiview()`
- `cuda_reject_outliers_depth(state)` — 替换 `reject_outliers_depth()`

**Step 1.5 — CUDA PnP RANSAC**

新文件 `src/algorithm/cuda/cuda_resection.h/.cu`：
- P3P minimal solver（每 thread 一个 RANSAC hypothesis）
- `cub::DeviceReduce::Max` 统计最佳 inlier count
- 替换现有 `gpu_ransac_pnp`（OpenGL 实现），成为 `kGpuCuda` backend

**Step 1.6 — 旧 OpenGL GPU 模块保留**

`gpu_geo_ransac` 和 `gpu_twoview_sfm` 继续保留，仅用于 `isat_geo` / `isat_twoview` 这两个 per-pair 工具。增量 SfM pipeline 完全切换到 CUDA 路径。两套 GPU 代码并存，不混用 context。

**阶段性验收指标**：
- `test_incremental_triangulation` / `test_sfm_diag2` 全通过
- 1000 张测试集上，Phase 1 后三角化 + 外点剔除 + resection 时间 ≤ Phase 0 的 1/8
- BA 时间不变（仍用 Ceres）

### Phase 2 — 自定义 CUDA Schur Complement BA（6–10 周）

**目标**：实现 GPU-native BA，整个 LM 迭代在 GPU 内完成，消除 BA 阶段的 CPU-GPU 传输，并支持增量 Hessian 更新。

**Step 2.1 — CUDA BA 核心实现**

新文件 `src/algorithm/cuda/cuda_ba.h/.cu`：
- `CudaBASolver` 类，维护 `d_H_cc`, `d_H_pp`, `d_inv_Hpp`, `d_S`, `d_H_cp_csr`
- `solve_global_ba(state, options)` — 完整 LM 迭代，内部 loop 不出 GPU
- `solve_local_ba(state, variable_cam_ids[], options)` — 局部 BA，相同 solver

**Jacobian 解析公式**：
直接移植 `bundle_adjustment_analytic.cpp` 中的手写解析 Jacobian（quaternion 的 Sola 公式 + Brown-Conrady 链式法则）到 CUDA device function。代码量约 150 行，已有 CPU 版本验证正确。

**Step 2.2 — 精度与数值稳定性策略**

- Jacobian 计算：**FP32**，使用 `__fmaf_rn` fused multiply-add
- H 矩阵累加：**FP32** + Kahan summation（对角块竟然 FP32 就够，因为条件数 < 1e5）
- Schur 消元（`cuSPARSE::SpMM`）：**FP64**
- `-lsvchol` 求解：**FP64**（`cusolverSpDcsrlsvchol`）
- 参数更新 Δx：**FP64** delta × **FP32** 参数 → **FP32** 存储

对于 RTX 系列 GPU，通过此混合精度策略，FP64 算量仅占总 BA 计算的约 20%，其余全走 FP32 高吞吐路径。

**Step 2.3 — 增量 Hessian 更新**

`CudaBASolver` 维护 `uint32_t hessian_epoch`，每次 resection batch 后：
- 仅对新增 obs 的对应 H 块做 `atomic += J^T W J`（incremental update）
- 已有 3D 点的 `d_H_pp` 只更新被新相机观测到的点（稀疏 update）
- 全局 BA 时: 使用增量 H + cuSOLVER 直接求解，比重建快 3–5×（取决于 batch size / total size 比）

**Step 2.4 — Ceres 仍保留为 reference/fallback**

`--use-ceres-ba` flag（默认 off when CUDA available）：
- 用于数值精度验证：对比 CUDA BA 和 Ceres BA 的 RMSE 差异
- CI 中用 Ceres 作为 ground truth，CUDA BA 结果差异 < 0.01px 为通过

---

## 5. VRAM 容量评估

典型任务规模：1000 张图 × 500K tracks × avg 8 obs/track = **4M observations**

| 数据 | 大小 | 内存 |
|------|------|------|
| poses: 1000 × 7 × 8B | FP64 | ~56 KB |
| intrinsics: 10 × 9 × 8B | FP64 | ~720 B |
| track_xyz: 500K × 3 × 4B | FP32 | ~6 MB |
| track_flags: 500K | uint8 | ~500 KB |
| obs_u/v: 4M × 4B × 2 | FP32 | ~32 MB |
| obs_img/track/cam/flags: 4M × 4B × 4 | uint32 | ~64 MB |
| CSR indices: 4M × 4B × 4 | int32 | ~64 MB |
| H_pp: 500K × 9 × 8B × 2 (+ inv) | FP64 | ~72 MB |
| H_cc: 1000 × 49 × 8B | FP64 | ~392 KB |
| Schur S（dense 1000×1000×7×7）| FP64 | ~392 MB |
| **Total** | | **~640 MB** |

RTX 3090 有 24GB VRAM，即使 5000 张图（~20× 规模）也只需约 **4–5 GB**，完全可行。超大场景（>10000 张）时 Schur complement 需要稀疏存储（cuSPARSE CSR），约 ~2 GB 以内。

---

## 6. 构建与依赖变化

新增依赖（`WITH_CUDA=ON` 时）：

```
CUDA Toolkit >= 11.8
  - cublas, cusparse, cusolver (通常随 toolkit 打包)
  - cub (header-only, toolkit 内置)
```

CMake 变更：
- `enable_language(CUDA)` 在顶层 CMakeLists（条件）
- `target_compile_options(... $<$<COMPILE_LANGUAGE:CUDA>:-O3 --use_fast_math ...>)`
- `target_link_libraries(... CUDA::cusparse CUDA::cusolver_static)`

Docker 镜像：基于 `nvidia/cuda:11.8.0-devel-ubuntu22.04`（已在 Dockerfile 中），加上 `-DWITH_CUDA=ON` 即可，无额外依赖。

---

## 7. 关键工程风险与应对

| 风险 | 详情 | 应对 |
|------|------|------|
| FP64 吞吐在消费级 GPU 慢 | RTX 4090 FP64 仅为 FP32 的 1/56 | 混合精度策略（第 3.4 节），Schur solve 是唯一 FP64 热点，通常 < 10ms |
| CSR 索引在 obs 删除后失效 | 外点剔除修改 d_obs_flags，CSR 行指针失效 | CSR 仅在 batch 边界（进入全局 BA 前）重建；外点剔除用 flag-based 方案，不改变 CSR 结构 |
| 增量 H 更新的数值漂移 | 多轮 incremental += 可能积累浮点误差 | 每隔 N 次全局 BA 做一次完整 H 重建（默认 N=5）；收敛用 Ceres 做 final polish |
| CUDA kernel 调试困难 | 相比 CPU 调试复杂 | 所有 kernel 有 CPU reference 实现，CI 跑对比测试；使用 `cuda-gdb` 和 `compute-sanitizer` |
| 多 GPU context（OpenGL + CUDA 并存）| isat_geo 等旧工具仍用 OpenGL EGL | 两者在不同进程运行，不共享 context；增量 SfM process 只用 CUDA，无冲突 |

---

## 8. 预期性能收益汇总

基于 RTX 3090，1000 张图，500K tracks，4M obs：

| 阶段 | 当前 CPU 时间 | Phase 1 估计 | Phase 2 估计 |
|------|------------|-------------|-------------|
| 三角化（全扫描） | ~8 min | ~20 sec（CUDA）| ~8 sec（+epoch skip）|
| 外点剔除（5 passes） | ~3 min | ~0.5 sec（CUDA）| ~0.5 sec |
| Resection (100 images) | ~2 min | ~15 sec（CUDA PnP）| ~15 sec |
| Local BA（per batch） | ~30 sec | ~30 sec（仍 Ceres）| ~3 sec（CUDA BA，小问题） |
| Global BA（定期）| ~5 min | ~5 min（仍 Ceres）| ~30 sec（CUDA Schur + incremental H） |
| **总计（1000张完整重建）** | **~45 min** | **~12 min** | **~3 min** |

---

## 9. 实施起点建议

基于 ROI 和风险评估，建议起始顺序：

1. **Step 1.1**（CMake CUDA 支持）— 1 天，无破坏性变更
2. **Step 1.2**（GpuSfMState 骨架）— 2–3 天，无外部依赖
3. **Step 1.4**（CUDA 外点剔除）— 最高 ROI，且算法最简单，约 1 周
4. **Step 1.3**（CUDA 三角化）— 配合 undistortion cache，约 1.5 周
5. **Step 1.5**（CUDA PnP）— 约 1 周
6. **Step 2.1**（CUDA BA，先做 local BA 这个小问题版本）— 约 3 周，分步验证
7. **Step 2.1**（CUDA BA，global BA）— 在 local 验证无误后扩展

每个 Step 完成后跑全套测试（`test_incremental_triangulation`, `test_sfm_diag2`, ETH3D benchmark）再进入下一步。
