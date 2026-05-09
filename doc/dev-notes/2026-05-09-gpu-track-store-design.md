# 2026-05-09 TrackStore GPU 持久化架构设计

**分支**：`feature/incremental-faster`  
**关联文档**：[2026-05-09-incremental-sfm-cuda-architecture.md](2026-05-09-incremental-sfm-cuda-architecture.md)

Track 构建完毕后，整个 TrackStore 一次性上传到 GPU VRAM，后续的 resection、triangulation、外点剔除全部在 GPU 上原地操作，直到 pipeline 结束。本文档记录该设计的完整细节。

---

## 1. 当前 TrackStore 内存布局分析

### 1.1 可直接上传的 flat SoA 数组

| 字段 | 类型 | 大小（M obs, T tracks）| 备注 |
|------|------|------|------|
| `track_xyz_` | `float[3T]` | 12T bytes | 三角化结果，GPU 写 |
| `track_flags_` | `uint8_t[T]` | T bytes | alive/retri/hasXYZ/skipBA，GPU 写 |
| `obs_u_` | `float[M]` | 4M bytes | 输入，只读 |
| `obs_v_` | `float[M]` | 4M bytes | 输入，只读 |
| `obs_scale_` | `float[M]` | 4M bytes | 输入，只读 |
| `obs_flags_` | `uint8_t[M]` | M bytes | alive/restorable，GPU 写 |
| `obs_track_id_` | `int[M]` | 4M bytes | 输入，只读 |
| `obs_image_id_` | `uint32_t[M]` | 4M bytes | 输入，只读 |
| `obs_feature_id_` | `uint32_t[M]` | 4M bytes | 输入，只读（resection 需要） |

以上数组均为 `std::vector<T>::data()` 连续内存，一次 `cudaMemcpy` 上传。

### 1.2 需要转换的 vector-of-vector 索引

```cpp
std::vector<std::vector<int>> track_obs_ids_;  // T × variable
std::vector<std::vector<int>> image_obs_ids_;  // N × variable
```

这两个是 GPU 无法直接使用的嵌套容器，需要在上传前转换为 **CSR（Compressed Sparse Row）** 格式：

```
track_obs_ptr[T+1]  int32  // track i 的 obs 从 track_obs_idx[ptr[i]] 到 ptr[i+1]
track_obs_idx[M]    int32  // obs 全局 id（obs 在 flat obs_* 数组中的下标）

img_obs_ptr[N+1]    int32  // 同上，按 image 维度
img_obs_idx[M]      int32
```

**关键不变量**：Track 构建完成后，observation 不会再新增（isat_tracks 输出的是静态 TrackStore）。因此 CSR 结构在整个 SfM pipeline 中**完全不变**，只需 CPU 侧一次转换后上传，此后只读。

只有以下字段在 GPU 上会被修改：
- `d_track_xyz`（三角化写入）
- `d_track_flags`（三角化 / 外点剔除写入）
- `d_obs_flags`（外点剔除写入）

---

## 2. GpuTrackStore 结构定义

```cuda
// src/algorithm/cuda/gpu_track_store.h

struct GpuTrackStore {
  // ── 规模 ───────────────────────────────────────────────────────────────
  int n_tracks;    // T
  int n_images;    // N
  int n_obs;       // M
  int n_cameras;   // C (distinct cameras)

  // ── Track 数据（GPU 可写）─────────────────────────────────────────────
  float*   d_track_xyz;    // [3T] float32: [x0,y0,z0, x1,y1,z1, ...]
  uint8_t* d_track_flags;  // [T]  bit0=kAlive bit1=kNeedsRetri bit2=kHasXYZ bit3=kSkipBA

  // ── Observation 数据（只读，不变）─────────────────────────────────────
  float*    d_obs_u;        // [M] 原始像素坐标
  float*    d_obs_v;        // [M]
  float*    d_obs_scale;    // [M]
  uint8_t*  d_obs_flags;    // [M] bit0=kAlive bit1=kRestorable  ← GPU 可写
  int*      d_obs_track;    // [M] → track index
  uint32_t* d_obs_img;      // [M] → image index
  uint32_t* d_obs_feat;     // [M] feature id (resection 用)

  // ── CSR 索引（只读，不变）────────────────────────────────────────────
  int* d_track_obs_ptr;  // [T+1]
  int* d_track_obs_idx;  // [M]   obs global id，按 track 排列
  int* d_img_obs_ptr;    // [N+1]
  int* d_img_obs_idx;    // [M]   obs global id，按 image 排列

  // ── Undistortion 缓存（GPU 可写，内参变化时重建）────────────────────
  float* d_obs_u_n;   // [M] 去畸变归一化坐标（供三角化用）
  float* d_obs_v_n;   // [M]
  uint32_t undist_epoch;  // 与 intrinsics_version 对比，决定是否需要重建

  // ── 相机 / 内参（与 GpuSfMState 共享指针）───────────────────────────
  // BA 结束后更新，直接使用 GpuSfMState 的 d_poses / d_intrinsics
  double*  d_poses;       // [N×7] qx qy qz qw Cx Cy Cz  (来自 GpuSfMState)
  double*  d_intrinsics;  // [C×9] fx σ cx cy k1 k2 k3 p1 p2
  uint32_t* d_img2cam;    // [N]   image_index → camera_index
  bool*    d_registered;  // [N]
};
```

> **`d_poses` / `d_intrinsics` 不在 GpuTrackStore 内独立分配，而是直接指向 GpuSfMState 的对应字段**，避免数据复制。BA 一旦在 GpuSfMState 内更新了 poses/intrinsics，GpuTrackStore 立刻看到最新值。

---

## 3. 上传流程（一次性）

```cpp
// src/algorithm/cuda/gpu_track_store.cu

GpuTrackStore* gpu_track_store_upload(const TrackStore& cpu_store,
                                      const std::vector<int>& img2cam,
                                      GpuSfMState* sfm_state) {
  // Step 1: 分配 GpuTrackStore
  auto* g = new GpuTrackStore{};
  g->n_tracks  = (int)cpu_store.num_tracks();
  g->n_images  = cpu_store.num_images();
  g->n_obs     = (int)cpu_store.num_observations();
  g->n_cameras = sfm_state->n_cameras;

  // Step 2: 直接上传 flat SoA（各数组一次 cudaMalloc + cudaMemcpy）
  upload_flat(g->d_track_xyz,   cpu_store.track_xyz_.data(),   3*T * sizeof(float));
  upload_flat(g->d_track_flags, cpu_store.track_flags_.data(), T   * sizeof(uint8_t));
  upload_flat(g->d_obs_u,       cpu_store.obs_u_.data(),       M   * sizeof(float));
  upload_flat(g->d_obs_v,       cpu_store.obs_v_.data(),       M   * sizeof(float));
  upload_flat(g->d_obs_scale,   cpu_store.obs_scale_.data(),   M   * sizeof(float));
  upload_flat(g->d_obs_flags,   cpu_store.obs_flags_.data(),   M   * sizeof(uint8_t));
  upload_flat(g->d_obs_track,   cpu_store.obs_track_id_.data(),M   * sizeof(int));
  upload_flat(g->d_obs_img,     cpu_store.obs_image_id_.data(),M   * sizeof(uint32_t));
  upload_flat(g->d_obs_feat,    cpu_store.obs_feature_id_.data(), M * sizeof(uint32_t));

  // Step 3: 构建 CSR（CPU 一次扫描，O(T + N + M)）
  build_and_upload_csr(cpu_store.track_obs_ids_,
                       &g->d_track_obs_ptr, &g->d_track_obs_idx, T, M);
  build_and_upload_csr(cpu_store.image_obs_ids_,
                       &g->d_img_obs_ptr, &g->d_img_obs_idx, N, M);

  // Step 4: 分配 undistortion 缓存（内容延迟计算）
  cudaMalloc(&g->d_obs_u_n, M * sizeof(float));
  cudaMalloc(&g->d_obs_v_n, M * sizeof(float));
  g->undist_epoch = UINT32_MAX;  // 强制首次重建

  // Step 5: 共享 GpuSfMState 的 poses/intrinsics/img2cam 指针
  g->d_poses      = sfm_state->d_poses;
  g->d_intrinsics = sfm_state->d_intrinsics;
  upload_flat(g->d_img2cam, img2cam.data(), N * sizeof(uint32_t));
  cudaMalloc(&g->d_registered, N * sizeof(bool));
  cudaMemset(g->d_registered, 0, N * sizeof(bool));

  return g;
}
```

**CSR 构建函数**（CPU 端，O(T + M)）：

```cpp
void build_and_upload_csr(const std::vector<std::vector<int>>& vv,
                          int** d_ptr, int** d_idx, int n_rows, int n_vals) {
  // host-side flattening
  std::vector<int> h_ptr(n_rows + 1);
  std::vector<int> h_idx;
  h_idx.reserve(n_vals);
  for (int i = 0; i < n_rows; ++i) {
    h_ptr[i] = (int)h_idx.size();
    for (int v : vv[i]) h_idx.push_back(v);
  }
  h_ptr[n_rows] = (int)h_idx.size();

  cudaMalloc(d_ptr, (n_rows + 1) * sizeof(int));
  cudaMalloc(d_idx, h_idx.size() * sizeof(int));
  cudaMemcpy(*d_ptr, h_ptr.data(), (n_rows + 1) * sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy(*d_idx, h_idx.data(), h_idx.size() * sizeof(int), cudaMemcpyHostToDevice);
}
```

**上传总开销**（典型 100K tracks，4M obs）：

| 数据 | 大小 | 单向 PCIe 时间（~12 GB/s）|
|------|------|------|
| Flat SoA 数组 | ~60 MB | ~5 ms |
| CSR indices | ~32 MB | ~3 ms |
| **合计** | **~92 MB** | **~8 ms** |

一次上传，整个 SfM 过程（数分钟）都不需要再传。

---

## 4. GPU 操作设计

### 4.1 Undistortion 缓存重建

在三角化之前，以及每次 BA 更新内参后，调用：

```cuda
__global__ void kernel_undistort_obs(
    const float* d_obs_u, const float* d_obs_v,
    const uint32_t* d_obs_img, const uint32_t* d_img2cam,
    const double* d_intrinsics,  // [C×9]
    const uint8_t* d_obs_flags,
    float* d_obs_u_n, float* d_obs_v_n,  // OUTPUT: normalized undistorted
    int M
) {
  // 每个 thread 处理一个 obs（只处理 kAlive 的）
  // 5 次 fixed-point iteration（FP32，比 CPU 的 15 次少，因为不需要 pixel-space 精度那么高）
  // 写 d_obs_u_n[i] = (u_undist - cx) / fx, d_obs_v_n[i] = (v_undist - cy) / fy
}
```

调用时机：
- 首次进入 pipeline 后
- Global BA 结束后，若内参变化 > 阈值（`|Δfx/fx| > 1e-4` 或 `|Δk1| > 1e-5`）

这样三角化 kernel 中不再有任何 undistort 逻辑，直接读 `d_obs_u_n`。

### 4.2 批量三角化

```cuda
// grid: n_active_tracks blocks，block_dim = min(64, max_obs_per_track)
__global__ void kernel_triangulate_batch(
    const int*     d_track_ids,    // [n_active] 待三角化的 track id 列表
    const int*     d_track_obs_ptr,
    const int*     d_track_obs_idx,
    const float*   d_obs_u_n,      // 已去畸变归一化坐标（只读缓存）
    const float*   d_obs_v_n,
    const uint32_t* d_obs_img,
    const uint8_t* d_obs_flags,    // 读：检查 kAlive
    const double*  d_poses,        // [N×7]
    const bool*    d_registered,
    float*         d_track_xyz,    // 写：三角化结果
    uint8_t*       d_track_flags,  // 写：set kHasXYZ
    int n_active
) {
  // block_idx → track_id
  // shared memory: float ATA[10] (4×4 上三角), float b[3], int n_inl
  // threads 并行计算各 obs 对 ATA 的贡献，warp reduce 累加
  // thread 0: power iteration → X → cheirality check → d_track_xyz 写 → flag 更新
}
```

对于 full-scan triangulation，`d_track_ids` 传入所有未收敛的 track（epoch 机制过滤稳定 track）。对于 batch triangulation，传入新注册图像的关联 track 子集。

### 4.3 外点剔除（3 种合并为一个 pass）

```cuda
// grid: ceil(M / 256) blocks
__global__ void kernel_reject_outliers(
    const float*   d_obs_u, d_obs_v,   // 原始像素坐标
    const uint32_t* d_obs_img,
    const int*     d_obs_track,
    const uint32_t* d_obs_feat,
    const uint8_t* d_obs_flags,        // 读
    const double*  d_poses,
    const double*  d_intrinsics,
    const uint32_t* d_img2cam,
    const bool*    d_registered,
    const float*   d_track_xyz,
    float*         d_reproj_errors,    // 中间输出：每个 obs 的重投影误差
    int M
) {
  // 每个 thread: obs_id → 投影 → ‖r‖ → d_reproj_errors[obs_id]
  // Brown-Conrady FP32，无分支
}

// 第二步：CPU 读取 d_reproj_errors，计算 MAD threshold
// (cub::DeviceRadixSort + nth_element, 或直接 cudaMemcpy 8KB 样本到 CPU 估算)
// 第三步：
__global__ void kernel_apply_outlier_flags(
    const float*   d_reproj_errors,
    const float    threshold_px,
    uint8_t*       d_obs_flags,     // 写：clear kAlive / set kRestorable
    uint8_t*       d_track_flags,   // 写：set kNeedsRetri（若 track 失去足够支撑）
    const int*     d_track_obs_ptr, // 用于统计每 track 存活 obs 数
    const int*     d_track_obs_idx,
    int M
) {}
```

**关于 MAD 阈值计算**：GPU 计算所有 `d_reproj_errors` 后，用 `cub::DeviceSelect::Flagged` 过滤有效误差，再 `cub::DeviceRadixSort` 排序取中位数。全程在 GPU 上完成，无需 cudaMemcpy。

### 4.4 Resection（CUDA P3P RANSAC）

```cuda
// 每个 thread 处理一个 RANSAC hypothesis
__global__ void kernel_p3p_ransac(
    const int*     d_obs_3d_indices,   // 当前候选图像的 track 有 3D 点的 obs list
    const float*   d_obs_u_n,          // 去畸变归一化坐标
    const float*   d_obs_v_n,
    const float*   d_track_xyz,
    const int*     d_sample_indices,   // [n_hyp × 3] 预生成的随机 minimal sample 下标
    float*         d_best_pose,        // [7] OUTPUT: qx qy qz qw Cx Cy Cz
    int*           d_best_inlier_count,
    float          threshold_n,        // 归一化坐标下的阈值
    int n_2d3d_pairs, int n_hyp
) {
  // 每个 thread: 取 3 个对应对 → P3P solver（4 solutions）
  //             count inliers → atomicMax 更新 d_best_inlier_count
  // block reduce 后 thread 0 写 d_best_pose
}
```

Resection 候选选择（`choose_next_resection_image`）仍在 CPU 执行（逻辑简单，不是热点）。一旦选出候选图像 id，立刻在 GPU 上完成 RANSAC。

RANSAC 结束后，pose 结果留在 `GpuSfMState::d_poses`，更新 `d_registered` 标志，**不需要 cudaMemcpy 到 CPU**。

---

## 5. CPU-GPU 同步点设计

### Phase 1（BA 仍在 CPU Ceres）

```
pipeline 开始
  │
  ├─ [一次性] cpu → gpu: track_store + project data
  │
  ├─ initial pair 选择（CPU，读 ViewGraph）
  │   ↓ 选出 (i, j)
  ├─ 两视图 BA（CPU，使用 Ceres，small problem）
  │   ↓ 得到初始 poses[i], poses[j], track_xyz
  ├─ [sync A] cpu → gpu: poses[i,j] + track_xyz（仅两张图的数据，< 1KB）
  │
  ├─ [loop: resection → triangulation → outlier rejection → local BA]
  │   ↓ 全程 GPU，d_poses/d_track_xyz/d_obs_flags 仅在 GPU 修改
  │   ↓
  │   ├─ (每 global BA 触发时)
  │   │   ├─ [sync B1] gpu → cpu: d_poses[N×7] + d_track_xyz[3T] + d_obs_flags[M]
  │   │   │   ← 约 50MB，~4ms
  │   │   ├─ Ceres global BA（CPU，数秒）
  │   │   └─ [sync B2] cpu → gpu: 更新后的 poses + intrinsics + track_xyz
  │   │       ← 约 50MB，~4ms
  │   │
  │   └─ (local BA 不需要同步：仅涉及 poses 的子集，可 cpu local BA 后 partial sync B2)
  │
  └─ pipeline 结束
      ├─ [sync C] gpu → cpu: d_track_xyz + d_track_flags + d_obs_flags（序列化用）
      └─ 写 poses.json / bundle.out / colmap / tracks.isat_tracks（CPU）
```

**同步频率分析**（1000 张图，全局 BA 每 50 张触发一次）：
- Global BA 触发次数：~20 次
- 每次同步：~8ms（4ms × 2 方向）
- 同步总开销：~160ms

相比 BA 本身（每次数秒），同步开销可忽略不计。

### Phase 2（BA 也在 GPU）

Phase 2 完成后，**sync B1 和 B2 完全消除**。Pipeline 从头到尾只有：
- 开始时 1 次 CPU→GPU（~8ms）
- 结束时 1 次 GPU→CPU（~8ms）
- BA 内 1 次 8-byte cudaMemcpy（cost value，每 LM 迭代 1 次）

---

## 6. 设计约束与不变量

### 6.1 观测不可新增（Immutable Observations After Upload）

上传后，CSR 结构和所有 `d_obs_*` 索引数组（d_obs_track, d_obs_img, d_obs_feat）是只读的。外点剔除只修改 `d_obs_flags`，不修改结构。这是 GPU CSR 可以永久有效的根本前提。

如果将来需要在 SfM 过程中支持动态添加 observation（目前设计不需要），需要引入 CSR 重建 kernel。

### 6.2 Track 删除 = Flag 操作

`mark_track_deleted` 只清 `track_flags::kAlive`，不改变 CSR 结构，GPU 端：
```cuda
atomicAnd(&d_track_flags[tid], ~track_flags::kAlive);
```

### 6.3 CPU TrackStore 在 Phase 1 期间为"只读 shadow"

CPU 端 TrackStore 仅用于：
- sync B1 前的 CPU 访问（读取序列化格式）
- 调试 / 断言

GPU 是数据的权威来源。同步点 B1 完成后，CPU TrackStore 通过 `cudaMemcpy` 更新，两者再次同步。

### 6.4 BA subset flag (kSkipFromBA) 在 GPU 上维护

Grid-NMS 子集选择：
```cuda
__global__ void kernel_grid_nms_select(
    const float* d_obs_u, d_obs_v,
    const double* d_intrinsics,
    const uint32_t* d_img2cam,
    const bool* d_registered,
    const uint8_t* d_track_flags,
    const int* d_img_obs_ptr, d_img_obs_idx,
    const int* d_obs_track,
    int grid_size,
    uint8_t* d_track_flags_out  // kSkipFromBA bit
);
```
选完后 sync B1 把 `d_track_flags` 带到 CPU，Ceres 的 `SetParameterBlockConstant` 根据 kSkipFromBA 决定是否固定 3D 点。

---

## 7. 数据合法性维护

下表列出每个可变字段在 GPU 上的所有写路径，确保没有遗漏：

| 字段 | 写入 kernel | 触发条件 |
|------|------------|---------|
| `d_track_xyz` | `kernel_triangulate_batch` | resection 后 batch tri；`kernel_triangulate_full_scan` 定期全扫 |
| `d_track_flags[kHasXYZ]` | `kernel_triangulate_batch` | 三角化成功 |
| `d_track_flags[kNeedsRetri]` | `kernel_apply_outlier_flags` | 某 track 的 obs 被删到 < 2 |
| `d_track_flags[kAlive]` | `kernel_apply_outlier_flags` | track 观测全部被删（角度/深度滤波） |
| `d_track_flags[kSkipFromBA]` | `kernel_grid_nms_select` | 每次 global BA 前 |
| `d_obs_flags[kAlive]` | `kernel_apply_outlier_flags` | 重投影误差 > MAD 阈值 |
| `d_obs_flags[kRestorable]` | `kernel_apply_outlier_flags` | 被 MAD 删除（可恢复） |
| `d_obs_flags[kRestorable→kAlive]` | `kernel_restore_observations` | BA 内参显著变化后 |
| `d_poses` | `kernel_update_params`（Phase 2）或 sync B2（Phase 1）| BA 完成后 |
| `d_registered` | `kernel_resection_apply_result` | resection 成功后 |
| `d_obs_u_n / d_obs_v_n` | `kernel_undistort_obs` | 首次 / 内参变化后 |

---

## 8. VRAM 占用（典型规模）

| 规模 | tracks | obs | VRAM（GpuTrackStore）|
|------|--------|-----|----------------------|
| 小（200 张） | 80K | 600K | ~60 MB |
| 中（1000 张）| 500K | 4M | ~110 MB |
| 大（5000 张）| 3M | 25M | ~650 MB |
| 超大（20K 张）| 15M | 120M | ~3 GB |

GpuSfMState（poses/intrinsics/H 矩阵）另计（见 CUDA 架构文档）。RTX 3090 24GB VRAM 支持到约 40K 张图以上不 OOM。

---

## 9. 下一步实施

1. 在 `TrackStore` 中新增 `build_csr_for_gpu()` 方法（CPU 扫描一遍生成 CSR，不改变现有接口）
2. 新建 `src/algorithm/cuda/gpu_track_store.h/.cu`，实现 `gpu_track_store_upload()` 和 `gpu_track_store_free()`
3. 新建 `src/algorithm/cuda/cuda_undistort.cu`，实现 `kernel_undistort_obs`
4. 改写 `incremental_triangulation.cpp/h`：若 `GpuTrackStore != nullptr`，走 CUDA 路径；否则 fallback CPU
5. 改写 `outlier rejection`：同上，双路径
6. `resection_batch.cpp`：CUDA P3P RANSAC 实现

每个模块都有 CPU fallback，确保 `WITH_CUDA=OFF` 时完全等价于现有行为。
