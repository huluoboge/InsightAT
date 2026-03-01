# GPU RANSAC 几何求解工具设计文档

## 1. 概述

### 1.1 功能

本工具基于 **OpenGL 4.3 Compute Shader + EGL 无窗口环境**，实现 GPU 并行加速的 RANSAC 算法，支持：

| 模型 | 最小样本集 | 求解算法 | 误差度量 |
|------|-----------|---------|---------|
| 单应矩阵 H (3×3) | 4 点 | 8×9 DLT + 9×9 Jacobi 零空间 | 平方前向传递距离 |
| 基础矩阵 F (3×3, 秩2) | 8 点 | 8×9 DLT + 零空间 + 秩2强制 SVD | 平方 Sampson 距离 |
| 本质矩阵 E (3×3, σ₁=σ₂,σ₃=0) | 8 点 | 8×9 DLT + 零空间 + 本质约束 SVD | 平方 Sampson 距离 |

### 1.2 核心优势

- **无头运行**：EGL surfaceless/pbuffer，不依赖 X11 / Wayland / 显示器，适合服务器和 Docker
- **数值稳定**：Hartley 各向同性归一化在 CPU 端完成，GPU 只在条件良好的归一化坐标上运算
- **零外部依赖**：仅需 OpenGL 4.3 + GLEW + EGL，输出通用 `float[9]` 行主序矩阵
- **可配置**：通过 `GeoRansacConfig` 调整迭代次数和 workgroup 大小

---

## 2. 架构设计

### 2.1 整体流程

```
CPU 侧                                  GPU 侧（每 iter 一个线程）
──────────────────────────────          ─────────────────────────────────────
输入 n 对匹配点
  │
  ▼
Hartley 归一化 T1, T2
  │  (零均值 + RMS距离=√2)
  ▼
生成 N×K 随机索引 (heap)
  │
  ▼
Upload  MatchBuf (SSBO 0)
        IdxBuf   (SSBO 1)
        ResBuf   (SSBO 2, write-only)
  │
  ▼
glDispatchCompute(N/64, 1, 1)  ──────►  取 K 个点构建 8×9 DLT 矩阵 A
                                         计算 AᵀA (9×9 对称)
                                         Jacobi 特征分解 → 零空间向量
                                         F/E: 3×3 SVD 强制几何约束
                                         遍历全部 n 点计算误差、统计内点数
  │
  ▼
glMemoryBarrier
glMapBuffer → CPU 找最优 iter
  │
  ▼
反归一化
  H: T2⁻¹ · Hn · T1
  F: T2ᵀ  · Fn · T1
  E: T2ᵀ  · En · T1
  │
  ▼
返回最优矩阵 + 内点数
```

### 2.2 GPU 数学核心

#### 零空间求解（9×9 对称 Jacobi）

对 8×9 系数矩阵 A，令 B = AᵀA（9×9对称），对 B 做 Jacobi 特征分解（20躺×36对），
最小特征值对应的特征向量即为 DLT 解。相比完整 Full SVD，实现更稳定，
迭代次数固定（无自适应收敛判断），适合 GPU 无分支计算。

#### 3×3 SVD（用于秩约束和本质约束）

对 3×3 矩阵 M，构造 MᵀM，再做 3×3 Jacobi 特征分解（12躺）：

- **F 秩2约束**：令最小奇异值 σ₃ = 0 后重组 U·diag(σ₁,σ₂,0)·Vᵀ
- **E 本质约束**：令 σ₁ = σ₂ = (σ₁+σ₂)/2, σ₃ = 0 后重组

#### 误差度量

**H 平方前向传递距离**（像素²）：

  e_H = ‖ (H x₁)_inhom − x₂ ‖²

**F/E 平方 Sampson 距离**（像素² 或归一化²）：

  e_S = (x₂ᵀ F x₁)² / [(Fx₁)₁² + (Fx₁)₂² + (Fᵀx₂)₁² + (Fᵀx₂)₂²]

---

## 3. API 调用指南

### 3.1 生命周期

```cpp
#include "gpu_geo_ransac.h"

// 进程启动时调用一次
GeoRansacConfig cfg{};          // 全零 = 默认值
cfg.num_iterations = 3000;      // 可根据需要调整，见 §4.2
cfg.local_size_x   = 64;        // 通常与 GPU warp/wavefront 大小对齐
gpu_geo_init(&cfg);             // 或 gpu_geo_init(NULL) 使用默认值

// ... 反复调用求解器（无额外初始化开销）...

// 进程退出前调用一次
gpu_geo_shutdown();
```

### 3.2 坐标约定

| 求解器 | 输入坐标 | 说明 |
|--------|---------|------|
| `gpu_ransac_H` | 像素坐标 | 直接传入 SiftGPU 输出的像素 (x, y) |
| `gpu_ransac_F` | 像素坐标 | 内部自动做 Hartley 归一化 |
| `gpu_ransac_E` | **归一化相机坐标** | 调用前必须乘以 K⁻¹：`x_n = (x_px − cx)/fx` |

### 3.3 阈值设置

> **重要**：所有阈值均为**平方**距离单位，与误差度量保持一致。

#### 单应矩阵 H — 平方前向传递误差（像素²）

```
推荐范围：1.0 ~ 4.0 px²（对应线性 1.0 ~ 2.0 px）

thresh = 1.0f   →  1.0 px  极高精度匹配
thresh = 2.25f  →  1.5 px  标准 SIFT 匹配（推荐默认）
thresh = 4.0f   →  2.0 px  噪声较大或低质量图像
```

#### 基础矩阵 F — 平方 Sampson 距离（像素²）

```
推荐范围：0.5 ~ 4.0 px²

thresh = 0.5f   高质量匹配 + 精确内参
thresh = 1.0f   标准场景（推荐默认）
thresh = 2.0f   噪声较大 / 宽基线
thresh = 4.0f   粗略估计 / 大噪声
```

#### 本质矩阵 E — 平方 Sampson 距离（归一化坐标²）

从像素阈值换算公式：`thresh_E ≈ (thresh_px / focal_length)²`

```
f ≈ 800 px,  1.5 px  →  thresh_E ≈ 3.5e-6f
f ≈ 800 px,  3.0 px  →  thresh_E ≈ 1.4e-5f
f ≈ 1200 px, 1.5 px  →  thresh_E ≈ 1.6e-6f
```

### 3.4 典型调用示例

```cpp
// ── H：平面场景 / 纯旋转 ─────────────────────────────────────────────────
float H[9];
int inliers = gpu_ransac_H(matches, n, H, 2.25f);
// H 为行主序 3×3，满足 x2_h ≅ H · x1_h（齐次坐标）

// ── F：一般场景（无标定）────────────────────────────────────────────────
float F[9];
int inliers = gpu_ransac_F(matches, n, F, 1.0f);
// 满足 x2_h^T · F · x1_h = 0

// ── E：标定相机 ──────────────────────────────────────────────────────────
// 先将像素坐标归一化
std::vector<Match2D> norm(n);
for (int i = 0; i < n; i++) {
    norm[i].x1 = (matches[i].x1 - cx) / fx;
    norm[i].y1 = (matches[i].y1 - cy) / fy;
    norm[i].x2 = (matches[i].x2 - cx) / fx;
    norm[i].y2 = (matches[i].y2 - cy) / fy;
}
float thresh_E = (1.5f / fx) * (1.5f / fx);
float E[9];
int inliers = gpu_ransac_E(norm.data(), n, E, thresh_E);
```

### 3.5 内点数判断建议

| 内点率 / 内点绝对数 | 建议 |
|-------------------|------|
| > 50% 且 > 20 对 | 结果可信，可直接使用 |
| 20~50% 且 > 10 对 | 可用，建议用内点集做一次精化（DLT/LM） |
| < 20% 或 < min_K | 匹配质量差；考虑放宽阈值或重新匹配 |
| 内点数 < 最小样本集 | 求解失败（返回 -1） |

---

## 4. 效率分析

### 4.1 理论复杂度

| 阶段 | 复杂度 | 备注 |
|------|--------|------|
| Hartley 归一化 | O(n) CPU | 两次遍历，< 0.01 ms |
| 随机索引生成 | O(N·K) CPU | N=3000, K=4/8 |
| SSBO 上传 | O(n + N·K) | n=1000 时 < 0.1 ms |
| GPU 并行求解 | O(K²·S / N_threads) | Jacobi 迭代 S=20 |
| **GPU 内点统计** | **O(n) per thread** | n 较大时为主要瓶颈 |
| 结果读回 | O(N×10 floats) ≈ 120 KB | |
| CPU 找最优 | O(N) | 可忽略 |

内点统计是瓶颈：每个线程独立遍历全部 n 个点，n=5000 时约需 10~20 ms。

### 4.2 调优建议

```cpp
// 1. 根据外点率估算理论迭代次数（RANSAC公式）
//    N = log(1 - confidence) / log(1 - (1 - outlier_rate)^K)
//
//    confidence=0.999, outlier_rate, K → N
//    e=30%, K=4(H) →   35 次
//    e=50%, K=4(H) →   72 次
//    e=30%, K=8(F) →  272 次
//    e=50%, K=8(F) → 1177 次
//    e=60%, K=8(F) → 4050 次  ← 需要 num_iterations=5000

// 2. 高质量 SIFT 匹配（外点率通常 < 50%）可降低迭代次数
cfg.num_iterations = 1000;   // 约为默认 3000 的 1/3 时间

// 3. 充分利用 GPU SM —— local_size_x 与 warp 大小对齐
cfg.local_size_x = 64;       // NVIDIA / Intel Xe 推荐
cfg.local_size_x = 32;       // 部分 AMD GCN 旧架构

// 4. init/shutdown 只做一次，反复调用 gpu_ransac_* 无额外开销
```

### 4.3 实测基准

运行 `./test_geo_ransac` 会自动打印热身耗时和 50 次统计（min/avg/max）。

以下为在 **NVIDIA GTX 1060 6GB**（N=2048 次迭代，workgroup=32）上实测结果：

| n（匹配对数） | 模型 | warm (ms) | min (ms) | avg (ms) | max (ms) |
|:---:|:---:|---:|---:|---:|---:|
| 100 | H | 49.74 | 43.77 | 44.18 | 48.64 |
| 100 | F | 44.30 | 43.70 | 43.97 | 46.22 |
| 100 | E | 43.88 | 43.23 | 43.84 | 45.23 |
| 300 | H | 43.90 | 43.82 | 43.92 | 44.25 |
| 300 | F | 43.95 | 43.72 | 43.92 | 44.33 |
| 300 | E | 43.81 | 43.71 | 43.94 | 45.08 |
| 500 | H | 43.97 | 43.86 | 44.19 | 44.93 |
| 500 | F | 44.34 | 44.10 | 44.30 | 44.91 |
| 500 | E | 44.20 | 43.97 | 44.46 | 45.46 |
| 1000 | H | 44.39 | 44.24 | 44.72 | 48.56 |
| 1000 | F | 44.51 | 44.39 | 44.58 | 44.98 |
| 1000 | E | 44.50 | 44.08 | 44.51 | 44.94 |

> **注意（双显卡系统）**：Linux 系统同时有 Intel 集成显卡 + NVIDIA 独显时，
> `eglGetDisplay(EGL_DEFAULT_DISPLAY)` 默认选 Intel，导致耗时约 **72~145 ms**（集成显卡）。
> 本库使用 `EGL_EXT_device_enumeration` + `EGL_EXT_platform_device`
> 自动枚举并优先选择 NVIDIA 设备（打印 `[gpu_geo] EGL device N vendor: NVIDIA`），
> 无需手动设置 `__NV_PRIME_RENDER_OFFLOAD` 等环境变量。

**关键观察（Jacobi 求解器）：**

1. **n=100~1000 耗时几乎相同（≈46 ms）**：真正瓶颈是 **GPU 寄存器溢出（Register Spilling）**。
   `null_vector` 内部使用 B[81]+V[81]+A[72]≈234 个动态索引 `float` 数组，GLSL 编译器无法
   将其全部映射到寄存器，强制溢出到 **Local Memory**（显存；≈100 周期延迟 vs 寄存器 1 周期）。
   已通过 `GL_TIME_ELAPSED` Timer Query 确认：GPU dispatch 本身耗时 44 ms，
   `glMemoryBarrier` 返回仅需 0.013 ms——因此**不是**同步开销。

2. **warm 首次调用稍慢**：GLSL shader JIT 编译 + SSBO 首次内存分配。

3. **IPI 求解器（deflation + 幂迭代）**：比 Jacobi 快 **37~75×**，且结果正确，
   见下方对比表。

### 4.4 Jacobi vs IPI 求解器对比

两种 null_vector 求解器（`gpu_geo_set_solver(0/1)`）的 50 次统计对比，
同一数据集、同一 RANSAC 框架，仅 null_vector 算法不同。

**GTX 1060 6GB，N=2048，workgroup=32，50 次均值：**

| n | 模型 | Jacobi avg (ms) | IPI avg (ms) | 加速比 |
|:---:|:---:|---:|---:|---:|
| 100 | H | 46.2 | 0.61 | **75.7×** |
| 100 | F | 46.2 | 0.73 | **63.3×** |
| 100 | E | 46.3 | 0.75 | **61.7×** |
| 300 | H | 46.4 | 0.74 | **62.7×** |
| 300 | F | 46.3 | 0.84 | **55.1×** |
| 300 | E | 46.4 | 0.86 | **53.9×** |
| 500 | H | 46.6 | 0.92 | **50.7×** |
| 500 | F | 46.6 | 1.01 | **46.1×** |
| 500 | E | 46.5 | 0.99 | **47.0×** |
| 1000 | H | 46.6 | 1.26 | **37.0×** |
| 1000 | F | 46.7 | 1.37 | **34.1×** |
| 1000 | E | 46.9 | 1.36 | **34.5×** |

**IPI 算法关键参数**（`null_vector_ipi` — Inverse Power Iteration (via Cholesky factorisation)Cholesky 逆迭代）：
- **Step 1**：计算 B = AᵀA；加微小正则化 μ = trace(B)/1000，使 B_μ 严格正定
- **Step 2**：原地 Cholesky 分解 B_μ = L·Lᵀ（下三角，覆写 B[81]），O(9³/6) ≈ 135 次操作
- **Step 3**：6 轮逆迭代（每轮前向代换 + 后向代换 + 规范化），**三次收敛**，适用于任何非退化 DLT 配置

**为何选择 Cholesky 逆迭代而非幂迭代**：  
幂迭代（偏转法）的收敛率为 `1 − λ₂/λmax`，其中 λ₂ 是 B 的第二小特征值。  
当 RANSAC 随机子样本产生近退化配置（λ₂ ≪ λmax）时，40 步幂迭代不足以收敛，  
导致零内点失效。Cholesky 逆迭代的收敛率为 `(μ/λ₂)^k`（k=迭代轮数），  
对任何特征值间隔均适用，6 轮即可精确求解。

---

## 5. 已知限制

| 限制 | 说明 |
|------|------|
| 线程安全 | 全局 EGL context / SSBO，**不支持多线程并发调用** |
| 最小点数 | H ≥ 4 对，F/E ≥ 8 对，否则返回 -1 |
| E 坐标 | 调用者负责 K⁻¹ 归一化，库内部仅做 Hartley 二次归一化 |
| 8点法限制 | E 使用 8点法而非 5点法；对纯平移场景无退化，但精度略低于 5点法 |
| 无精化步骤 | 输出 RANSAC 最优模型；工业使用建议用内点集再做一次全点 DLT/LM 精化 |
