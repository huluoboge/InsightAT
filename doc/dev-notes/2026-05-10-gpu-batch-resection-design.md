# 2026-05-10 GPU 批量 Resection 设计记录

**背景**：当前增量 SfM 的整体方向已经明确为“除 Ceres BA 外，其他阶段尽量全部留在 GPU 内完成”。围绕 resection，我们进一步收敛出一套新的设计原则和实现方案，先在此记录，作为后续重构与实现的基线。

---

## 1. 设计目标

本次 resection 重构的目标不是只替换一个求解器，而是统一到一套新的执行原则：

1. **GPU 到 GPU 的数据流通**
   只要数据已经在 GPU 显存中，后续算子之间就不应再绕回 CPU。`TrackStore`、观测、track、姿态中间状态，应尽量以 GPU 常驻状态为主。

2. **批量计算优先**
   Resection 不应按“每次 1 张图像”串行执行。应支持“每次一批候选图像”同时求解，这样更符合 GPU 的并行特性，也更符合航测 SfM 的吞吐需求。

3. **使用去畸变后的归一化坐标**
   Resection 阶段不再承担畸变建模。输入给 PnP / 投影误差核函数的应是 **在当前内参 K 下** 已在 GPU 上更新过的去畸变归一化坐标 `xn, yn`，即用 pinhole 在归一化平面上做误差与优化。BA 会改变 K，因此该坐标是**按需重算**的派生量，而非「Track 上传时算一次即可」。

4. **核函数与策略解耦**
   Kernel 本身只负责高效执行，不绑定具体策略。像迭代次数、阈值、批大小、最小内点数等，都通过参数控制。

5. **先测试，再接管主流程**
   这是一次架构级重构，不是局部替换。任何新内核都必须配套单元测试，并使用仿真数据验证正确性、稳定性和退化场景表现。

---

## 2. 总体原则

### 2.1 GPU 常驻状态是主状态

增量 SfM 的中间状态应以 GPU 内存为主，而不是 CPU 数据结构为主。对 resection 而言，理想状态是：

- `track xyz` 常驻 GPU
- `obs -> track`、`obs -> image`、`obs valid` 常驻 GPU
- 去畸变后的 `obs_xn / obs_yn` 常驻 GPU
- 当前已注册影像、相机位姿、track 有效性等状态常驻 GPU

这样 resection、triangulation、outlier reject 之间就可以直接共享同一份 GPU 状态，而不是每个模块都重复做 CPU 打包、上传、读回。

### 2.2 CPU 只保留必要职责

在这一轮设计中，CPU 仍保留以下职责：

- 启动 pipeline 与调度高层流程
- Ceres BA（当前仍是 CPU）
- 极少量标量级控制逻辑
- 单元测试框架与结果校验

但 CPU 不应继续承担以下工作：

- 为每个 resection image 打包 3D-2D 对应关系
- 为每个假设生成大规模中间数组
- 在每张图像之间串行做 hypothesis 搜索
- 在内核之间搬运本来已经在 GPU 上的数据

---

## 3. Resection 的目标形态

### 3.1 从“单图像 resection”改为“批量 resection”

当前 `run_batch_resection()` 的接口是 batch，但实现本质上还是：

```text
for image in candidates:
    resection_single_image(image)
```

这不符合 GPU 的最佳使用方式。新的目标形态是：

```text
一次输入 K 张候选图像
→ GPU 为每张图像并行运行固定次数的 PnP RANSAC
→ 每张图像各自选出最优 pose hypothesis
→ 对通过阈值的图像并行做 pose refine
→ 输出一批成功注册的图像
```

也就是说，真正的并行维度有两层：

1. **图像维度并行**：同一批候选图像同时求解
2. **hypothesis 维度并行**：每张图像内部并行跑固定次数的 RANSAC

这比“串行多次单图像 resection”更符合 GPU 的吞吐模型。

### 3.2 使用去畸变后的 pinhole 模型

本次设计明确采用：

- 在读取 `d_obs_xn` / `d_obs_yn` 参与 resection / refine **之前**，按当前 `d_intrinsics` 在 GPU 上完成去畸变（例如在每次 BA 写回内参之后整表重算）
- resection 内核只读已与 K 同步的归一化坐标
- 投影误差、内点判断、pose refine 均使用 pinhole 模型

这样做的原因：

1. 计算链更短，Jacobian 更简单
2. 数值行为更清晰，易于调试和单测
3. 更容易在批量 kernel 中保持高吞吐
4. 符合“畸变处理前移”的总体架构

因此，resection 阶段不再把 Brown-Conrady 畸变作为核心的一部分，而是要求输入已经被 GPU 去畸变。

---

## 4. 求解器方案

### 4.1 采用 P3P RANSAC，而不是现有 GLSL DLT PnP

当前已有的 `gpu_geo_ransac.cpp` 中 `gpu_ransac_pnp()` 基于 OpenGL Compute Shader，核心是 DLT 风格的 PnP RANSAC。它有几个问题：

1. 依赖 OpenGL / EGL 数据路径，和 CUDA 持久状态割裂
2. 输入输出是上传-读回模式，不适合 GPU 常驻架构
3. 对航测常见的近平面 / nadir 场景，DLT 数值稳定性不如 P3P 最小解路径

因此，本轮设计目标不是继续增强 `gpu_geo_ransac.cpp`，而是在 `sfm_cuda_kernels` 中新增 **纯 CUDA 的 batch P3P RANSAC**。

### 4.2 P3P 的理由

P3P 是更适合当前场景的最小求解器：

- 最小样本数是 3，对固定 2000 次迭代更友好
- 更适合 GPU 上大规模并行 hypothesis 搜索
- 对航测场景比基于 DLT 的 6 点估计更合理

当前倾向使用 **Lambda Twist** 一类的稳定 P3P 实现，作为 device solver。

---

## 5. 采样策略

### 5.1 固定 2000 次并行迭代

本次讨论明确：**每张图像固定并行跑 2000 次 hypothesis**。理由如下：

1. 从经验上看，CPU 版本通常用不到 2000 次
2. GPU 更适合固定次数、少分支的并行执行
3. 固定 2000 次比“自适应停止”更简单，控制逻辑更少
4. 直接算满并取最优 hypothesis，更符合 GPU 风格

因此，内核层面默认支持固定 2000 次，但迭代次数仍应参数化，例如：

- `num_iterations`
- `min_inliers`
- `reproj_threshold`

### 5.2 采样是否由 CPU 预生成

本次讨论的判断如下：

#### 方案 A：CPU 预生成采样序列，再上传 GPU

优点：

- 实现简单
- 可复现性强
- 不依赖 cuRAND 或设备端随机状态

缺点：

- 每次 batch resection 都要重新组织并上传索引
- 随着“多图像 × 多迭代”扩大，虽然传输量不大，但会把一次本可纯 GPU 的流程又切回 CPU 参与
- 不符合长期“GPU 到 GPU 数据流”的总体原则

#### 方案 B：GPU 内部用无状态伪随机哈希生成采样

优点：

- 不需要 cuRAND 状态数组
- 不需要 CPU 预生成大块 sample index
- 只需传一个 `seed` 或 `frame_seed`
- 更符合零拷贝的方向

缺点：

- 需要设计简单可靠的 hash / pseudo-random 采样逻辑
- 需要注意同一 hypothesis 内 3 个点不能重复

### 5.3 当前建议

当前设计建议采用：

**GPU 内部基于整数哈希的无状态伪随机采样**。

原因：

1. 速度上几乎没有负担
2. 避免 CPU 参与 sample generation
3. 不需要维护设备端 RNG state
4. 更符合长期 GPU 常驻数据流目标

换句话说：

- 不推荐引入复杂的 cuRAND 状态
- 也不推荐长期依赖 CPU 预生成 sample 序列
- 更适合在 kernel 内根据 `(image_id, iter_id, seed)` 直接生成 3 个采样索引

---

## 6. 内核接口设计原则

新内核的设计原则是“面向数据，不面向流程函数”。也就是说，新模块不应依赖 CPU 版 `TrackStore` 的访问模式，而应直接操作 GPU 上的扁平数组和 CSR 索引。

建议新增模块：

- `src/algorithm/modules/sfm/cuda/cuda_pnp_ransac.cuh`
- `src/algorithm/modules/sfm/cuda/cuda_pnp_ransac.cu`

并加入现有 `sfm_cuda_kernels` target。

### 6.1 推荐输入

建议 kernel 直接读取以下 GPU 数据：

- 去畸变后的 `obs_xn`, `obs_yn`
- `obs_track_idx`
- `obs_image_idx`
- `track_xyz`
- `track_valid`
- `registered`
- 每个 image 的观测 CSR（或 image→obs 索引）

### 6.2 推荐输出

按 batch image 输出：

- `best_R[batch_size][9]`
- `best_t[batch_size][3]`
- `best_inlier_count[batch_size]`
- `success_flag[batch_size]`
- 可选：`inlier_mask` 或 `best_hypothesis_id`

### 6.3 参数化控制

内核层应支持以下可调参数：

- `num_iterations`
- `min_inliers`
- `reproj_threshold`
- `max_candidates_per_image`
- `seed`

这样就能把“策略”放在上层，把“执行”留在 kernel 内部。

---

## 7. 精化（Refine）方案

本次讨论的结论是：**精化阶段也应顺应去畸变后的 pinhole 设计**。

也就是说，新的 refine path 不应再以“带畸变像素坐标”为主要输入，而应提供一个更简单、更纯净的 pinhole refine 版本：

- 输入：去畸变后的归一化坐标
- 模型：`Xc = R * X + t`
- 残差：归一化平面或 pinhole 像素平面上的 reprojection error
- 求解：小规模 LM / GN，在 GPU 上做并行累计和小矩阵求解

后续可以复用 `cuda_resection.cu` 的设计思路，但需要新增一个更单纯的 undistorted / pinhole refine 通道。

---

## 8. 单元测试要求

这是本次讨论中明确强调的要求：**新内核必须先做单元测试**。

建议至少覆盖以下测试：

1. **基础正确性测试**
   随机生成相机姿态和 3D 点，投影得到无噪声 2D 点，验证 batch P3P 是否能恢复 pose。

2. **噪声鲁棒性测试**
   在 2D 点上加入小噪声，验证 fixed-iteration RANSAC + refine 的稳定性。

3. **外点测试**
   注入一定比例错误匹配，验证 2000 次并行 hypothesis 是否能稳定选出正确模型。

4. **批量测试**
   一次输入多张图像，验证不同 image 的结果互不干扰。

5. **航测 / nadir 退化场景测试**
   用近似共面的 3D 点和小基线构造数据，检查 P3P 路径是否仍然可工作，并分析失败条件。

6. **参数化测试**
   不同 `num_iterations`、`threshold`、`min_inliers` 下结果是否符合预期。

测试目标不是只验证“能跑通”，而是确认：

- 数值正确
- 批量执行正确
- 退化场景行为可解释
- 参数变化行为稳定

---

## 9. 与现有架构的关系

### 9.1 与 `gpu_geo_ransac.cpp` 的关系

`gpu_geo_ransac.cpp` 当前是 OpenGL Compute Shader 架构，适合工具链式“上传-求解-读回”的独立任务，例如：

- `isat_geo`
- 早期单次 PnP / E / F / H 求解

但它不是本轮 batch resection 的长期承载位置。新的 batch resection 更适合放在 `sfm_cuda_kernels` 内，与 `CudaSfMState` 共存。

### 9.2 与 `run_batch_resection()` 的关系

`run_batch_resection()` 的现有接口可以保留，但实现应逐步从：

```text
batch API + 单图串行实现
```

演进为：

```text
batch API + GPU 批量并行实现
```

也就是说，函数名可以继续用，但内部语义要真正变成 batch。

---

## 10. 当前方案结论

本次讨论形成的结论如下：

1. **总体原则成立**：不同函数之间应尽量 GPU 到 GPU 数据流通，track 等状态以 GPU 常驻为主。
2. **Resection 必须支持 batch**：不能继续按单图像串行调用，必须一次处理多张图像。
3. **Resection 应基于与当前 K 一致的去畸变坐标**：在 GPU 上按内参版本按需重算 `xn, yn`，resection 内核只处理 pinhole 模型。
4. **固定 2000 次并行迭代是合理的**：符合 GPU 计算习惯，也足够工程实用。
5. **内核应参数化而非绑死策略**：迭代次数等参数由上层传入。
6. **内核必须先有单元测试**：仿真数据验证正确性与稳定性，是本次重构的硬要求。
7. **新实现位置明确**：放在 `sfm_cuda_kernels`，而不是继续堆在 OpenGL PnP 路径上。

---

## 11. 下一步实现建议

建议按以下顺序推进：

1. 新增 `cuda_pnp_ransac.cu/.cuh`，先实现单图像 batch-hypothesis 版本
2. 做独立单元测试，确认 P3P + fixed 2000 iteration 正确
3. 扩展到多图像 batch resection
4. 新增 undistorted / pinhole refine kernel
5. 接入 `run_batch_resection()`
6. 最后再清理旧的 OpenGL PnP 路径在增量 SfM 中的角色

这个顺序的好处是：先把最核心、最容易验证的 CUDA PnP RANSAC 单元单独做对，再逐步接入完整 pipeline，降低重构风险。