# 2026-05-17 多策略 Seed 评估 CLI 设计草案

本文档定义一个新的 CLI 级别“自系统”来评估增量 SfM 的初始图像对与早期增长稳定性。目标不是继续调单一初始对分数，而是将“初始 seed 选择”升级为一个多策略、短程试跑、几何健康度驱动的独立过程。

---

## 1. 问题定义

当前增量 SfM 的初始对选择存在以下结构性问题：

1. 当前流程本质上是“候选排序后，遇到第一个通过门限的 pair 就接受”。
2. 单一静态分数无法同时覆盖航片、ETH3D、COLMAP 小场景等不同分布。
3. `resection -> triangulation -> re-triangulation` 具备一定自恢复能力，因此坏 seed 在前几张图像上可能仍然“看起来成功”。
4. 只看前几张图像是否成功注册，无法识别“局部结构已经进入错误 basin，但系统尚未死亡”的情况。
5. 当前 early BA + 固定位姿点优化 + 外点剔除，可能把两视图初始点拟合得很好，但这些 3D 点不一定代表真实几何。

已观测到的典型坏例子：

1. 初始对三角化角满足门限，但后续注册图像少。
2. 初始阶段前 3 张到 10 张图像都能成功 resection，但第 4 张附近点云整体交叉、深度关系错误。
3. 固定权重排序在一种数据上有效，换一批 COLMAP 数据或 ETH 数据后失效。

因此，问题不应再被建模为“找一个最好的静态分数”，而应建模为“从多个候选 seed / 策略中，找出最能进入正确几何 basin 的 early-stage 增长路径”。

---

## 2. 设计目标

本系统目标如下：

1. 将初始对选择从单次启发式，升级为多策略并行试跑后的择优过程。
2. 每个策略在独立 CLI 进程中运行，避免在单个进程内复制和回滚复杂可变状态。
3. 评估重点从“注册成功率”转移到“几何健康度 + 状态稳定性 + 增长质量”。
4. 第一版尽可能复用现有 `incremental_sfm_pipeline.cpp` 的能力，不重写完整 pipeline。
5. 输出机器可读 JSON 结果，便于将来接入 benchmark、批量实验和自动选优。

非目标：

1. 第一版不追求自动预测全局最佳参数。
2. 第一版不追求跨策略共享内存态或线程内并行复用 TrackStore。
3. 第一版不追求直接把最佳试跑状态无缝续跑为正式全量 SfM。

---

## 3. 总体思路

总体采用“两层决策”结构：

### 3.1 第一层：多策略 seed shortlist

为同一数据构造多组有明确偏好的初始化策略。每组策略在“初始对选择”上有不同的偏置，例如：

1. `wide_baseline`：偏重大角度、强基线。
2. `support_first`：偏重 F/E 内点数、有效三角化点数、图连通度。
3. `balanced`：平衡型策略，作为通用基线。
4. `conservative`：更保守，更强调前向运动限制、几何刚性与 early stability。

每组策略只负责提出自己的候选初始对与 early-stage 调度逻辑，不直接宣称自己是最终最优。

### 3.2 第二层：短程局部 SfM 试跑评估

对每组策略，独立运行一个短程 local SfM 试验：

1. 选择自己的初始对。
2. 最多注册 `N_eval_images` 张图像，建议第一版默认 6 张。
3. 记录 early-stage 每轮的增长统计、几何刚性统计、churn 统计和优化稳定性统计。
4. 输出试验结果 JSON。

最终不是简单选择“注册图像最多”的策略，而是选择“增长足够、几何健康、状态不振荡”的策略。

---

## 4. 为什么采用独立 CLI

采用独立 CLI，而不是在现有 pipeline 内部用线程/分支复制状态，原因如下：

1. `TrackStore`、poses、registered flags、triangulation / retriangulation 状态均可变，复制和回滚复杂且易错。
2. 在一个线程内为多个 seed 维护多套状态会显著增加实现复杂度与调试成本。
3. 独立进程天然隔离，失败试验可直接丢弃目录，不污染主流程。
4. CLI 结果易落盘、易 benchmark、易复现实验，符合项目 CLI-first 架构。
5. 将来如果需要并行跑多个策略，可以直接由外层调度多个进程，而不是在算法层引入线程安全负担。

这也是本设计明确选择“自系统 / 过程”的原因。

---

## 5. 并发调用架构（isat_sfm 协调层）

虽然 `isat_seed_eval` 是独立 CLI，但它在实际使用中通常由更高层级的编排器（外层调度）驱动。本章描述 `isat_sfm` 如何协调调用多个 `isat_seed_eval` 进程。

### 5.1 整体流程

```
isat_sfm.cpp (orchestrator)
  │
  ├─ [Seed Eval Step] (新增可选步骤)
  │   │
  │   ├─ Fork Process 1: isat_seed_eval --strategy balanced -o <work>/eval_balanced/
  │   ├─ Fork Process 2: isat_seed_eval --strategy wide_baseline -o <work>/eval_wide_baseline/
  │   ├─ Fork Process 3: isat_seed_eval --strategy support_first -o <work>/eval_support_first/
  │   └─ Fork Process 4: isat_seed_eval --strategy conservative -o <work>/eval_conservative/
  │   
  │   └─ Wait for all 4 processes (可选并发 / 串行)
  │       └─ Aggregate results from 4 × report.json
  │           └─ Write <work>/seed_eval_summary.json
  │
  └─ [Incremental SfM Step] (跳过或用最佳seed初始化)
      └─ isat_incremental_sfm ... [--seed best_seed.json]  (Phase C功能)
```

### 5.2 参数设计：`--strategy` 选项

`isat_seed_eval` 的命令行参数设计：

```bash
isat_seed_eval \
  -t traces.isat_tracks \
  -p project.iat \
  -m pairs.json \
  -g geo_dir/ \
  -o output_dir/ \
  --max-eval-images 6 \
  [--strategy STRATEGY]  # 新增：单个策略模式
  [-j|--threads N] \
  [-v|--verbose]
```

其中：

1. **不指定 `--strategy`**：运行全部 4 策略（default mode），适合完整评估
2. **指定 `--strategy balanced`**：仅运行 balanced，快速测试模式
3. **指定 `--strategy wide_baseline|support_first|conservative`**：相应单策略模式

输出结构：

- 如果 `--strategy` 指定单策略，输出目录中仅有该策略的目录结构
- 如果不指定，输出所有 4 个策略的结果 + 聚合 `report.json`

### 5.3 并发与隔离

#### 5.3.1 进程级隔离

每个 `isat_seed_eval` 进程：

1. 有独立的工作目录 `<work>/eval_${strategy}/`
2. 读同一份 tracks、geo、pairs，但在内存中独立加载
3. 各自执行短程试跑，互不干扰
4. 各自写入 `summary.json`

#### 5.3.2 并发启动方式

`isat_sfm` 中建议用以下方式：

```cpp
// 伪代码
std::vector<std::thread> workers;
std::vector<int> exit_codes(4, -1);
std::vector<std::string> strategies = {"balanced", "wide_baseline", "support_first", "conservative"};

for (size_t i = 0; i < strategies.size(); ++i) {
  workers.emplace_back([i, &strategies, &exit_codes, ...] {
    fs::path strategy_out = work_path / ("eval_" + strategies[i]);
    fs::create_directories(strategy_out);
    
    std::vector<std::string> cmd = {
      tool_path("isat_seed_eval"),
      "-t", tracks_path.string(),
      "-p", project_path.string(),
      "-m", pairs_json.string(),
      "-g", geo_dir.string(),
      "-o", strategy_out.string(),
      "--max-eval-images", "6",
      "--strategy", strategies[i]
    };
    // 添加verbosity
    for (const auto& va : g_verbosity_args)
      cmd.push_back(va);
    
    exit_codes[i] = run(cmd);  // 或 run_capture 以收集 ISAT_EVENT
  });
}

// Wait for all
for (auto& t : workers)
  if (t.joinable())
    t.join();

// Check all succeeded
for (int rc : exit_codes) {
  if (rc != 0) {
    LOG(ERROR) << "seed_eval strategy failed";
    return 1;
  }
}

// Aggregate
std::string agg_err;
if (!aggregate_seed_eval_results(work_path, &agg_err)) {
  LOG(ERROR) << agg_err;
  return 1;
}
```

### 5.4 结果汇总

`aggregate_seed_eval_results()` 负责：

1. 读取 4 个 `eval_${strategy}/summary.json`
2. 提取各策略的关键指标
3. 排名，输出 `seed_eval_summary.json`

`seed_eval_summary.json` 格式示例：

```json
{
  "total_strategies": 4,
  "evaluation_window": 6,
  "results": [
    {
      "rank": 1,
      "strategy": "balanced",
      "score": 85.3,
      "metrics": {
        "registered_images": 6,
        "positive_depth_ratio": 0.98,
        "point_deletion_rate": 0.03,
        "point_recovery_rate": 0.5,
        "churn_rate": 0.02,
        "ba_point_shift_p90": 0.45
      },
      "best_seed_info": {
        "image1_id": 0,
        "image2_id": 1,
        "num_inliers": 350,
        "median_angle_deg": 18.5
      }
    },
    { "rank": 2, "strategy": "support_first", ... },
    { "rank": 3, "strategy": "conservative", ... },
    { "rank": 4, "strategy": "wide_baseline", ... }
  ],
  "recommended_for_full_sfm": {
    "strategy": "balanced",
    "reason": "稳定几何 + 足够增长 + 低churn"
  }
}
```

### 5.5 与现有 isat_sfm steps 的集成

推荐在 `isat_sfm` 的 step 列表中新增：

```
current steps: create, extract, match, tracks, incremental_sfm
new steps:     create, extract, match, tracks, [seed_eval], incremental_sfm
```

新增 CLI 参数：

```bash
--steps create,extract,match,tracks,seed_eval,incremental_sfm  # 完整带 seed_eval
--steps create,extract,match,tracks,incremental_sfm             # 跳过 seed_eval (default)
--seed-eval-enable                                              # 简化开关
--seed-eval-max-images 6                                        # seed_eval 评估窗口
```

---

## 6. 拟新增 CLI

建议新增一个工具：

`isat_seed_eval`

其职责是：

1. 读取 tracks / project / pairs / geo。
2. 根据配置生成多个策略实例。
3. 对每个策略独立执行短程 seed 试跑。
4. 收集统计并输出策略排名结果。

建议命令形式（支持多策略或单策略模式）：

```bash
# 模式A：全策略评估
isat_seed_eval \
  -t tracks.isat_tracks \
  -p project.iat \
  -m pairs.json \
  -g geo_dir/ \
  -o seed_eval_out/ \
  --max-eval-images 6 \
  -v

# 模式B：单策略快速测试
isat_seed_eval \
  -t traces.isat_tracks \
  -p project.iat \
  -m pairs.json \
  -g geo_dir/ \
  -o seed_eval_out/ \
  --max-eval-images 6 \
  --strategy balanced \
  -v

# 完整参数示例（保留兼容性）
isat_seed_eval \
  -t tracks.isat_tracks \
  -p project.iat \
  -m pairs.json \
  -g geo_dir/ \
  -o seed_eval_out/ \
  --max-eval-images 6 \
  --strategies balanced,wide_baseline,support_first,conservative \
  -v
```

建议输出：

1. `seed_eval_out/report.json`：所有策略的总报告。
2. `seed_eval_out/best_seed.json`：最佳策略摘要。
3. `seed_eval_out/strategy_xx_*/`：每个策略的独立工作目录和明细 JSON。

stdout / stderr 约束：

1. stdout 只输出 `ISAT_EVENT <json>`。
2. stderr 使用 glog 日志。

---

## 7. 与现有 `isat_incremental_sfm` 的关系

### 6.1 第一版边界

第一版不要求 `isat_seed_eval` 直接把最佳试跑态无缝交给 `isat_incremental_sfm`。第一版仅输出：

1. 最佳策略 ID
2. 最佳初始对
3. early-stage 统计结果
4. 建议配置

这样工程边界清晰，先验证判别力，再决定是否做状态续跑。

### 6.2 第二版可选方向

若第一版判别力良好，可以继续扩展：

1. 让 `isat_seed_eval` 输出最佳初始 pair 与策略 profile。
2. 让 `isat_incremental_sfm` 支持读取 `best_seed.json` 作为初始化覆盖。
3. 再往后，才考虑输出可恢复的中间状态。

---

## 8. 策略空间设计

第一版不做大规模自动参数搜索，而是做少量、可解释的策略组。

建议第一版四组：

### 7.1 `balanced`

目标：作为通用基线。

特点：

1. 第二图候选排序使用平衡权重。
2. 初始 pair 角度、内点、点数、连通度均参与。
3. 保持现有 gate，但不过度偏向单一信号。

### 7.2 `wide_baseline`

目标：偏向航片或规则覆盖场景中的高基线 seed。

特点：

1. 强化 triangulation angle / median angle。
2. 更强调前向运动限制。
3. 可接受稍低的连通度，但要求基线明显更强。

### 7.3 `support_first`

目标：偏向 COLMAP 通用场景、小场景、纹理丰富场景。

特点：

1. 强调 F/E 内点数。
2. 强调 `num_valid_points`。
3. 强调 view graph degree / 后续可扩展性。
4. 角度作为辅助项，且饱和。

### 7.4 `conservative`

目标：优先选择 early-stage 几何更稳、更不易 fold 的 seed。

特点：

1. 更严格前向运动限制。
2. 更严格正深度 / cheirality 约束。
3. early-stage 优化更保守，减少自由度。
4. 对删除-恢复 churn、BA 后大改变量更敏感。

---

## 9. 短程试跑长度

当前经验表明，很多坏 seed 在第 4 张图像附近已经暴露出“点云交叉 / 结构翻折 / 状态振荡”，即使系统表面上仍在成功注册。

因此，短程试跑建议：

1. 第一版默认 `max_eval_images = 6`。
2. 允许配置 `4 <= N <= 10`。
3. 默认不要一开始就跑到 10 张，以控制成本。

建议停止条件：

1. 达到 `max_eval_images`。
2. 连续 `k` 次 resection 失败。
3. 几何健康度分数跌破硬阈值。
4. 发生明显结构崩坏，例如正深度比例严重下降。

---

## 10. 评估指标体系

最终目标不是单看“注册成功”，而是衡量几何是否稳定地进入正确 basin。

### 9.1 增长类指标

作用：衡量 early-stage 是否能长起来，但不应成为唯一决策依据。

建议指标：

1. 已注册图像数。
2. 成功 resection 次数。
3. 每轮 PnP inlier 数中位数。
4. 每轮新增 triangulated track 数。

### 9.2 刚性几何指标

作用：衡量结构是否物理合理、是否开始翻折。

建议指标：

1. 正深度比例。
2. cheirality flip 比例。
3. 新增三角化点的 triangulation angle 分布：median、p10、低于阈值比例。
4. 深度排序反转比例。
5. 结构穿插 / 交叉近似检测指标。

其中正深度与 cheirality 属于刚性约束，必须高权重使用。

### 9.3 生命周期稳定性指标

作用：直接衡量当前几何是否稳定，是否在 delete / restore 振荡。

建议指标：

1. 点删除率：每轮 BA / rejection 后被删的 triangulated tracks 比例。
2. 点恢复率：re-triangulation 恢复的点数占最近删除点数比例。
3. 点 churn 率：同一批点在最近若干轮反复 delete / restore 的比例。
4. observation churn 率：观测被删、恢复、再删的比例。
5. 新点存活率：新三角化点活过 `n` 轮的比例。

这类指标对当前 InsightAT 的 pipeline 特别重要，因为系统本身具备一定自恢复能力，仅看“活着没活着”不足以识别坏 seed。

### 9.4 优化稳定性指标

作用：检测过拟合与错误 basin 内的局部自洽。

建议指标：

1. BA 后点位移中位数 / p90。
2. BA 后相机位姿改变量。
3. 固定位姿点优化的误差收益。
4. 刚做完 fixed-pose point optimize 后又被删除的比例。

如果 early-stage 每轮 BA 都导致相机或点大幅移动，通常说明模型尚未进入稳定 basin。

---

## 11. 最终评分原则

不建议用单一硬阈值比较所有策略，而建议构造一个综合健康分：

$$
score = w_g \cdot growth - w_c \cdot churn - w_h \cdot geometry\_instability - w_o \cdot overfit\_signals
$$

其中：

1. `growth`：注册图像数、PnP inlier、中位新增点数等。
2. `churn`：点删除率、点恢复率、点 churn、obs churn。
3. `geometry_instability`：正深度下降、cheirality flip、角度退化、深度排序反转。
4. `overfit_signals`：点位移过大、位姿改变量大、fixed-pose optimize 后迅速失效。

评分原则：

1. 注册数不是唯一主导项。
2. 刚性几何错误应有强惩罚。
3. 生命周期振荡应有高权重惩罚。
4. 如果某策略增长稍慢但几何明显更稳定，应优先考虑该策略。

---

## 12. 工作目录与输出格式

建议目录结构：

```text
seed_eval_out/
  report.json
  best_seed.json
  strategy_00_balanced/
    config.json
    summary.json
    iter_0001.json
    iter_0002.json
    ...
  strategy_01_wide_baseline/
    ...
  strategy_02_support_first/
    ...
  strategy_03_conservative/
    ...
```

每个 `summary.json` 至少包含：

1. strategy 名称
2. 选中的初始 pair
3. 最大注册图像数
4. 各类统计指标摘要
5. 最终综合分
6. 是否出现硬失败

`report.json` 包含：

1. 输入数据摘要
2. 所有策略排名
3. 最佳策略与理由
4. 可供主流程复用的建议配置

---

## 13. 与现有 pipeline 的复用边界

### 12.1 建议复用的部分

第一版应尽量复用以下现有逻辑：

1. `ViewGraph` 与初始 pair 候选获取。
2. `try_initial_pair_candidate()` 的几何验证逻辑。
3. `run_batch_resection()` / `choose_next_resection_image()`。
4. `run_batch_triangulation()` / `run_retriangulation()`。
5. `run_local_ba_dispatch()` 的局部 BA 调度。

### 12.2 建议新增的部分

应新增但尽量薄的一层：

1. `SeedEvalStrategy` 抽象：给出各策略的候选排序偏好与 early-stage 配置。
2. `SeedEvalStats`：收集每轮增长 / 几何 / churn / 优化稳定性指标。
3. `SeedEvalRunner`：对单一策略执行短程试跑。
4. `SeedEvalReportWriter`：负责 JSON 输出。

### 12.3 第一版不建议做的事情

1. 不要在现有 `run_incremental_sfm_pipeline()` 内部直接引入多策略并行。
2. 不要第一版就做完整中间状态序列化恢复。
3. 不要第一版就把所有参数做成自动网格搜索。

---

## 14. 实施路线

建议按三步走。

### Phase A：设计验证版

目标：验证指标是否有判别力。

内容：

1. 实现 `isat_seed_eval` CLI 骨架。
2. 支持 4 个固定策略。
3. 每个策略最多注册到 6 张。
4. 先统计 6 个核心指标：
   - 注册图像数
   - 正深度比例
   - 点删除率
   - 点恢复率
   - 点 churn 率
   - BA 后点位移 p90
5. 输出 JSON 排名，不自动接正式 SfM。

### Phase B：判别力增强版

目标：让“第 4 张左右的几何崩坏”能稳定被识别。

内容：

1. 增加 cheirality flip 比例。
2. 增加 triangulation angle 分布退化统计。
3. 增加深度排序反转指标。
4. 增加 fixed-pose point optimize 后又被删的比例。

### Phase C：主流程集成版

目标：将 seed-eval 结果反馈给正式增量 SfM。

内容：

1. `isat_incremental_sfm` 支持读取 `best_seed.json`。
2. 将最佳策略的初始 pair / 配置注入正式流程。
3. 后续若需要，再考虑恢复中间状态。

---

## 15. 第一版 MVP 建议

第一版建议严格控制范围：

1. 新增 CLI：`isat_seed_eval`
2. 固定四种策略：`balanced` / `wide_baseline` / `support_first` / `conservative`
3. 默认每组最多注册 6 张图像
4. 核心指标 6 项：
   - 注册图像数
   - 正深度比例
   - 点删除率
   - 点恢复率
   - 点 churn 率
   - BA 后点位移 p90
5. 输出 `report.json` + `best_seed.json`
6. 不自动续跑正式全量 SfM

这样能以较小工程量验证这套系统是否真正优于继续调单一初始对排序分数。

---

## 16. 结论

本设计的核心判断是：

1. 初始 seed 选择不应再被视为“单个 pair 的静态评分问题”。
2. 更合理的建模方式，是“多策略生成候选 + 独立 CLI 短程局部 SfM 试跑 + 几何健康度选优”。
3. InsightAT 当前 pipeline 的 delete / restore / retriangulation 特性，使得“点删除-恢复 churn”成为特别有价值的稳定性指标。
4. 采用独立 CLI 进程而非单进程多状态复制，可以显著降低实现复杂度和调试成本。

如果该系统判别力足够好，它将把“初始对选错导致整次空三跑偏”的风险，从一次性静态赌运气，变成一个更稳健的、有证据支撑的 early-stage 选择过程。