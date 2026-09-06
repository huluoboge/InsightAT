# Track Graph、软 Split 与 Merge 实现报告

日期：2026-09-07
分支：`feature/track-graph`
设计文档：`doc/dev-notes/2026-09-06-track-split-graph-design.md`

## 1. 实现范围

- `TrackStore` 增加 `graph_id`、`parent_track_id`、split parent 状态和已有 observation attach API。
- 新增 `TrackGraphStore`，使用 flat SoA + CSR 保存真实 verified correspondence。
- adjacency 使用 `uint16_t`，低 15 位为 local node index，最高位为 mask。
- CSR 构建去重无向边，拒绝 self edge、越界 local index和超过 15-bit 的 graph。
- 基于固定 parent-local observation 顺序进行 constrained component replay，并执行 one-feature-per-image 约束。
- 实现 soft split、同 graph merge、mask 双向同步和完整事务回滚。
- split parent 保留原始 observation list；child 使用自己的 observation list，不复制 graph payload。
- child 不继承 parent XYZ，设置重三角化标志；parent 转为历史容器。
- 新增 `.isat_graph` IDC sidecar 的保存、加载和严格 CSR 校验。
- `.isat_tracks` schema `1.4` 保存 graph lineage、parent/child observation ownership 和 canonical source ID。
- `isat_tracks` 在 UF 后通过真实 edge spool 构建 graph，禁止从 observation 集合生成完全图。
- graph 初始 degree `<= 3` 的 observation 删除；degree `>= 4` 的 track 建 graph。
- `--stats` 增加 graph 数、node 数、edge 数和 CSR validity 输出。

## 2. 单元测试

新测试目标：`test_track_graph`。覆盖 adjacency 编解码、mask、最大 15-bit index、edge 去重、多 graph CSR offset、对称 adjacency、非法 graph size、越界 edge、自环、malformed serialized CSR、empty graph sentinel、soft split、merge、事务回滚、deleted observation、one-feature-per-image 约束和 consistency validator 错误路径。

验证结果：

```text
All track graph tests passed
All existing TrackStore state-cache tests PASSED.
```

使用 ASan/UBSan 编译运行的新核心测试也通过，未发现内存错误。

coverage 结果：`TrackGraphStore.cpp` 业务代码行覆盖率 100%，所有核心 public 函数均被测试调用。gcov 报告中的少量未执行分支是 C++ 标准库分配失败和异常展开生成的隐式分支，不属于 Track Graph 业务判断分支；旧 `TrackStore` 覆盖率不作为本次要求的一部分。

## 3. 100RGB 真实数据验证

输入目录：`/home/jones/Data/01-benchmark/03-insightat/scenes/100RGB/results/insightat/work`。

使用真实输入 `pairs_matched.json`、`match/`、`geo/` 和 `images_all.json`。输出写入独立目录 `/tmp/insightat-100rgb-track-graph`，没有覆盖原始 `work/tracks.isat_tracks`。

| 指标 | 结果 |
|---|---:|
| images | 344 |
| tracks | 425,116 |
| serialized observations | 2,141,686 |
| removed low-degree observations | 449,005 |
| graphs | 232,735 |
| graph nodes | 1,692,681 |
| undirected graph edges | 4,620,038 |
| adjacency entries | 9,240,076 |
| view graph pairs | 6,676 |
| peak RSS | 627,384 KiB，约 613.7 MiB |
| wall time | 2:08.82 |
| exit status | 0 |

输出文件约为：`tracks.isat_tracks` 82 MiB，`tracks.isat_graph` 34 MiB。

随后执行 `isat_tracks --stats -o /tmp/insightat-100rgb-track-graph/tracks.isat_tracks --quiet`，结果为：

```text
track_graphs=232735
graph_nodes=1692681
graph_undirected_edges=4620038
graph_adjacency_entries=9240076
graph_csr_valid=true
```

sidecar 被 loader 成功加载，所有 graph 的 CSR 对称性和 local index 校验通过。边数量来自真实 verified correspondence spool，不是 observation 完全图推导结果。

## 4. 构建说明

使用独立 CPU Ceres 配置目录 `build-track-graph-cpu` 构建，并关闭与本次功能无关且需要网络下载 RapidCheck 的 Render property tests：

```text
cmake -S . -B build-track-graph-cpu \
  -DCMAKE_BUILD_TYPE=Release \
  -DINSIGHTAT_BUILD_RENDER_TESTS=OFF \
  -DINSIGHTAT_ENABLE_SIFTGPU=OFF \
  -DCeres_DIR=/usr/lib/x86_64-linux-gnu/cmake/Ceres \
  -DCUDAToolkit_ROOT=/nonexistent
```

完整默认配置在本机还会遇到两个环境问题：Render tests 的 RapidCheck FetchContent 无法访问 GitHub，以及用户目录 CUDA Ceres 包的 cuDSS imported-target scope 与系统 CMake 版本不兼容。新增 `INSIGHTAT_BUILD_RENDER_TESTS` 选项允许在离线环境中关闭无关测试依赖；本次功能目标已在 CPU Ceres 配置下完整构建并验证。

## 5. 未包含范围

自动几何候选生成和完整 incremental SfM 端到端重建没有作为新 graph 功能的一部分改动；本次真实数据验证覆盖了新的 track builder、`.isat_tracks` lineage、`.isat_graph` sidecar 加载和 CSR consistency。旧代码的覆盖率和旧测试行为不纳入本次 100% 要求。
