# Track Graph、软 Split 与 Merge 设计

## 1. 设计结论

新增独立的 `TrackGraphStore`，使用 flat SoA + CSR 保存高阶 track 的原始
correspondence graph。`TrackStore` 继续保存当前 BA、三角化、observation
生命周期和当前 observation 归属。

```text
Observation
    全局 obs_id
    obs_track_id：当前活动 track

TrackStore::Track
    当前活动 track 或历史父 track
    自己的 observation id list
    graph_id
    parent_track_id
    XYZ / flags

TrackGraphStore::Graph
    graph_id
    父 track 的固定 observation 顺序
    CSR row offset
    adjacency value：低 15 位为邻居 local node，高位为 mask
```

规则固定为：

```text
初始 degree <= 3：不创建 graph，异常直接删除 observation
初始 degree >= 4：创建一个 graph，只保存真实原始 correspondence
禁止从 observation 集合生成完全图
split 和 merge 都回到同一个父 graph 重算 component
子 track 自己保存 observation list，但不复制 graph
跨 graph merge 不支持
初始 Track 构建只运行一次 UF
```

Graph 全部常驻内存是第一版目标。文件可以使用独立的 `.isat_graph` sidecar，
但运行时不以延迟加载为前提。

## 2. 当前 TrackStore

当前 observation 是独立的 SoA 记录：

```text
obs_id -> image_index, feature_id, u, v, scale, obs_track_id
```

当前 track 保存全局 observation ID：

```text
track_obs_ids_[track_id] = [obs_id0, obs_id1, ...]
```

当前 image 反向索引为：

```text
image_obs_ids_[image_index] = [obs_id, ...]
```

`image_obs_ids_` 只保存 observation ID，不保存 track ID。因此 split 时不需要
重建 image -> observation 结构索引；只更新 `obs_track_id_`，并为子 track
建立自己的 observation list。

现有逻辑删除规则保持不变：删除 observation 清除 `kAlive`，不移动 observation
数组；删除 track 清除 `kAlive`，不复用 track_id。父 track 的 observation 顺序
因此可以作为 graph local node 的稳定映射。

## 3. Track ID、graph ID 和 lineage

一个初始高阶 track P 创建一个 graph G：

```text
P.graph_id = G
```

P split 后：

```text
C0.graph_id = G
C1.graph_id = G
parent_track_id[C0] = P
parent_track_id[C1] = P
```

`graph_id` 表示共享哪一份原始拓扑；`parent_track_id` 表示历史来源。多次
split 可以形成 `P -> C0 -> C00, C01`，所有后代仍可共享同一个 graph_id。

TrackStore 增加：

```cpp
std::vector<uint32_t> track_graph_id_;
std::vector<int32_t> track_parent_id_;
```

可选的 flat lineage 索引：

```cpp
std::vector<int32_t> track_first_child_;
std::vector<int32_t> track_next_sibling_;
```

```cpp
constexpr uint32_t kInvalidGraphId = 0xffffffffu;
constexpr int32_t kInvalidTrackId = -1;
```

建议增加 `track_flags::kSplitParent`。split 后父 track 是历史容器，不再与
child 同时作为 BA 中的活动 3D point。

## 4. Graph 边语义和 degree 规则

一条 graph edge 只表示输入 verified match/geo 中真实存在的一条 correspondence：

```text
(image_a, feature_a) <-> (image_b, feature_b)
```

当前 observation 由 `(image_index, feature_id)` 唯一确定，同一对 observation
最多对应一条 match edge，不需要多重 edge ID。

禁止：

```text
12 个 observation -> 自动生成 66 条完全图边
```

实际保存多少 edge，只取决于原始 match 中有多少真实 correspondence。初始 graph
只保存两个 endpoint 最终属于同一个、且初始 degree >= 4 的 track 的 edge。

```text
初始 degree <= 3：graph_id = kInvalidGraphId，异常直接删除 observation
初始 degree >= 4：创建一个 graph，保存真实内部 edge
```

split 后 child 即使只有 2 或 3 个 observation，也继续保留原 graph_id；不创建
新 graph。

## 5. TrackGraphStore 的 SoA + CSR

不使用每个 track 一个 `std::vector<Edge>`，也不在子 track 中复制 graph。建议：

```cpp
class TrackGraphStore {
public:
  std::vector<uint32_t> graph_owner_track_id_;
  std::vector<uint64_t> graph_node_offset_;
  std::vector<uint64_t> graph_adj_offset_;
  std::vector<uint16_t> graph_adj_neighbor_;
};
```

字段含义：

```text
graph_owner_track_id_[g]
    graph g 的原始父 track id

graph_node_offset_[g]
    graph g 的 global CSR row 起点
    graph g 的 row 范围为
    [graph_node_offset_[g], graph_node_offset_[g + 1])

graph_adj_offset_[r]
    global CSR row r 的邻接范围起点
    row r 的邻接项为
    [graph_adj_offset_[r], graph_adj_offset_[r + 1])

graph_adj_neighbor_[p]
    低 15 位：同一 graph 内的 local node index
    最高位：mask，1 表示该邻接项无效
```

数组长度：

```text
graph_owner_track_id：num_graphs
graph_node_offset：num_graphs + 1
graph_adj_offset：total_graph_nodes + 1
graph_adj_neighbor：total_adjacency_entries
```

父 track observation list 是 node 到全局 observation 的映射：

```text
global_obs_id = track_obs_ids_[owner_track_id][local_node]
```

Graph 不重复存储这份 observation ID list。

邻接值编码：

```cpp
using GraphAdjValue = uint16_t;
constexpr GraphAdjValue kGraphAdjMask = 0x8000u;
constexpr GraphAdjValue kGraphAdjIndexMask = 0x7fffu;
```

```text
0x0007：邻居 local node 7，有效
0x8007：邻居 local node 7，已 mask
```

低 15 位限制 graph node 数不超过 32767。当前最大 track degree 是 1046；构建
时必须检查，不能静默截断。

一条无向 edge `u <-> v` 在 CSR 中保存两个邻接项：`row(u) -> v` 和
`row(v) -> u`。每条无向 edge 使用 4 bytes payload。设置或清除 mask 时必须
同步修改两个方向。第一版可在两个 CSR row 内线性查找反向项。

父 track 的 observation list 是稳定的 graph node 顺序：

```text
P.obs_ids[0] -> graph local node 0
P.obs_ids[1] -> graph local node 1
...
```

split 后不能清空、重排或压缩父 list。逻辑删除也不改变 local index；replay
时跳过已删除 observation。若父 list 被 compact/reorder，必须同时重建 graph。

## 6. 内存估算

令 `N` 为 graph node 数、`E` 为无向 edge 数、`G` 为 graph 数。64-bit offset 的
主要内存为：

```text
graph_owner_track_id  4 * G bytes
graph_node_offset     8 * (G + 1) bytes
graph_adj_offset      8 * (N + 1) bytes
graph_adj_neighbor    4 * E bytes
```

当前数据：

```text
images                  3,930
tracks              3,376,269
observations        31,724,754
平均 degree              9.396
最大 degree              1,046
degree >= 4 tracks   1,312,867
degree >= 4 observations 27,040,491
```

100M 条无向 edge 时：

```text
graph_adj_neighbor       约 381 MiB
graph_adj_offset         约 206 MiB（按约 27.04M 高阶 graph node）
graph node/owner offset  约 15 MiB（约 1.31M graph）
```

合计约 602 MiB，不含 vector capacity。500M 条无向 edge 时，邻接 payload 约
1.86 GiB，Graph SoA 总体约 2.06 GiB。`track_graph_id_` 和 `track_parent_id_`
各约 13 MiB。完整 graph 常驻内存是第一版接受的成本。

如果确认所有 row 和邻接项小于 `2^32`，内存实现可以使用 `uint32_t offset`；
文件 schema 仍需记录 offset 宽度。

## 7. Graph 构建流程

第一遍读取 match/geo，只运行一次 UF：

```text
创建 feature node
执行 one-feature-per-image 约束下的 UF
得到 node -> final track
构建 TrackStore observation list
统计最终 degree
```

最终 track 和 degree 未确定前，不能把 edge 写入最终 graph。

重新读取真实 verified edge，对每条 edge：

```text
1. endpoint node -> final track / global obs_id
2. 两端必须属于同一个 final track
3. final track degree >= 4
4. 找到 graph_id
5. 通过父 track observation list 得到两个 local node
6. 对两个 local node 的邻接度各加一
```

计数后 prefix sum 得到 `graph_adj_offset`，再次填充 `graph_adj_neighbor`。可选
实现：

```text
A. 原始输入读取三遍：逻辑最简单
B. 原始输入读取两遍：第二遍暂存紧凑 endpoint pair
C. 过滤 edge 写磁盘 spool：RAM 最低
```

无论 A/B/C，UF 都只运行一次。第一版优先 B，临时内存过大再切换 C。构建时
检查 local node < 32768、两端 graph_id 相同、无重复无向 edge、CSR 对称。

## 8. 软 Split

软 split 不复制 graph，也不从父 track 移除原始 observation：

```text
父 P：保留原始 obs list，graph_id = G，转为历史 split parent
子 C0：新 track_id，graph_id = G，parent_track_id = P，自己的 obs list
子 C1：新 track_id，graph_id = G，parent_track_id = P，自己的 obs list
```

子 track 必须保存自己的 observation list。仅更新 `obs_track_id_` 而不建立
child list 会破坏现有 BA、三角化和重投影访问模型。

split 步骤：

```text
1. 找到 graph_id = G 的 owner parent P
2. 在 G 的 parent-local node 上运行 constrained UF
3. 跳过 mask 邻接项和已删除 observation
4. 维持 one-feature-per-image 约束
5. 得到 connected components；component <= 1 则失败
6. 每个 component 创建 child track
7. local node -> P.obs_ids -> global obs_id
8. 填充 child observation list
9. 更新 obs_track_id_[obs_id] = child_track_id
10. P 标记为历史父 track
11. child 不继承 P 的 XYZ，清除并进入重三角化队列
```

不能用 `add_observation()` 创建新 observation；需要增加：

```cpp
attach_existing_observation(new_track_id, obs_id);
```

它只更新 `obs_track_id_` 和 child list，不创建新 obs_id，也不重复加入
`image_obs_ids_`。

## 9. Merge 与重新分区

merge 不能直接拼接 child 的 observation list。权威来源始终是共同的父 graph：

```text
修改 graph mask
在固定 parent-local node 上重新计算 components
按新的 components 重新分配 observation
```

只有 `track_a.graph_id == track_b.graph_id` 时允许通过本机制 merge；不同 graph
需要额外 candidate edge，属于非目标。

完整 rollback：

```text
1. 恢复该事务设置的两侧 adjacency mask
2. 恢复 P 为 active
3. 将 parent obs 的 obs_track_id 改回 P
4. child 标记为 history/inactive
5. 清除或重新计算 P 的 XYZ
```

部分 merge 也必须在 graph G 的全部 node 上重算 components：恢复需要恢复的
mask，保留其他 mask，创建/复用新的 child，一次性更新所有受影响的
`obs_track_id`，旧 child 标为 history。

## 10. Mask 事务和统一入口

候选 split 在几何验证通过前不能永久污染共享 graph。第一版只允许一个活动
transaction：

```cpp
struct GraphMaskTransaction {
  uint32_t graph_id;
  std::vector<uint64_t> changed_adj_positions;
  std::vector<int32_t> created_track_ids;
  std::vector<int32_t> changed_obs_ids;
  std::vector<int32_t> old_obs_track_ids;
};
```

失败时恢复 mask、obs_track_id、child/parent 状态、XYZ 和 dirty 状态。split 和
merge 都调用：

```cpp
rebuild_active_tracks_from_graph(graph_id, transaction);
```

该入口负责重放 masked CSR、应用 one-feature-per-image 约束、计算 components、
创建/复用 child、更新 observation list 和 `obs_track_id_`，并处理旧 track 与
XYZ 状态。

## 11. 一致性要求

重分区完成后必须满足：

```text
每个有效 observation 的 obs_track_id 指向活动 track
该 observation 出现在对应活动 track 的 observation list
image_obs_ids 中每个 observation 只出现一次
父 track 原始 observation list 未改变
子 track 没有 graph payload，只引用 graph_id
父 track 不与 child 同时参与 BA
每个 component 中同一 image 至多一个 observation
CSR local index 未超出 graph node 范围
无向 edge 的两个方向 mask 状态一致
```

建议提供 debug-only：

```cpp
validate_track_graph_consistency(graph_id);
```

## 12. 文件格式

```text
tracks.isat_tracks
    TrackStore observation、track、graph_id、parent_id

tracks.isat_graph
    TrackGraphStore graph SoA
```

运行时第一版把 graph 全部加载到内存；sidecar 只是避免把大 graph blob 塞入
已有 metadata header。

Track blobs：

```text
track_graph_id       uint32[num_tracks]
track_parent_id      int32[num_tracks]
track_first_child    int32[num_tracks]（可选）
track_next_sibling   int32[num_tracks]（可选）
```

Graph blobs：

```text
graph_owner_track_id  uint32[num_graphs]
graph_node_offset     uint64[num_graphs + 1]
graph_adj_offset      uint64[num_graph_nodes + 1]
graph_adj_neighbor    uint16[num_adjacency_entries]
```

```text
num_adjacency_entries = 2 * num_undirected_edges
```

metadata 至少记录 `schema_version`、`num_graphs`、`num_graph_nodes`、
`num_undirected_edges`、`min_split_degree`、`adj_mask_bit`、`adj_index_bits` 和
`offset_bits`。加载时验证 owner track、父 observation 顺序、offset 长度和
邻接 local index。

## 13. 测试和实现阶段

构建测试：degree <= 3 没有 graph_id；degree >= 4 有且只有一个 graph；不生成
完全图；不保存跨初始 track edge；每条无向 edge 恰有两个对称邻接项；默认
mask 有效；local node 在 15 位范围内；不启用 graph 时 TrackStore 结果不变。

Split/Merge 测试：构造两个 component 由 bridge 连接的 graph，mask bridge 后
创建两个 child，验证 child list 与 `obs_track_id` 一致、parent list/local
index 不变、child XYZ 清除并进入重三角化；恢复 mask 后得到一个 component，
旧 child 不再 active，失败事务能恢复全部状态。

实现顺序：

```text
Phase 1：TrackStore graph_id/parent_id，TrackGraphStore CSR SoA，mask 编解码，
         attach_existing_observation，一致性检查
Phase 2：保留一次 UF，构建 degree >= 4 graph，生成 CSR
Phase 3：parent graph replay、软 split、observation 重新分配
Phase 4：同 graph merge、rollback、mask transaction
Phase 5：自动 split 候选和几何验证
```

## 14. 决策表

| 问题 | 决策 |
|---|---|
| Graph 放在哪里 | 独立 `TrackGraphStore`，track 只存 `graph_id` |
| Graph 表达 | SoA + CSR 稀疏邻接表 |
| 邻居索引 | `uint16_t` 低 15 位 |
| mask | `uint16_t` 最高位，1 表示无效 |
| 无向 edge | CSR 中保存两个对称邻接项 |
| 子 track 是否记录 observation | 是，自己的 global obs_id list |
| 子 track 是否复制 graph | 否，共享父 graph |
| 父 track 是否保留 observation list | 是，顺序固定作为 graph node 映射 |
| split 如何做 | 修改父 graph mask，重算 component，重新分配 |
| merge 如何做 | 恢复父 graph mask，重算 component，重新分配 |
| 初始 degree <= 3 | 不建 graph，异常直接删 observation |
| 跨 graph merge | 不支持 |
| 初始 UF | 只运行一次 |
| Graph 常驻内存 | 第一版接受 |
