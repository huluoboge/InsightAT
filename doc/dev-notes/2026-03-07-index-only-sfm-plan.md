# Index-only SfM 导出与工具链重构计划

## 目标

- **isat_project** 导出 ATTask 时产出**紧致数组结构**：图像列表带 `image_index`（0..n-1），相机列表带 `camera_index`（0..m-1），每张图带 `camera_index`。
- **后续所有算法（tools）** 统一使用 **index**：特征/匹配/几何等文件名均为 index（如 `{image_index}.isat_feat`、`{idx1}_{idx2}.isat_match`），pairs 等 JSON 使用 `image1_index`/`image2_index`（或保留 `image1_id`/`image2_id` 但语义为 index）。
- **不再维护** image_id 与 index 的 IdMapping，导出后全链路 index。

## 1. 导出格式（isat_project extract）

### 1.1 输出 JSON 结构

- **images**：数组，顺序即 image_index（0..n-1）。每项：
  - `image_index`：int，与数组下标一致，用于后续所有工具。
  - `image_id`：原始数据库 image_id，仅用于核对/追溯。
  - `path`：图像路径。
  - `camera_index`：int，指向 `cameras` 数组下标。
  - 可选：`gnss`、`imu` 等与现有一致。
- **cameras**：数组，顺序即 camera_index（0..m-1）。每项：`fx`、`fy`、`cx`、`cy`、`width`、`height` 及可选元数据。
- **metadata**：与现有一致（format_version、task_id、坐标系等）。

同一文件中同时包含 `images` + `cameras`，形成「图像列表 + 相机列表」的紧致结构。

### 1.2 实现要点

- 遍历 ATTask 选中的 ImageGroup，按固定顺序（先组、组内按图）展开为 images[]，依次赋 `image_index = 0, 1, …`。
- 先建 cameras[]：对每个选中的 group 用 `pickCameraFromTask` 取相机，去重并按出现顺序填入 cameras，记录 group_id → camera_index；每张图取其所在组的 `camera_index`。

## 2. 工具链约定

- **特征**：`{image_index}.isat_feat`（不再用 image_id 命名）。
- **匹配**：`{image1_index}_{image2_index}.isat_match`。
- **几何**：`{image1_index}_{image2_index}.isat_geo`。
- **Pairs JSON**：推荐字段 `image1_index`、`image2_index`；为兼容可同时支持 `image1_id`/`image2_id` 且语义为 index。
- 内参：由导出中的 `cameras` + 每图的 `camera_index` 解析，无需再通过 group_id/camera_id 映射。

## 3. 各模块修改清单

| 模块 | 修改内容 |
|------|----------|
| **isat_project** | runExtract：输出 images[].image_index、images[].camera_index 与 cameras[]；runIntrinsics 可保留，用于仅导出相机场景。 |
| **pair_json_utils** | 支持从 pair 读取 `image1_index`/`image2_index`（或回退到 `image1_id`/`image2_id`），返回 index。 |
| **isat_intrinsics.h** | 支持从「带 cameras 的 image list」按 image_index → cameras[camera_index] 构建内参；保留原有 image_id→camera_id 逻辑以兼容旧 JSON。 |
| **isat_extract** | 从 list 读 `image_index`（缺省用数组下标）；输出 `{image_index}.isat_feat`。 |
| **isat_retrieve** | 输出 pairs 使用 `image1_index`、`image2_index`。 |
| **isat_match** | 从 pairs 取 index，特征/匹配路径按 index 拼接。 |
| **isat_geo** | 从 pairs 取 index；内参优先从 list 内 cameras + camera_index 解析。 |
| **isat_twoview** | 同上，路径与内参均按 index。 |
| **isat_tracks** | pairs 与路径按 index；Phase 1/2 均以 geo F/E inlier 为准；TrackStore 内部 image_index。 |
| **isat_calibrate** | 读取 twoview/路径按 index。 |
| **view_graph_loader** | 已改为 index，无需 IdMapping。 |
| **full_track_builder** | 已按 index 读 pairs/路径；可移除 IdMapping 参数。 |

## 4. 与设计文档的对应

- 实现后，`doc/design/12_implementation_details.md` 中「SfM 内部 ID 重编码与向量化相机表示」一节应更新为：导出即 index，不再需要 IdMapping；文件名与 JSON 均以 index 为准。
- `doc/design/image_id_design.md` 可补充：导出阶段产生 image_index + cameras 紧致结构，后续工具仅使用 index。

---

*本文档为 2026-03-07 重构计划，具体以代码与 design 文档为准。*
