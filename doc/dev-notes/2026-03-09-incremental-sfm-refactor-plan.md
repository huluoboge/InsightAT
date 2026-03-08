# 2026-03-09 增量 SfM 重构实施计划

基于 `isat_tracks` 输出的 track IDC 进行增量重建的整体重构计划，包含三大算法与实施顺序。相关代码：`src/algorithm/tools/isat_tracks.cpp`、`src/algorithm/modules/sfm/incremental_sfm*.cpp`、`view_graph`、`resection`、`track_store`。

---

## 1. 背景与输入

- **输入**：`isat_tracks` 输出的单个 `.isat_tracks` IDC（含 `image_indices`、观测、无两视图 3D）。
- **现状**：当前增量重建在 `src/algorithm/modules/sfm/incremental_sfm_engine.cpp` 中通过 `build_full_track_store_from_pairs` 从 pairs 重建 TrackStore，与 isat_tracks 分离；初始对仅由 ViewGraph 选一次，无“试到成功为止”的循环；resection 为单图；三角化与去畸变分散在多处。需整体重构。

---

## 2. 算法一：初始图像对选择

- **数据来源**：从 geo 中取得**非退化**的 pair 列表（例如 E_ok、twoview_ok、stable，可结合 `src/algorithm/modules/sfm/view_graph.h` 的 `PairGeoInfo` / degeneracy）。
- **流程**：按某种**优先级排序**（如 score = f(stable, num_valid_points, connectivity)），然后**循环**尝试：
  - 选当前优先级最高的未尝试 pair；
  - 对该对做两视图重建（三角化 + 两视图 BA）；
  - 若成功（如 track 数、RMSE 满足阈值）则**选定**并退出；
  - 否则标记失败、试下一对，直至成功或穷尽。
- **与现有代码**：可复用 `src/algorithm/modules/sfm/view_graph.cpp` 的建图与打分，扩展为“多候选 + 试算”循环；`run_initial_pair_loop`（`incremental_sfm.cpp`）改为在循环内调用，成功才返回。

---

## 3. 算法二：基于 track 的 resection（PnP）

- **思路**：用 track 中已三角化的 3D 点与未注册图像的 2D 观测做 PnP（RANSAC + pose-only BA），与现有 `resection_single_image`（`src/algorithm/modules/sfm/resection.h`）一致，但策略需明确：
  - **图像选择策略**：例如按 3D–2D 对应数量、或网格覆盖（`resection_image_grid_coverage`）排序，选择下一批待注册图像。
  - **批量策略**：不限定“每次只 resection 一张图”；可设计为每次选 **k 张**图像分别做 PnP（或按依赖关系分批），以利于并行或减少 BA 轮数。
- **去畸变**：所有 2D 观测按**当前相机状态**（image → camera_index → Intrinsics）做去畸变后再进 PnP/BA，与现有 `resection.cpp` 中 `K.has_distortion()` 分支一致；需保证 image–camera 索引一致且高效。

---

## 4. 算法三：三角化（Triangulate）与 re-triangulation、Local BA

- **批量三角化**：一次对 **n 张**新注册图像相关的 track 做三角化（多视图 DLT 或等效），而不是严格“每注册一张就只三角化该张”。接口上可设计为：输入当前已注册集合与待三角化 track 列表，按可见视图批量计算 3D。
- **畸变**：所有参与三角化的 2D 观测**必须先去畸变**；去畸变参数来自**当前相机状态**（image_index → camera_index → Intrinsics），保证与 BA 使用同一套参数。
- **图像–相机索引**：设计明确“image_index → camera_index”的映射（与 export list 的 camera_index 一致），所有观测、内参访问均通过该映射高效索引。
- **Re-triangulation**：对因重投影误差大而被误删的 track，通过 **re-triangulation** 重新计算 3D 再评估，避免误删；与 `src/algorithm/modules/sfm/track_store.h` 的 `track_needs_retriangulation` / `set_track_retriangulation_flag` 配合。
- **Local BA**：支持**局部 BA** 策略（例如只优化最近注册的若干相机 + 相关 track），在文档中写明策略目标（稳定性/速度）与触发条件（如每注册 m 张做一次 local BA）。

---

## 5. 数据流与入口

- **Track 来源**：从 `.isat_tracks` IDC **直接加载** TrackStore + image_indices（复用 `isat_tracks.cpp` 的 `load_track_store_from_idc` 逻辑），不再在增量 SfM 内部从 pairs 建 track。
- **相机/内参**：由 image list（含 cameras + camera_index）构建 per-image intrinsics，与 isat_geo/isat_twoview 一致；image_index = list 下标，camera_index 从 list 中读取，用于去畸变与 BA。

---

## 6. 实施顺序

1. **Track 从 IDC 加载**：isat_incremental_sfm 或新入口支持 `-t tracks.isat_tracks`，内部用 `load_track_store_from_idc` 得到 Store + image_indices，去掉对 `build_full_track_store_from_pairs` 的依赖。
2. **初始对循环**：在 ViewGraph 上实现“非退化 pair 排序 + 循环试算（三角化 + 两视图 BA），成功即停”。
3. **Resection 策略**：明确图像选择与批量 resection 的接口与策略（单图 vs 批量 k 图），并统一去畸变与 image–camera 索引。
4. **批量三角化与去畸变**：实现“一次 n 张”的三角化路径，所有 2D 按当前相机去畸变；与现有 `triangulate_tracks_for_new_image` / `retriangulate_two_view_tracks` 整合或替换。
5. **Re-triangulation 与 Local BA**：在 pipeline 中接入 re-triangulation 标记与重算；设计并实现 local BA 的触发与范围。

---

*本文档为 2026-03-09 增量 SfM 重构计划，具体以代码实现为准。*
