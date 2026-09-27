# 需求：isat_sfm 工作目录结构整理（方案 B）

- **状态**：需求记录（未实现）
- **日期**：2026-09-27
- **范围**：`isat_sfm` / `isat_incremental_sfm` / `isat_undistort` / UI 任务目录约定
- **目标**：中间产物与重建导出分层清晰；Bundler 与 COLMAP 导出并列；根目录只保留项目入口文件

## 1. 背景与问题

当前典型 `work/` 布局混杂：

```
work/
  images_all.json
  project.iat
  feat/  feat_retrieval/
  match/
  geo/
  pairs_retrieve.json      ← 散落在根
  pairs_matched.json       ← 散落在根
  tracks.isat_tracks       ← 散落在根
  seed_eval_all/
  incremental_sfm/
    poses.json
    bundle.out / list.txt  ← 与 colmap 不对等
    colmap/sparse/0/
    tracks.isat_tracks
```

问题：

1. SfM 导出里 Bundler（`bundle.out` + `list.txt`）直接落在 `incremental_sfm/` 根下，而 COLMAP 已有子目录，结构不对等。
2. 配对 JSON、tracks 阶段产物堆在 `work/` 根上，和入口文件、特征/匹配目录混在一起，不利于续跑与文档说明。

## 2. 方案选择

采用 **方案 B**：

- Bundler 单独建目录，与 `colmap/` 并列；
- 同时将 `pairs_*`、tracks 阶段产物收纳到对应子目录。

不做「仅改 Bundler 并列、不动根目录 JSON/tracks」的最小方案 A。

## 3. 目标目录结构

```
work/
  # 项目入口（留在根）
  images_all.json
  project.iat
  sfm_timing.json            # 可选，管线汇总

  # 特征
  feat/
  feat_retrieval/

  # 匹配（含配对清单）
  match/
    pairs_retrieve.json      # 检索得到的候选对
    pairs_matched.json       # 匹配后保留的对
    *.isat_match

  # 几何
  geo/
    pairs.json               # 几何后的 view graph（保持现状位置亦可，见备注）
    *.isat_geo

  # Tracks 阶段产物
  tracks/
    tracks.isat_tracks

  # Seed 评估（保持独立目录）
  seed_eval_all/

  # 增量 SfM 结果与导出
  incremental_sfm/
    poses.json
    tracks.isat_tracks       # SfM 结束后带位姿嵌入的 tracks 副本（现状保留语义）
    bundler/
      bundle.out
      list.txt
    colmap/
      sparse/0/              # 带畸变 OPENCV/FULL_OPENCV（与 Bundler 同语义，不去畸变）
      images/                # 仅 isat_undistort 可选步骤写出

  # 可选调试
  sfm_interval/
    iter_NNNN/
      bundle.out
      list.txt
```

### 备注

- **COLMAP（SfM 默认写出）**：保持带畸变相机模型 + 畸变像素观测，与 Bundler 一致；**不在默认导出路径做去畸变**。
- **`isat_undistort`**：仍为可选步骤，专供 3DGS 等需要 PINHOLE + 去畸变图像的下游；输出可继续落在 `incremental_sfm/colmap/` 下，或后续再拆 `colmap_undistorted/`（本需求不强制）。
- **`geo/pairs.json`**：已在 `geo/` 内，可维持；若实现时希望统一命名，可在实现 PR 中一并说明，不作为本需求阻塞项。

## 4. 功能需求

| ID | 需求 | 验收要点 |
|----|------|----------|
| R1 | `write_bundler` 默认写到 `<sfm_out>/bundler/` | 存在 `bundler/bundle.out` 与 `bundler/list.txt`；`incremental_sfm/` 根下不再直接放这两个文件 |
| R2 | COLMAP 路径保持 `<sfm_out>/colmap/sparse/0/` | 与现状一致；与 `bundler/` 并列 |
| R3 | `isat_sfm` 将 `pairs_retrieve.json` / `pairs_matched.json` 写到 `match/` | 根目录不再出现这两个文件；下游 match/geo 调用路径同步 |
| R4 | tracks 阶段输出写到 `tracks/tracks.isat_tracks` | `isat_tracks -o`、`isat_incremental_sfm -t`、seed_eval、续跑路径同步 |
| R5 | `isat_sfm` 汇总日志 / UI 打开 Bundler 的路径指向新目录 | 例如 `at_bundler_viewer <work>/incremental_sfm/bundler` |
| R6 | 颜色导出逻辑不因目录搬迁失效 | 仍能自动发现 `work/feat/`（或显式 `-f`）；有 colors 则 COLMAP/Bundler 点有色 |

## 5. 非目标

- 不改变 SfM 算法、相机模型语义（默认 COLMAP 仍不去畸变）。
- 不强制迁移用户已有旧 `work/` 数据；实现时可提供迁移说明，不做自动静默搬迁。
- 不在本需求中实现 `colmap_undistorted/` 再拆分（可另开需求）。

## 6. 兼容与迁移

实现时建议：

1. **新任务**：一律按目标结构写。
2. **旧任务续跑**：读取路径可短暂双读（先新路径，回退旧根路径），或在文档中要求用户手动移动文件；双读策略在实现 PR 中选定并写清。
3. 更新：`isat_sfm` 帮助/结束摘要、`doc/develop/design/05_cli_io_conventions.md`（若涉及）、UI `at_task_panel` 子目录列表与导出路径。

## 7. 涉及改动面（实现时）

- [`src/cli/isat_incremental_sfm.cpp`](../../src/cli/isat_incremental_sfm.cpp)：`write_bundler` 输出子目录；结束日志路径
- [`src/cli/isat_sfm.cpp`](../../src/cli/isat_sfm.cpp)：`pairs_*`、`tracks_path`、`sfm_out` 子路径；传给子进程的参数；摘要打印
- [`src/cli/isat_undistort.cpp`](../../src/cli/isat_undistort.cpp)：若依赖相对路径，仅同步文档/调用处
- [`src/ui/panels/at_task_panel.cpp`](../../src/ui/panels/at_task_panel.cpp)：创建子目录列表、Bundler/COLMAP 打开路径
- 相关 README / CLI 约定文档

## 8. 验收清单

- [ ] 全新跑通 `create → … → incremental_sfm` 后，目录符合第 3 节
- [ ] `bundler/` 与 `colmap/sparse/0/` 并列，根下无散落的 `bundle.out` / `list.txt`
- [ ] `work/` 根下无 `pairs_retrieve.json` / `pairs_matched.json` / `tracks.isat_tracks`
- [ ] `at_bundler_viewer` 与文档中的示例路径可用
- [ ] 带 `feat/colors` 时 COLMAP/Bundler 点颜色仍正确
