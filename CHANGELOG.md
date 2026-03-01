# Changelog

All notable changes to the InsightAT project are documented in this file.

The design vision and architecture are described in [doc/design/](doc/design/) and [doc/design/parallel_hybrid_sf_m_design_philosophy.md](doc/design/parallel_hybrid_sf_m_design_philosophy.md). This changelog focuses on implementation and refactoring milestones.

---

## [Unreleased]

### 设计对齐与规划

- **架构**：遵循三层模型（Project Layer / AT Task Layer / Output Layer），算法与 UI 完全分离，CLI-First。
- **SfM 路线**：按《Parallel Hybrid SfM 架构设计》采用两层设计——**第一层**：粗 SfM = Cluster 内并行重建 + Merge + **Global BA**（得到粗的全局一致结果）；**第二层**（规划中）：原始分辨率特征、位姿引导匹配、高精度相对位姿、Global BA 策略。
- **数据格式**：IDC (Insight Data Container) 规范已定，支持 `.isat_feat` / `.isat_match` / `.isat_pose` 等，8 字节对齐、自描述 JSON 头。

### 已实现（与设计文档对应）

- **CLI 工具链**：`isat_extract`（特征提取）、`isat_match`（匹配）、`isat_retrieve`（检索）、`isat_geo`（几何验证）、`isat_train_vlad` / `isat_train_vocab`、`isat_project`、`isat_twoview`、`isat_tracks`、`isat_calibrate`、`isat_camera_estimator`；独立进程 `CameraEstimator`。
- **数据层**：`database` 模块（`database_types.h`）提供 CoordinateSystem、InputPose、Measurement、ATTask、Project、ImageGroup、CameraModel 等，Cereal 序列化与版本控制。
- **算法模块**：特征提取（SIFT GPU、SuperPoint 可选）、检索（VLAD、词汇树、GNSS、Sequence）、匹配、几何（GPU RANSAC）、两视图重建与焦距优化、track 构建。
- **UI**：Qt 主窗口、项目/任务/坐标系配置、相机参数与图像组管理、与 ProjectDocument 绑定；无 3D 渲染依赖。
- **规范**：CLI I/O 约定（stdout/stderr、`ISAT_EVENT`）、编码风格与 IDC 格式见 [doc/design/](doc/design/)。

### 进行中 / 计划中（相对设计文档）

- **第一层 SfM**：Cluster 内两视图/层级式增量重建 → **Merge**（Sim3/对齐）→ **第一层 Global BA**；简化畸变模型；GNSS+姿态下的航高/投影匹配策略待完善。
- **第二层 SfM**（规划中）：原始分辨率特征、位姿引导的匹配、高精度相对位姿估计、Global BA 策略；完整畸变模型。
- **流水线集成**：从检索→匹配→几何→tracks→第一层 Global BA 的端到端脚本与任务描述（如 JSON 任务列表）仍在完善。

### 重构与清理

- **Common 迁移**：原 `Common` 拆分为轻量 `util`（string_utils、numeric、insight_global）；Coordinates 迁入 `ui/utils`；exif 链迁入 `algorithm/io/exif`；未使用代码已删除。
- **Gui 移除**：旧版 Qt Gui 目录已删除；仅保留新主窗口 UI，QString 转换工具置于 `ui/utils/QStringConvert.h`。
- **Render 移除**：未使用的 OpenGL 渲染模块已删除；主程序仅依赖 UI + database，无 3D 查看器。
- **主程序命名**：可执行文件与 CMake 目标统一为 `InsightAT`（不再使用 InsightAT_New）。

### 文档

- 设计文档索引：[doc/design/index.md](doc/design/index.md)；架构、数据模型、坐标系与旋转、序列化、UI、函数式 AT 工具包见对应编号文档。
- 实现与工具说明见 [doc/dev-notes/](doc/dev-notes/)、[doc/tools/](doc/tools/)。

---

## 版本说明

- **Unreleased**：当前开发状态，API 与数据格式可能变动，不建议用于生产环境。
- 未来版本号将随首个稳定发布或语义化版本（如 0.1.0）一起引入。
