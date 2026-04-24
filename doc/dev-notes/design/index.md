# InsightAT 设计文档索引

本目录为 **InsightAT** 的正式设计文档，按阅读与实现顺序组织（序号 01–12）。算法与 UI 实现应以本文档集为准。

---

## 01. 整体算法设计

- **[01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md)**  
  第一层：粗 SfM（Cluster + Merge + Global BA）；第二层：高精度 SfM（原始分辨率、位姿引导匹配、高精度相对位姿、Global BA 策略）。面向航空/倾斜摄影、大规模数据。

---

## 02. 代码规范

- **[02_coding_style.md](02_coding_style.md)**  
  命名、文件组织、注释、头文件、命名空间、GPU 与序列化约定、CLI 工具规范等。新代码必须遵守；格式由 `.clang-format` 执行。

---

## 03. 整体目录组织

- **[03_directory_organization.md](03_directory_organization.md)**  
  源码目录划分：`algorithm/`、`database/`、`util/`、`ui/` 及各自职责，与三层架构（Project / AT Task / Output）的对应关系。

---

## 04. CLI-First 实现

- **[04_functional_at_toolkit.md](04_functional_at_toolkit.md)**  
  CLI-First 架构、算法独立性、分布式就绪、IDC 自描述数据格式、工具间通过文件交换的流水线设计。

---

## 05. IO 交互标准

- **[05_cli_io_conventions.md](05_cli_io_conventions.md)**  
  所有 `isat_*` 工具的 stdout/stderr/退出码约定；机器可读输出格式 `ISAT_EVENT <单行 JSON>`；日志级别与管道友好用法。

---

## 06. UI 设计

- **[06_ui_framework.md](06_ui_framework.md)**  
  Qt Document/View、ProjectDocument 与 MainWindow、Model/View、组件化 Widget、信号槽驱动工作流及 UI 规范。

---

## 07. UI 与项目序列化

- **[07_serialization.md](07_serialization.md)**  
  项目与任务数据的序列化设计。底层支持 Cereal 版本化；**当前为便于调试与互操作，项目/配置等采用 JSON 格式**，二进制方案见文档与 `database/` 实现。

---

## 08–09. 技术细节设计（单独列出）

以下为具体技术选型与约定，实现时需严格对齐：

| 序号 | 文档 | 内容 |
|------|------|------|
| 08 | **[08_coordinate_and_rotation.md](08_coordinate_and_rotation.md)** | 坐标系类型（EPSG/WKT/ENU/Local）、旋转约定（Omega-Phi-Kappa vs Yaw-Pitch-Roll）、角单位与内部标准化 |
| 09 | **[09_data_model.md](09_data_model.md)** | Project、ImageGroup、Image、Measurement、ATTask 等核心数据结构（与 `database/` 对应） |
| — | **旋转专题** | 更详细的旋转表示与转换见 [design/rotation/](rotation/rotation_readme.md) |

---

## 10–12. 其他参考

| 序号 | 文档 | 说明 |
|------|------|------|
| 10 | **[10_introduction.md](10_introduction.md)** | 软件目标与核心概念（综述） |
| 11 | **[11_architecture_overview.md](11_architecture_overview.md)** | 三层架构与数据流向（与 03 目录组织互补） |
| 12 | **[12_implementation_details.md](12_implementation_details.md)** | 核心实现方案：坐标解析、畸变模型、任务快照、**SfM ID 重编码与向量化相机表示（§2.4）** |

---

## AI Agent 阅读顺序建议

1. **算法与流水线**：[01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md) → [04_functional_at_toolkit.md](04_functional_at_toolkit.md)
2. **代码与 CLI 行为**：[02_coding_style.md](02_coding_style.md) → [05_cli_io_conventions.md](05_cli_io_conventions.md)
3. **数值与坐标**：[08_coordinate_and_rotation.md](08_coordinate_and_rotation.md)，必要时 [rotation/rotation_readme.md](rotation/rotation_readme.md)
4. **UI 与持久化**：[06_ui_framework.md](06_ui_framework.md) → [07_serialization.md](07_serialization.md)
