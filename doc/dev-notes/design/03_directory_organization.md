# 整体目录组织

本文描述 InsightAT 源码目录划分及其与设计架构的对应关系。

---

## 1. 源码目录结构

```
src/
├── algorithm/           # 核心算法（无 Qt 依赖）
│   ├── modules/         # 算法模块（extraction, retrieval, matching, geometry 等）
│   ├── io/              # IDC 读写、exif 等
│   └── tools/           # CLI 可执行程序（isat_*.cpp）
├── database/            # 数据模型与序列化（Cereal，见 05_serialization）
├── util/                # 轻量公共工具（string_utils, numeric 等，无 Qt）
└── ui/                  # Qt 界面（唯一允许依赖 Qt 的层）
    └── utils/           # UI 用工具（Coordinates, QStringConvert 等）
```

- **algorithm**：严禁依赖 Qt；可 headless、可容器化；与 [04_functional_at_toolkit.md](04_functional_at_toolkit.md) 及 [01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md) 对应。
- **database**：核心类型定义（Project、ATTask、CoordinateSystem 等）及序列化，见 [09_data_model.md](09_data_model.md)、[07_serialization.md](07_serialization.md)。
- **util**：跨层小工具，无业务、无 Qt。
- **ui**：Qt 主窗口、对话框、Widget；依赖 database，见 [06_ui_framework.md](06_ui_framework.md)。

---

## 2. 与三层架构的对应

| 架构层 | 职责 | 主要目录 |
|--------|------|----------|
| **Project Layer** | 全局配置与原始输入（坐标系、相机、图像、测量） | `database/`（类型）、`ui/`（编辑与展示） |
| **AT Task Layer** | 空三任务与解算（InputSnapshot、优化、输出） | `algorithm/`、`database/`（ATTask 等） |
| **Output Layer** | 按输出坐标系与约定导出结果 | `algorithm/`（导出逻辑）、CLI 与文件 |

---

## 3. 可执行产物

- **InsightAT**：Qt 主程序（链接 ui、database、util 等），带主窗口与项目/任务管理。
- **isat_***：各 CLI 工具（特征提取、匹配、检索、几何、标定等），仅依赖 algorithm/database/util，无 Qt。
- **CameraEstimator**：独立进程的相机内参估计工具。
