# InsightAT

## 📋 概述

**InsightAT** 是一个专业的摄影测量空三（Aerial Triangulation）处理软件。本文件包括设计原则、架构、系统架构、文件格式规范、CLI 工具、数据模型和开发规范。

---

## 📌 项目状态（Project Status）

- **当前阶段**：**积极开发中**，尚未达到生产就绪（not production-ready）。
- **设计依据**：整体思路与阶段划分以设计文档为准：
  - [设计文档索引](doc/design/index.md)（架构、数据模型、坐标系、序列化、UI、AT 工具包）
  - [Parallel Hybrid SfM 架构设计](doc/design/parallel_hybrid_sf_m_design_philosophy.md)（粗到细两层、拓扑优先、Pose Graph 融合）
- **已实现**：CLI 工具链（特征提取 / 匹配 / 检索 / 几何 / 两视图 / tracks / 标定等）、IDC 数据格式、database 类型与序列化、Qt 主窗口与项目/任务/坐标系配置。详见 [CHANGELOG.md](CHANGELOG.md) 中「已实现」与「进行中/计划中」。
- **可能变动**：API、IDC 细节和项目文件格式在开发过程中可能调整，请以仓库与 CHANGELOG 为准。
- **反馈与贡献**：欢迎试用、提 Issue 和 PR；用于生产前请自行评估稳定性与兼容性。

## 🎯 核心设计原则

### 1. 全GPU加速
- **所有核心算法都在GPU上运行**：特征提取、特征匹配、图像检索、几何计算， SfM
- **目标**：快速、鲁棒、高精度开源空三软件，可处理大规模航拍数据

### 2. CLI-First架构
- **所有算法都提供独立的命令行工具**：可单独运行，无需UI
- **无Qt依赖**：算法层（`src/algorithm/`）严禁依赖Qt
- **头优先设计**：优先保证命令行工具的完整性和易用性

### 2.1 CLI 输出与交互规范（必读）

- **CLI I/O 规范**：[doc/design/CLI_IO_CONVENTIONS.md](doc/design/CLI_IO_CONVENTIONS.md)
  - 约定 **stdout 只输出机器可读结果**、stderr 输出日志/进度
  - 约定统一的 **`ISAT_EVENT <single-line-json>`** 输出格式，便于脚本与管道使用
  - **日志级别**：所有 isat_* 工具统一支持 `--log-level=error|warn|info|debug` 及 `-v`/`-q`，详见该文档第 4 节「日志级别」

### 3. 基于文件的交互
- **结构化文件格式**：类似GLB容器格式，包含：
  ```
  [文件头]：格式标识、版本号
  [JSON描述]：元数据、坐标系信息、任务配置
  [二进制体]：特征数据、匹配结果、位姿等
  ```
- **自描述性**：每个文件都包含完整的元数据和数据结构定义
- **版本控制**：Ui使用Cereal序列化库，支持向后兼容

### 4. UI与算法完全分离
- **三层架构**：
  - **Project Layer**：全局配置和原始输入数据（无Qt依赖）
  - **AT Task Layer**：空三任务和算法执行（无Qt依赖）
  - **UI Layer**：Qt界面，仅负责展示和用户交互
- **算法可独立运行**：可在headless服务器、Docker容器中运行

### 5. 分布式能力
- **Docker支持**：每个CLI工具都可以容器化
- **任务列表驱动**：支持JSON格式的任务描述文件
- **云原生**：适合在Kubernetes、AWS Batch等环境中运行

## 🏗️ 系统架构

### 三层架构模型

```
┌─────────────────────────────────────────────────────┐
│                    PROJECT LAYER                    │
│    (全局配置 & 原始输入数据 - 无Qt依赖)               │
├─────────────────────────────────────────────────────┤
│  • ProjectInformation: 项目基本信息                 │
│  • input_coordinate_system: 全局输入坐标系定义       │
│  • Camera[i]: 相机参数库                            │
│  • Image[j]: 图像数据（含InputPose）                │
│  • Measurements: GNSS/IMU/GCP/SLAM测量数据          │
└─────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────┐
│                    AT TASK LAYER                    │
│      (空三任务 - 树形嵌套，每层冻结输入)              │
├─────────────────────────────────────────────────────┤
│  ATTask[k]:                                          │
│  ├─ InputSnapshot: 冻结的原始输入数据                │
│  ├─ Initialization: 前任务结果（可选）              │
│  ├─ Processing: BA优化（GPU加速）                   │
│  ├─ output_coordinate_system: 输出坐标系配置         │
│  └─ child_tasks[]: 嵌套的子任务                     │
└─────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────┐
│                    OUTPUT LAYER                     │
│      (按任务配置的输出坐标系导出结果)                 │
└─────────────────────────────────────────────────────┘
```

### SfM 算法架构：两层设计

具体设计思想：[SfM 架构](doc/design/parallel_hybrid_sf_m_design_philosophy.md)

**第一层：粗 SfM 结果（Cluster + Merge + Global BA）**

- 目标：得到**粗的、全局一致的** SfM 结果，为第二层提供初值。
- 流程：Cluster 内并行重建 → Merge（融合/对齐）→ **Global BA**。
- 已实现/进行中：
  - [X] 分辨率：可降分辨率（如不超过 4000×4000）以控制规模
  - [X] 特征提取：GPU（SIFT、SuperPoint），特征数量可限（如 ≤10000）
  - [X] 图像检索：VLAD、词汇树、GNSS、Sequence
  - [X] 特征匹配与几何验证：GPU 匹配、RANSAC（F/E）
  - [ ] 两视图几何重建、层级式增量、Merge、**第一层 Global BA**（进行中）
  - 畸变：简化模型（如仅径向畸变）

**第二层：高精度 SfM（规划中）**

- 目标：在第一层粗结果基础上达到高精度。
- 计划：**原始分辨率**特征、**位姿引导**的匹配、**高精度相对位姿**、**Global BA 策略**；完整畸变模型。


## 📁 文件格式规范: IDC (Insight Data Container) 格式


**设计目标**
- 兼顾性能（二进制）与可读性（JSON）
- 自描述：无需外部文档即可理解数据结构
- 版本化：支持未来格式演进
- AI 友好：Agent 可直接解析 JSON 头了解数据内容
- **8 字节对齐**：Binary Payload 对齐优化，支持高性能 SIMD、GPU 上传和跨平台兼容

**文件结构**

```
┌─────────────────────────────────────┐
│ Magic Header: "ISAT" (4 bytes)     │  ← 魔数标识
├─────────────────────────────────────┤
│ Format Version: uint32_t (4 bytes) │  ← 格式版本号（当前版本：1）
├─────────────────────────────────────┤
│ JSON Size: uint64_t (8 bytes)      │  ← JSON 描述符长度
├─────────────────────────────────────┤
│                                     │
│ JSON Descriptor                     │  ← 自描述元数据
│ (UTF-8, Variable Length)            │
│                                     │
├─────────────────────────────────────┤
│ Padding (0-7 bytes)                 │  ← **对齐填充**：保证下一部分 8 字节对齐
├─────────────────────────────────────┤
│                                     │
│ Binary Payload                      │  ← 原始数据块（8 字节对齐起始）
│ (Little-Endian, Variable Length)    │
│                                     │
└─────────────────────────────────────┘
```

### 主要文件类型

1. **特征文件** (`.isat_feat`)
   - 存储图像特征点、描述符、尺度等信息
   - 支持SIFT、SuperPoint等多种特征类型

2. **匹配文件** (`.isat_match`)
   - 存储图像对之间的匹配关系
   - 包含内点索引、基础矩阵等信息

3. **位姿文件** (`.isat_pose`)
   - 存储相机位姿、坐标系信息
   - 支持多种旋转约定（OmegaPhiKappa/YawPitchRoll）

4. **任务配置文件** (`.isat_task`)
   - 描述处理任务的JSON文件
   - 包含输入输出路径、算法参数等

## 🛠️ 工具链（CLI工具）

### 核心工具概览

| 工具 | 功能 | GPU加速 | 输出格式 |
|------|------|---------|----------|
| `isat_extract` | 特征提取（SIFT/SuperPoint） | ✅ | `.isat_feat` |
| `isat_retrieve` | 图像检索（GNSS/VLAD/BoW/Sequence） | ⚠️（CPU为主） | `.json` |
| `isat_match` | 特征匹配 | ✅ | `.isat_match` |
| `isat_geo` | 几何验证和位姿估计 | ✅ | `.isat_pose` |
| `isat_train_vlad` | 训练VLAD编码器 | ⚠️（CPU为主）| `.vlad.bin` |
| `isat_train_vocab` | 训练词汇树 | ⚠️（CPU为主） | `.dbow3` |

### 详细工具说明

具体工具可直接调用CLI -h来查看

## 📊 数据模型

### 核心数据结构

1. **CoordinateSystemDescriptor** - 坐标系描述
   - 类型：EPSG/WKT/ENU/Local
   - 旋转约定：OmegaPhiKappa（摄影测量）/YawPitchRoll（航空学）
   - 原点/参考点（可选）

2. **Measurement** - 统一测量框架
   - 类型：GNSS/IMU/GCP/SLAM
   - 所有测量都可以包含协方差（不确定度）
   - 在BA中作为先验约束使用

3. **InputPose** - 输入位姿
   - 原始GNSS/IMU测量值
   - 按项目的输入坐标系解释

4. **ATTask** - 空三任务
   - InputSnapshot：冻结的输入数据
   - Initialization：前任务结果（可选）
   - 树形嵌套结构

### 序列化机制

- **使用Cereal库**：二进制序列化，支持版本控制
- **向后兼容**：自动处理旧版本文件
- **可选字段**：使用`std::optional`，空字段不占用序列化空间

## 🚀 开发规范

### 代码组织结构

```
src/
├── algorithm/           # 核心算法（无 Qt 依赖）
│   ├── modules/        # 算法模块
│   │   ├── extraction/ # 特征提取（SIFT, SuperPoint）
│   │   ├── retrieval/  # 图像检索（VLAD, 词汇树）
│   │   ├── matching/   # 特征匹配
│   │   └── geometry/   # 几何计算
│   ├── io/exif/        # EXIF 解析（相机内参估计用）
│   └── tools/          # CLI 工具（isat_*）
├── database/           # 数据结构和序列化（Cereal）
├── util/               # 轻量公共工具（string_utils, numeric）
└── ui/                 # Qt 界面（与算法分离）
    └── utils/          # UI 用工具（Coordinates, QStringConvert）
```
### 文档

- **编码规范**：[doc/design/CODING_STYLE.md](doc/design/CODING_STYLE.md)
  - 命名规范、注释格式、头文件组织、CLI 工具约定、GPU 代码规范、序列化规范等
  - **CLI 工具实现文件**（`src/algorithm/tools/isat_*.cpp`）须在**文件开头**写规范的文件头注释：文件名、工具标题、功能概述、Pipeline/输出说明、Usage 示例，详见该文档 §1.3
  - 代码格式由项目根目录的 [.clang-format](.clang-format) 自动执行（`clang-format -i <file>`）
- **CLI I/O 规范**：[doc/design/CLI_IO_CONVENTIONS.md](doc/design/CLI_IO_CONVENTIONS.md)

```
doc/
├── design/
│   ├── CODING_STYLE.md       # 代码编写规范（命名/注释/格式/设计）
│   └── CLI_IO_CONVENTIONS.md # CLI 输入输出约定
└── dev-notes/           # 开发过程记录（过程稿，以 design/ 与代码为准）
```
### 添加新算法模块

1. **在`src/algorithm/modules/`下创建新目录**
2. **实现GPU加速的核心算法**
3. **提供CLI工具接口**
4. **定义结构化文件格式**
5. **编写单元测试**

### 开发原则

1. **GPU优先**：新算法必须支持GPU加速
2. **CLI接口**：每个算法模块都必须提供独立的命令行工具
3. **文件格式**：使用标准的ISAT文件格式
4. **无Qt依赖**：算法层严禁使用Qt
5. **版本控制**：数据格式必须支持版本升级


