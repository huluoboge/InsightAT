# InsightAT 

## 📋 概述

**InsightAT** 是一个专业的摄影测量空三（Aerial Triangulation）处理软件。本文件包括设计原则、架构、系统架构、文件格式规范、CLIG工具、数据模型、和开发规范。

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

### SfM算法架构：粗到细两层设计

具体设计思想： [SfM架构](./parallel_hybrid_sf_m_design_philosophy.md)
第一层：粗粒度初始位姿估计
  - 核心思想：提供相对位姿准确， 鲁棒性强，速度快速的整体粗粒度重建
  - [X] 分辨率：不超过4000×4000
  - 特征提取：
   - [X] 快速GPU特征检测（SIFT、SuperPoint）特征数量不超过10000
  - 图像检索：
      - [x] VLAD、词汇树、GNSS，Sequence
      - [ ] 如果有GNSS+姿态那么根据世界基础地形数据，按照椭球高计算航高，然后根据投影来精确获得匹配图像
  - [X] 特征匹配：GPU加速匹配
  - [X] 几何验证：RANSAC + 基础矩阵/本质矩阵，GPU RANSAC  
  - [ ] 两视图几何重建： 通过两视图几何，估计F, 估计E， 前方交会，生成3D点，估计内参，BA优化 
  - 已知内参，准确使用E来估计inlier，进行两视图几何重建，筛选三维点> 50的pair
  - 层级式SfM： 所有两视图重建成功的可以直接尝试求解sRt，剩下的不断进行增量重建。
  - 优化：使用PBA（Parallel Bundle Adjustment）
  - 畸变模型：简化模型（仅径向畸变）
  

第二层：细粒度优化
  -  分辨率：原始图像分辨率
  -  特征提取和匹配： 利用初始位姿/BA引导，子像素稀疏特征、光流进行匹配及优化（最小二乘匹配）
  - 优化器：GPU cuda实现+ceres cuda实现
  - 目标：高精度，稀疏特征匹配+全局BA。
  - 畸变： 完整畸变模型


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
├── algorithm/           # 核心算法（无Qt依赖）
│   ├── modules/        # 算法模块
│   │   ├── extraction/ # 特征提取（SIFT, SuperPoint）
│   │   ├── retrieval/  # 图像检索（VLAD, 词汇树）
│   │   ├── matching/   # 特征匹配
│   │   └── geometry/   # 几何计算
│   └── tools/          # CLI工具
│       ├── isat_extract.cpp
│       ├── isat_retrieve.cpp
│       ├── isat_match.cpp
│       └── isat_geo.cpp
├── database/           # 数据结构和序列化
├── Common/            # 公共工具函数
└── ui/                # Qt界面（与算法分离）
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
└── implemention/
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


