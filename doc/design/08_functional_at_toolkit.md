# 08. Functional AT Toolkit Architecture

> **文档版本**: 1.0  
> **创建日期**: 2026-02-11  
> **目标**: 定义 InsightAT 的函数式算法工具包架构——一个独立于项目数据库、支持分布式计算、基于 CLI 和自描述数据的摄影测量处理流水线。

---

## 1. 核心设计理念

### 1.1 架构原则

**算法独立性 (Algorithm Independence)**
- 所有算法模块独立于 `Project` 和 `ProjectDocument`，仅依赖纯数据结构
- 算法间通过**自描述文件**传递数据，而非共享内存或数据库
- 每个算法都是**无状态**的纯函数，相同输入保证相同输出

**CLI-First 设计 (CLI-First Design)**
- 所有算法都提供**命令行接口**，支持独立运行和测试
- 每个工具必须实现 `-h` 帮助信息，描述输入输出格式
- 标准错误流（`stderr`）输出日志和进度，标准输出流（`stdout`）可用于数据重定向

**分布式就绪 (Distributed-Ready)**
- 基于**文件系统**的数据交换，天然支持 Docker 容器化
- 工具间通过文件路径传递数据，支持本地和网络存储
- 无需中央数据库或状态管理器

**低层级控制 (Low-Level Control)**
- 优先使用 **Eigen3**, **Ceres Solver**, **RansacLib** 等底层库
- 避免依赖 OpenCV 的高级封装（如 `solvePnP`），保持算法透明度
- 所有数学运算可追溯、可调试、可替换

**策略模式适配 (Strategy Pattern)**
- 同一步骤支持多种算法实现（如特征提取：SIFT/ORB/AKAZE）
- 通过适配器接口实现算法可互换，无需修改上层流程
- 支持多种匹配策略（GPS 引导、IMU 约束、穷举搜索等）

---

## 2. 数据格式标准

### 2.1 IDC (Insight Data Container) 格式

**设计目标**
- 兼顾性能（二进制）与可读性（JSON）
- 自描述：无需外部文档即可理解数据结构
- 版本化：支持未来格式演进
- AI 友好：Agent 可直接解析 JSON 头了解数据内容

**文件结构**

```
┌─────────────────────────────────────┐
│ Magic Header: "ISAT" (4 bytes)     │  ← 魔数标识
├─────────────────────────────────────┤
│ Format Version: uint32_t (4 bytes) │  ← 格式版本号
├─────────────────────────────────────┤
│ JSON Size: uint64_t (8 bytes)      │  ← JSON 描述符长度
├─────────────────────────────────────┤
│                                     │
│ JSON Descriptor                     │  ← 自描述元数据
│ (UTF-8, Variable Length)            │
│                                     │
├─────────────────────────────────────┤
│                                     │
│ Binary Payload                      │  ← 原始数据块
│ (Little-Endian, Variable Length)    │
│                                     │
└─────────────────────────────────────┘
```

**JSON 描述符示例**

```json
{
  "schema_version": "1.0",
  "container_id": "550e8400-e29b-41d4-a716-446655440000",
  "task_type": "feature_extraction",
  "algorithm": {
    "name": "SIFT_OpenCV",
    "version": "4.8.0",
    "parameters": {
      "nfeatures": 8000,
      "contrastThreshold": 0.04
    }
  },
  "blobs": [
    {
      "name": "keypoints",
      "dtype": "float32",
      "shape": [8000, 2],
      "offset": 0,
      "size": 64000
    },
    {
      "name": "descriptors",
      "dtype": "uint8",
      "shape": [8000, 128],
      "offset": 64000,
      "size": 1024000
    }
  ],
  "metadata": {
    "image_path": "/data/images/IMG_0001.jpg",
    "timestamp": "2026-02-11T10:30:00Z",
    "execution_time_ms": 1250
  }
}
```

**数据类型规范**
- 所有二进制数据强制 **Little-Endian** 编码
- 支持的基本类型：`uint8`, `uint16`, `uint32`, `uint64`, `float32`, `float64`
- 复杂类型通过 `shape` 字段描述（例如 `[N, M]` 表示 N×M 矩阵）

### 2.2 简单数据交换格式

对于轻量级数据（如配置参数、任务描述），直接使用 **纯 JSON 文件**。

**示例：相机内参文件**
```json
{
  "camera_id": 1,
  "model": "PINHOLE",
  "width": 3840,
  "height": 2160,
  "fx": 3600.0,
  "fy": 3600.0,
  "cx": 1920.0,
  "cy": 1080.0,
  "distortion": {
    "model": "RADIAL_TANGENTIAL",
    "k1": -0.12,
    "k2": 0.05,
    "p1": 0.001,
    "p2": -0.002
  }
}
```

---

## 3. 目录结构设计

```
src/algorithm/
├── base/                       # 头文件库：几何、数学原语
│   ├── geometry/
│   │   ├── camera_models.h     # 相机投影模型
│   │   ├── epipolar.h          # 对极几何
│   │   ├── triangulation.h     # 三角测量
│   │   └── pnp.h               # PnP 求解器接口
│   ├── math/
│   │   ├── rotation.h          # 旋转表示转换
│   │   ├── numeric.h           # 数值优化工具
│   │   └── robust.h            # RANSAC 框架
│   └── types.h                 # 基础类型定义（基于 Eigen）
│
├── modules/                    # 无状态算法核心
│   ├── extraction/
│   │   ├── sift_extractor.h
│   │   ├── orb_extractor.h
│   │   └── feature_interface.h # 特征提取抽象接口
│   ├── matching/
│   │   ├── brute_force_matcher.h
│   │   ├── flann_matcher.h
│   │   ├── geometric_filter.h  # F/E 矩阵验证
│   │   └── gps_guided_matcher.h # GPS 约束匹配
│   ├── retrieval/
│   │   ├── vocabulary_tree.h   # 词袋检索
│   │   └── gps_indexer.h       # 空间索引
│   ├── geometry/
│   │   ├── two_view_solver.h   # 双视图几何
│   │   ├── resection.h         # 空间后方交会
│   │   └── intersection.h      # 前方交会
│   └── optimization/
│       ├── bundle_adjuster.h   # BA 优化器
│       ├── pose_graph.h        # 位姿图优化
│       └── ceres_cost_functions.h # Ceres 代价函数
│
├── io/                         # I/O 与序列化
│   ├── idc_reader.h            # IDC 读取器
│   ├── idc_writer.h            # IDC 写入器
│   ├── json_utils.h            # JSON 辅助工具
│   └── endian.h                # 字节序转换
│
└── tools/                      # CLI 工具集
    ├── isat_extract.cpp        # 特征提取
    ├── isat_retrieve.cpp       # 图像检索
    ├── isat_match.cpp          # 特征匹配
    ├── isat_geometry.cpp       # 两视图几何
    ├── isat_mapper.cpp         # 增量/全局重建
    └── isat_adjust.cpp         # 光束法平差
```

---

## 4. 算法流水线

### 4.1 完整处理流程

```
┌──────────────┐
│  Image List  │ (JSON: 图像路径、GNSS/IMU 数据)
└──────┬───────┘
       │
       ▼
┌──────────────────┐
│ isat_extract     │ → .isat_feat (每张图一个 IDC 文件)
│ (特征提取)       │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ isat_retrieve    │ → .isat_pairs.json (候选匹配对列表)
│ (图像检索)       │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ isat_match       │ → .isat_match (每对图像一个 IDC 文件)
│ (特征匹配)       │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ isat_geometry    │ → .isat_pairs_verified.json (几何验证后的对)
│ (两视图几何)     │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ isat_mapper      │ → .isat_sparse (稀疏点云 + 相机位姿)
│ (增量/全局重建)  │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ isat_adjust      │ → .isat_sparse_refined (优化后的稀疏模型)
│ (光束法平差)     │
└──────────────────┘
```

### 4.2 混合重建策略

**阶段 A: 低分辨率增量重建**
- 使用降采样图像进行快速增量 SfM
- 目标：获得稳定的初始相机位姿
- 策略：保守的特征匹配 + 严格的几何验证

**阶段 B: 高精度优化**
- 使用原始分辨率图像重新提取特征
- 基于阶段 A 的位姿进行引导式匹配
- 剔除粗差，执行全局 Bundle Adjustment

**阶段 C: 分块优化（可选）**
- 对大规模数据集进行空间分块
- 每块独立优化后进行边界对齐
- 最终执行全局位姿图优化

---

## 5. CLI 工具接口标准

### 5.1 通用规范

**命令行格式**
```bash
isat_<command> [OPTIONS] <input> <output>
```

**必须支持的选项**
- `-h, --help`: 打印详细帮助信息
- `-v, --verbose`: 详细日志输出到 stderr
- `-q, --quiet`: 仅输出错误信息
- `--version`: 打印工具版本

**日志与进度**
- 所有日志输出到 **stderr**（使用 glog）
- 进度信息格式：`PROGRESS: <float>` (0.0 ~ 1.0)
- 标准输出流保留用于数据管道

### 5.2 示例：特征提取工具

**命令行接口**
```bash
isat_extract -h
```

**输出帮助信息**
```
InsightAT Feature Extraction Tool v1.0

USAGE:
  isat_extract [OPTIONS] -i <image_list.json> -o <output_dir>

OPTIONS:
  -h, --help              Print this help message
  -i, --input <file>      Input image list (JSON format)
  -o, --output <dir>      Output directory for .isat_feat files
  -a, --algorithm <name>  Feature algorithm: sift|orb|akaze (default: sift)
  -n, --nfeatures <int>   Maximum features per image (default: 8000)
  --gpu                   Enable GPU acceleration (if available)

INPUT FORMAT (JSON):
{
  "images": [
    {"path": "/data/IMG_0001.jpg", "camera_id": 1},
    {"path": "/data/IMG_0002.jpg", "camera_id": 1}
  ]
}

OUTPUT FORMAT:
  One .isat_feat file per image (IDC format with JSON header)

EXAMPLE:
  isat_extract -i images.json -o features/ -a sift -n 10000
```

---

## 6. UI 集成策略

### 6.1 ProjectDocument 作为编排器

`ProjectDocument` 不再直接调用算法，而是：
1. 将 `ATTask` 的 `InputSnapshot` 导出为 JSON 文件
2. 构建 CLI 命令并通过 `QProcess` 启动工具
3. 监听 `stderr` 获取进度和日志
4. 读取生成的 IDC 文件，将结果导入 `ATTask` 的 `results` 字段

### 6.2 工作流示例代码

```cpp
// 伪代码：执行特征提取
void ATTaskExecutor::runFeatureExtraction(const ATTask& task) {
    // 1. 导出图像列表
    QString inputJson = exportImageList(task);
    QString outputDir = task.working_directory + "/features";
    
    // 2. 构建命令
    QStringList args;
    args << "-i" << inputJson
         << "-o" << outputDir
         << "-a" << "sift"
         << "-n" << "8000";
    
    // 3. 启动进程
    QProcess process;
    process.setProgram("isat_extract");
    process.setArguments(args);
    
    connect(&process, &QProcess::readyReadStandardError, [&]() {
        QString log = process.readAllStandardError();
        parseProgressLog(log); // 解析进度
        emit progressUpdated(m_currentProgress);
    });
    
    process.start();
    process.waitForFinished(-1);
    
    // 4. 读取结果
    loadFeaturesFromIDC(outputDir);
}
```

---

## 7. 外部库集成

### 7.1 推荐依赖

| 库名称 | 用途 | 集成方式 |
|--------|------|----------|
| **Eigen3** | 线性代数 | Header-only |
| **Ceres Solver** | 非线性优化 | CMake find_package |
| **RansacLib** | 鲁棒估计 | 子模块或 Header-only |
| **OpenCV** | 图像 I/O 和特征提取 | 仅用于底层算子 |
| **FLANN** | 快速最近邻搜索 | CMake find_package |
| **nlohmann/json** | JSON 解析（可选） | Cereal 已足够 |

### 7.2 避免的依赖

- ❌ Qt (算法层完全禁用)
- ❌ OpenCV 高级 API (`solvePnP`, `recoverPose` 等)
- ❌ PCL (点云库，过重)

---

## 8. 实施路线图

### Phase 1: 基础设施 (2 周)
- [ ] 实现 `IDCWriter` 和 `IDCReader`
- [ ] 创建 `src/algorithm/base` 头文件库
- [ ] 编写第一个 CLI 工具：`isat_extract`

### Phase 2: 核心模块 (4 周)
- [ ] 特征提取模块（SIFT/ORB 适配器）
- [ ] 特征匹配模块（暴力/FLANN）
- [ ] 两视图几何验证

### Phase 3: 重建引擎 (6 周)
- [ ] 增量 SfM 实现
- [ ] Ceres BA 集成
- [ ] 混合重建策略实现

### Phase 4: UI 集成 (2 周)
- [ ] `ProjectDocument` 编排器
- [ ] 进度监控与错误处理
- [ ] 3D 可视化集成

---

## 9. 未来扩展

### 9.1 分布式计算
- Docker 镜像打包各个工具
- Kubernetes 编排大规模任务
- 云存储（S3/MinIO）数据交换

### 9.2 算法演进
- 集成 GLOMAP 进行全局优化
- 支持 AliceVision 的特征提取器
- 深度学习特征（SuperPoint/SuperGlue）

### 9.3 策略优化
- 基于场景类型自动选择算法组合
- 自适应参数调优（网格搜索）
- 持续学习：从历史任务中学习最佳策略

---

## 10. 总结

本设计文档定义了一个**高度模块化、分布式就绪、自描述**的摄影测量算法工具包。核心要点：

1. **算法独立性**：完全解耦于项目数据库
2. **CLI-First**：所有功能均可独立运行和测试
3. **自描述数据**：IDC 格式兼顾性能与可读性
4. **低层级控制**：使用 Eigen/Ceres/RansacLib
5. **策略模式**：支持多种算法实现和自动选择

此架构为未来的云原生、AI 驱动的摄影测量工作流奠定了坚实基础。
