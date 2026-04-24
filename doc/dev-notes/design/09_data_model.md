# 03 - 核心数据模型

InsightAT 的数据结构定义在 `insight::database` 命名空间中。其设计原则是“结构体驱动”，配合 Cereal 进行序列化。

## 1. Project (项目)
项目的根节点，包含所有核心元素。
- `ProjectInformation`: 基本信息（名称、作者、路径）。
- `input_coordinate_system`: 全局输入参考系。
- `std::vector<CameraModel>`: 相机参数库。
- `std::vector<ImageGroup>`: 图像分组。
- `std::vector<ATTask>`: 任务树。

## 2. ImageGroup (图像组)
管理图像与其相机参数的关联，支持两种模式：
- **GroupLevel**: 整个组共享一个相机参数（适用于普通航飞）。
- **ImageLevel**: 每张图像可以有独立的相机参数（适用于多相机系统或自标定）。

## 3. Image (图像)
- `image_id`: 唯一标识。
- `path`: 绝对或相对路径。
- `InputPose`: 原始输入的先验位姿。
- `std::optional<CameraModel>`: 仅在 ImageLevel 模式下有效。

## 4. Measurement (测量数据)
统一的测量框架，所有测量均包含**协方差（Covariance）**。
- `kGNSS`: 位置 (x, y, z)。
- `kIMU`: 姿态、加速度、角速度。
- `kGCP`: 地面控制点 (三维坐标 + 图像观测像素)。
- `kSLAM`: 帧间相对位姿。

## 5. ATTask (空三任务)
- `InputSnapshot`: 冻结的输入数据。包含原始测量的副本，确保 BA 过程中的先验约束不会被后续用户操作修改。
- `Initialization`: 初始值来源（可能是前一个任务的输出）。
- `OutputConfig`: 任务指定的输出坐标系和约定。
- `optimized_poses`: 解算后的精化位姿。
- `child_tasks`: 嵌套的实验/迭代。

## 6. 数据完整性设计
- **KeyType**: 使用统一的 ID 类型（通常为 `uint32_t`）进行关联。
- **Optional**: 广泛使用 `std::optional` 处理不完整数据，序列化时会智能跳过空字段。
