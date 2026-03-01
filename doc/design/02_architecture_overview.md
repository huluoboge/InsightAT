# 02 - 系统架构

InsightAT 采用了清晰的层级架构，将原始数据与计算过程解耦。

## 1. 三层架构模型

### 第一层：Project Layer (全局配置与原始输入)
- **职责**：管理项目元数据、输入坐标系定义、相机库、以及最原始的测量数据（GNSS/IMU/GCP）。
- **技术约束**：**严禁依赖 Qt**。仅使用标准 C++、Eigen 和 Cereal。

### 第二层：AT Task Layer (空三任务层)
- **职责**：执行具体的空三解算。
- **机制**：
    - **Input Snapshot**：在创建任务时，系统会抓取 Project 的当前状态（测量值）并冻结。
    - **树形结构**：Task 支持嵌套（子任务）。子任务可以继承父任务的输出位姿作为初值，但依然引用同一份原始 GNSS 快照。
    - **隔离性**：BA 优化过程只修改 Task 内部的位姿副本，不干扰原始 Project。
- **技术约束**：核心算法逻辑位于 `src/algorithm/`，**严禁依赖 Qt**。确保算法可以脱离界面在 headless 环境下运行。

### 第三层：Output Layer (输出层)
- **职责**：根据选定的输出坐标系和旋转约定，将解算结果（DBPose）导出为特定格式。
- **技术约束**：导出逻辑应保持与 UI 解耦，支持命令行调用。

## 2. 技术栈
- **核心逻辑与算法**：C++ 17, Eigen3, OpenCV (No Qt)
- **UI 框架**：Qt 5.15 (QtWidgets) - 仅限于 `src/ui/`
- **数据库/序列化**：Cereal (Binary format)
- **渲染**：OpenGL
- **坐标变换**：GDAL / PROJ

## 3. 模块划分
```bash
src/
├── database/    # 核心数据结构、序列化逻辑
├── algorithm/   # 空三解算、几何计算、导出工具
├── render/      # OpenGL 3D 查看器实现
├── ui/          # 会话、对话框、小部件
│   ├── dialogs/ # 对话框 (NewProject, ImageGroup)
│   ├── models/  # Qt Model/View 适配
│   └── widgets/ # 复用组件 (CameraModel, CoordinateSystem)
└── Common/      # 全局宏、工具函数
```

## 4. 数据流向
1. **输入阶段**：用户通过 UI 输入坐标系和相机参数，导入图像和测量文件，存入 `Project`。
2. **任务初始化**：用户创建 `ATTask`，系统自动创建 `InputSnapshot`。
3. **计算阶段**：算法模块读取 `InputSnapshot` 进行解算，产生优化的位姿。
4. **结果持久化**：整个 `Project`（包含所有 `ATTask`）被序列化为二进制 `.db` 文件。
