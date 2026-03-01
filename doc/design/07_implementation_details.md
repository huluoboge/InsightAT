# 07 - 实现细节与开发规范

本节总结了开发过程中的关键实现结论和应当遵循的代码规范。

## 1. 开发规范

### 1.1 依赖解耦规则 (Critical)
为了保证算法的可移植性和可测试性，软件遵循严格的层级依赖限制：
- **`src/algorithm/`**：**禁止包含任何 Qt 头文件或链接 Qt 库**。所有字符串处理应使用 `std::string`，集合使用 STL 容器（如 `std::vector`），数学计算使用 Eigen。
- **`src/database/`**：**禁止依赖 Qt**。数据结构必须是纯粹的 C++ 类/结构体，以便在非 GUI 环境下进行序列化和反序列化。
- **`src/ui/`**：允许使用 Qt。负责将 `std::string` 转为 `QString`，将 STL 容器转为 Qt Model，以及调用算法层接口。

### 1.2 命名空间
- 核心逻辑：`namespace insight`
- 数据库结构：`namespace insight::database`
- UI 组件：`namespace insight::ui`
- 渲染引擎：`namespace insight::render`

### 1.2 异常与错误处理
- 底层库（算法、IO）优先使用异常或返回 `std::optional` / `Expected<T>`。
- UI 层必须捕获异常并转为 `QMessageBox` 提示。
- 使用 **Glog** 进行结构化日志记录。

### 1.3 资源管理
- 倾向于通过 `std::unique_ptr` 或 `std::shared_ptr` 管理具有所有权的资源。
- Qt 对象遵循父子生命周期管理，但非 QObject 类型必须手动控权。

## 2. 核心实现方案 (Key Implementations)

### 2.1 坐标系解析 (Spatial Reference)
- 软件维护了一个 `PROJCS_Database.csv` 和 `GEOGCS_Database.csv` 映射。
- `SpatialReferenceTool` 负责在这些本地数据库与 GDAL / PROJ 接口间建立联系。
- 重点：**坐标原点漂移补偿**。对于大范围坐标系（如 UTM），软件内部会计算一个局部原点偏移量，以减小浮点数精度带来的几何噪声。

### 2.2 相机畸变模型
- 支持 **Pinhole**, **Brown-Conrady**, **Fisheye** 等模型。
- 畸变参数序列采用标量存储：`k1, k2, k3, k4, p1, p2, b1, b2`。
- 坐标归一化是在去畸变前进行的标准流程。

### 2.3 任务快照 (Task Snapshot)
- 由于 `Project` 包含大量数据，快照采用 **“影子拷贝 (Shallow-to-Mid Copy)”**。
- 不会拷贝原始二进制图像数据，只拷贝包含元数据和测量值的结构体。
- 所有的 `ATTask` 共享同一套 `ImageGroup` 的引用，但拥有独立的位姿数组。

## 3. 工具化标准 (Tooling Standards)

为了支持 AI Agent 的自动化调用和开发者的调试，所有 `src/algorithm/` 下的独立进程工具必须：
1. **提供 `--help` 选项**：详细说明输入输出格式，方便 AI Agent 自学用法。
2. **支持双模输入**：
   - **JSON 文件/流 模式**：通过 `--json-file` 或 `stdin` 接收结构化 JSON 数据，与内部 IPC 协议对齐。
   - **CSV 模式**：支持通过 `--csv-file` 传入简单的图像路径列表（每行一个路径）。
3. **静默标准**：计算结果输出到 `stdout` (JSON 格式)，所有日志、进度信息及错误提示必须输出到 `stderr`。

## 4. 待办与扩展 (Future Outlook)
- **版本 2.0 计划**：考虑引入 SQLite 作为数据后端的选项，以支持极大规模（百万级）影像匹配。
- **标准化输出**：实现向 COLMAP、Agisoft XML、以及标准 POS 格式的导出器。
