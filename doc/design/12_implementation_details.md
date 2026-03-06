# 07 - 实现细节与开发规范

本节总结了开发过程中的关键实现结论和应当遵循的代码规范。

## 1. 开发规范

### 1.1 依赖解耦规则 (Critical)
为了保证算法的可移植性和可测试性，软件遵循严格的层级依赖限制：
- **`src/algorithm/`**：**禁止包含任何 Qt 头文件或链接 Qt 库**。所有字符串处理应使用 `std::string`，集合使用 STL 容器（如 `std::vector`），数学计算使用 Eigen。**算法层不依赖 `src/database/`**：相机内参、畸变等由算法侧最小结构描述（如 `insight::camera::Intrinsics`），仅包含 fx, fy, cx, cy 与 5 个畸变参数；由调用边界（CLI 从 JSON 加载、或 UI 从 database 导出）填充后传入算法，database 只负责导出算法所需的 JSON 或由工具层做一次转换。
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
- 畸变参数序列采用标量存储：`k1, k2, k3, p1, p2`（与 Bentley ContextCapture 透视模型 5 参数对齐）。
- 坐标归一化是在去畸变前进行的标准流程。
- **算法层**：使用 `insight::camera::Intrinsics`（仅 fx, fy, cx, cy, k1–p2），定义于 `src/algorithm/modules/camera/camera_types.h`，算法不依赖 database；resection、去畸变、reject_outliers 等均接受该类型。**database 层**：`CameraModel` 存完整元数据与序列化；需调用算法时由工具/UI 从 JSON 或 `CameraModel` 填充 `Intrinsics` 后传入。

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

### 2.4 SfM 内部 ID 重编码与向量化相机表示

#### 背景

图像 ID（`image_id`）和相机 ID（`camera_id`）由项目数据库分配，是稀疏的正整数（例如 1001, 1005, 2008…）。若直接将这些 ID 用作数组下标，会导致大量空洞（`image_degree_` 等 vector 按 max_id 分配），内存浪费，且无法被 GPU 直接消费。

#### 设计方案

在 **AT 任务开始、导出参数时建立 `IdMapping`，算法结束后用映射还原原始 ID**，中间全程使用密集的内部索引 `[0, N)`。

```
原始 image_id: 1001, 1005, 2008, 2012   (稀疏)
内部 index  :    0,    1,    2,    3   (密集)

原始 camera_id: 1, 3           (稀疏)
内部 index  :   0, 1           (密集)
```

**`IdMapping`**（`src/algorithm/modules/sfm/id_mapping.h`）：
- `original_image_ids[i]`：内部下标 i → 原始 image_id
- `original_camera_ids[j]`：内部下标 j → 原始 camera_id
- `image_to_camera[i]`：内部图像下标 → 内部相机下标
- `original_to_internal_image`/`original_to_internal_camera`：反查表（仅在"边界加载"时使用）

在 `isat_intrinsics.h` 中提供 `build_id_mapping_from_image_list(path)` 一次性构建映射，顺序即 image list JSON 数组顺序。

#### MultiCameraSetup 向量化

`incremental_sfm_engine.h` 中的 `MultiCameraSetup` 以密集向量存储（去掉 `unordered_map`）：

```cpp
struct MultiCameraSetup {
  std::vector<int>              image_to_camera; // 内部图像索引 -> 内部相机索引
  std::vector<camera::Intrinsics> cameras;       // 内部相机索引 -> 内参+畸变
};
```

好处：
- 内存连续，CPU cache 友好，SIMD 可直接访问
- 后续 GPU BA：直接 `memcpy` 到 device，无需序列化

#### 流程约定

```
构建 IdMapping（build_id_mapping_from_image_list）
       ↓
构建向量化 MultiCameraSetup（image_to_camera + cameras）
       ↓
SfM pipeline（ViewGraph、TrackStore、BA —— 全用内部索引）
       ↓
输出时用 original_image_id(i) 还原为原始 ID
```

**重要约定**：同一 AT 任务内所有工具（isat_geo、isat_match、isat_tracks、isat_incremental_sfm）共用同一份 image list，以保证各阶段内部索引对应关系一致。文件名（`.isat_geo`、`.isat_match` 等）仍使用原始 image_id 拼接，读取时通过 `original_image_id(idx)` 还原路径；SfM 内部对图像的所有引用均为密集索引。

#### 与 GPU BA 的关系

向量化后，`cameras` 数组可直接作为 Ceres/GPU BA 的参数块数组（`camera_index_for_image[i]` 给出哪张图用哪个相机参数块），同构于 COLMAP 的 `camera_id → camera params` 表示，便于未来切换 GPU BA 后端（sba、g2o 等）。

#### 相关文件

| 文件 | 职责 |
|---|---|
| `src/algorithm/modules/sfm/id_mapping.h` | `IdMapping` 结构定义 |
| `src/algorithm/tools/isat_intrinsics.h` | `build_id_mapping_from_image_list()` |
| `src/algorithm/modules/sfm/incremental_sfm_engine.h` | 向量化 `MultiCameraSetup` |
| `src/algorithm/modules/sfm/view_graph_loader.cpp` | 加载时转换 pair ID 为内部索引 |
| `src/algorithm/modules/sfm/incremental_sfm.cpp` | `run_initial_pair_loop` 支持 id_mapping |
| `src/algorithm/tools/isat_incremental_sfm.cpp` | CLI 层构建映射、输出时还原 |

## 4. 待办与扩展 (Future Outlook)
- **版本 2.0 计划**：考虑引入 SQLite 作为数据后端的选项，以支持极大规模（百万级）影像匹配。
- **标准化输出**：实现向 COLMAP、Agisoft XML、以及标准 POS 格式的导出器。
