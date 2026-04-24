# 04 - 坐标系与旋转标准

这是 InsightAT 最具技术细节的部分，确保了摄影测量解算的严谨性。

## 1. 坐标系统 (Coordinate System)
底层通过 GDAL/PROJ 管理，使用 `CoordinateSystemDescriptor` 描述：

### 支持类型
- **EPSG**: 直接使用 EPSG 代码（如 4326, 3857）。
- **WKT**: OpenGIS 标准的字符串描述。
- **ENU (East-North-Up)**: 以特定经纬度点为原点的局部东北天坐标系。
- **Local**: 抽象的局部坐标系（无地球物理关联）。

### 高程基准
支持椭球高和海平面高（需通过 `reference_point` 定义）。

## 2. 旋转表示法 (Rotation Representation)

为了消除跨学科合作中的歧义，InsightAT 严格区分了两种旋转体系：

### 2.1 Omega-Phi-Kappa (ω, φ, κ)
- **领域**：经典摄影测量。
- **定义**：Z-Y-X **外旋** (Extrinsic)。
- **适用点**：航空摄影测量、共线方程。

### 2.2 Yaw-Pitch-Roll (偏航/俯仰/翻滚)
- **领域**：无人机、导航、机器人。
- **定义**：Z-Y'-X'' **内旋** (Intrinsic)。
- **坐标框架**：通常基于 NED（北东地）或 ENU。

## 3. 内部标准化
- **角单位**：统一使用弧度 (Radians) 进行数学计算，UI 层转为度 (Degrees)。
- **工具库**：`rotation_utils.h` 提供这两种约定与四元数 (Quaternions) / 旋转矩阵 (Matrix3d) 之间的显性相互转换。
- **配置一致性**：`CoordinateSystemDescriptor::rotation_convention` 记录了当前数据的约定，转换时由框架自动处理，避免硬编码错误。
