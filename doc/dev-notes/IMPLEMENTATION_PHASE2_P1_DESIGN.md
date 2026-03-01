# Phase 2 - P1 设计文档：DBPose 增强与坐标系标准化

**日期**: 2026-02-08  
**状态**: 🔄 设计文档（准备实现）  
**目标**: 支持用户自定义坐标系，区分输入/输出坐标系，内部统一处理

---

## 📋 核心设计原则

### 用户场景

```
用户输入 (支持多种格式)
    ↓
[DBPose with input_coordinate_system]
    ↓
内部处理 (ECEF/ENU + 四元数，无精度损失)
    ↓
用户输出 (自选目标坐标系)
    ↓
[DBPose with output_coordinate_system]
```

### 关键设计决策

1. **坐标系表示**:
   - ❌ 不用固定的 `UTM`, `ENU` 枚举
   - ✅ 用 **EPSG 码**或 **WKT 字符串**
   - 用户可自定义任意投影（高斯克吕格、Mercator等）

2. **两个坐标系字段**:
   - `input_coordinate_system`: 用户输入数据的坐标系
   - `output_coordinate_system`: 用户期望的输出坐标系
   - 内部自动转换

3. **内部处理**:
   - 统一转到 **ECEF** 或 **ENU**（参考点固定）
   - 旋转统一用 **四元数**（避免万向锁）
   - 坐标系转换时，不改变旋转（使用相同四元数）

4. **精度保证**:
   - 输入/输出转换在需要时进行（Lazy conversion）
   - 内部ECEF/ENU不涉及三角函数丢失精度
   - 四元数保证无奇异值（vs欧拉角的gimbal lock）

---

## 🏗️ 详细设计

### 1. 坐标系元数据结构

用户通过UI选择坐标系，系统自动包含所有必要参数。支持四种坐标系类型：

```cpp
namespace insight {

/**
 * 坐标系描述 - 用户友好的设计
 * 
 * 用户通过UI选择坐标系，无需手动指定椭球或其他参数。
 * 
 * 1. EPSG 码: "EPSG:4326" (WGS84), "EPSG:3857" (Web Mercator)
 *    - 包括地理坐标系、投影坐标系、ECEF等
 *    - 椭球和投影参数自动包含
 *    - 例: EPSG:4978 用于ECEF, EPSG:4326 用于WGS84 LLH
 * 
 * 2. WKT 字符串: "PROJCS[...]" 或 "GEOGCS[...]"
 *    - OGC标准定义
 *    - 椭球和投影参数自动包含
 * 
 * 3. ENU (东北天本地坐标系): "ENU:39.9045,116.4074,50.0"
 *    - 格式自动解析参考点: ENU:lat,lon,alt
 *    - 参考点在WGS84坐标系中
 *    - 用于小范围局部处理（避免大数值）
 * 
 * 4. LOCAL (本地/未知): "My_Local_CRS" 或用户任意定义
 *    - 用户不知道坐标系时使用
 *    - 按用户提供的坐标处理，无自动转换
 *    - 可配置坐标原点以提高精度
 * 
 * 参考: ContextCapture metadata.xml 结构、OSGB瓦片格式
 */
struct CoordinateSystemDescriptor {
    enum Type {
        kEPSG = 0,       // EPSG code (包括投影、地理、ECEF等)
        kWKT = 1,        // OGC WKT 字符串
        kENU = 2,        // 东北天本地坐标系 (需要参考点)
        kLocal = 3       // 本地/未知坐标系
    };
    
    Type type = kEPSG;
    std::string definition;  // "EPSG:4326", WKT string, "ENU:39.9,116.4,50", or local name
    
    /**
     * 坐标原点 - 用于投影坐标系精度优化
     * 
     * 投影坐标系中的坐标值很大（如UTM: 500000m+），直接存储会损失浮点精度。
     * 设置坐标原点（如tile中心）后，存储的是相对坐标，精度更高。
     * 
     * 参考ContextCapture生成的OSGB瓦片的metadata.xml定义。
     * 
     * 示例:
     * - 投影坐标系(UTM)中的点: (234567.89, 5678901.23, 100.5)
     * - 设置原点为: (234500.0, 5678900.0, 100.0)
     * - 存储为相对坐标: (67.89, 1.23, 0.5)
     * - 浮点精度大幅提高
     */
    struct Origin {
        double x = 0.0;  // X坐标原点
        double y = 0.0;  // Y坐标原点
        double z = 0.0;  // Z坐标原点（可选）
        
        bool IsZero() const { 
            return x == 0.0 && y == 0.0 && z == 0.0; 
        }
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
        }
    };
    
    /**
     * 参考点 - 用于ENU坐标系
     * 
     * ENU需要一个参考点（通常是测量区域的中心）来定义局部坐标系。
     * 参考点本身在WGS84地理坐标系中。
     * 
     * 从 "ENU:39.9045,116.4074,50.0" 自动解析，或手动设置。
     */
    struct ReferencePoint {
        double lat = 0.0;  // WGS84 latitude (degrees, -90~90)
        double lon = 0.0;  // WGS84 longitude (degrees, -180~180)
        double alt = 0.0;  // WGS84 ellipsoidal height (meters)
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(lat), CEREAL_NVP(lon), CEREAL_NVP(alt));
        }
    };
    
    std::optional<Origin> origin;              // 坐标原点（投影坐标系）
    std::optional<ReferencePoint> reference;   // 参考点（ENU）
    
    /**
     * 从ENU字符串自动解析参考点。
     * 输入: "ENU:39.9045,116.4074,50.0"
     * 解析为: ReferencePoint{lat=39.9045, lon=116.4074, alt=50.0}
     */
    bool ParseENUReference() {
        if (type != kENU || definition.empty()) return false;
        
        size_t pos = definition.find(':');
        if (pos == std::string::npos) return false;
        
        std::string coords_str = definition.substr(pos + 1);
        std::vector<std::string> parts;
        if (!insight::split(coords_str, ',', parts) || parts.size() != 3) 
            return false;
        
        try {
            ReferencePoint ref;
            ref.lat = std::stod(parts[0]);
            ref.lon = std::stod(parts[1]);
            ref.alt = std::stod(parts[2]);
            
            // 验证范围
            if (ref.lat < -90.0 || ref.lat > 90.0 || 
                ref.lon < -180.0 || ref.lon > 180.0) 
                return false;
            
            reference = ref;
            return true;
        } catch (...) {
            return false;
        }
    }
    
    /**
     * 获取可读的坐标系描述。
     */
    std::string ToString() const {
        std::ostringstream oss;
        if (type == kEPSG) {
            oss << definition << " (EPSG)";
        } else if (type == kWKT) {
            oss << "[WKT] " << definition.substr(0, 50);
            if (definition.length() > 50) oss << "...";
        } else if (type == kENU) {
            oss << definition << " (ENU)";
            if (reference) {
                oss << " ref[" << reference->lat << "," 
                    << reference->lon << "," << reference->alt << "]";
            }
        } else if (type == kLocal) {
            oss << definition << " (Local)";
        }
        if (origin && !origin->IsZero()) {
            oss << " origin[" << origin->x << "," 
                << origin->y << "," << origin->z << "]";
        }
        return oss.str();
    }
    
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(type), CEREAL_NVP(definition));
        ar(CEREAL_NVP(origin), CEREAL_NVP(reference));
    }
};

}  // namespace insight
```

### 2. 增强 DBPose 结构

DBPose现在包含完整的坐标系信息。DBPose中的坐标始终解释为input_coordinate_system中定义的坐标系。

```cpp
/**
 * Enhanced Camera pose with coordinate system metadata.
 * 
 * DESIGN:
 * - Position (x, y, z) 存储在 input_coordinate_system 中
 * - Rotation 统一用四元数（避免gimbal lock）
 * - 可选的欧拉角字段仅用于用户交互
 * - 坐标系信息来自Project级别（所有DBPose共享同一输入坐标系）
 * 
 * WORKFLOW:
 * 1. 用户导入数据
 *    → Project.input_coordinate_system 定义所有pose的输入坐标系
 *    → 读入pose数据 (x, y, z)，按Project.input_coordinate_system解释
 * 2. 内部处理
 *    → 转换到内部ECEF或ENU（可选）
 *    → 存储四元数
 * 3. 输出处理
 *    → ATTask.output_coordinate_system 定义输出格式
 *    → 从Project.input_coordinate_system转到ATTask.output_coordinate_system
 */
struct DBPose {
    KeyType image_id = UndefinedKey;
    
    // ─────────────────────────────────────────
    // POSITION (坐标)
    // ─────────────────────────────────────────
    
    // 当前存储的位置坐标
    // 这些坐标按 Project.input_coordinate_system 解释
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    /**
     * 坐标系描述 (仅在P1初期使用)
     * 
     * P1设计目标: 所有DBPose共享Project级别的input_coordinate_system
     * 但为了向后兼容和灵活性，暂时保留此字段。
     * 
     * 如果此字段非空，优先级高于Project.input_coordinate_system
     * 否则使用Project.input_coordinate_system
     */
    CoordinateSystemDescriptor input_coordinate_system;
    
    // ─────────────────────────────────────────
    // ROTATION (旋转)
    // ─────────────────────────────────────────
    
    // 四元数 (推荐，避免gimbal lock)
    // 格式: (x, y, z, w) 其中w是标量部分
    double quaternion_x = 0.0;
    double quaternion_y = 0.0;
    double quaternion_z = 0.0;
    double quaternion_w = 1.0;  // 初始=单位四元数(无旋转)
    
    // 欧拉角 (可选，仅用于用户交互)
    // 仅当 euler_convention != kNone 时有效
    double omega = 0.0;
    double phi = 0.0;
    double kappa = 0.0;
    
    enum class EulerAngleConvention {
        kNone = 0,               // 不存储/使用欧拉角
        kOmegaPhiKappa = 1,      // ISPRS photogrammetry (Z-Y-X extrinsic)
        kYawPitchRoll = 2        // Aviation (Z-Y-X intrinsic)
    };
    
    EulerAngleConvention euler_convention = EulerAngleConvention::kNone;
    
    int angleUnit = 0;  // 0=degrees, 1=radians (仅用于欧拉角的I/O)
    
    // ─────────────────────────────────────────
    // METADATA
    // ─────────────────────────────────────────
    
    /**
     * 旋转矩阵方向（四元数和欧拉角都遵循此约定）
     * World_to_Camera: p_camera = R · p_world (标准光学习惯)
     * Camera_to_World: p_world = R · p_camera
     */
    enum class RotationMatrixType {
        kWorld_to_Camera = 0,    // 标准：p_camera = R · p_world
        kCamera_to_World = 1     // 逆: p_world = R · p_camera
    };
    RotationMatrixType rotation_type = RotationMatrixType::kWorld_to_Camera;
    
    // GPS/RTK 测量权重（用于bundle adjustment）
    float weight_x = 1.0;
    float weight_y = 1.0;
    float weight_z = 1.0;
    
    // ─────────────────────────────────────────
    // LEGACY/DEPRECATED FIELDS (向后兼容)
    // ─────────────────────────────────────────
    
    /**
     * ENU坐标 (已弃用，为向后兼容保留)
     * 这些字段是冗余的，应该按需计算
     * TODO: P2中移除
     */
    double enuX = 0.0;
    double enuY = 0.0;
    double enuZ = 0.0;
````
    
    // 旋转矩阵方向（四元数本身不区分，但文档需要）
    enum class RotationMatrixType {
        kWorld_to_Camera = 0,    // 标准：p_camera = R · p_world
        kCamera_to_World = 1
    };
    RotationMatrixType rotation_type = RotationMatrixType::kWorld_to_Camera;
    
    // GPS/RTK 测量权重
    float weight_x = 1.0;
    float weight_y = 1.0;
    float weight_z = 1.0;
    
    // ─────────────────────────────────────────
    // HELPERS (不序列化，计算属性)
    // ─────────────────────────────────────────
    
    /**
     * 将当前DBPose从 input_coordinate_system 转换到目标坐标系。
     * 
     * 使用GDAL库进行投影转换：
     * input (x,y,z) 按 input_coordinate_system 解释
     *   ↓ [GDAL 转换] ↓
     * 目标坐标系中的 (x,y,z)
     * 
     * 旋转不变（四元数保持不变，因为是World→Camera的相对旋转）
     */
    DBPose ConvertToCoordinateSystem(
        const CoordinateSystemDescriptor& target_crs) const;
    
    /**
     * 获取内部标准格式：ECEF坐标 + 四元数。
     * 用于内部算法处理（BA、SfM等）。
     */
    struct InternalRepresentation {
        Vec3 ecef_position;        // ECEF 坐标
        Quaternion rotation;       // 四元数
    };
    
    InternalRepresentation GetInternalRepresentation() const;
    
    /**
     * 从内部格式转回DBPose。
     */
    static DBPose FromInternalRepresentation(
        const InternalRepresentation& internal,
        const CoordinateSystemDescriptor& output_crs);
    
    // 序列化
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(image_id));
        ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
        ar(CEREAL_NVP(input_coordinate_system));
        ar(CEREAL_NVP(quaternion_x), CEREAL_NVP(quaternion_y),
           CEREAL_NVP(quaternion_z), CEREAL_NVP(quaternion_w));
        ar(CEREAL_NVP(omega), CEREAL_NVP(phi), CEREAL_NVP(kappa));
        ar(CEREAL_NVP(euler_convention), CEREAL_NVP(angleUnit));
        ar(CEREAL_NVP(rotation_type));
        ar(CEREAL_NVP(weight_x), CEREAL_NVP(weight_y), CEREAL_NVP(weight_z));
    }
};

}  // namespace insight
```

### 3. 用户界面流程

#### 用户输入阶段

```cpp
// 用户在UI中选择输入坐标系
// 选项：
// 1. "EPSG:4326" (WGS84 LLH)
// 2. "EPSG:3857" (Web Mercator) 
// 3. Custom EPSG code
// 4. Custom WKT string
// 5. "Local ENU" (输入reference point)

CoordinateSystemDescriptor input_crs;

// Case 1: WGS84 LLH
input_crs.type = CoordinateSystemDescriptor::kEPSG;
input_crs.definition = "EPSG:4326";
input_crs.enu_reference.reset();  // not needed

// Case 2: Local ENU
input_crs.type = CoordinateSystemDescriptor::kPredefined;
input_crs.definition = "ENU";
input_crs.enu_reference = {lat, lon, height};  // reference point in WGS84

// 读取用户输入的pose
DBPose pose;
pose.x = user_input.lon;  // EPSG:4326 的 lon
pose.y = user_input.lat;  // EPSG:4326 的 lat
pose.z = user_input.height;
pose.input_coordinate_system = input_crs;

// 欧拉角（可选）
pose.omega = user_input.omega;
pose.phi = user_input.phi;
pose.kappa = user_input.kappa;
pose.euler_convention = DBPose::EulerAngleConvention::kOmegaPhiKappa;
pose.angleUnit = 0;  // degrees
```

#### 空三处理阶段

```cpp
// 内部算法需要无精度损失的坐标系
// 使用ECEF和四元数

// 1. 将所有输入pose转到内部格式
std::vector<DBPose::InternalRepresentation> internal_poses;
for (const auto& pose : input_poses) {
    internal_poses.push_back(pose.GetInternalRepresentation());
}

// 2. 进行BA、SfM等算法（使用ECEF + quaternion）
auto result = BundleAdjustment(internal_poses, observations);

// 3. 更新pose（仍在ECEF+四元数中）
for (int i = 0; i < poses.size(); ++i) {
    poses[i] = DBPose::FromInternalRepresentation(
        result.optimized_poses[i],
        poses[i].input_coordinate_system  // 保持原坐标系
    );
}
```

#### 用户输出阶段

```cpp
// 用户选择输出坐标系
CoordinateSystemDescriptor output_crs;
output_crs.type = CoordinateSystemDescriptor::kEPSG;
output_crs.definition = "EPSG:2385";  // 高斯克吕格 (例如)

// 或者选择自己的WKT
output_crs.type = CoordinateSystemDescriptor::kWKT;
output_crs.definition = "PROJCS[...]";  // WKT string

// 转换所有pose
std::vector<DBPose> output_poses;
for (const auto& pose : final_poses) {
    output_poses.push_back(pose.ConvertToCoordinateSystem(output_crs));
}

// 导出
ExportPoses(output_poses, output_file);
```

---

## 🔧 实现任务分解

### Task 1: 添加坐标系描述结构
**文件**: `src/Common/db_types.h`  
**内容**:
- `CoordinateSystemDescriptor` 结构体
- 支持EPSG/WKT/预定义名称
- ENU参考点支持

### Task 2: 增强 DBPose
**文件**: `src/Common/db_types.h`
**内容**:
- 添加 `input_coordinate_system` 字段
- 添加 `quaternion_*` 字段（替代/补充欧拉角）
- 添加 `rotation_type` 字段
- 添加 `euler_convention` 枚举
- 更新序列化（Cereal）

### Task 3: 坐标系转换函数
**文件**: `src/Common/rotation_utils.h/cpp`  
**新增函数**:
```cpp
// ECEF ↔ LLH (WGS84)
Vec3 ECEF_to_LLH(const Vec3& ecef);
Vec3 LLH_to_ECEF(const Vec3& llh);

// ECEF ↔ ENU (给定参考点)
Vec3 ECEF_to_ENU(const Vec3& ecef, const Vec3& ref_llh);
Vec3 ENU_to_ECEF(const Vec3& enu, const Vec3& ref_llh);

// 一般投影转换 (使用GDAL)
Vec3 TransformCoordinate(
    const Vec3& position,
    const CoordinateSystemDescriptor& from,
    const CoordinateSystemDescriptor& to);
```

### Task 4: DBPose 转换方法
**文件**: `src/Common/db_types.cpp`  
**实现**:
- `DBPose::ConvertToCoordinateSystem()`
- `DBPose::GetInternalRepresentation()`
- `DBPose::FromInternalRepresentation()`
- 四元数 ↔ 欧拉角转换（仅用户I/O）

### Task 5: 单元测试
**文件**: `src/Common/test_rotation_utils_p1.cpp`  
**测试内容**:
- ECEF ↔ LLH 往返精度
- ECEF ↔ ENU 往返精度
- 坐标转换时旋转不变性
- 四元数稳定性

---

## 📊 关键指标

| 指标 | 要求 | 说明 |
|------|------|------|
| **坐标转换精度** | < 1 mm | ECEF <→> LLH 往返 |
| **ENU参考点精度** | < 1 mm | 在100km范围内 |
| **四元数精度** | < 1e-10 | 避免gimbal lock |
| **向后兼容** | 100% | 现有的纯欧拉角数据可读 |
| **GDAL依赖** | 可选 | 只有使用复杂投影时需要 |

---

## 🗂️ 文件结构

```
src/Common/
├── db_types.h                    [修改：添加坐标系字段]
├── db_types.cpp                  [修改：实现转换方法]
├── rotation_utils.h              [修改：添加坐标转换函数]
├── rotation_utils.cpp            [修改：实现坐标转换]
└── test_rotation_utils_p1.cpp    [新增：P1测试]
```

---

## 🚀 实现计划

1. **Day 1**: 设计+实现 Task 1-2 (坐标系结构 + DBPose增强)
2. **Day 2**: 实现 Task 3-4 (转换函数 + DBPose方法)
3. **Day 3**: 实现 Task 5 (单元测试 + 验证)
4. **Day 4**: 编译、测试、文档

**预计工作量**: 3-4 天

---

## ⚠️ 重要决策点

### 1. GDAL 依赖

**选项A**: 硬依赖
- 优点：功能完整（支持任意投影）
- 缺点：增加编译依赖

**选项B**: 软依赖（推荐）
- 只提供基本转换（ECEF ↔ LLH, ECEF ↔ ENU）
- 复杂投影使用 `rotation_utils` 中的接口但不实现
- 用户可选安装GDAL以获得完整功能

### 2. 四元数 vs 欧拉角存储

**设计**: 
- **首选**: 四元数（避免gimbal lock）
- **备选**: 欧拉角（仅用户交互，不序列化）
- **过渡**: 读取旧文件时自动转换

### 3. 向后兼容性

现有文件格式（Version 2）：
```
#image_id;x;y;z;omega;phi;kappa;weight_x;weight_y;weight_z;angleUnit;coordinate;eulerAngle
```

升级方案：
- 读取时：欧拉角自动转为四元数 ✓
- 写入时：新格式（JSON v3）或兼容格式
- 迁移策略：可选的格式转换工具

---

## 📚 相关文档

- [doc/design/rotation/rotation_standards.md](../design/rotation/rotation_standards.md)
- [doc/dev-notes/CODE_ANALYSIS_ROTATION.md](CODE_ANALYSIS_ROTATION.md)
- [src/Common/ROTATION_UTILS_IMPLEMENTATION.md](rotation_utils.md)

---

**状态**: 🔄 准备实现  
**下一步**: 从Task 1开始编码
