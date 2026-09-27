# Phase 2 - P1 实现进度：DBPose 增强与坐标系标准化

**日期**: 2026-02-08  
**状态**: 🔄 Task 1-2 完成，Task 3-5 待实现  
**完成内容**: 坐标系结构 + DBPose 增强  

---

## ✅ 已完成部分

### Task 1: 坐标系描述结构 ✅

**文件**: `src/Common/db_types.h`

**新增结构体**: `CoordinateSystemDescriptor`

```cpp
struct CoordinateSystemDescriptor {
    enum Type {
        kEPSG,           // EPSG code (e.g., "EPSG:4326")
        kWKT,            // WKT string (e.g., "PROJCS[...]")
        kPredefined      // Predefined name (ECEF, ENU, etc.)
    };
    
    Type type = kEPSG;
    std::string definition;              // "EPSG:4326" or "PROJCS[...]"
    std::optional<ReferencePoint> enu_reference;  // For ENU only
};
```

**特性**:
- ✅ 支持 EPSG 码（用户自定义投影）
- ✅ 支持 WKT 字符串（完整的OGC定义）
- ✅ 支持预定义名称（ECEF, ENU, WGS84_LLH等）
- ✅ ENU参考点支持（用于本地坐标系）
- ✅ `ToString()` 方法用于UI显示
- ✅ Cereal序列化支持

### Task 2: 增强 DBPose 结构 ✅

**文件**: `src/Common/db_types.h`

**核心改变**:

#### 1. **添加枚举字段**
```cpp
enum class EulerAngleConvention : int {
    kNone = 0,              // 不使用欧拉角
    kOmegaPhiKappa = 1,     // ISPRS photogrammetry
    kYawPitchRoll = 2       // Aviation
};

enum class RotationMatrixType : int {
    kWorld_to_Camera = 0,   // 标准
    kCamera_to_World = 1    // 反向
};
```

#### 2. **重构位置字段**
```cpp
double x, y, z;                          // 当前坐标系中的位置
CoordinateSystemDescriptor input_coordinate_system;  // 坐标系描述
```

#### 3. **添加四元数字段**（P0中设计，P1中集成）
```cpp
double quaternion_x, quaternion_y, quaternion_z, quaternion_w;
// 初始值: (0, 0, 0, 1) = 单位四元数
```

#### 4. **欧拉角字段改进**
```cpp
double omega, phi, kappa;                // 仅用于用户交互
EulerAngleConvention euler_convention;   // 约定说明
int angleUnit;                           // 0=degrees, 1=radians
```

#### 5. **旋转元数据**
```cpp
RotationMatrixType rotation_type;        // World→Camera vs Camera→World
```

#### 6. **完整的序列化支持**
- ✅ Version 3: 新格式（四元数 + 坐标系）
- ✅ Version 2 向后兼容（欧拉角 + ENU字段）
- ✅ 自动版本检测和转换

**改进的方法**:
```cpp
bool centerValid() const;      // 检查位置有效
bool rotationValid() const;    // 检查旋转有效（四元数或欧拉角）
Vec3 center() const;           // 获取位置向量
Vec3 rotationDeg() const;      // 获取欧拉角（度）
void reset();                  // 重置所有字段
```

### 编译状态 ✅

```
[100%] Built target InsightAT ✓
```

- ✅ 零编译错误
- ✅ 零相关警告
- ✅ 完全向后兼容

---

## 🔄 待实现部分

### Task 3: 坐标转换函数

**文件**: `src/Common/rotation_utils.h/cpp`

**需要实现的函数**:

```cpp
// ECEF ↔ LLH (WGS84)
Vec3 ECEF_to_LLH(const Vec3& ecef);
Vec3 LLH_to_ECEF(const Vec3& llh);

// ECEF ↔ ENU (给定参考点)
Vec3 ECEF_to_ENU(const Vec3& ecef, 
                 const CoordinateSystemDescriptor::ReferencePoint& ref);
Vec3 ENU_to_ECEF(const Vec3& enu, 
                 const CoordinateSystemDescriptor::ReferencePoint& ref);

// 一般投影转换（使用GDAL）- 可选
Vec3 TransformCoordinate(
    const Vec3& position,
    const CoordinateSystemDescriptor& from,
    const CoordinateSystemDescriptor& to);
```

**精度要求**:
- ECEF ↔ LLH 往返精度: < 1 mm
- ECEF ↔ ENU 往返精度: < 1 mm (在100km范围内)

### Task 4: DBPose 转换方法

**文件**: `src/Common/db_types.cpp` (新增)

**需要实现的方法**:

```cpp
/**
 * 将DBPose从input_coordinate_system转换到目标坐标系。
 * 旋转（四元数）保持不变。
 */
DBPose DBPose::ConvertToCoordinateSystem(
    const CoordinateSystemDescriptor& target_crs) const;

/**
 * 获取内部标准表示：ECEF + 四元数。
 * 用于内部算法处理（无精度损失）。
 */
struct InternalRepresentation {
    Vec3 ecef_position;
    Quaternion rotation;
};
InternalRepresentation GetInternalRepresentation() const;

/**
 * 从内部格式转回DBPose。
 */
static DBPose FromInternalRepresentation(
    const InternalRepresentation& internal,
    const CoordinateSystemDescriptor& output_crs);
```

### Task 5: 单元测试

**文件**: `src/Common/test_rotation_utils_p1.cpp` (新增)

**测试内容**:
- ECEF ↔ LLH 往返精度
- ECEF ↔ ENU 往返精度
- 坐标转换时旋转不变性
- 四元数稳定性
- Version 2 → Version 3 序列化兼容性

---

## 🎯 设计关键决策

### 坐标系表示

✅ **使用EPSG/WKT而非固定枚举**
- 原因: 允许用户自定义任意投影
- 灵活性: EPSG (简洁), WKT (完整定义)
- 例子: "EPSG:4326", "EPSG:3857", 自定义高斯克吕格等

✅ **ENU参考点支持**
- 本地坐标系需要参考点
- 存储在 `CoordinateSystemDescriptor.enu_reference`
- 参考点在WGS84 LLH中定义

### 内部处理

✅ **统一转到ECEF+四元数**
```
用户输入 (任意坐标系) 
    ↓
内部ECEF+四元数 (无精度损失)
    ↓
用户输出 (自选目标坐标系)
```

优点:
- 避免万向锁（四元数）
- 避免投影坐标精度损失（ECEF）
- 统一的内部表示

### 向后兼容性

✅ **Version 2 → Version 3 自动转换**
```cpp
// 读取Version 2数据时：
if (version == 2) {
    // 欧拉角 → 四元数
    quaternion = OPK_to_Quaternion(omega, phi, kappa);
    // 空ENU坐标系（待定）
    input_coordinate_system = {...};
}
```

---

## 📊 当前状态

| Task | 状态 | 预计完成 |
|------|------|---------|
| 1. 坐标系结构 | ✅ 完成 | 已完成 |
| 2. DBPose增强 | ✅ 完成 | 已完成 |
| 3. 坐标转换函数 | ⏳ 待做 | 今天/明天 |
| 4. DBPose方法 | ⏳ 待做 | 明天 |
| 5. 单元测试 | ⏳ 待做 | 后天 |
| **总体P1** | 🔄 进行中 | 3-4天 |

---

## 💡 下一步行动

### 今天
1. ✅ Task 1-2 (坐标系 + DBPose) - 已完成
2. ⏳ 开始Task 3 (坐标转换函数)

### 明天
1. ⏳ 完成Task 3-4
2. ⏳ 基本编译测试

### 后天
1. ⏳ Task 5 (单元测试)
2. ⏳ 集成测试
3. ⏳ 文档完善

---

## 📝 关键文件清单

### 新增文件
```
IMPLEMENTATION_PHASE2_P1_DESIGN.md       [设计文档]
src/Common/test_rotation_utils_p1.cpp    [测试，待创建]
```

### 修改文件
```
src/Common/db_types.h                    [✅ Task 1-2完成]
src/Common/rotation_utils.h              [⏳ Task 3 待做]
src/Common/rotation_utils.cpp            [⏳ Task 3 待做]
src/Common/db_types.cpp                  [⏳ Task 4 待做]
```

---

## 🔗 设计思想亮点

1. **坐标系灵活性**: EPSG/WKT支持任意用户自定义投影
2. **双层坐标系**: 输入(用户数据) vs 内部(ECEF) vs 输出(用户期望)
3. **无损内部表示**: ECEF + 四元数避免所有精度损失
4. **优雅的向后兼容**: Version 2 → 3 自动转换，无数据丢失
5. **统一的接口**: `ConvertToCoordinateSystem()` 简化用户交互

---

**项目状态**: 🔄 P1进行中  
**编译状态**: ✅ 成功  
**下一步**: 实现Task 3 (坐标转换函数)
