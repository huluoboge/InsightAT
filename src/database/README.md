# InsightAT Database Module (`src/database/`)

## 概述

`src/database/` 目录包含 InsightAT 的核心数据库类型定义，用于管理坐标系、测量数据、位姿、**相机参数**、和空三任务。该模块采用清洁的代码组织，包括完整的 Cereal 序列化支持。

## 文件结构

```
src/database/
├── CMakeLists.txt                             # 编译配置
├── database_types.h                           # 统一的头文件（所有类型定义）
├── database_types.cpp                         # 实现文件（所有方法实现）
├── README.md                                  # 本文档
├── IMAGEGROUP_DESIGN.md                       # ImageGroup 详细设计
├── IMAGEGROUP_SUMMARY.md                      # ImageGroup 快速参考
├── ATTASK_IMAGEGROUP_INTEGRATION.md           # ATTask 集成指南
└── PROJECT_STRUCTURE_DESIGN.md                # Project 结构体设计 ⭐⭐ 新增
```

## 核心类型

### 1. CoordinateSystem（坐标系）

定义和管理四种坐标系类型：

```cpp
enum class Type {
    kEPSG,      // EPSG代码 (e.g., "EPSG:4326")
    kWKT,       // OGC WKT标准定义
    kENU,       // ENU本地坐标系 (需要参考点)
    kLocal      // Local本地/未知坐标系
};

enum class RotationConvention {
    kNone,              // 无旋转信息
    kOmegaPhiKappa,     // 摄影测量 (ω,φ,κ)
    kYawPitchRoll       // 航空学 (Y,P,R)
};
```

**特点**：
- 支持坐标原点设置（用于投影坐标系精度优化）
- 支持ENU参考点（lat/lon/alt）
- 版本化序列化（Version 1+）

**示例**：
```cpp
insight::database::CoordinateSystem cs;
cs.type = CoordinateSystem::Type::kENU;
cs.rotation_convention = CoordinateSystem::RotationConvention::kOmegaPhiKappa;
cs.reference = CoordinateSystem::ReferencePoint{39.9045, 116.4074, 50.0};
std::string desc = cs.ToString();
bool valid = cs.IsValid();
```

### 2. InputPose（输入位姿）

轻量级的位姿测量表示：

```cpp
struct InputPose {
    // 位置（可选）
    double x, y, z;
    bool has_position;
    
    // 旋转（可选）Omega-Phi-Kappa
    double omega, phi, kappa;
    bool has_rotation;
    
    // 角度单位
    enum class AngleUnit { kDegrees, kRadians };
    AngleUnit angle_unit;
};
```

**方法**：
- `Reset()` - 清空所有数据
- `HasData() const` - 检查是否有任何数据
- `IsValid() const` - 验证数据有效性
- `ToString() const` - 获取人类可读的描述

### 3. Measurement（测量框架）

统一的测量数据框架，支持四种类型：

```cpp
enum class Type {
    kGNSS,      // GPS/GNSS定位
    kIMU,       // 惯性测量单元
    kGCP,       // 地面控制点
    kSLAM,      // SLAM相对定位
    kOther      // 其他类型
};
```

#### 3.1 GNSS 测量
```cpp
struct GNSSMeasurement {
    double x, y, z;                    // 位置
    double cov_xx, cov_yy, cov_zz;    // 对角协方差
    double cov_xy, cov_xz, cov_yz;    // 非对角协方差
    uint8_t num_satellites;            // 卫星数量
    double hdop, vdop;                 // DOP值
};
```

#### 3.2 IMU 测量
```cpp
struct IMUMeasurement {
    // 姿态（可选）
    bool has_attitude;
    double roll, pitch, yaw;
    double cov_att_xx, cov_att_yy, cov_att_zz;
    
    // 加速度（可选）
    bool has_accel;
    double accel_x, accel_y, accel_z;
    double cov_acc_xx, cov_acc_yy, cov_acc_zz;
    
    // 角速度（可选）
    bool has_gyro;
    double gyro_x, gyro_y, gyro_z;
    double cov_gyr_xx, cov_gyr_yy, cov_gyr_zz;
};
```

#### 3.3 GCP 测量
```cpp
struct GCPMeasurement {
    uint32_t gcp_id;                   // 控制点ID
    double x, y, z;                    // 世界坐标
    double cov_xx, cov_yy, cov_zz;    // 坐标协方差
    double pixel_x, pixel_y;           // 图像像素坐标
};
```

#### 3.4 SLAM 测量
```cpp
struct SLAMMeasurement {
    uint32_t reference_image_id;       // 参考图像ID
    double rel_x, rel_y, rel_z;        // 相对位置
    double rel_qx, rel_qy, rel_qz, rel_qw;  // 四元数
    double confidence;                 // 置信度 [0, 1]
};
```

**主要成员**：
```cpp
struct Measurement {
    Type type;                         // 测量类型
    uint32_t image_id;                // 所属图像ID
    int64_t timestamp;                // 时间戳（毫秒）
    
    // 可选的测量数据
    std::optional<GNSSMeasurement> gnss;
    std::optional<IMUMeasurement> imu;
    std::optional<GCPMeasurement> gcp;
    std::optional<SLAMMeasurement> slam;
};
```

**方法**：
- `IsValid() const` - 验证测量数据一致性
- `ToString() const` - 获取人类可读的描述

### 4. ATTask（空三任务）

自动三角测量任务及其快照设计：

```cpp
struct ATTask {
    // ── 输入快照（创建时冻结）
    struct InputSnapshot {
        CoordinateSystem input_coordinate_system;  // 输入坐标系
        std::vector<Measurement> measurements;     // 所有原始测量
    };
    
    // ── 初始化信息（可选）
    struct Initialization {
        uint32_t prev_task_id;                     // 前任务ID
        std::map<uint32_t, OptimizedPose> initial_poses;  // 初始位姿
    };
    
    // ── 任务数据
    std::string id;                                // 任务唯一ID
    InputSnapshot input_snapshot;                  // 冻结的输入状态
    std::optional<Initialization> initialization;  // 初始化信息
    CoordinateSystem output_coordinate_system;     // 输出坐标系
    std::map<uint32_t, OptimizedPose> optimized_poses;  // 优化结果
};
```

## 设计特点

### 1. 快照设计（Snapshot Pattern）
- **目的**：版本控制和追踪性
- **实现**：ATTask 在创建时冻结 Project 的当前输入状态
- **优势**：
  - 每个任务有独立快照，不受后续修改影响
  - 可重复性：运行相同Task总是用相同输入
  - 可追踪性：知道何时输入被冻结

### 2. 约束与初始化的分离
- **约束来源**：InputSnapshot 中的 measurements（原始数据，冻结）
- **初始化来源**：Initialization 中的前任务结果（用于加速，非约束）
- **优势**：清晰的数据语义，避免混淆

### 3. 灵活的测量框架
- 支持多种测量类型
- 每种类型都有完整的协方差矩阵
- 使用 `std::optional` 支持稀疏数据
- 每个测量都有单独的验证方法

### 4. 完整的序列化支持
- 使用 Cereal 库序列化
- 版本控制（CEREAL_CLASS_VERSION）
- 支持向前兼容扩展

## 编译集成

### CMakeLists.txt 配置

```cmake
# src/database/CMakeLists.txt
add_library(InsightATDatabase OBJECT
    database_types.cpp
    database_types.h
)

target_link_libraries(InsightATDatabase PUBLIC 
    Eigen3::Eigen
    glog::glog
)
```

### 使用方式

```cpp
#include "database/database_types.h"

using namespace insight::database;

// 创建坐标系
CoordinateSystem cs;
cs.type = CoordinateSystem::Type::kENU;

// 创建测量
Measurement meas;
meas.type = Measurement::Type::kGNSS;
meas.gnss = GNSSMeasurement{...};

// 创建任务
ATTask task;
task.input_snapshot.input_coordinate_system = cs;
task.input_snapshot.measurements.push_back(meas);
```

## 代码统计

| 文件 | 行数 | 大小 |
|------|------|------|
| database_types.h | 310 | 11.6K |
| database_types.cpp | 233 | 8.0K |
| CMakeLists.txt | - | 3.5K |
| **总计** | **543** | **23.1K** |

## 编译状态

✅ **编译成功**
- 项目：InsightAT
- 编译时间：2026-02-08 14:35
- 输出：1.7MB 可执行文件
- 错误数：0
- 警告数：10（来自GUI模块，与数据库无关）

## 命名规范

- **文件名**：`snake_case.h / .cpp`
- **类名/结构体**：`PascalCase`
- **成员变量**：`snake_case`（public）或 `snake_case_`（private）
- **方法名**：`PascalCase`（公共）或 `snake_case_`（私有）
- **常数/枚举值**：`kPascalCase`

## 依赖关系

```
database_types.h/cpp
├── cereal/cereal.hpp           (头文件库，序列化)
├── Eigen3                       (线性代数)
├── glog                         (日志)
└── C++17 标准库
```

## 扩展指南

### 添加新的测量类型

1. 在 `Measurement::Type` 枚举中添加新类型
2. 创建新的 `struct XxxMeasurement`
3. 在 `Measurement` 中添加 `std::optional<XxxMeasurement> xxx;`
4. 实现 `XxxMeasurement::IsValid()` 方法
5. 更新 `Measurement::IsValid()` 的 switch 语句
6. 在 `Measurement::ToString()` 中添加格式化输出

### 添加新的坐标系类型

1. 在 `CoordinateSystem::Type` 枚举中添加新类型
2. 在 `ToString()` 中处理新类型的输出
3. 在 `IsValid()` 中添加验证逻辑
4. 更新文档

## 最近更新（v2.0）

### ✨ ATTask 与 ImageGroup 集成

ATTask 现已完全集成 ImageGroup 设计：

**新增字段**：
- `ATTask::InputSnapshot::image_groups` - 输入图像分组

**新增方法**：
- `ATTask::GetCameraForImage(group_id, image_id)` - 获取特定图像的相机参数
- `ATTask::FindGroupByImageId(image_id)` - 查找包含图像的分组
- `ATTask::GetTotalImageCount()` - 获取所有图像总数

**版本升级**：
- `ATTask` v1 → v2
- `ATTask::InputSnapshot` v1 → v2

详见：[ATTASK_IMAGEGROUP_INTEGRATION.md](ATTASK_IMAGEGROUP_INTEGRATION.md)

## 相关资源

- **前驱工作**：P0 Rotation Utilities（`src/Common/rotation_utils.h`）
- **ImageGroup 设计**：[IMAGEGROUP_DESIGN.md](IMAGEGROUP_DESIGN.md)（508 行）
- **ImageGroup 参考**：[IMAGEGROUP_SUMMARY.md](IMAGEGROUP_SUMMARY.md)（550 行）
- **ATTask 集成**：[ATTASK_IMAGEGROUP_INTEGRATION.md](ATTASK_IMAGEGROUP_INTEGRATION.md)（450+ 行）⭐ 新增
- **序列化库**：https://github.com/USCiLab/cereal
- **GNSS数据格式**：NMEA 0183, RTCM 3.x
- **坐标系参考**：OGC WKT 2.0, EPSG 数据库

## 注意事项

1. **线程安全**：当前实现不是线程安全的。如需并发访问，请添加同步机制。
2. **内存管理**：使用 `std::optional` 而不是指针，简化内存管理。
3. **浮点精度**：所有坐标使用 `double`（64位）以保持高精度。
4. **协方差矩阵**：存储为 3x3 矩阵的对角线和上三角元素。
5. **四元数归一化**：SLAM 测量的四元数应已归一化，验证时允许 ±1% 误差。
6. **ImageGroup 使用**：详见 [IMAGEGROUP_SUMMARY.md](IMAGEGROUP_SUMMARY.md) 的"快速入门"章节。

---

**创建日期**：2026-02-08  
**更新日期**：2026-02-08  
**创建人**：InsightAT Team  
**版本**：2.0（ATTask + ImageGroup 集成）


## 5. Project（项目容器）✨ 新增

项目结构体是高层容器，包含完整的项目信息和所有数据：

**核心字段**：
- name, uuid, creation_time, description, author - 项目元数据
- input_coordinate_system - 输入坐标系
- measurements - 所有测量数据（GNSS/IMU/GCP/SLAM）
- image_groups - 所有图像分组
- initial_pose - 初始位姿（可选）
- at_tasks - 空三处理任务列表

**核心方法**：
- GetTotalImageCount() - 获取所有图像总数
- GetMeasurementCountByType() - 按类型统计测量数据
- FindGroupByImageId() - 查找图像所属分组
- GetCameraForImageId() - 查找图像的相机参数
- IsValid() - 验证项目数据完整性
- ToString() / GetSummary() - 获取项目描述

详见：[PROJECT_STRUCTURE_DESIGN.md](PROJECT_STRUCTURE_DESIGN.md)（600+ 行）

---

**版本**：3.0（Project 顶层容器）

## 4. CameraModel（相机参数）✨ 升级

完整的Brown-Conrady畸变模型，支持灵活的参数优化配置。

### 4.1 设计原则：数据和算法分离

```
数据结构层 (database_types.h)
  ↑
  └─→ CameraModel / OptimizationFlags（纯数据）
       ├─ IsValid() / ToString() / GetSummary()
       └─ HasDistortion() / Reset()

算法层 (camera_utils.h) - 独立实现
  ├─ 参数估计：EstimateFromExif(), EstimateFromEquivalentFocalLength()
  ├─ BA支持：BuildOptimizationMask(), GetOptimizationParameterCount()
  └─ 几何计算：ApplyDistortion(), ProjectPoint3D(), UnprojectNormalized()
```

**优势**：
- ✅ 相机参数结构稳定，不受算法变更影响
- ✅ 算法独立，易于复用和扩展
- ✅ 添加新相机类型或新算法无需修改数据结构
- ✅ 清晰的职责分离，易于测试和维护

### 4.2 核心参数

| 分类 | 字段 | 单位 | 用途 |
|------|------|------|------|
| **分辨率** | width, height | px | 图像大小 |
| **传感器** | sensor_width/height_mm | mm | 物理大小（初值估计） |
| | pixel_size_um | µm | 像素物理尺寸 |
| | focal_length_35mm | mm | 35mm等效焦距 |
| **内参** | focal_length | px | 焦距 |
| | principal_point_x/y | px | 主点坐标 |
| | aspect_ratio | - | 宽高比 (fy/fx) |
| | skew | - | 像素偏斜 |
| **径向畸变** | k1, k2, k3, k4 | - | 1阶～4阶 |
| **切向畸变** | p1, p2 | - | 装配偏差 |
| **棱镜畸变** | b1, b2 | - | 光学缺陷 |
| **元数据** | camera_name, make, model | - | 相机识别 |
| **优化标记** | optimization_flags | - | BA配置 |

### 4.3 优化标记（OptimizationFlags）

```cpp
struct OptimizationFlags {
    bool focal_length, principal_point_x, principal_point_y;
    bool aspect_ratio, skew;
    bool k1, k2, k3, k4;  // 径向
    bool p1, p2;          // 切向
    bool b1, b2;          // 棱镜
};
```

**使用场景**：
```cpp
// 场景1：简单参数化（广角镜头）
flags.focal_length = true;
flags.principal_point_x = true;
flags.principal_point_y = true;
flags.k1 = true;
// k2, k3, p1, p2等固定

// 场景2：完全参数化（精密测量）
flags.focal_length = true;
flags.k1 = flags.k2 = flags.k3 = true;
flags.p1 = flags.p2 = true;

// 场景3：固定内参（已标定相机）
// 所有flags = false（仅优化位姿）
```

### 4.4 方法

| 方法 | 返回值 | 用途 |
|------|--------|------|
| IsValid() | bool | 验证参数一致性 |
| ToString() | string | 详细多行描述 |
| GetSummary() | string | 单行摘要 |
| HasDistortion() | bool | 检查是否有畸变 |
| Reset() | void | 重置所有参数 |

### 4.5 相关算法（独立模块）

这些算法**不是**CameraModel的方法，而在独立的camera_utils.h中：

```cpp
// 参数估计
bool EstimateFromEquivalentFocalLength(...);
bool EstimateFromExif(...);
void GetFocalLengthRange(...);

// BA支持
std::vector<bool> BuildOptimizationMask(...);
int GetOptimizationParameterCount(...);

// 几何计算
void ApplyDistortion(...);
void RemoveDistortion(...);
bool ProjectPoint3D(...);
void UnprojectNormalized(...);
```

详见：[CAMERA_MODEL_DESIGN.md](CAMERA_MODEL_DESIGN.md)（500+ 行）

---

**版本**：4.0（CameraModel 升级 + 数据/算法分离）
**更新日期**：2026-02-08

