# Phase 2 - 完整架构设计：坐标系 & 测量数据管理

**日期**: 2026-02-08  
**状态**: 🎯 架构设计（准备实现）  
**作者**: 与用户讨论确定  

---

## 📐 整体系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         PROJECT LAYER                           │
│  (全局配置 & 原始输入数据)                                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ ProjectInformation (基本信息)                              │   │
│  │  - name, description, author, type, altitude, etc.      │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ input_coordinate_system (全局输入坐标系定义)                   │
│  │  - 所有原始测量数据(GNSS, IMU, GCP)都用此坐标系                │
│  │  - Type: EPSG / WKT / ENU / Local                        │   │
│  │  - RotationConvention: OmegaPhiKappa / YawPitchRoll      │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Camera[i] (相机配置 - 多相机支持)                            │   │
│  │  ├─ id, name, intrinsics, distortion                    │   │
│  │  └─ Image[j]  (该相机下的所有图像)                          │   │
│  │     ├─ id, name, path                                  │   │
│  │     ├─ InputPose (可选的输入位姿测量)                       │   │
│  │     │  ├─ position (x, y, z) - 可选                     │   │
│  │     │  ├─ rotation (ω, φ, κ) - 可选                     │   │
│  │     │  └─ angle_unit (degrees/radians)                 │   │
│  │     └─ Measurements[] (可能有多个)                      │   │
│  │        └─ GNSS/IMU/GCP/SLAM data with covariance      │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                       AT TASK LAYER                              │
│  (空三任务 - 树形嵌套，每层冻结输入)                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ATTask[k] - Task创建时冻结Project的输入数据                      │
│  ├─ InputSnapshot (冻结的原始输入)                               │
│  │  ├─ input_coordinate_system (copy from Project)             │
│  │  └─ Measurements[] (copy from Project)                      │
│  │     └─ GNSS先验约束在整个BA过程中保持不变                      │
│  ├─ Initialization (可选 - 上一个Task的结果用于初始化)           │
│  │  ├─ prev_task_id                                        │   │
│  │  └─ initial_poses (from previous ATTask output)         │   │
│  ├─ Processing (BA optimization)                          │   │
│  │  └─ GNSS constraints → optimized_poses                 │   │
│  ├─ output_coordinate_system (输出坐标系配置)                    │
│  └─ child_tasks[] (支持嵌套)                               │   │
│                                                                   │
│  TreeStructure Example:                                         │
│  ├─ ATTask_1 (初始空三)                                         │
│  │  ├─ input: Original GNSS measurements                       │
│  │  └─ output: DBPose (refined)                               │
│  ├─ ATTask_1_1 (迭代优化 v1)                                   │
│  │  ├─ input: Same GNSS measurements (unchanged!)             │
│  │  ├─ init: ATTask_1 results                                 │
│  │  └─ output: DBPose (better)                               │
│  └─ ATTask_1_2 (迭代优化 v2)                                   │
│     ├─ input: Same GNSS measurements (unchanged!)             │
│     ├─ init: ATTask_1_1 results                               │
│     └─ output: DBPose (best)                                  │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                         OUTPUT LAYER                             │
│  (按task的output_coordinate_system输出结果)                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔧 核心数据结构设计

### 1. CoordinateSystemDescriptor (扩展)

定义坐标系 + 旋转约定（全局统一）

```cpp
struct CoordinateSystemDescriptor {
    enum Type {
        kEPSG = 0,      // EPSG code (椭球和投影参数自动包含)
        kWKT = 1,       // OGC WKT string
        kENU = 2,       // ENU: "ENU:lat,lon,alt"
        kLocal = 3      // Local/unknown
    };
    
    // ─────────────────────────────────────────
    // 坐标系定义
    // ─────────────────────────────────────────
    Type type = kEPSG;
    std::string definition;  // "EPSG:4326", WKT, "ENU:39.9,116.4,50", etc.
    
    // ─────────────────────────────────────────
    // 旋转约定 (P1 NEW)
    // ─────────────────────────────────────────
    enum RotationConvention {
        kNone = 0,              // 无旋转信息
        kOmegaPhiKappa = 1,     // 摄影测量 (Z-Y-X extrinsic)
        kYawPitchRoll = 2       // 航空学 (Z-Y-X intrinsic)
    };
    RotationConvention rotation_convention = kNone;
    
    // ─────────────────────────────────────────
    // 可选参数
    // ─────────────────────────────────────────
    struct Origin {
        double x = 0.0, y = 0.0, z = 0.0;
        bool IsZero() const { return x == 0.0 && y == 0.0 && z == 0.0; }
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
        }
    };
    
    struct ReferencePoint {
        double lat = 0.0, lon = 0.0, alt = 0.0;
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(lat), CEREAL_NVP(lon), CEREAL_NVP(alt));
        }
    };
    
    std::optional<Origin> origin;
    std::optional<ReferencePoint> reference;
    
    // Serialization with version control
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        if (version == 0) {
            // Legacy: version 0
            ar(CEREAL_NVP(type), CEREAL_NVP(definition));
        } else {
            // Version 1+: includes rotation_convention
            ar(CEREAL_NVP(type), CEREAL_NVP(definition));
            ar(CEREAL_NVP(rotation_convention));
            ar(CEREAL_NVP(origin), CEREAL_NVP(reference));
        }
    }
};

CEREAL_CLASS_VERSION(CoordinateSystemDescriptor, 1);
```

### 2. Measurement (新增)

统一的测量数据框架，支持多种传感器

```cpp
/**
 * 通用测量数据 - 支持GNSS, IMU, GCP, SLAM等
 * 
 * 关键特性：
 * - 所有测量都有不确定度（协方差）
 * - 所有测量在input_coordinate_system中表示
 * - 易于扩展新的测量类型
 * 
 * BA中的使用：
 * - 测量数据 → 先验约束（在整个BA过程中不变）
 * - 权重矩阵 = 协方差矩阵的逆
 */
struct Measurement {
    enum Type {
        kGNSS = 0,      // GPS/RTK 位置 (x, y, z)
        kIMU = 1,       // IMU 姿态 & 加速度 & 角速度
        kGCP = 2,       // Ground Control Point 三维坐标
        kSLAM = 3,      // Visual SLAM 相对位姿
        kOther = 4
    };
    
    Type type;
    KeyType image_id;           // 关联的图像ID
    double timestamp = -1.0;    // 可选：时间戳
    
    // ─────────────────────────────────────────
    // GNSS 位置测量
    // ─────────────────────────────────────────
    struct GNSSMeasurement {
        double x, y, z;         // 位置 (坐标系由Project.input_crs定义)
        
        // 协方差矩阵（对角线元素）
        double cov_xx, cov_yy, cov_zz;
        
        // 可选：完整协方差
        double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0;
        
        // 可选：诊断信息
        int num_satellites = -1;
        double hdop = -1.0, vdop = -1.0;
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
            ar(CEREAL_NVP(cov_xx), CEREAL_NVP(cov_yy), CEREAL_NVP(cov_zz));
            ar(CEREAL_NVP(cov_xy), CEREAL_NVP(cov_xz), CEREAL_NVP(cov_yz));
            ar(CEREAL_NVP(num_satellites), CEREAL_NVP(hdop), CEREAL_NVP(vdop));
        }
    };
    
    // ─────────────────────────────────────────
    // IMU 惯导测量
    // ─────────────────────────────────────────
    struct IMUMeasurement {
        // 姿态 (按Project.input_crs.rotation_convention解释)
        bool has_attitude = false;
        double omega = 0.0, phi = 0.0, kappa = 0.0;  // 或 yaw, pitch, roll
        double cov_omega = 0.0, cov_phi = 0.0, cov_kappa = 0.0;
        
        // 加速度
        bool has_acceleration = false;
        double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
        double cov_accel_x = 0.0, cov_accel_y = 0.0, cov_accel_z = 0.0;
        
        // 角速度
        bool has_angular_velocity = false;
        double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
        double cov_gyro_x = 0.0, cov_gyro_y = 0.0, cov_gyro_z = 0.0;
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(has_attitude), CEREAL_NVP(omega), CEREAL_NVP(phi), 
               CEREAL_NVP(kappa), CEREAL_NVP(cov_omega), CEREAL_NVP(cov_phi), 
               CEREAL_NVP(cov_kappa));
            ar(CEREAL_NVP(has_acceleration), CEREAL_NVP(accel_x), 
               CEREAL_NVP(accel_y), CEREAL_NVP(accel_z));
            ar(CEREAL_NVP(cov_accel_x), CEREAL_NVP(cov_accel_y), CEREAL_NVP(cov_accel_z));
            ar(CEREAL_NVP(has_angular_velocity), CEREAL_NVP(gyro_x), 
               CEREAL_NVP(gyro_y), CEREAL_NVP(gyro_z));
            ar(CEREAL_NVP(cov_gyro_x), CEREAL_NVP(cov_gyro_y), CEREAL_NVP(cov_gyro_z));
        }
    };
    
    // ─────────────────────────────────────────
    // GCP 地面控制点
    // ─────────────────────────────────────────
    struct GCPMeasurement {
        std::string gcp_id;             // 控制点标识符
        double x, y, z;                 // 三维坐标
        double cov_xx, cov_yy, cov_zz;  // 坐标方差
        double cov_xy = 0.0, cov_xz = 0.0, cov_yz = 0.0;
        
        // 可选：图像上的观测
        double pixel_x = -1.0, pixel_y = -1.0;
        double pixel_cov_x = -1.0, pixel_cov_y = -1.0;
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(gcp_id), CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z));
            ar(CEREAL_NVP(cov_xx), CEREAL_NVP(cov_yy), CEREAL_NVP(cov_zz));
            ar(CEREAL_NVP(cov_xy), CEREAL_NVP(cov_xz), CEREAL_NVP(cov_yz));
            if (version > 0) {
                ar(CEREAL_NVP(pixel_x), CEREAL_NVP(pixel_y),
                   CEREAL_NVP(pixel_cov_x), CEREAL_NVP(pixel_cov_y));
            }
        }
    };
    
    // ─────────────────────────────────────────
    // SLAM 相对位姿
    // ─────────────────────────────────────────
    struct SLAMMeasurement {
        KeyType reference_image_id;     // 参考图像ID
        
        // 相对位置
        double dx, dy, dz;
        double cov_dx, cov_dy, cov_dz;
        
        // 相对旋转（四元数）
        double qx, qy, qz, qw;
        double cov_qx, cov_qy, cov_qz;  // 旋转不确定度
        
        double confidence = 1.0;        // 匹配置信度 [0, 1]
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(reference_image_id));
            ar(CEREAL_NVP(dx), CEREAL_NVP(dy), CEREAL_NVP(dz));
            ar(CEREAL_NVP(cov_dx), CEREAL_NVP(cov_dy), CEREAL_NVP(cov_dz));
            ar(CEREAL_NVP(qx), CEREAL_NVP(qy), CEREAL_NVP(qz), CEREAL_NVP(qw));
            ar(CEREAL_NVP(cov_qx), CEREAL_NVP(cov_qy), CEREAL_NVP(cov_qz));
            ar(CEREAL_NVP(confidence));
        }
    };
    
    // ─────────────────────────────────────────
    // 测量数据存储
    // ─────────────────────────────────────────
    std::optional<GNSSMeasurement> gnss;
    std::optional<IMUMeasurement> imu;
    std::optional<GCPMeasurement> gcp;
    std::optional<SLAMMeasurement> slam;
    
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(type), CEREAL_NVP(image_id), CEREAL_NVP(timestamp));
        
        // 根据type序列化相应的数据
        if (type == kGNSS) ar(CEREAL_NVP(gnss));
        if (type == kIMU) ar(CEREAL_NVP(imu));
        if (type == kGCP) ar(CEREAL_NVP(gcp));
        if (type == kSLAM) ar(CEREAL_NVP(slam));
    }
};

CEREAL_CLASS_VERSION(GCPMeasurement, 1);
CEREAL_CLASS_VERSION(Measurement, 1);
```

### 3. InputPose (新增)

轻量化的输入位姿表示

```cpp
/**
 * 输入位姿 - 存储在Image中的原始测量位姿
 * 
 * 坐标解释：
 * - (x, y, z) 按 Project.input_coordinate_system 解释
 *   例如：EPSG:4326 → (lon, lat, alt)
 *        UTM → (easting, northing, height)
 * 
 * 旋转解释：
 * - (omega, phi, kappa) 或 (yaw, pitch, roll)
 *   按 Project.input_crs.rotation_convention 解释
 */
struct InputPose {
    // ─────────────────────────────────────────
    // 位置信息
    // ─────────────────────────────────────────
    double x = 0.0, y = 0.0, z = 0.0;
    bool has_position = false;
    
    // ─────────────────────────────────────────
    // 旋转信息 (按rotation_convention解释)
    // ─────────────────────────────────────────
    double omega = 0.0, phi = 0.0, kappa = 0.0;
    bool has_rotation = false;
    
    // ─────────────────────────────────────────
    // 角度单位
    // ─────────────────────────────────────────
    int angle_unit = 0;  // 0=degrees, 1=radians
    
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(x), CEREAL_NVP(y), CEREAL_NVP(z), CEREAL_NVP(has_position));
        ar(CEREAL_NVP(omega), CEREAL_NVP(phi), CEREAL_NVP(kappa), CEREAL_NVP(has_rotation));
        ar(CEREAL_NVP(angle_unit));
    }
};

CEREAL_CLASS_VERSION(InputPose, 1);
```

### 4. ATTask InputSnapshot (新增)

冻结的输入快照

```cpp
struct ATTask {
    /**
     * 输入快照 - 创建任务时冻结Project的当前输入状态
     * 
     * 冻结原因：
     * - 版本控制：每个任务有独立快照
     * - 可重复性：运行相同Task总是用相同输入
     * - 追踪性：知道何时输入被冻结
     */
    struct InputSnapshot {
        CoordinateSystemDescriptor input_crs;   // 从Project copy
        std::vector<Measurement> measurements;   // 所有原始测量
        std::map<KeyType, std::vector<Measurement>> meas_by_image;  // 快速查询
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(input_crs), CEREAL_NVP(measurements));
            // meas_by_image 不序列化，运行时从measurements重建
        }
    };
    
    /**
     * 初始化信息 - 可选的前任务结果
     * 
     * 用途：
     * - 加速迭代收敛
     * - 但不作为约束（约束来自input_snapshot的measurements）
     */
    struct Initialization {
        KeyType prev_task_id = UndefinedKey;  // 前任务ID
        std::map<KeyType, DBPose> initial_poses;  // image_id → pose
        
        template <class Archive>
        void serialize(Archive& ar, std::uint32_t const version) {
            ar(CEREAL_NVP(prev_task_id), CEREAL_NVP(initial_poses));
        }
    };
    
    // ─────────────────────────────────────────
    // 成员
    // ─────────────────────────────────────────
    std::string id;
    InputSnapshot input_snapshot;
    std::optional<Initialization> initialization;
    
    CoordinateSystemDescriptor output_crs;
    std::map<KeyType, DBPose> optimized_poses;  // 输出：优化后的位姿
    
    std::vector<ATTask> child_tasks;  // 嵌套支持
    
    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version) {
        ar(CEREAL_NVP(id), CEREAL_NVP(input_snapshot), 
           CEREAL_NVP(initialization), CEREAL_NVP(output_crs),
           CEREAL_NVP(optimized_poses), CEREAL_NVP(child_tasks));
    }
};

CEREAL_CLASS_VERSION(ATTask::InputSnapshot, 1);
CEREAL_CLASS_VERSION(ATTask::Initialization, 1);
CEREAL_CLASS_VERSION(ATTask, 1);
```

---

## 📊 数据流示例

### 场景1：简单的航拍测量

```
项目创建
├─ 设置 input_crs = "EPSG:4326" (WGS84)
├─ 旋转约定 = "OmegaPhiKappa"
└─ 导入图像 + GNSS数据
   ├─ Image_1: (lon, lat, alt) = (116.40, 39.90, 100.0)
   ├─ Image_2: (lon, lat, alt) = (116.41, 39.91, 105.0)
   └─ 创建Measurement[i] {type=kGNSS, image_id, x, y, z, cov...}

创建ATTask_1
├─ input_snapshot copy:
│  ├─ input_crs = "EPSG:4326" + OmegaPhiKappa (冻结)
│  └─ measurements[] (冻结)
├─ BA优化：
│  ├─ 先验约束：GNSS measurements
│  ├─ 未知数：pose (position + orientation)
│  └─ 输出：optimized_poses[]
└─ output_crs = "EPSG:3857" (投影坐标系)

输出
└─ 结果按EPSG:3857坐标系导出
```

### 场景2：迭代优化

```
ATTask_1 (初始空三)
├─ input_snapshot: GNSS measurements
└─ output: optimized_poses_1

ATTask_1_1 (迭代v1)
├─ input_snapshot: Same GNSS measurements (frozen, unchanged!)
├─ initialization: from ATTask_1.output
└─ output: optimized_poses_1_1 (更好)

ATTask_1_2 (迭代v2)
├─ input_snapshot: Same GNSS measurements (still frozen!)
├─ initialization: from ATTask_1_1.output
└─ output: optimized_poses_1_2 (最优)
```

**关键**：虽然initial_poses来自前一个Task，但GNSS约束始终来自input_snapshot，保证一致性。

---

## 🔄 序列化版本控制

### Version Strategy

```cpp
// CoordinateSystemDescriptor
Version 0: type, definition (legacy)
Version 1: + rotation_convention, origin, reference

// Measurement & subclasses
Version 0: basic fields
Version 1: + optional diagnostic fields (hdop, vdop, pixel coordinates)

// ATTask
Version 0: legacy structure
Version 1: + InputSnapshot design
```

**向后兼容**：
- 旧文件（version 0）自动升级到version 1
- rotation_convention 默认值 = kNone
- origin, reference 可选

---

## 🎯 下一步实现

1. ✅ 更新CoordinateSystemDescriptor (添加rotation_convention)
2. ✅ 实现Measurement结构
3. ✅ 实现InputPose
4. ✅ 更新DBImage (添加input_pose字段)
5. ✅ 实现ATTask InputSnapshot & Initialization
6. ✅ 单元测试 & 编译验证
7. ✅ UI集成（后续）

