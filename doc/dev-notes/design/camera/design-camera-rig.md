# CameraRig 多相机配置设计

**Date**: 2026-02-08  
**Status**: ✅ Implemented and Compiled  
**Compiler**: 0 errors, 0 warnings

---

## 1. 设计目标

支持 **多相机固定几何关系** 的完整表示，特别是倾斜摄影中常见的场景：

- **五相机配置**（垂直向下 + 四斜视）
- **相对位置可知或未知**（已标定 vs 待优化）
- **BA中的约束管理**（根据标定状态应用不同约束）

---

## 2. 核心概念

### 2.1 CameraRig 结构体

```cpp
struct CameraRig {
    enum CalibrationStatus { kUnknown, kKnown, kPartial };
    
    struct CameraMount {
        uint32_t camera_id;              // 相机ID
        std::string position_name;       // 位置名称（"Nadir", "Forward"等）
        
        // 相对于Rig基准坐标系的位置和姿态
        double rel_x, rel_y, rel_z;      // 相对位置 (米)
        double rel_qx/qy/qz/qw;          // 相对姿态 (四元数)
        
        // 位置/姿态的不确定度（已标定时）
        double cov_pos_xx/yy/zz;         // 位置方差
        double cov_rot_xx/yy/zz;         // 旋转方差
    };
    
    uint32_t rig_id;                     // Rig ID（全局唯一）
    std::string rig_name;                // Rig名称（如"DJI-Zenmuse-5"）
    CalibrationStatus calib_status;      // 标定状态
    std::vector<CameraMount> mounts;     // 所有相机挂载点
    std::string description;             // 描述
};
```

### 2.2 ImageGroup 扩展

添加了 **kRigBased 模式** 和 **RigMountInfo 结构体**：

```cpp
struct ImageGroup {
    enum CameraMode { kGroupLevel, kImageLevel, kRigBased };  // 新增kRigBased
    
    struct RigMountInfo {
        uint32_t rig_id;        // 所属Rig ID
        uint32_t camera_id;     // 该Rig内的相机ID
    };
    
    std::optional<RigMountInfo> rig_mount_info;  // Rig模式时的挂载点信息
};
```

### 2.3 Project 集中管理

```cpp
struct Project {
    std::map<uint32_t, CameraRig> camera_rigs;  // 所有相机配置
    
    // 查询方法
    const CameraRig* GetCameraRig(uint32_t rig_id) const;
    const CameraModel* GetCameraForRigMount(uint32_t rig_id, uint32_t camera_id) const;
    bool ValidateRig(uint32_t rig_id) const;
};
```

---

## 3. 设计模式

### 3.1 单一Rig管理多相机

```
Rig 1: DJI-Zenmuse-5 (五相机配置)
├─ Camera 0 (Nadir)     @ (0,    0,    0)    # 垂直向下
├─ Camera 1 (Forward)   @ (50mm, 0,    20mm) # 前斜视 ~45°
├─ Camera 2 (Backward)  @ (-50mm, 0,   20mm) # 后斜视 ~45°
├─ Camera 3 (Left)      @ (0,    -50mm, 20mm) # 左斜视 ~45°
└─ Camera 4 (Right)     @ (0,     50mm, 20mm) # 右斜视 ~45°
```

### 3.2 ImageGroup关联Rig

```
ImageGroup 1 (Rig-based mode)
├─ rig_mount_info: {rig_id: 1, camera_id: 0}  # 来自Nadir相机
├─ images: [100张图像来自Nadir相机]
└─ group_camera: 指向Rig1中Camera0的相机参数

ImageGroup 2 (Rig-based mode)
├─ rig_mount_info: {rig_id: 1, camera_id: 1}  # 来自Forward相机
├─ images: [100张图像来自Forward相机]
└─ group_camera: 指向Rig1中Camera1的相机参数

... 继续对应其他4个相机 ...
```

### 3.3 BA中的约束应用

```cpp
// Bundle Adjustment 中
for (auto& group : project.image_groups) {
    if (group.camera_mode == ImageGroup::CameraMode::kRigBased) {
        const auto* rig = project.GetCameraRig(group.rig_mount_info->rig_id);
        
        if (rig->calib_status == CameraRig::CalibrationStatus::kKnown) {
            // 固定相对位置约束
            for (const auto& mount : rig->mounts) {
                fix_relative_pose(mount.camera_id, 
                                mount.rel_x/y/z,
                                mount.rel_qx/qy/qz/qw);
            }
        } else if (rig->calib_status == CameraRig::CalibrationStatus::kUnknown) {
            // 不应用约束，自由优化
        } else {
            // kPartial: 应用部分约束
        }
    }
}
```

---

## 4. 标定状态管理

### 4.1 三种标定状态

| 状态 | 含义 | BA中的处理 |
|------|------|-----------|
| **kUnknown** | 相对位置未知，需优化 | 相对位置作为变量参与优化 |
| **kKnown** | 相对位置已精确标定，固定 | 相对位置固定，减少优化变量 |
| **kPartial** | 部分参数已知，部分待优化 | 已知参数固定，其他参数优化 |

### 4.2 标定工作流

```
1. 未标定的Rig (kUnknown)
   ↓
   [离线标定或BA过程中优化]
   ↓
2. 部分已标定 (kPartial)  ← 可选中间状态
   ↓
   [继续优化剩余参数]
   ↓
3. 完全已标定 (kKnown)
   ↓
   [在后续BA中固定相对关系，提高稳定性]
```

---

## 5. API 使用示例

### 5.1 创建五相机Rig

```cpp
CameraRig rig;
rig.rig_id = 1;
rig.rig_name = "DJI-Zenmuse-5";
rig.calib_status = CameraRig::CalibrationStatus::kKnown;
rig.description = "Five-camera tilt system on DJI platform";

// 添加垂直相机
rig.mounts.push_back({
    camera_id: 0,
    position_name: "Nadir",
    rel_x: 0.0, rel_y: 0.0, rel_z: 0.0,
    rel_qx: 0.0, rel_qy: 0.0, rel_qz: 0.0, rel_qw: 1.0
});

// 添加前斜视相机（45°俯角）
rig.mounts.push_back({
    camera_id: 1,
    position_name: "Forward",
    rel_x: 0.05, rel_y: 0.0, rel_z: 0.02,
    rel_qx: 0.258, rel_qy: 0.0, rel_qz: 0.0, rel_qw: 0.966  // 约45°绕X轴
});

// ... 继续添加其他4个相机 ...

// 验证Rig
assert(rig.IsValid());

// 添加到Project
project.camera_rigs[1] = rig;
```

### 5.2 创建关联到Rig的ImageGroup

```cpp
// 为Nadir相机创建分组
ImageGroup nadir_group;
nadir_group.group_id = 100;
nadir_group.group_name = "Nadir Images";
nadir_group.camera_mode = ImageGroup::CameraMode::kRigBased;
nadir_group.rig_mount_info = {rig_id: 1, camera_id: 0};

// 从camera_models中获取Camera0的参数
const CameraModel* camera_model = project.GetCameraForRigMount(1, 0);
if (camera_model) {
    nadir_group.group_camera = *camera_model;
}

// 添加Nadir图像
for (int i = 0; i < num_nadir_images; ++i) {
    Image img;
    img.image_id = i;
    img.filename = "nadir_" + std::to_string(i) + ".tif";
    nadir_group.AddImage(img);
}

project.image_groups.push_back(nadir_group);

// 为Forward相机创建分组（类似过程）
// ... 继续为其他4个相机创建分组 ...
```

### 5.3 查询和验证

```cpp
// 获取Rig信息
const auto* rig = project.GetCameraRig(1);
if (rig) {
    std::cout << rig->ToString() << std::endl;
}

// 验证Rig完整性
if (!project.ValidateRig(1)) {
    LOG(ERROR) << "Rig 1 validation failed";
}

// 获取特定相机的参数
const auto* cam_model = project.GetCameraForRigMount(1, 0);
if (cam_model) {
    std::cout << "Nadir camera: f=" << cam_model->focal_length << std::endl;
}

// 遍历Rig中的所有相机挂载点
for (const auto& mount : rig->mounts) {
    std::cout << "Camera " << mount.camera_id << " @ " << mount.position_name
              << " rel_pos=(" << mount.rel_x << ", " << mount.rel_y << ", " 
              << mount.rel_z << ")" << std::endl;
}
```

### 5.4 BA中应用约束

```cpp
// 初始化BA问题
BA_problem ba;

for (const auto& group : project.image_groups) {
    if (group.camera_mode != ImageGroup::CameraMode::kRigBased) {
        continue;
    }
    
    const auto* rig = project.GetCameraRig(group.rig_mount_info->rig_id);
    if (!rig) continue;
    
    // 根据标定状态应用约束
    if (rig->calib_status == CameraRig::CalibrationStatus::kKnown) {
        // 应用固定相对位置约束
        const auto& ref_mount = rig->mounts[0];  // 以第一个相机为基准
        
        for (size_t i = 1; i < rig->mounts.size(); ++i) {
            const auto& mount = rig->mounts[i];
            
            // 创建相对位姿约束
            RelativePoseConstraint constraint;
            constraint.camera_id1 = ref_mount.camera_id;
            constraint.camera_id2 = mount.camera_id;
            constraint.rel_t = {mount.rel_x, mount.rel_y, mount.rel_z};
            constraint.rel_q = {mount.rel_qx, mount.rel_qy, mount.rel_qz, mount.rel_qw};
            constraint.weight = 100.0;  // 强约束
            
            ba.AddConstraint(constraint);
        }
    }
}

// 执行BA求解
ba.Solve();
```

---

## 6. 数据结构完整性验证

### 6.1 CameraMount::IsValid()

检查项：
- camera_id 有效（非-1）
- rel_x/y/z 是有限浮点数
- rel_qx/qy/qz/qw 是有限浮点数

### 6.2 CameraRig::IsValid()

检查项：
- rig_id 有效（非-1）
- mounts 非空（至少1个相机）
- 所有挂载点有效
- 所有 camera_id 唯一（无重复）

### 6.3 Project::ValidateRig()

检查项：
- Rig 本身有效
- 所有挂载的相机都有对应的相机参数

---

## 7. 序列化和版本管理

### 7.1 版本变更

```cpp
CEREAL_CLASS_VERSION(insight::database::CameraRig, 1);
CEREAL_CLASS_VERSION(insight::database::CameraRig::CameraMount, 1);
CEREAL_CLASS_VERSION(insight::database::ImageGroup, 2);  // 新增rig_mount_info
CEREAL_CLASS_VERSION(insight::database::ImageGroup::RigMountInfo, 1);
CEREAL_CLASS_VERSION(insight::database::Project, 3);     // 新增camera_rigs
```

### 7.2 向后兼容

- Project v2 → v3：自动处理新增 camera_rigs 字段
- ImageGroup v1 → v2：自动处理新增 rig_mount_info 字段
- 旧格式的Rig数据不包含在Measurement中

---

## 8. 工程实现细节

### 8.1 文件变更

**database_types.h:**
- 新增 CameraRig 结构体（完整的嵌套结构）
- ImageGroup 新增 CameraMode::kRigBased 和 RigMountInfo
- Project 新增 camera_rigs 容器和查询方法

**database_types.cpp:**
- CameraRig::IsValid()、ToString()、GetSummary()
- CameraMount::IsValid()、ToString()
- Project::GetCameraRig()
- Project::GetCameraForRigMount()
- Project::ValidateRig()

### 8.2 编译结果

```
✅ Compilation: SUCCESS
  - 0 errors
  - 0 warnings
  
新增内容：
  - CameraRig struct with 6 methods
  - CameraMount struct with 2 methods
  - ImageGroup::RigMountInfo struct
  - 3个Project查询方法
  - 完整的序列化支持
```

---

## 9. 设计对比

### 9.1 vs 旧设计

| 维度 | 旧设计 | 新设计 |
|------|--------|--------|
| **多相机表示** | 不支持 | ✅ 完整支持 |
| **相对位置管理** | N/A | ✅ 统一管理 |
| **标定状态** | 无 | ✅ 三状态支持 |
| **BA约束** | 难以应用 | ✅ 明确的约束API |
| **数据一致性** | 分散 | ✅ 集中(Project) |

### 9.2 可扩展性

- **支持更多相机数量**：只需添加更多CameraMount
- **支持不同Rig配置**：多个独立的CameraRig实例
- **支持混合模式**：同一Project中可有多种模式的ImageGroup

---

## 10. 典型应用场景

### 10.1 倾斜摄影五相机系统

```
项目：城市三维模型重建
├─ Rig 1: DJI-Zenmuse-5 (已标定)
│  └─ calib_status: kKnown
│
├─ ImageGroup 1-5: 五个相机的图像分组
│  ├─ Group 1: Nadir (1000张)
│  ├─ Group 2: Forward (1000张)
│  ├─ Group 3: Backward (1000张)
│  ├─ Group 4: Left (1000张)
│  └─ Group 5: Right (1000张)
│
└─ BA约束: 固定五相机的相对位置关系
   → 减少优化变量，提高稳定性
```

### 10.2 多源数据融合

```
项目：混合数据处理
├─ Rig 1: 主无人机五相机 (kKnown)
├─ Rig 2: 斜挂二级相机 (kPartial)
├─ ImageGroup: 各种数据源的图像
└─ BA: 根据标定状态灵活应用约束
```

---

## 11. 总结

**CameraRig设计的核心价值：**
- ✅ **表达多相机固定关系** - 完整支持倾斜摄影等复杂系统
- ✅ **灵活的标定管理** - 从未知到已知的逐步标定工作流
- ✅ **BA中的约束应用** - 根据标定状态自动应用相应约束
- ✅ **数据集中管理** - Project 统一存储所有相机配置
- ✅ **易于扩展** - 支持任意数量的相机和Rig配置

这是一个**生产级别**的设计，直接支持现代倾斜摄影和多相机系统的实际需求。
