# 相机模型设计文档（Camera Model Design）

**版本**: 2.0 (Brown-Conrady Complete Model with Optimization Flags)  
**日期**: 2026-02-08  
**作者**: InsightAT Team

---

## 1. 设计原则

### 1.1 数据和算法分离

这是本设计最重要的原则：

```
┌─────────────────────────────────────────────────────────────┐
│                    应用层 (Application)                      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│  算法层 (Algorithm Layer) - camera_utils.h                  │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ 参数估计 (Parameter Estimation)                          ││
│  │ - EstimateFromExif()                                   ││
│  │ - EstimateFromEquivalentFocalLength()                  ││
│  │ - GetFocalLengthRange()                                ││
│  │                                                         ││
│  │ 约束和优化 (Constraints & Optimization)                 ││
│  │ - IsParameterOptimized()                               ││
│  │ - GetOptimizationParameterCount()                      ││
│  │ - BuildOptimizationMask()                              ││
│  │                                                         ││
│  │ 畸变计算 (Distortion Calculation)                       ││
│  │ - ApplyDistortion()                                    ││
│  │ - RemoveDistortion()                                   ││
│  │ - ProjectPoint3D()                                     ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│  数据结构层 (Data Structure Layer) - database_types.h      │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ CameraModel (纯数据结构)                                 ││
│  │ - 所有参数字段                                           ││
│  │ - IsValid() / ToString() / GetSummary()                ││
│  │ - HasDistortion() / Reset()                            ││
│  │                                                         ││
│  │ OptimizationFlags (优化配置数据)                         ││
│  │ - 参数优化标记                                           ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

**设计目的**：
- ✅ CameraModel 仅是**数据容器**，不包含算法
- ✅ 算法独立于结构体，便于复用和扩展
- ✅ 易于添加新的相机类型和算法
- ✅ 算法可以在不修改结构体的情况下演进

---

## 2. CameraModel 结构体设计

### 2.1 结构概览

```cpp
struct CameraModel {
    // 1. 相机类型
    Type type;                    // Pinhole / BrownConrady / ...
    
    // 2. 传感器物理信息
    width, height;                // 分辨率（像素）
    sensor_width_mm, sensor_height_mm;  // 传感器大小（物理）
    pixel_size_um;                // 像素大小（微米）
    focal_length_35mm;            // 35mm等效焦距
    
    // 3. 内参（Pinhole模型）
    focal_length;                 // 焦距（像素）
    principal_point_x/y;          // 主点（像素）
    aspect_ratio;                 // 宽高比（fy/fx）
    skew;                         // 像素偏斜
    
    // 4. Brown-Conrady 畸变参数
    k1, k2, k3, k4;              // 径向畸变
    p1, p2;                       // 切向畸变
    b1, b2;                       // 薄棱镜畸变
    
    // 5. 元数据
    camera_name, make, model;     // 相机识别信息
    lens_model;                   // 镜头型号
    serial_number;                // 序列号
    
    // 6. 优化配置
    OptimizationFlags optimization_flags;
    
    // 7. 方法（仅数据验证和格式化）
    bool IsValid() const;
    std::string ToString() const;
    std::string GetSummary() const;
    bool HasDistortion() const;
    void Reset();
};
```

### 2.2 数据字段详解

#### 2.2.1 传感器物理信息

| 字段 | 单位 | 用途 | 示例值 |
|------|------|------|--------|
| `width` | 像素 | 图像宽度 | 5472 |
| `height` | 像素 | 图像高度 | 3648 |
| `sensor_width_mm` | mm | 传感器物理宽度（用于初值估计） | 35.0 |
| `sensor_height_mm` | mm | 传感器物理高度 | 23.0 |
| `pixel_size_um` | µm | 像素物理大小 | 6.5 |
| `focal_length_35mm` | mm | 35mm等效焦距 | 50.0 |

**设计注意**：
- 物理信息用于**初值估计**，由工具函数独立处理
- 不直接用于算法，只作为参考信息
- 某些应用场景可为空（如从内参标定文件直接加载）

#### 2.2.2 内参（Pinhole）

| 字段 | 单位 | 范围 | 物理意义 |
|------|------|------|---------|
| `focal_length` | px | > 0 | 焦距 (f = f_mm / pixel_size_um) |
| `principal_point_x` | px | 0 ~ width | 主点X坐标（图像中心） |
| `principal_point_y` | px | 0 ~ height | 主点Y坐标 |
| `aspect_ratio` | 无 | > 0 | fy/fx（通常=1.0） |
| `skew` | 无 | ≈ 0 | 像素偏斜（现代相机通常=0） |

**标准针孔投影模型**：
```
[u]   [f·ar   0    ppx] [x/z]
[v] = [ 0    f    ppy] [y/z]
[1]   [ 0    0     1  ] [ 1 ]
```

#### 2.2.3 Brown-Conrady 畸变参数

完整的8参数模型：

```
径向畸变：
  r² = u_norm² + v_norm²
  r_dist = r(1 + k1·r² + k2·r⁴ + k3·r⁶ + k4·r⁸)

切向畸变：
  δu_tan = p1(2u_norm·v_norm + p2(r² + 2u_norm²)) 
         + b1·u_norm + b2·v_norm
  δv_tan = p2(2u_norm·v_norm + p1(r² + 2v_norm²)) 
         + b1·v_norm + b2·u_norm

最终畸变像素：
  u_dist = u_norm·r_dist + δu_tan
  v_dist = v_norm·r_dist + δv_tan
```

**参数范围参考**：

| 参数 | 典型范围 | 物理意义 |
|------|----------|---------|
| k1 | -1e-4 ~ 1e-2 | 主要径向畸变（桶形/枕形） |
| k2 | -1e-4 ~ 1e-4 | 高阶径向修正 |
| k3 | -1e-6 ~ 1e-6 | 极高阶径向修正 |
| k4 | ≈ 0 | 罕见使用 |
| p1 | -1e-4 ~ 1e-4 | 由装配偏差引起 |
| p2 | -1e-4 ~ 1e-4 | 由装配偏差引起 |
| b1, b2 | ≈ 0 | 普通相机通常为0 |

### 2.3 OptimizationFlags 设计

```cpp
struct OptimizationFlags {
    // 内参优化标记
    bool focal_length = false;
    bool principal_point_x = false;
    bool principal_point_y = false;
    bool aspect_ratio = false;
    bool skew = false;
    
    // 畸变参数优化标记
    bool k1 = false;
    bool k2 = false;
    bool k3 = false;
    bool k4 = false;
    bool p1 = false;
    bool p2 = false;
    bool b1 = false;
    bool b2 = false;
};
```

**使用场景**：

1. **简单参数化（广角镜头）**
   ```cpp
   flags.focal_length = true;      // 优化焦距
   flags.principal_point_x = true;
   flags.principal_point_y = true;
   flags.k1 = true;                // 仅优化k1
   // k2, k3, p1, p2 固定为0或初值
   ```

2. **完全参数化（精密测量）**
   ```cpp
   flags.focal_length = true;
   flags.principal_point_x = true;
   flags.principal_point_y = true;
   flags.k1 = true;
   flags.k2 = true;
   flags.k3 = true;
   flags.p1 = true;
   flags.p2 = true;
   // k4, b1, b2 通常不优化
   ```

3. **固定内参（已标定）**
   ```cpp
   // 所有 flags = false（默认）
   // 仅优化位姿
   ```

---

## 3. 方法设计

### 3.1 数据验证方法

#### `bool IsValid() const`

验证相机参数的逻辑一致性：

```cpp
bool CameraModel::IsValid() const {
    // 1. 分辨率检查
    if (width == 0 || height == 0) return false;
    
    // 2. 焦距检查（>0）
    if (focal_length <= 0.0) return false;
    
    // 3. 焦距合理范围（0.3~10x 宽度）
    double focal_ratio = focal_length / width;
    if (focal_ratio < 0.3 || focal_ratio > 10.0) return false;
    
    // 4. 主点在合理范围
    if (principal_point_x < -width || principal_point_x > width * 2.0)
        return false;
    if (principal_point_y < -height || principal_point_y > height * 2.0)
        return false;
    
    // 5. 宽高比合理
    if (aspect_ratio <= 0.0 || aspect_ratio > 5.0) return false;
    
    return true;
}
```

### 3.2 格式化方法

#### `std::string ToString() const`

返回详细的多行格式化字符串，包括：
- 相机类型和分辨率
- 传感器物理信息
- 内参和主点
- 畸变参数
- 元数据（制造商、型号等）
- 优化标记

#### `std::string GetSummary() const`

返回单行摘要，例如：
```
Camera [5472x3648] f=3516.5 (Canon EOS 5D Mark IV) {distorted}
```

### 3.3 其他方法

#### `bool HasDistortion() const`
检查是否有任何畸变参数非零。

#### `void Reset()`
重置所有参数为默认值（仅数据，不重置优化标记）。

---

## 4. 算法独立实现（camera_utils.h）

以下算法**不应**作为CameraModel的成员方法，而应在独立的工具模块中实现：

### 4.1 参数估计算法

```cpp
// src/camera/camera_utils.h

namespace insight {
namespace camera {

/**
 * 从35mm等效焦距估计内参初值
 * 
 * 原理：基于传感器大小和镜头规格估计焦距
 *   f_pixel = f_35mm * width / 36.0  (假设全画幅)
 */
bool EstimateFromEquivalentFocalLength(
    double f_35mm,
    uint32_t width, uint32_t height,
    double sensor_width_mm, double sensor_height_mm,
    double& out_focal_length);

/**
 * 从EXIF信息估计内参
 */
bool EstimateFromExif(
    const SimpleExifHeader& exif,
    const std::string& image_path,
    CameraModel& out_camera);

/**
 * 获取焦距的有效范围
 */
void GetFocalLengthRange(
    uint32_t width, uint32_t height,
    double& min_focal, double& max_focal);

}  // namespace camera
}  // namespace insight
```

### 4.2 BA优化支持算法

```cpp
/**
 * 构建Bundle Adjustment优化掩码
 */
std::vector<bool> BuildOptimizationMask(
    const CameraModel& camera);

/**
 * 获取需要优化的参数个数
 */
int GetOptimizationParameterCount(
    const OptimizationFlags& flags);

/**
 * 根据参数索引设置相机参数值
 */
void SetCameraParameter(
    CameraModel& camera,
    int param_idx,
    double value);

/**
 * 获取相机的参数向量
 */
std::vector<double> GetParameterVector(
    const CameraModel& camera);
```

### 4.3 几何计算算法

```cpp
/**
 * 应用Brown-Conrady畸变
 */
void ApplyDistortion(
    double u_norm, double v_norm,
    const CameraModel& camera,
    double& u_dist, double& v_dist);

/**
 * 移除Brown-Conrady畸变（迭代求解）
 */
void RemoveDistortion(
    double u_dist, double v_dist,
    const CameraModel& camera,
    double& u_norm, double& v_norm);

/**
 * 3D点投影到图像
 */
bool ProjectPoint3D(
    double x, double y, double z,
    const CameraModel& camera,
    double& u, double& v,
    bool apply_distortion = true);

/**
 * 归一化图像坐标反投影到3D
 */
void UnprojectNormalized(
    double u_norm, double v_norm,
    const CameraModel& camera,
    double depth,
    double& x, double& y, double& z);
```

---

## 5. 设计优势

### 5.1 灵活性

```cpp
// 相同的数据结构，不同的优化策略
CameraModel cam = LoadCameraModel("canon.xml");

// 策略1：只优化k1
cam.optimization_flags.k1 = true;
bundle_adjustment.AddCamera(cam);

// 策略2：完全优化
cam.optimization_flags.k1 = true;
cam.optimization_flags.k2 = true;
cam.optimization_flags.k3 = true;
cam.optimization_flags.p1 = true;
cam.optimization_flags.p2 = true;
bundle_adjustment.AddCamera(cam);

// 策略3：固定内参（只优化位姿）
// optimization_flags 全为false
bundle_adjustment.AddCamera(cam);
```

### 5.2 扩展性

添加新的相机模型无需修改已有的数据结构：

```cpp
// 新增 Rational多项式模型
namespace insight {
namespace camera {
    bool FitRationalModel(
        const std::vector<cv::Point2d>& observed,
        const std::vector<cv::Point3d>& object,
        CameraModel& camera);
}
}
```

### 5.3 可维护性

- 数据结构稳定，不频繁变更
- 算法模块独立演进
- 测试和调试更容易
- 版本管理更清晰

---

## 6. 使用示例

### 6.1 从EXIF创建相机

```cpp
#include "database_types.h"
#include "camera_utils.h"

CameraModel cam;
if (insight::camera::EstimateFromExif(exif, image_path, cam)) {
    cam.camera_name = "Canon EOS 5D Mark IV";
    
    // 配置优化策略
    cam.optimization_flags.focal_length = true;
    cam.optimization_flags.k1 = true;
    
    if (cam.IsValid()) {
        project.image_groups[0].ApplyCameraModel(cam, 
            ImageGroup::CameraMode::kGroupLevel);
    }
}
```

### 6.2 在BA中使用

```cpp
// 提取优化掩码
std::vector<bool> mask = 
    insight::camera::BuildOptimizationMask(camera_model);

// 执行BA
bundle_adjustment.SetCameraModel(camera_model);
bundle_adjustment.SetOptimizationMask(mask);
bundle_adjustment.Solve();
```

### 6.3 参数估计

```cpp
// 从35mm等效焦距估计
double focal_px;
if (insight::camera::EstimateFromEquivalentFocalLength(
        50.0,  // 35mm等效焦距
        5472, 3648,  // 分辨率
        35.0, 23.0,  // 传感器大小
        focal_px)) {
    camera_model.focal_length = focal_px;
    camera_model.principal_point_x = 5472 / 2.0;
    camera_model.principal_point_y = 3648 / 2.0;
}
```

---

## 7. 版本管理

### 7.1 Cereal序列化版本

```cpp
CEREAL_CLASS_VERSION(insight::database::OptimizationFlags, 1);
CEREAL_CLASS_VERSION(insight::database::CameraModel, 2);
// v2: 添加了 OptimizationFlags 字段
```

### 7.2 前向兼容性

新版本代码能加载旧版本数据（v1 CameraModel）：
- OptimizationFlags 使用默认值（所有为false）
- 其他字段保持一致

---

## 8. 参考资源

- **ISPRS Photogrammetry Standards**: OmegaPhiKappa 欧拉角定义
- **OpenCV Camera Model**: 参考实现
- **ContextCapture ImageGroup**: 分组设计灵感
- **Brown-Conrady Model**: 标准畸变模型参考

---

## 9. 设计时间线

| 版本 | 日期 | 描述 |
|-----|------|------|
| 1.0 | 2026-02-01 | 初始Pinhole模型 |
| 2.0 | 2026-02-08 | 完整Brown-Conrady模型 + 优化标记 + 数据/算法分离 |

---

## 附录：字段参考表

### 所有参数列表

| 组别 | 字段名 | 类型 | 默认值 | 单位 | 说明 |
|------|--------|------|--------|------|------|
| **Type** | type | enum | kBrownConrady | - | 相机模型类型 |
| **分辨率** | width | uint32_t | 0 | px | 图像宽度 |
| | height | uint32_t | 0 | px | 图像高度 |
| **传感器** | sensor_width_mm | double | 0.0 | mm | 传感器宽度 |
| | sensor_height_mm | double | 0.0 | mm | 传感器高度 |
| | pixel_size_um | double | 0.0 | µm | 像素大小 |
| | focal_length_35mm | double | 0.0 | mm | 35mm等效焦距 |
| **内参** | focal_length | double | 0.0 | px | 焦距 |
| | principal_point_x | double | 0.0 | px | 主点X |
| | principal_point_y | double | 0.0 | px | 主点Y |
| | aspect_ratio | double | 1.0 | - | 宽高比 |
| | skew | double | 0.0 | - | 偏斜 |
| **径向畸变** | k1 | double | 0.0 | - | 1阶 |
| | k2 | double | 0.0 | - | 2阶 |
| | k3 | double | 0.0 | - | 3阶 |
| | k4 | double | 0.0 | - | 4阶 |
| **切向畸变** | p1 | double | 0.0 | - | X方向 |
| | p2 | double | 0.0 | - | Y方向 |
| **棱镜畸变** | b1 | double | 0.0 | - | X分量 |
| | b2 | double | 0.0 | - | Y分量 |
| **元数据** | camera_name | string | "" | - | 相机名称 |
| | make | string | "" | - | 制造商 |
| | model | string | "" | - | 型号 |
| | lens_model | string | "" | - | 镜头型号 |
| | serial_number | uint32_t | 0 | - | 序列号 |
| **优化** | optimization_flags | struct | - | - | 优化配置 |

---

**END OF DOCUMENT**
