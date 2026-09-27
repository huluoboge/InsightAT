# ImageGroup 相机分组架构设计

## 概述

ImageGroup 是一个灵活的图像分组和相机参数管理框架，用于处理复杂的摄影测量场景。设计参考了 ContextCapture 的 ImageGroup 概念，提供两种相机参数管理模式。

## 核心问题

在现实的摄影测量项目中，存在以下常见情况：

1. **单相机多图像**（最常见）
   - 同一台相机拍摄多张照片
   - 所有图像共享相同的内参
   - 示例：无人机航拍、固定相机序列

2. **多相机混合**（复杂场景）
   - 多台不同的相机
   - 每台相机有自己的内参
   - 示例：多个固定摄像头、多个无人机

3. **相机漂移**（高精度要求）
   - 相同相机，但内参随时间漂移
   - 每个时间段（或每张图）需要不同的内参
   - 示例：长时间航拍、温度敏感环境

## 解决方案：两种模式

### 1. GroupLevel（组级模式）- 默认

**适用场景**：单相机或相同相机参数的图像集合

```cpp
ImageGroup::CameraMode = kGroupLevel

group_camera = CameraModel { ... }
images = [
    Image { id: 1, filename: "photo_001.jpg", camera: nullptr },
    Image { id: 2, filename: "photo_002.jpg", camera: nullptr },
    Image { id: 3, filename: "photo_003.jpg", camera: nullptr }
]
```

**特点**：
- 所有图像共享一个 `group_camera`
- 每个 Image 的 camera 字段为空
- 内存占用低：只存储一份相机参数
- 管理简洁：更新相机参数只需修改 group_camera

**应用示例**：
```cpp
// 创建分组
ImageGroup group;
group.group_id = 1;
group.group_name = "DJI Phantom 4 Flight";
group.camera_mode = ImageGroup::CameraMode::kGroupLevel;

// 设置组级相机参数
CameraModel drone_camera;
drone_camera.camera_name = "FC6310";
drone_camera.sensor_width = 5280;
drone_camera.sensor_height = 3956;
drone_camera.focal_length = 3648.0;  // 焦距（像素）
group.group_camera = drone_camera;

// 添加图像（不需要单独设置相机）
for (int i = 1; i <= 100; ++i) {
    Image img;
    img.image_id = i;
    img.filename = "photo_" + std::to_string(i) + ".jpg";
    group.AddImage(img);
}
```

### 2. ImageLevel（图像级模式）- 灵活

**适用场景**：混合相机、相机漂移等复杂情况

```cpp
ImageGroup::CameraMode = kImageLevel

group_camera = nullptr
images = [
    Image { id: 1, filename: "photo_001.jpg", camera: CameraModel { ... } },
    Image { id: 2, filename: "photo_002.jpg", camera: CameraModel { ... } },
    Image { id: 3, filename: "photo_003.jpg", camera: CameraModel { ... } }
]
```

**特点**：
- 每个图像有自己的 camera 参数
- group_camera 为空
- 内存占用高：存储多份相机参数
- 灵活性强：支持任意的相机参数组合

**应用示例**：
```cpp
// 创建分组
ImageGroup group;
group.group_id = 1;
group.group_name = "Multi-Camera Setup";
group.camera_mode = ImageGroup::CameraMode::kImageLevel;

// 相机1：佳能 EOS 5D Mark IV
CameraModel camera1;
camera1.camera_name = "Canon EOS 5D Mark IV";
camera1.focal_length = 3840.0;

// 相机2：尼康 Z6
CameraModel camera2;
camera2.camera_name = "Nikon Z6";
camera2.focal_length = 4256.0;

// 添加图像，每个图像指定自己的相机
Image img1; img1.image_id = 1; img1.camera = camera1;
Image img2; img2.image_id = 2; img2.camera = camera1;
Image img3; img3.image_id = 3; img3.camera = camera2;

group.AddImage(img1);
group.AddImage(img2);
group.AddImage(img3);
```

## 类型定义

### CameraModel - 相机参数

```cpp
struct CameraModel {
    enum class Type {
        kPinhole,           // 标准针孔模型（支持）
        kBrownConrady,      // 棕色-康拉迪畸变
        kFisheye,           // 鱼眼镜头
        kOther
    };

    // Pinhole 模型
    uint32_t sensor_width, sensor_height;   // 传感器分辨率
    double focal_length;                    // 焦距（像素）
    double principal_x, principal_y;        // 主点
    double pixel_size;                      // 像素大小（微米）

    // 畸变参数（棕色-康拉迪）
    double k1, k2, p1, p2, k3;             // 径向和切向畸变

    // 元数据
    std::string camera_name;                // 相机型号
    std::string make, model;                // 制造商和型号
};
```

### Image - 单张图像

```cpp
struct Image {
    uint32_t image_id;                      // 图像ID（在分组内唯一）
    std::string filename;                   // 图像文件名
    InputPose input_pose;                   // 初始位姿估计
    std::optional<CameraModel> camera;      // 图像级相机（仅在图像级模式时使用）
};
```

### ImageGroup - 图像分组

```cpp
struct ImageGroup {
    enum class CameraMode {
        kGroupLevel,        // 组级：所有图像共享一个相机
        kImageLevel         // 图像级：每个图像有自己的相机
    };

    uint32_t group_id;                      // 分组ID
    std::string group_name;                 // 分组名称
    CameraMode camera_mode;                 // 工作模式

    std::optional<CameraModel> group_camera;  // 组级相机（仅在组级模式时使用）
    std::vector<Image> images;              // 分组内的图像

    std::string description;                // 分组描述
    int64_t creation_time;                  // 创建时间戳
};
```

## API 接口

### 应用相机参数

```cpp
void ImageGroup::ApplyCameraModel(const CameraModel& camera, CameraMode mode);
```

- 在 GroupLevel 模式：设置 group_camera
- 在 ImageLevel 模式：为所有未设置相机的图像应用此参数

**使用示例**：
```cpp
CameraModel camera;
camera.camera_name = "DJI FC6310";
// ... 设置相机参数 ...

// 应用到分组（组级模式）
group.ApplyCameraModel(camera, ImageGroup::CameraMode::kGroupLevel);
```

### 获取相机参数

```cpp
const CameraModel* ImageGroup::GetCameraForImage(uint32_t image_id) const;
```

- 自动处理两种模式的差异
- GroupLevel：返回 group_camera
- ImageLevel：返回该图像自己的 camera

**使用示例**：
```cpp
const CameraModel* cam = group.GetCameraForImage(image_id);
if (cam) {
    std::cout << "焦距: " << cam->focal_length << " px\n";
}
```

### 添加图像

```cpp
bool ImageGroup::AddImage(const Image& image);
```

- 检查 image_id 重复
- 返回操作成功标志

**使用示例**：
```cpp
Image img;
img.image_id = 42;
img.filename = "photo.jpg";
if (group.AddImage(img)) {
    std::cout << "图像添加成功\n";
}
```

### 模式转换

#### 转换到图像级模式

```cpp
bool ImageGroup::ConvertToImageLevel();
```

- 将 group_camera 复制到所有图像
- 清空 group_camera
- 返回转换是否成功

**使用场景**：需要为不同图像设置不同的相机参数

```cpp
// 初始：组级模式，只有一个group_camera
ImageGroup group = CreateGroupLevelGroup();

// 转换为图像级模式
if (group.ConvertToImageLevel()) {
    // 现在可以修改各个图像的相机参数
    group.images[2].camera->focal_length = 4000.0;  // 修改第3张图的焦距
}
```

#### 转换到组级模式

```cpp
bool ImageGroup::ConvertToGroupLevel();
```

- 检查所有图像的相机参数是否相同
- 如果相同，将参数移到 group_camera
- 清空所有图像的 camera 字段
- 返回转换是否成功（失败时：参数不一致）

**使用场景**：如果所有图像实际上使用相同相机，转换为更高效的组级模式

```cpp
// 初始：图像级模式
ImageGroup group = CreateImageLevelGroup();

// 所有图像确认使用相同相机后，转换为组级模式
if (group.ConvertToGroupLevel()) {
    // 现在更高效地存储（只一份相机参数）
    std::cout << "转换成功，已优化内存占用\n";
} else {
    // 图像相机参数不一致，无法转换
    std::cout << "无法转换：图像使用不同相机\n";
}
```

## 设计优势

### 1. 灵活性

- 支持从简单到复杂的各种场景
- 可在两种模式间动态切换
- 无需重新加载数据

### 2. 效率

- GroupLevel 模式下内存占用最小
- GroupLevel 模式下参数更新快速
- ImageLevel 模式下支持复杂场景

### 3. 清晰的语义

```cpp
// 获取图像的相机参数，无需关心是哪种模式
const CameraModel* camera = group.GetCameraForImage(image_id);
```

### 4. 数据一致性

- `IsValid()` 检查模式和数据一致性
- 两种模式都有明确的验证规则
- 防止不一致的状态

### 5. 可序列化

- 完整的 Cereal 支持
- 版本控制（Version 1）
- 支持持久化存储和加载

## 实现细节

### 内存布局对比

**GroupLevel 模式**：
```
ImageGroup
├── group_id: uint32
├── group_name: string
├── camera_mode: enum (=kGroupLevel)
├── group_camera: CameraModel ★ 有值
├── images: vector<Image>
│   ├── Image { id: 1, camera: null }
│   ├── Image { id: 2, camera: null }
│   └── Image { id: 3, camera: null }
└── ...

总大小 = group_camera + images(3 * small_image)
```

**ImageLevel 模式**：
```
ImageGroup
├── group_id: uint32
├── group_name: string
├── camera_mode: enum (=kImageLevel)
├── group_camera: null
├── images: vector<Image>
│   ├── Image { id: 1, camera: CameraModel ★ }
│   ├── Image { id: 2, camera: CameraModel ★ }
│   └── Image { id: 3, camera: CameraModel ★ }
└── ...

总大小 = images(3 * (small_image + camera))
```

### 验证规则

**GroupLevel 模式**：
- ✓ group_camera 必须存在且有效
- ✓ 所有 Image.camera 必须为空
- ✓ 所有 image_id 唯一

**ImageLevel 模式**：
- ✓ group_camera 必须为空
- ✓ 所有 Image.camera 必须存在且有效
- ✓ 所有 image_id 唯一

## 使用场景

### 场景 1: 无人机航拍

```cpp
// DJI Phantom 4 采集 100 张图像
ImageGroup group;
group.group_name = "Site A - DJI P4";
group.camera_mode = ImageGroup::CameraMode::kGroupLevel;

CameraModel camera;
camera.camera_name = "FC6310";
camera.sensor_width = 5280;
camera.sensor_height = 3956;
camera.focal_length = 3648.0;
group.group_camera = camera;

// 快速添加 100 张图像，无需重复设置相机参数
for (int i = 1; i <= 100; ++i) {
    Image img;
    img.image_id = i;
    img.filename = "photo_" + std::to_string(i) + ".jpg";
    group.AddImage(img);
}
```

### 场景 2: 多相机混合采集

```cpp
// 三台不同的相机
ImageGroup group;
group.group_name = "Multi-camera Survey";
group.camera_mode = ImageGroup::CameraMode::kImageLevel;

// 相机 1: 佳能
CameraModel canon; canon.camera_name = "Canon 5D Mark IV"; ...
// 相机 2: 尼康
CameraModel nikon; nikon.camera_name = "Nikon Z6"; ...
// 相机 3: 索尼
CameraModel sony; sony.camera_name = "Sony A7R IV"; ...

// 添加图像，指定各自的相机
AddImageWithCamera(group, 1, "photo_1.jpg", canon);
AddImageWithCamera(group, 2, "photo_2.jpg", canon);
AddImageWithCamera(group, 3, "photo_3.jpg", nikon);
AddImageWithCamera(group, 4, "photo_4.jpg", sony);
```

### 场景 3: 相机漂移补偿

```cpp
// 初始：所有图像使用同一相机参数
ImageGroup group = CreateGroupLevelGroup();

// 长时间航拍后，检测到焦距漂移
// 转换到图像级模式以提供更精确的参数
group.ConvertToImageLevel();

// 为不同时间段的图像调整焦距
for (int i = 0; i < 50; ++i) {
    group.images[i].camera->focal_length = 3640.0;  // 开始时
}
for (int i = 50; i < 100; ++i) {
    group.images[i].camera->focal_length = 3650.0;  // 中期
}
for (int i = 100; i < 150; ++i) {
    group.images[i].camera->focal_length = 3660.0;  // 后期
}
```

## 代码统计

- **新增类**: 3 个（CameraModel, Image, ImageGroup）
- **新增方法**: 13 个
- **代码行数**: ~450 行（头文件 + 实现）
- **编译状态**: ✅ 成功（0 错误）

## 与现有架构的集成

```
Project
├── input_coordinate_system: CoordinateSystem
├── image_groups: vector<ImageGroup>  ★ 新增
│   └── ImageGroup
│       ├── images: vector<Image>
│       │   └── Image
│       │       ├── input_pose: InputPose
│       │       └── camera: CameraModel ★ 新增
│       └── group_camera: CameraModel ★ 新增
└── tasks: vector<ATTask>
    └── ATTask
        └── input_snapshot
            └── measurements: vector<Measurement>
```

## 扩展方向

### 1. 相机参数优化接口

```cpp
struct OptimizationResult {
    CameraModel optimized_camera;
    double reprojection_error;
};

OptimizationResult OptimizeCameraParameters(const ImageGroup& group);
```

### 2. 相机参数导入/导出

```cpp
bool ImportFromMetadata(ImageGroup& group, const std::string& metadata_file);
bool ExportCameraParameters(const ImageGroup& group, const std::string& output_file);
```

### 3. 相机数据库

```cpp
class CameraDatabase {
    CameraModel GetCamera(const std::string& name);
    std::vector<std::string> ListAvailableCameras();
};
```

### 4. 相机参数验证和校准

```cpp
ValidationReport ValidateCameraParameters(const ImageGroup& group);
CalibratedCamera CalibrateFromCheckboard(const std::vector<std::string>& calibration_images);
```

---

**创建日期**：2026-02-08  
**设计版本**：1.0  
**编译状态**：✅ 成功
