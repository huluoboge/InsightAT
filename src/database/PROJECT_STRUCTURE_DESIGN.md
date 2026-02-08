# Project 结构体设计文档

## 概述

`Project` 是 InsightAT 的顶层数据结构，代表一个完整的摄影测量项目。它包含：
- 项目元数据（名称、UUID、创建时间、作者等）
- 所有输入数据（坐标系、测量数据、图像分组）
- 初始位姿信息
- 一个或多个空三处理任务

## 核心设计原则

### 1. 层级关系

```
Project（项目）
├── 项目元数据
│   ├── name（项目名称）
│   ├── uuid（唯一标识）
│   ├── creation_time（创建时间）
│   ├── description（描述）
│   └── author（作者）
│
├── 输入数据（Inputs）
│   ├── input_coordinate_system（输入坐标系）
│   ├── measurements[]（测量数据）
│   └── image_groups[]（图像分组）
│
├── 初始化信息
│   └── initial_pose（初始位姿）
│
└── 处理结果（Outputs）
    └── at_tasks[]（空三任务）
```

### 2. 设计特点

✅ **完整性** - 包含项目从输入到输出的全部信息  
✅ **独立性** - Project 可以独立序列化/反序列化  
✅ **可扩展性** - 易于添加新的元数据字段  
✅ **可追踪性** - 完整的时间戳和版本控制  
✅ **灵活性** - 支持多个 ATTask（虽然通常只有一个）

## 结构体详解

```cpp
struct Project {
    // ── 项目元数据（基本信息）
    std::string name;                       ///< 项目名称
    std::string uuid;                       ///< 项目唯一标识符（UUID v4）
    int64_t creation_time = 0;              ///< 项目创建时间（Unix 时间戳，秒）
    std::string description;                ///< 项目描述
    std::string author;                     ///< 项目作者/操作员
    
    // ── 项目版本管理
    std::string project_version = "1.0";    ///< 项目文件格式版本
    int64_t last_modified_time = 0;         ///< 最后修改时间（Unix 时间戳，秒）
    
    // ── 项目标签（可选）
    std::vector<std::string> tags;          ///< 项目标签（用于分类和搜索）
    
    // ── 输入数据（Inputs）
    CoordinateSystem input_coordinate_system;  ///< 输入坐标系
    std::vector<Measurement> measurements;     ///< 所有输入测量数据（GNSS/IMU/GCP/SLAM）
    std::vector<ImageGroup> image_groups;      ///< 所有输入图像分组
    
    // ── 初始位姿（可选）
    std::optional<InputPose> initial_pose;     ///< 初始位姿估计（可选，用于初始化）
    
    // ── 空三任务（通常一个项目只有一个主任务）
    std::vector<ATTask> at_tasks;              ///< 空三处理任务列表
};
```

## 字段说明

### 项目元数据

| 字段 | 类型 | 说明 | 示例 |
|------|------|------|------|
| name | string | 项目名称，必填 | "2024年城市测量" |
| uuid | string | 项目UUID v4，唯一标识 | "550e8400-e29b-41d4-a716-446655440000" |
| creation_time | int64 | 创建时间戳（Unix 秒） | 1707398400 |
| description | string | 项目描述 | "高精度城市建筑三维测量项目" |
| author | string | 项目作者/操作员 | "张三" |
| project_version | string | 项目格式版本 | "1.0" |
| last_modified_time | int64 | 最后修改时间戳 | 1707484800 |
| tags | vector<string> | 项目标签 | ["urban", "buildings", "high-precision"] |

### 输入数据

#### input_coordinate_system
- **类型**：`CoordinateSystem`
- **说明**：定义所有输入数据（测量、位姿）所在的坐标系
- **支持**：EPSG、WKT、ENU、Local
- **示例**：
  ```cpp
  project.input_coordinate_system.type = CoordinateSystem::Type::kENU;
  project.input_coordinate_system.reference = {39.9045, 116.4074, 50.0};  // 北京
  ```

#### measurements
- **类型**：`std::vector<Measurement>`
- **说明**：所有输入测量数据（GNSS、IMU、GCP、SLAM）
- **内容**：
  - GNSS 测量：GPS/北斗 坐标和精度
  - IMU 测量：惯性测量（姿态、加速度、角速度）
  - GCP 测量：地面控制点
  - SLAM 测量：相机相对位姿
- **关联**：通过 `image_id` 关联到具体图像

#### image_groups
- **类型**：`std::vector<ImageGroup>`
- **说明**：所有输入图像分组（可以有多个）
- **支持模式**：
  - GroupLevel：单个分组内所有图像使用同一相机参数
  - ImageLevel：每个图像可以有不同的相机参数
- **示例**：
  ```cpp
  Project project;
  
  // 第一个分组：无人机图像（GroupLevel）
  ImageGroup drone_group;
  drone_group.group_id = 1;
  drone_group.group_name = "DJI Phantom 4";
  drone_group.camera_mode = ImageGroup::CameraMode::kGroupLevel;
  // ... 添加 100 张图像
  project.image_groups.push_back(drone_group);
  
  // 第二个分组：地面相机（ImageLevel）
  ImageGroup camera_group;
  camera_group.group_id = 2;
  camera_group.group_name = "Canon + Nikon";
  camera_group.camera_mode = ImageGroup::CameraMode::kImageLevel;
  // ... 添加混合相机图像
  project.image_groups.push_back(camera_group);
  ```

### 初始化信息

#### initial_pose
- **类型**：`std::optional<InputPose>`
- **说明**：可选的初始位姿估计（用于初始化空三计算）
- **用途**：
  - 为空三提供初始值，加速收敛
  - 在无 GNSS 数据时提供参考
- **示例**：
  ```cpp
  InputPose pose;
  pose.x = 2665000.0; pose.y = 4516500.0; pose.z = 3400.0;
  pose.has_position = true;
  pose.omega = 0.0; pose.phi = 0.0; pose.kappa = 90.0;
  pose.has_rotation = true;
  pose.angle_unit = InputPose::AngleUnit::kDegrees;
  
  project.initial_pose = pose;
  ```

### 处理结果

#### at_tasks
- **类型**：`std::vector<ATTask>`
- **说明**：空三处理任务列表
- **通常情况**：一个项目只有一个主 ATTask
- **可选用途**：
  - 实验性处理（多个参数组合）
  - 增量处理（多个批次的图像）
  - 参数优化（不同参数的对比）

## 核心方法

### 统计方法

#### GetTotalImageCount()
```cpp
size_t GetTotalImageCount() const;
```
获取项目中所有分组的总图像数。

**示例**：
```cpp
Project project = LoadProject("project.json");
size_t total = project.GetTotalImageCount();
std::cout << "Total images: " << total << std::endl;
```

#### GetMeasurementCountByType()
```cpp
size_t GetMeasurementCountByType(Measurement::Type type) const;
```
获取特定类型的测量数据个数。

**示例**：
```cpp
size_t gnss_count = project.GetMeasurementCountByType(Measurement::Type::kGNSS);
size_t imu_count = project.GetMeasurementCountByType(Measurement::Type::kIMU);
std::cout << "GNSS: " << gnss_count << ", IMU: " << imu_count << std::endl;
```

### 验证方法

#### IsValid()
```cpp
bool IsValid() const;
```
验证项目数据的完整性和一致性。

**检查项目**：
- 项目名称和 UUID 非空
- 作者非空
- 创建时间有效
- 输入坐标系有效
- 所有测量数据有效
- 所有图像分组有效
- 初始位姿有效（如果存在）

**示例**：
```cpp
if (project.IsValid()) {
    std::cout << "Project is valid" << std::endl;
} else {
    std::cout << "Project has validation errors" << std::endl;
}
```

### 查询方法

#### FindGroupByImageId()
```cpp
const ImageGroup* FindGroupByImageId(uint32_t image_id) const;
```
根据图像 ID 查找所属的分组。

**示例**：
```cpp
const ImageGroup* group = project.FindGroupByImageId(image_id=100);
if (group) {
    std::cout << "Image 100 in group: " << group->group_id << std::endl;
}
```

#### GetCameraForImageId()
```cpp
const CameraModel* GetCameraForImageId(uint32_t image_id) const;
```
根据图像 ID 查找对应的相机参数（自动搜索所有分组）。

**示例**：
```cpp
const CameraModel* camera = project.GetCameraForImageId(image_id=100);
if (camera) {
    std::cout << "Focal length: " << camera->focal_length << " px" << std::endl;
}
```

### 格式化方法

#### ToString()
```cpp
std::string ToString() const;
```
获取完整的项目描述（多行格式）。

**输出示例**：
```
Project {
  Name: 2024年城市测量
  UUID: 550e8400-e29b-41d4-a716-446655440000
  Author: 张三
  CreationTime: 1707398400 (Unix timestamp)
  LastModified: 1707484800 (Unix timestamp)
  Version: 1.0
  Description: 高精度城市建筑三维测量项目
  Tags: urban, buildings, high-precision
  InputCoordinateSystem: ENU (reference: 39.9045, 116.4074, 50.0)
  Measurements: 500
    - GNSS: 350
    - IMU: 100
    - GCP: 30
    - SLAM: 20
  ImageGroups: 2
  TotalImages: 250
    - Group 1: 100 images, mode=GroupLevel
    - Group 2: 150 images, mode=ImageLevel
  InitialPose: present
  ATTasks: 1
}
```

#### GetSummary()
```cpp
std::string GetSummary() const;
```
获取项目摘要（一到两行）。

**输出示例**：
```
2024年城市测量 (UUID: 550e8400...)
  Images: 250 | Measurements: 500 | Author: 张三
  GNSS: 350 | IMU: 100 | GCP: 30 | SLAM: 20
```

## 使用场景

### 场景 1：无人机航拍项目

```cpp
// 创建项目
Project project;
project.name = "DJI 城市航拍";
project.uuid = GenerateUUID();  // 生成UUID v4
project.creation_time = time(nullptr);
project.author = "李四";
project.description = "使用 DJI Phantom 4 对城市进行航拍测量";
project.tags = {"UAV", "DJI", "urban", "2024"};

// 设置输入坐标系
project.input_coordinate_system.type = CoordinateSystem::Type::kENU;
project.input_coordinate_system.reference = {39.9045, 116.4074, 50.0};

// 创建图像分组
ImageGroup group;
group.group_id = 1;
group.group_name = "DJI Phantom 4";
group.camera_mode = ImageGroup::CameraMode::kGroupLevel;

CameraModel camera;
camera.camera_name = "DJI Zenmuse X5S";
camera.focal_length = 3664.0;
camera.sensor_width = 5472;
camera.sensor_height = 3648;

group.ApplyCameraModel(camera, ImageGroup::CameraMode::kGroupLevel);

// 添加 200 张图像
for (int i = 1; i <= 200; ++i) {
    Image img;
    img.image_id = i;
    img.filename = "DJI_" + std::to_string(i) + ".jpg";
    img.input_pose.x = 2665000.0 + i * 10.0;  // 沿X方向移动
    img.input_pose.y = 4516500.0;
    img.input_pose.z = 3400.0;
    img.input_pose.has_position = true;
    group.AddImage(img);
}

project.image_groups.push_back(group);

// 添加 GNSS 测量数据
for (int i = 1; i <= 200; ++i) {
    Measurement meas;
    meas.type = Measurement::Type::kGNSS;
    meas.image_id = i;
    meas.timestamp = i * 1000;  // 毫秒
    meas.gnss = Measurement::GNSSMeasurement{
        2665000.0 + i * 10.0, 4516500.0, 3400.0, 2, 0.1, 0.1, 0.15
    };
    project.measurements.push_back(meas);
}

// 验证项目
if (project.IsValid()) {
    std::cout << project.ToString() << std::endl;
    
    // 查询：获取第 100 张图像的相机
    const CameraModel* cam = project.GetCameraForImageId(100);
    std::cout << "Focal length: " << cam->focal_length << std::endl;  // 3664.0
}
```

### 场景 2：多相机混合项目

```cpp
// 创建项目
Project project;
project.name = "建筑物多相机测量";
project.uuid = GenerateUUID();
project.creation_time = time(nullptr);
project.author = "王五";
project.description = "使用 Canon 和 Nikon 测量建筑物";
project.tags = {"buildings", "multi-camera", "architectural"};

// 设置坐标系
project.input_coordinate_system.type = CoordinateSystem::Type::kLocal;

// 第一个分组：Canon 相机（100 张图）
ImageGroup canon_group;
canon_group.group_id = 1;
canon_group.group_name = "Canon EOS 5D";
canon_group.camera_mode = ImageGroup::CameraMode::kGroupLevel;

CameraModel canon_camera;
canon_camera.camera_name = "Canon EOS 5D Mark IV";
canon_camera.focal_length = 3600.0;
canon_camera.sensor_width = 5760;
canon_camera.sensor_height = 3840;
canon_group.ApplyCameraModel(canon_camera, ImageGroup::CameraMode::kGroupLevel);

for (int i = 1; i <= 100; ++i) {
    Image img; img.image_id = i; img.filename = "canon_" + std::to_string(i) + ".jpg";
    canon_group.AddImage(img);
}
project.image_groups.push_back(canon_group);

// 第二个分组：Nikon 相机（100 张图）
ImageGroup nikon_group;
nikon_group.group_id = 2;
nikon_group.group_name = "Nikon D850";
nikon_group.camera_mode = ImageGroup::CameraMode::kGroupLevel;

CameraModel nikon_camera;
nikon_camera.camera_name = "Nikon D850";
nikon_camera.focal_length = 3700.0;
nikon_camera.sensor_width = 5760;
nikon_camera.sensor_height = 3840;
nikon_group.ApplyCameraModel(nikon_camera, ImageGroup::CameraMode::kGroupLevel);

for (int i = 101; i <= 200; ++i) {
    Image img; img.image_id = i; img.filename = "nikon_" + std::to_string(i) + ".jpg";
    nikon_group.AddImage(img);
}
project.image_groups.push_back(nikon_group);

// 设置初始位姿
InputPose initial;
initial.x = 0.0; initial.y = 0.0; initial.z = 0.0;
initial.has_position = true;
project.initial_pose = initial;

// 查询：项目摘要
std::cout << project.GetSummary() << std::endl;
```

### 场景 3：项目序列化

```cpp
// 保存项目
#include <cereal/archives/json.hpp>
#include <fstream>

void SaveProject(const Project& project, const std::string& filename) {
    std::ofstream out(filename);
    cereal::JSONOutputArchive ar(out);
    ar(project);
}

void LoadProject(Project& project, const std::string& filename) {
    std::ifstream in(filename);
    cereal::JSONInputArchive ar(in);
    ar(project);
}

// 使用
Project project = CreateProject(...);
SaveProject(project, "my_project.json");

Project loaded_project;
LoadProject(loaded_project, "my_project.json");
if (loaded_project.IsValid()) {
    std::cout << loaded_project.ToString() << std::endl;
}
```

## 数据验证

### 验证流程

```
Project
├── IsValid()
│   ├── 检查 name 非空 ✓
│   ├── 检查 uuid 非空 ✓
│   ├── 检查 author 非空 ✓
│   ├── 检查 creation_time > 0 ✓
│   ├── 检查 input_coordinate_system.IsValid() ✓
│   ├── 检查所有 measurements[].IsValid() ✓
│   ├── 检查所有 image_groups[].IsValid() ✓
│   └── 检查 initial_pose（如果存在）.IsValid() ✓
└── 返回 true 如果全部通过
```

### 常见验证错误

| 错误 | 原因 | 解决方案 |
|------|------|--------|
| Empty name | 项目名称未设置 | 设置 `project.name = "项目名"` |
| Empty UUID | UUID 未生成 | 使用 `GenerateUUID()` 生成 |
| Invalid creation_time | 时间戳为 0 或负数 | 使用 `time(nullptr)` 获取当前时间 |
| Invalid coordinate system | 坐标系不完整 | 设置完整的坐标系定义 |
| Invalid image group | 分组数据不一致 | 验证所有 images 都有相机参数 |

## 序列化支持

### Cereal 配置

```cpp
CEREAL_CLASS_VERSION(insight::database::Project, 1);
```

### 向前兼容性设计

项目支持多个序列化格式：
- **JSON**：人类可读，便于编辑和调试
- **XML**：结构清晰，适合配置管理
- **Binary**：高效存储，适合大型项目

**示例**：
```cpp
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>

// JSON 格式
{
    "name": "项目名称",
    "uuid": "550e8400-e29b-41d4-a716-446655440000",
    "creation_time": 1707398400,
    "author": "张三",
    "input_coordinate_system": { ... },
    "measurements": [ ... ],
    "image_groups": [ ... ],
    "at_tasks": [ ... ]
}
```

## 性能考虑

### 内存使用

```
Project 内存估算（1000 张图像）：
├── 基本字段：~500 bytes
├── measurements (1000)：~150 KB
├── image_groups (2)：~200 KB
│   └── images (1000)：~100 KB
└── at_tasks：~50 KB
─────────────────────────
总计：~500 KB
```

### 时间复杂度

| 操作 | 复杂度 | 说明 |
|------|--------|------|
| GetTotalImageCount() | O(m) | m = 分组数 |
| GetMeasurementCountByType() | O(n) | n = 测量数据数 |
| FindGroupByImageId() | O(m·k) | k = 每组平均图像数 |
| GetCameraForImageId() | O(m·k) | 两步查询 |
| IsValid() | O(n+m·k) | 全量验证 |

## 最佳实践

### 项目创建

```cpp
// ✅ 推荐
Project project;
project.name = "明确的项目名称";
project.uuid = GenerateUUID();  // 使用库函数
project.creation_time = time(nullptr);
project.author = "操作员名称";
project.description = "详细的项目描述";
project.tags = {"分类标签1", "分类标签2"};
```

### 数据添加

```cpp
// ✅ 推荐顺序
1. 设置坐标系
2. 添加图像分组和图像
3. 添加测量数据
4. （可选）设置初始位姿
5. 调用 IsValid() 验证

// ❌ 避免
- 添加图像但不设置相机参数
- 添加 GNSS 但不设置坐标系
- 在验证前修改数据
```

### 查询使用

```cpp
// ✅ 推荐
const CameraModel* cam = project.GetCameraForImageId(image_id);
if (cam) {
    // 使用相机参数
}

// ❌ 避免
// 直接访问 image_groups，容易出错
auto& group = project.image_groups[0];  // 假设索引
```

## 集成检查表

- ✅ 设置项目基本信息（名称、UUID、作者）
- ✅ 设置输入坐标系
- ✅ 添加图像分组和图像
- ✅ 添加测量数据
- ✅ （可选）设置初始位姿
- ✅ 调用 IsValid() 验证
- ✅ 序列化保存项目
- ✅ 反序列化加载项目
- ✅ 查询验证数据完整性

## 未来扩展

### 计划中的功能

1. **项目配置管理**
   ```cpp
   struct ProjectConfig {
       std::string processing_method;  // "bundle", "incremental"
       std::map<std::string, double> parameters;
       std::vector<std::string> disabled_features;
   };
   ```

2. **项目统计和报告**
   ```cpp
   struct ProjectStatistics {
       size_t total_images;
       size_t total_measurements;
       double coverage_ratio;
       std::vector<std::string> warnings;
   };
   ```

3. **增量处理支持**
   ```cpp
   struct ProjectIncrement {
       std::vector<ImageGroup> new_image_groups;
       std::vector<Measurement> new_measurements;
       int64_t timestamp;
   };
   ```

4. **项目模板**
   ```cpp
   Project CreateProjectFromTemplate(const std::string& template_name);
   ```

---

**版本**：1.0  
**日期**：2026-02-08  
**状态**：✅ 完成  
**编译**：✅ 成功  
