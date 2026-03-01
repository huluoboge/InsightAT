# GCP 集中化存储设计

**Date**: 2026-02-08  
**Status**: ✅ Implemented and Compiled  
**Compiler**: 0 errors, 0 warnings

---

## 1. 设计决策

### 问题回顾

初始设计中，GCPMeasurement 结构体被嵌入在 Measurement 联合类型中：

```cpp
// ❌ 旧设计：GCP散布在Measurement中
struct Measurement {
    enum Type { kGNSS, kIMU, kGCP, kSLAM };
    std::optional<GCPMeasurement> gcp;  // 数据冗余
};
```

**问题：**
1. **数据冗余** - 相同的GCP信息可能在多个Measurement中重复
2. **一致性困难** - 修改GCP时需要同步多个地方
3. **查询低效** - 获取某图像的所有GCP需要遍历所有Measurement并过滤
4. **设计混乱** - GCP本质上是"参考数据"，不是"观测数据"

### 新设计方案

采用 **Single Source of Truth + 懒加载索引** 策略：

```cpp
// ✅ 新设计：GCP集中存储
struct Project {
    // GCP 中心数据库（唯一来源）
    std::map<uint32_t, GCPMeasurement> gcp_database;
    
    // 缓存索引（按需构建）
    mutable std::map<uint32_t, std::vector<uint32_t>> image_to_gcp_cache;
    mutable bool cache_valid = false;
    
    // 查询方法
    std::vector<uint32_t> GetGCPsForImage(uint32_t image_id) const;
    const GCPMeasurement* GetGCP(uint32_t gcp_id) const;
    void InvalidateGCPCache() const;
    bool RebuildGCPCache() const;
};
```

---

## 2. 核心改动

### 2.1 GCPMeasurement 现在是独立顶级结构体

```cpp
// ✅ 独立定义（不再嵌套在Measurement内）
struct GCPMeasurement {
    struct Observation {
        uint32_t image_id;
        double pixel_x, pixel_y;
        double pixel_cov_x, pixel_cov_y;
    };
    
    uint32_t gcp_id;
    std::string gcp_name;
    
    double x, y, z;                    // 3D坐标
    double cov_xx, cov_yy, cov_zz;    // 坐标方差
    double cov_xy, cov_xz, cov_yz;    // 坐标协方差
    
    std::vector<Observation> observations;  // 所有图像观测
};
```

### 2.2 Measurement 结构体简化

```cpp
struct Measurement {
    enum Type { kGNSS, kIMU, kGCP, kSLAM };  // kGCP 标记为已弃用
    
    std::optional<GNSSMeasurement> gnss;
    std::optional<IMUMeasurement> imu;
    // ❌ 移除：std::optional<GCPMeasurement> gcp;
    std::optional<SLAMMeasurement> slam;
};
```

### 2.3 Project 中的 GCP 管理

```cpp
struct Project {
    // 核心数据存储
    std::map<uint32_t, GCPMeasurement> gcp_database;  // gcp_id -> GCPMeasurement
    
    // 缓存索引（懒加载）
    mutable std::map<uint32_t, std::vector<uint32_t>> image_to_gcp_cache;
    mutable bool cache_valid = false;
};
```

---

## 3. API 接口

### 3.1 获取指定图像的所有GCP

```cpp
std::vector<uint32_t> Project::GetGCPsForImage(uint32_t image_id) const;
```

**使用场景：**
```cpp
// 处理图像时获得该图像的所有GCP观测
Project project = load_project();
std::vector<uint32_t> gcp_ids = project.GetGCPsForImage(10);

for (uint32_t gcp_id : gcp_ids) {
    const auto* gcp = project.GetGCP(gcp_id);
    // 处理GCP约束
}
```

**实现细节：**
- 首次调用时自动触发缓存构建 (RebuildGCPCache)
- 后续调用直接返回缓存结果，O(1) 查询
- 若 cache_valid = false，重新构建

### 3.2 获取指定GCP的完整信息

```cpp
const GCPMeasurement* Project::GetGCP(uint32_t gcp_id) const;
```

**使用场景：**
```cpp
const auto* gcp = project.GetGCP(42);
if (gcp && gcp->IsValid()) {
    // 使用GCP的3D坐标和所有观测
    std::cout << "GCP: " << gcp->gcp_name << " @ (" 
              << gcp->x << ", " << gcp->y << ", " << gcp->z << ")\n";
    
    // 访问所有观测
    for (const auto& obs : gcp->observations) {
        std::cout << "  Observed in Image " << obs.image_id 
                  << " @ (" << obs.pixel_x << ", " << obs.pixel_y << ")\n";
    }
}
```

### 3.3 缓存失效和重建

```cpp
// 修改GCP数据库后调用此方法
void Project::InvalidateGCPCache() const {
    cache_valid = false;
    image_to_gcp_cache.clear();
}

// 手动重建缓存
bool Project::RebuildGCPCache() const;
```

**使用场景：**
```cpp
// 添加或修改GCP
project.gcp_database[100] = new_gcp_measurement;

// 使缓存失效
project.InvalidateGCPCache();

// 下次调用GetGCPsForImage时会自动重建缓存
auto gcp_ids = project.GetGCPsForImage(10);  // 自动重建缓存
```

---

## 4. 优势分析

### 4.1 Single Source of Truth

| 操作 | 优势 |
|------|------|
| **修改GCP** | 只需修改一处，全局生效 |
| **删除GCP** | 自动从所有图像观测中消失 |
| **验证一致性** | 无需跨表同步检查 |

### 4.2 性能优化

| 场景 | 性能 |
|------|------|
| **获取图像的GCP** | O(1) - 缓存查询 |
| **首次构建缓存** | O(n*m) - n个GCP, m个观测/GCP |
| **后续查询** | O(1) - 缓存直接返回 |
| **内存占用** | 优化 - 无数据冗余 |

### 4.3 代码清晰度

```cpp
// ✅ 清晰的语义
project.gcp_database[42];           // GCP中心数据库
project.GetGCPsForImage(10);        // 图像观测到的GCPs
project.GetGCP(42);                 // 获取GCP信息
project.InvalidateGCPCache();       // 管理缓存
```

vs

```cpp
// ❌ 混乱的语义
for (auto& m : measurements) {
    if (m.type == Type::kGCP) {
        // 在Measurement中搜索GCP？
    }
}
```

---

## 5. 缓存机制详解

### 5.1 懒加载策略

```
客户端调用 GetGCPsForImage(10)
    ↓
检查 cache_valid?
    ├─ 否 → RebuildGCPCache()
    │       ├─ 遍历所有GCP及其observations
    │       ├─ 构建 image_id -> [gcp_ids] 映射
    │       └─ 设置 cache_valid = true
    │
└─ 是 → 直接返回缓存结果 O(1)
```

### 5.2 缓存重建实现

```cpp
bool Project::RebuildGCPCache() const {
    image_to_gcp_cache.clear();
    
    // 遍历所有GCP
    for (const auto& [gcp_id, gcp_meas] : gcp_database) {
        // 遍历每个观测
        for (const auto& obs : gcp_meas.observations) {
            // 建立image_id -> gcp_id映射
            image_to_gcp_cache[obs.image_id].push_back(gcp_id);
        }
    }
    
    cache_valid = true;
    return true;
}
```

### 5.3 缓存失效触发

任何修改GCP数据库的操作后：

```cpp
project.gcp_database[42] = new_gcp;
project.InvalidateGCPCache();  // 重要！

// 或者自动失效（如果使用引用修改）
project.gcp_database.at(42).observations.push_back(new_obs);
project.InvalidateGCPCache();  // 必须手动调用
```

---

## 6. 版本管理

### 6.1 Cereal 版本变更

```cpp
// 新增：独立的GCPMeasurement
CEREAL_CLASS_VERSION(insight::database::GCPMeasurement, 1);
CEREAL_CLASS_VERSION(insight::database::GCPMeasurement::Observation, 1);

// 修改：Measurement 不再包含GCP
CEREAL_CLASS_VERSION(insight::database::Measurement, 1);  // 保持v1，只移除字段

// 修改：Project 增加gcp_database
CEREAL_CLASS_VERSION(insight::database::Project, 2);  // 升级到v2
```

### 6.2 序列化更新

**Project 序列化：**
```cpp
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    // ... 基础字段 ...
    
    ar(CEREAL_NVP(measurements), CEREAL_NVP(image_groups));
    
    if (version > 1) {
        ar(CEREAL_NVP(gcp_database));  // v2新增
    }
    
    ar(CEREAL_NVP(at_tasks));
}
```

---

## 7. 迁移路径

### 7.1 从旧设计迁移

**旧文件格式：**
```cpp
Measurement {
    type: kGCP
    gcp: { gcp_id: 42, x: 100, observations: [...] }
}
```

**新文件格式：**
```cpp
Project {
    gcp_database: {
        42: { gcp_id: 42, x: 100, observations: [...] }
    }
}
```

**迁移代码示例：**
```cpp
// 加载旧数据时的转换
for (auto& meas : old_measurements) {
    if (meas.type == Type::kGCP) {
        project.gcp_database[meas.gcp->gcp_id] = *meas.gcp;
    }
}

// 现有代码无需改动，使用Project的新API
auto gcps = project.GetGCPsForImage(image_id);
```

---

## 8. 设计对比

### 8.1 旧设计 vs 新设计

| 维度 | 旧设计 | 新设计 |
|------|--------|--------|
| **存储位置** | Measurement[]中 | Project.gcp_database中 |
| **数据冗余** | ❌ 高（多Measurement重复） | ✅ 无（单一存储） |
| **查询图像GCP** | ❌ O(n*m)遍历 | ✅ O(1)缓存查询 |
| **修改GCP** | ❌ 需同步多处 | ✅ 修改一处 |
| **缓存管理** | ❌ 无 | ✅ 懒加载+失效机制 |
| **内存效率** | ❌ 低 | ✅ 高 |
| **代码清晰度** | ❌ 混乱 | ✅ 清晰 |

---

## 9. 编译验证

```
✅ Compilation: SUCCESS
  - 0 errors
  - 0 warnings (database_types related)
  
Modified Files:
  - src/database/database_types.h (lines adjusted)
    * GCPMeasurement 移出 Measurement
    * Project 添加 gcp_database 和缓存
    * 版本号升级 (Project v1→v2)
  
  - src/database/database_types.cpp
    * GCPMeasurement::IsValid() 独立实现
    * Measurement::ToString/IsValid 更新
    * Project GCP 方法实现：
      - GetGCPsForImage()
      - GetGCP()
      - InvalidateGCPCache()
      - RebuildGCPCache()
```

---

## 10. 后续使用指南

### 10.1 创建和管理GCP

```cpp
// 1. 创建GCP测量
GCPMeasurement gcp;
gcp.gcp_id = 42;
gcp.gcp_name = "Corner A";
gcp.x = 1000.0;
gcp.y = 2000.0;
gcp.z = 100.0;

// 2. 添加观测
gcp.observations.push_back({
    image_id: 10,
    pixel_x: 123.45,
    pixel_y: 567.89
});

gcp.observations.push_back({
    image_id: 20,
    pixel_x: 234.56,
    pixel_y: 456.78
});

// 3. 存储到Project
project.gcp_database[42] = gcp;
project.InvalidateGCPCache();
```

### 10.2 查询GCP

```cpp
// 获取图像10的所有GCP
auto gcp_ids = project.GetGCPsForImage(10);

for (uint32_t gcp_id : gcp_ids) {
    const auto* gcp = project.GetGCP(gcp_id);
    if (gcp) {
        // 使用GCP数据
    }
}
```

### 10.3 Bundle Adjustment 集成

```cpp
// BA中的GCP约束残差计算
for (uint32_t image_id = 0; image_id < num_images; ++image_id) {
    // 获取该图像的所有GCP
    auto gcp_ids = project.GetGCPsForImage(image_id);
    
    for (uint32_t gcp_id : gcp_ids) {
        const auto* gcp = project.GetGCP(gcp_id);
        
        // 计算该GCP在该图像中的观测
        for (const auto& obs : gcp->observations) {
            if (obs.image_id == image_id) {
                // 计算投影误差
                double residual = compute_reprojection_error(
                    pose[image_id], gcp->x/y/z,
                    obs.pixel_x/y
                );
                
                // 应用可选的像素权重
                double weight = 1.0 / (1.0 + obs.pixel_cov_x * obs.pixel_cov_y);
                cost += weight * residual * residual;
            }
        }
    }
}
```

---

## 11. 总结

**核心设计原则：**
- ✅ **Single Source of Truth** - GCP 在 Project 中集中存储
- ✅ **Lazy Caching** - 索引按需构建，性能高效
- ✅ **Explicit Invalidation** - 缓存失效清晰可控
- ✅ **Semantic Clarity** - API 语义明确，易于理解
- ✅ **Bundle Adjustment Ready** - 直接支持BA算法

这个设计确保数据一致性，同时保持高效的查询性能，是"实际应用优先"的务实方案。
