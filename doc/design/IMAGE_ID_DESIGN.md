# 图像ID设计更新说明

## 设计问题

**原设计**：使用文件名作为特征文件标识
```
IMG_1234.jpg → IMG_1234.isat_feat
IMG_5678.jpg → IMG_5678.isat_feat
```

**问题**：
- ❌ 文件名可能重复（不同目录下同名文件）
- ❌ 文件名可能被重命名，导致特征文件失效
- ❌ 文件名不具备唯一性保证

## 新设计

**使用image_id作为唯一标识符**：
```json
{
  "images": [
    {"id": 1, "path": "/path/to/IMG_1234.jpg"},
    {"id": 2, "path": "/path/to/IMG_1234.jpg"}  // 同名文件不冲突
  ]
}
```

特征文件命名：
```
1.isat_feat  ← 基于ID
2.isat_feat
```

**优势**：
- ✅ ID全局唯一（数据库自增ID）
- ✅ 不受文件名/路径影响
- ✅ 支持GNSS/IMU等元数据
- ✅ 便于数据库关联管理

---

## 新图像列表格式 (v2.0)

### 必需字段
```json
{
  "images": [
    {
      "id": 1,                            // 必需：唯一图像ID（整数）
      "path": "/absolute/path/to/image.jpg" // 必需：图像文件路径
    }
  ]
}
```

### 完整字段（含GNSS/IMU）
```json
{
  "images": [
    {
      "id": 1,
      "path": "/path/to/DSC_0001.JPG",
      "camera_id": 1,                     // 可选：相机ID（默认1）
      
      "gnss": {                           // 可选：GNSS数据
        "x": 500123.456,                  // 东向（米）
        "y": 4500789.012,                 // 北向（米）
        "z": 125.5,                       // 高程（米）
        "cov_xx": 0.1,                    // 协方差（米²）
        "cov_yy": 0.1,
        "cov_zz": 0.2,
        "cov_xy": 0.0,
        "cov_xz": 0.0,
        "cov_yz": 0.0,
        "num_satellites": 12,             // 卫星数量
        "hdop": 0.8,                      // 水平精度因子
        "vdop": 1.2                       // 垂直精度因子
      },
      
      "imu": {                            // 可选：IMU数据（角度单位：度）
        "roll": 0.5,                      // 横滚角（度）
        "pitch": -1.2,                    // 俯仰角（度）
        "yaw": 45.3,                      // 航向角（度）
        "cov_att_xx": 0.1,                // 姿态协方差（弧度²）
        "cov_att_yy": 0.1,
        "cov_att_zz": 0.1
      }
    }
  ],
  
  "metadata": {                           // 可选：全局元数据
    "format_version": "2.0",
    "coordinate_system": "EPSG:32649",    // 坐标系统
    "angle_unit": "degrees"               // IMU角度单位
  }
}
```

---

## 工作流更新

### 1. 特征提取（isat_extract）

**新命令**（基于ID）：
```bash
./isat_extract \
  -i image_list_v2.json \      # JSON中包含ID
  -o features/ \               # 输出: 1.isat_feat, 2.isat_feat, ...
  --output-retrieval features-retrieval/ \
  --nfeatures 10000 \
  --nfeatures-retrieval 1500 \
  --resize-retrieval 1024 \
  --uint8 --nms -v
```

**输出特征文件**：
```
features/1.isat_feat          ← 匹配特征
features/2.isat_feat
features-retrieval/1.isat_feat ← 检索特征
features-retrieval/2.isat_feat
```

### 2. 检索（isat_retrieve）

```bash
./isat_retrieve \
  -f features-retrieval/ \
  -i image_list_v2.json \      # 读取ID和GNSS/IMU
  -o pairs.json \
  -s gps+vlad \                # 空间+视觉检索
  --vlad-codebook codebook.vcbt \
  --distance-threshold 200 \
  --max-neighbors 50 -v
```

**输出pairs.json**：
```json
{
  "pairs": [
    {
      "image1_id": "1",         ← 基于ID
      "image2_id": "2",
      "feature1_file": "features-retrieval/1.isat_feat",
      "feature2_file": "features-retrieval/2.isat_feat",
      "score": 0.85,
      "spatial_distance": 15.3
    }
  ]
}
```

### 3. 匹配（isat_match）

```bash
./isat_match \
  -p pairs.json \              # Pair列表（含image_id）
  -o matches/ \
  -t mutual \
  --ratio 0.75 -v
```

输出：
```
matches/1-2.isat_matches      ← 基于ID的配对
matches/1-5.isat_matches
```

---

## 迁移指南

### 从旧格式（v1.0）迁移到新格式（v2.0）

**旧格式**：
```json
{
  "images": [
    {"path": "/path/to/IMG_1234.jpg", "camera_id": 1}
  ]
}
```

**新格式**：
```json
{
  "images": [
    {"id": 1, "path": "/path/to/IMG_1234.jpg", "camera_id": 1}
  ]
}
```

**Python迁移脚本示例**：
```python
import json

# 读取旧格式
with open('old_image_list.json') as f:
    old_data = json.load(f)

# 转换为新格式（分配ID）
new_data = {"images": []}
for idx, img in enumerate(old_data['images'], start=1):
    new_img = {
        "id": idx,
        "path": img['path'],
        "camera_id": img.get('camera_id', 1)
    }
    
    # 可选：添加GNSS/IMU数据
    if 'gnss' in img:
        new_img['gnss'] = img['gnss']
    if 'imu' in img:
        new_img['imu'] = img['imu']
    
    new_data['images'].append(new_img)

# 保存新格式
with open('image_list_v2.json', 'w') as f:
    json.dump(new_data, f, indent=2)

print(f"Migrated {len(new_data['images'])} images")
```

---

## 数据库集成建议

### InsightAT数据库schema
```sql
CREATE TABLE images (
  id INTEGER PRIMARY KEY AUTOINCREMENT,  -- 图像ID（自增）
  path TEXT NOT NULL,                    -- 图像路径
  camera_id INTEGER DEFAULT 1,
  
  -- GNSS数据
  gnss_x REAL,
  gnss_y REAL,
  gnss_z REAL,
  gnss_cov_xx REAL,
  gnss_cov_yy REAL,
  gnss_cov_zz REAL,
  
  -- IMU数据
  imu_roll REAL,
  imu_pitch REAL,
  imu_yaw REAL,
  
  -- 特征文件路径（自动生成）
  feature_file TEXT GENERATED ALWAYS AS (id || '.isat_feat') VIRTUAL
);
```

### 导出JSON接口
```python
def export_image_list(db_connection):
    """从数据库导出v2.0格式JSON"""
    cursor = db_connection.execute("""
        SELECT id, path, camera_id, 
               gnss_x, gnss_y, gnss_z, gnss_cov_xx, gnss_cov_yy, gnss_cov_zz,
               imu_roll, imu_pitch, imu_yaw
        FROM images ORDER BY id
    """)
    
    images = []
    for row in cursor:
        img = {"id": row[0], "path": row[1], "camera_id": row[2]}
        
        if row[3] is not None:  # Has GNSS
            img["gnss"] = {
                "x": row[3], "y": row[4], "z": row[5],
                "cov_xx": row[6], "cov_yy": row[7], "cov_zz": row[8]
            }
        
        if row[9] is not None:  # Has IMU
            img["imu"] = {"roll": row[9], "pitch": row[10], "yaw": row[11]}
        
        images.append(img)
    
    return {"images": images}
```

---

## FAQ

**Q: 旧的特征文件还能用吗？**  
A: 可以，但需要手动重命名：`IMG_1234.isat_feat` → `1.isat_feat`（根据数据库ID）

**Q: 如何批量处理已有数据？**  
A: 使用迁移脚本为现有图像分配ID，然后重命名特征文件

**Q: ID必须从1开始吗？**  
A: 不需要，任何正整数都可以，只要保证唯一性

**Q: 相同文件路径可以有不同ID吗？**  
A: 可以！这正是新设计的优势：支持重复成像、多次处理等场景

**Q: GNSS/IMU数据是必需的吗？**  
A: 不是，只有`id`和`path`是必需的。GNSS/IMU仅在使用GPS检索或IMU约束时需要

---

## 兼容性

- ✅ `isat_extract`: 支持v2.0格式（读取`id`字段）
- ✅ `isat_retrieve`: 支持v2.0格式（读取`id` + GNSS/IMU）
- ⏳ `isat_match`: 需要更新以支持基于ID的pair文件
- ⏳ `isat_train_vlad`: 无需更改（直接读取特征文件）
- ⏳ GUI: 需要更新以支持新格式

**后续工作**：
1. 更新`isat_match`支持新的pair格式
2. 更新GUI程序支持v2.0 JSON格式
3. 添加格式验证工具
4. 更新所有文档和示例

---

## 总结

**核心改进**：
- 从**文件名依赖** → **ID驱动**设计
- 增强数据完整性和可追溯性
- 支持更复杂的检索策略（GPS、IMU、视觉）
- 为数据库集成奠定基础

**示例文件位置**：
- 格式示例：`data/image_list_format_v2.json`
- 迁移脚本：待添加到 `tools/migrate_v1_to_v2.py`
