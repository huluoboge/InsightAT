# Feature Matching Module Design

> **版本**: 1.0  
> **创建日期**: 2026-02-12  
> **目标**: 定义 InsightAT 特征匹配模块的架构、数据格式和实现策略

---

## 1. 设计原则

### 1.1 单一职责原则

**匹配模块的唯一职责：执行特征匹配算法**

```
┌─────────────────┐
│ isat_retrieve   │ → .isat_pairs.json (决定配对策略)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ isat_match      │ → .isat_match (执行匹配算法)  ← 本模块
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│isat_build_graph │ → .isat_graph.json (几何验证)
└─────────────────┘
```

**职责边界：**
- ✅ 读取检索模块提供的图像对列表
- ✅ 调用 SiftGPU 执行匹配
- ✅ 应用 ratio test 过滤
- ✅ 输出 IDC 格式的匹配结果
- ❌ 不决定匹配哪些图像对（由 `isat_retrieve` 负责）
- ❌ 不执行几何验证（由 `isat_build_graph` 负责）

### 1.2 函数式设计

**核心匹配函数签名：**
```cpp
namespace insight::algorithm::matching {

struct MatchResult {
    std::vector<std::pair<uint16_t, uint16_t>> indices;  // 特征点索引对
    std::vector<Eigen::Vector4f> coords_pixel;           // 像素坐标 [x1,y1,x2,y2]
    std::vector<float> distances;                        // 匹配距离
    size_t num_matches;
};

// 纯函数：输入特征，输出匹配
MatchResult matchTwoImages(
    const std::string& feat1_path,
    const std::string& feat2_path,
    const MatchOptions& options
);

}  // namespace insight::algorithm::matching
```

### 1.3 GPU 资源管理策略

**问题：** SiftGPU 需要 OpenGL 上下文，多线程环境下如何管理？

**解决方案：** 单实例 + 异步任务队列

```cpp
// 主线程创建单个 SiftMatchGPU 实例
SiftMatchGPU matcher(max_sift_features);
matcher.VerifyContextGL();  // 验证 OpenGL 上下文

// 工作线程通过队列提交任务
TaskQueue<MatchTask> gpu_queue(1);  // 队列大小=1，串行GPU任务

// CPU 多线程加载特征 → GPU 单线程匹配 → CPU 多线程后处理
```

**参考实现：** `src/algorithm/tools/isat_extract.cpp:180-195`（GPU stage 单线程模式）

---

## 2. 数据格式定义

### 2.1 输入：.isat_pairs.json（检索模块输出）

```json
{
  "schema_version": "1.0",
  "retrieval_method": "exhaustive",
  "retrieval_params": {
    "max_distance_meters": 50.0,
    "min_overlap_ratio": 0.3
  },
  "pairs": [
    {
      "image1_id": "IMG_0001",
      "image2_id": "IMG_0002",
      "feature1_file": "features/IMG_0001.isat_feat",
      "feature2_file": "features/IMG_0002.isat_feat",
      "priority": 1.0
    }
  ]
}
```

### 2.2 输出：.isat_match（IDC 格式）

**Binary Payload 结构：**
```cpp
struct MatchBlobs {
    // Blob 1: 索引对（紧凑，用于 track 构建）
    uint16_t indices[N][2];        // shape: [N, 2], dtype: uint16
    
    // Blob 2: 像素坐标（用于几何计算）
    float coords_pixel[N][4];      // shape: [N, 4], dtype: float32
                                   // [x1_pixel, y1_pixel, x2_pixel, y2_pixel]
    
    // Blob 3: 匹配距离（质量评估）
    float distances[N];            // shape: [N], dtype: float32
};
```

**JSON 元数据示例：**
```json
{
  "schema_version": "1.0",
  "container_id": "550e8400-e29b-41d4-a716-446655440000",
  "task_type": "feature_matching",
  "algorithm": {
    "name": "SiftGPU",
    "version": "1.1",
    "parameters": {
      "ratio_test": 0.8,
      "max_matches": 10000
    }
  },
  "image_pair": {
    "image1_id": "IMG_0001",
    "image2_id": "IMG_0002",
    "image1_size": [3840, 2160],
    "image2_size": [3840, 2160]
  },
  "camera_intrinsics": {
    "camera1": {"fx": 3600, "fy": 3600, "cx": 1920, "cy": 1080, "confidence": "low"},
    "camera2": {"fx": 3600, "fy": 3600, "cx": 1920, "cy": 1080, "confidence": "low"}
  },
  "blobs": [
    {"name": "indices", "dtype": "uint16", "shape": [8532, 2], "offset": 0, "size": 34128},
    {"name": "coords_pixel", "dtype": "float32", "shape": [8532, 4], "offset": 34128, "size": 136512},
    {"name": "distances", "dtype": "float32", "shape": [8532], "offset": 170640, "size": 34128}
  ],
  "metadata": {
    "num_matches": 8532,
    "execution_time_ms": 125,
    "timestamp": "2026-02-12T10:30:00Z"
  }
}
```

**关键设计决策：**
1. **存储像素坐标而非归一化坐标**
   - 原因：初始内参可能不准确（标记为 `"confidence": "low"`）
   - F 矩阵估计直接使用像素坐标
   - E 矩阵估计时动态归一化

2. **使用 uint16 索引**
   - 假设：单张图像特征点数量 < 65536
   - 节省存储：4 bytes vs 8 bytes (uint32)

3. **存储匹配距离**
   - 用途：质量评估、异常值检测
   - 可选的阈值过滤

---

## 3. SiftGPU 集成

### 3.1 SiftMatchGPU API

**相关文件：** `third_party/SiftGPU/SiftMatch.cu`, `SiftGPU.h`

**核心函数：**
```cpp
// 1. 基本匹配（无几何约束）
int GetSiftMatch(
    int max_match,           // 最大匹配数量
    uint32_t match_buffer[], // 输出：匹配索引对 [2*num_matches]
    float distmax = 0.7,     // ratio test 阈值
    float ratiomax = 0.8,    // Lowe's ratio test
    int mutual_best_match = 1 // 双向最近邻
);

// 2. 引导式匹配（使用预计算的 F/H 矩阵）
int GetGuidedSiftMatch(
    int max_match,
    uint32_t match_buffer[],
    float H[3][3],           // Homography 矩阵（可选）
    float F[3][3],           // Fundamental 矩阵（可选）
    float distmax = 0.7,
    float ratiomax = 0.8,
    float hdistmax = 32,     // H 矩阵误差阈值
    float fdistmax = 16,     // F 矩阵误差阈值（Sampson 距离）
    int mutual_best_match = 1
);
```

**几何验证实现（GPU Shader）：**
- F 矩阵：Sampson 距离 `x2^T F x1 / sqrt(dot(temp, temp))`
- H 矩阵：重投影距离

**参考：** `SiftMatch.cu:785-850`（`MultiplyDescriptorG` 函数）

### 3.2 数据结构转换

**SiftGPU → InsightAT 数据流：**
```cpp
// 1. 读取 IDC 特征文件
struct FeatureData {
    std::vector<Eigen::Vector4f> keypoints;  // [x, y, scale, orientation]
    std::vector<uint8_t> descriptors;        // [N * 128]
};

// 2. 上传到 SiftGPU
matcher.SetDescriptors(0, num_feat1, descriptors1.data());
matcher.SetDescriptors(1, num_feat2, descriptors2.data());

// 3. 执行匹配
std::vector<uint32_t> match_buffer(max_matches * 2);
int num_matches = matcher.GetSiftMatch(
    max_matches, 
    match_buffer.data(),
    /*distmax=*/0.7,
    /*ratiomax=*/0.8
);

// 4. 转换为 InsightAT 格式
MatchResult result;
for (int i = 0; i < num_matches; ++i) {
    uint16_t idx1 = match_buffer[2*i];
    uint16_t idx2 = match_buffer[2*i+1];
    result.indices.push_back({idx1, idx2});
    
    // 提取像素坐标
    Eigen::Vector4f coords;
    coords << keypoints1[idx1].head<2>(), keypoints2[idx2].head<2>();
    result.coords_pixel.push_back(coords);
    
    // 计算描述子距离
    float dist = computeDescriptorDistance(
        &descriptors1[idx1 * 128],
        &descriptors2[idx2 * 128]
    );
    result.distances.push_back(dist);
}
```

---

## 4. 实现架构

### 4.1 模块文件结构

```
src/algorithm/modules/matching/
├── sift_matcher.h          # SiftGPU 封装
├── sift_matcher.cpp
├── match_types.h           # 数据结构定义
└── DESIGN.md               # 本文档

src/algorithm/io/
├── idc_reader.h            # IDC 读取器
├── idc_reader.cpp
├── idc_writer.h            # 已存在
└── idc_writer.cpp

src/algorithm/tools/
├── isat_match.cpp          # CLI 工具
└── CMakeLists.txt
```

### 4.2 核心类设计

**sift_matcher.h：**
```cpp
namespace insight::algorithm::matching {

struct MatchOptions {
    float ratio_test = 0.8f;
    float distance_max = 0.7f;
    int max_matches = -1;  // -1 表示无限制
    bool mutual_best_match = true;
};

class SiftMatcher {
public:
    explicit SiftMatcher(int max_features = 10000);
    ~SiftMatcher();
    
    // 禁止拷贝（GPU 资源管理）
    SiftMatcher(const SiftMatcher&) = delete;
    SiftMatcher& operator=(const SiftMatcher&) = delete;
    
    // 核心匹配函数
    MatchResult match(
        const FeatureData& features1,
        const FeatureData& features2,
        const MatchOptions& options = MatchOptions()
    );
    
    // 引导式匹配（使用预计算矩阵）
    MatchResult matchGuided(
        const FeatureData& features1,
        const FeatureData& features2,
        const Eigen::Matrix3f* F = nullptr,
        const Eigen::Matrix3f* H = nullptr,
        const MatchOptions& options = MatchOptions()
    );
    
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace insight::algorithm::matching
```

**idc_reader.h：**
```cpp
namespace insight::algorithm::io {

class IDCReader {
public:
    explicit IDCReader(const std::string& filepath);
    
    // 读取 JSON 元数据
    nlohmann::json getMetadata() const;
    
    // 读取指定 blob
    template <typename T>
    std::vector<T> readBlob(const std::string& blob_name);
    
    // 特征文件专用接口
    struct FeatureBlobs {
        std::vector<Eigen::Vector4f> keypoints;
        std::vector<uint8_t> descriptors;
    };
    FeatureBlobs readFeatures();
    
private:
    std::string filepath_;
    nlohmann::json metadata_;
    size_t payload_offset_;
};

}  // namespace insight::algorithm::io
```

### 4.3 CLI 工具流程

**isat_match.cpp 伪代码：**
```cpp
int main(int argc, char* argv[]) {
    // 1. 解析命令行参数
    CmdLine cmd("InsightAT Feature Matching Tool");
    std::string pairs_json, output_dir;
    float ratio_test = 0.8f;
    
    cmd.add(make_option('i', pairs_json, "input")
        .doc("Input pairs list (JSON, from isat_retrieve)"));
    cmd.add(make_option('o', output_dir, "output")
        .doc("Output directory for .isat_match files"));
    cmd.add(make_option('r', ratio_test, "ratio")
        .doc("Ratio test threshold (default: 0.8)"));
    
    // 2. 读取图像对列表
    auto pairs = loadPairsJSON(pairs_json);
    
    // 3. 初始化 GPU 匹配器（主线程）
    SiftMatcher matcher(10000);
    
    // 4. 多线程处理
    TaskQueue<MatchTask> load_queue(num_threads);   // CPU 加载
    TaskQueue<MatchTask> gpu_queue(1);              // GPU 匹配（单线程）
    TaskQueue<MatchTask> write_queue(num_threads);  // CPU 写入
    
    for (const auto& pair : pairs) {
        // Stage 1: 加载特征（并行）
        load_queue.push([&](int idx) {
            FeatureData feat1 = readFeatures(pair.feature1_file);
            FeatureData feat2 = readFeatures(pair.feature2_file);
            return {feat1, feat2, pair};
        });
        
        // Stage 2: GPU 匹配（串行）
        gpu_queue.push([&](auto task) {
            auto result = matcher.match(task.feat1, task.feat2, options);
            return {result, task.pair};
        });
        
        // Stage 3: 写入文件（并行）
        write_queue.push([&](auto task) {
            writeMatchIDC(task.result, task.pair, output_dir);
        });
    }
    
    return 0;
}
```

---

## 5. 性能考量

### 5.1 GPU 上传开销

**问题：** CPU → GPU 数据传输是主要瓶颈

**策略：**
1. **批量匹配**：一次上传多组特征，减少传输次数
2. **异步传输**：使用 OpenGL PBO（Pixel Buffer Object）
3. **特征缓存**：对于多次匹配的图像，缓存 GPU 描述子

**未来优化（Phase 3）：**
```cpp
class SiftMatcherCached {
    // GPU 端缓存已上传的描述子
    std::unordered_map<std::string, int> descriptor_cache_;
    
    MatchResult matchCached(
        const std::string& image1_id,
        const std::string& image2_id
    );
};
```

### 5.2 内存估算

**单张图像特征：**
- 10000 特征点 × (4 float keypoint + 128 uint8 descriptor)
- = 10000 × (16 + 128) = 1.44 MB

**单对匹配输出：**
- 8000 匹配 × (4 bytes 索引 + 16 bytes 坐标 + 4 bytes 距离)
- = 8000 × 24 = 192 KB

**批量处理内存（1000 对）：**
- 输入特征：~3 GB（假设 2000 张图像）
- 输出匹配：~192 MB
- GPU 显存：~2 GB（SiftGPU 内部缓冲）

---

## 6. 测试策略

### 6.1 单元测试

```cpp
TEST(SiftMatcher, BasicMatching) {
    // 1. 加载合成数据
    FeatureData feat1 = generateSyntheticFeatures(1000);
    FeatureData feat2 = transformFeatures(feat1, /*rotation=*/10°, /*translation=*/{100, 50});
    
    // 2. 执行匹配
    SiftMatcher matcher;
    auto result = matcher.match(feat1, feat2);
    
    // 3. 验证结果
    EXPECT_GT(result.num_matches, 800);  // 期望 80% 正确匹配
    EXPECT_LT(computeReprojectionError(result), 2.0);  // 重投影误差 < 2 像素
}
```

### 6.2 集成测试

```bash
# 1. 手动创建 pairs.json
echo '{
  "pairs": [
    {
      "image1_id": "IMG_0001",
      "image2_id": "IMG_0002",
      "feature1_file": "features/IMG_0001.isat_feat",
      "feature2_file": "features/IMG_0002.isat_feat"
    }
  ]
}' > test_pairs.json

# 2. 运行匹配
./isat_match -i test_pairs.json -o matches/ -r 0.8 -v

# 3. 验证输出
python3 scripts/validate_match.py matches/IMG_0001_IMG_0002.isat_match
```

---

## 7. 未来扩展

### 7.1 多种匹配器支持（策略模式）

```cpp
// 抽象接口
class FeatureMatcher {
public:
    virtual MatchResult match(const FeatureData&, const FeatureData&) = 0;
    virtual ~FeatureMatcher() = default;
};

// 具体实现
class SiftGPUMatcher : public FeatureMatcher { /*...*/ };
class FLANNMatcher : public FeatureMatcher { /*...*/ };
class BruteForceMatcher : public FeatureMatcher { /*...*/ };
```

### 7.2 深度学习匹配器

**集成 SuperGlue/LightGlue：**
- 输入：图像 + 关键点
- 输出：匹配置信度矩阵
- 挑战：PyTorch C++ API 集成

### 7.3 CUDA RANSAC 集成

**在匹配阶段直接验证几何：**
```cpp
MatchResult matchWithGeometry(
    const FeatureData& feat1,
    const FeatureData& feat2,
    bool enable_cuda_ransac = false
);
// 如果启用 CUDA RANSAC，返回已过滤的内点
```

---

## 8. 参考资料

- **SiftGPU 文档**: `third_party/SiftGPU/README.md`
- **SiftMatch CUDA 实现**: `third_party/SiftGPU/SiftMatch.cu`
- **IDC 格式规范**: `doc/design/08_functional_at_toolkit.md`
- **isat_extract 参考**: `src/algorithm/tools/isat_extract.cpp`

---

## 9. 实施检查清单

### Phase 1: 核心实现（当前）
- [ ] `modules/matching/match_types.h` - 数据结构定义
- [ ] `modules/matching/sift_matcher.h/cpp` - SiftGPU 封装
- [ ] `io/idc_reader.h/cpp` - IDC 读取器
- [ ] `tools/isat_match.cpp` - CLI 工具
- [ ] 单元测试：合成数据验证
- [ ] 集成测试：真实图像对匹配

### Phase 2: 优化与扩展
- [ ] GPU 描述子缓存
- [ ] 批量匹配优化
- [ ] 异步 GPU 传输
- [ ] 性能 benchmark

### Phase 3: 高级功能
- [ ] 引导式匹配（使用预计算 F 矩阵）
- [ ] 策略模式抽象
- [ ] CUDA RANSAC 集成

---

**最后更新**: 2026-02-12  
**审阅状态**: 待实现
