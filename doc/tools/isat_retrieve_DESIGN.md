# Image Retrieval System Design (isat_retrieve)

> **版本**: 2.0  
> **日期**: 2026-02-13  
> **架构风格**: 函数式编程 + 泛型 + 策略函数派发

---

## 1. 设计原则

### 1.1 函数式编程优先
- **纯函数**: 所有检索算法实现为无状态纯函数
- **不变性**: 输入数据只读，返回新的结果对象
- **组合性**: 通过函数组合构建复杂策略
- **避免继承**: 使用泛型、模板、std::function 替代虚函数

### 1.2 类型安全与泛型
```cpp
// 策略函数签名
using RetrievalFunction = std::function<
    std::vector<ImagePair>(
        const std::vector<ImageInfo>&,  // 输入图像列表
        const RetrievalOptions&         // 配置参数
    )
>;

// 泛型过滤器
template <typename Pred>
std::vector<ImagePair> filterPairs(
    const std::vector<ImagePair>& pairs,
    Pred predicate
);
```

### 1.3 数据结构
```cpp
struct ImageInfo {
    uint32_t image_id = 0;   // 数字 ID（与 project/database 一致）；文件名用 to_string(id)
    std::string image_path;
    std::string feature_file;  // 如 feature_dir + "/" + to_string(image_id) + ".isat_feat"
    int camera_id = 1;
    
    // 可选的 GNSS/IMU 数据
    std::optional<GNSSData> gnss;
    std::optional<IMUData> imu;
};

struct GNSSData {
    double x, y, z;                  // 位置（投影坐标）
    double cov_xx, cov_yy, cov_zz;  // 协方差
    uint8_t num_satellites = 0;
};

struct IMUData {
    double roll, pitch, yaw;         // 姿态角（弧度）
    double cov_att_xx, cov_att_yy, cov_att_zz;
};

struct ImagePair {
    int image1_idx;
    int image2_idx;
    double score;                    // 相似度分数 [0, 1]
    std::string method;              // "gps" | "vlad" | "vocab_tree"
    
    // 可选的元数据
    std::optional<double> spatial_distance;
    std::optional<double> visual_similarity;
};

struct RetrievalOptions {
    // GPS 检索参数
    double distance_threshold = 200.0;  // 米
    double angle_threshold = 45.0;      // 度
    int max_neighbors = 50;
    
    // VLAD 参数
    int vlad_clusters = 64;
    int top_k = 20;
    
    // 词汇树参数
    std::string vocab_file;
    
    // 通用参数
    int max_pairs = -1;  // -1 = 无限制
    bool verbose = false;
};
```

---

## 2. 检索策略函数

### 2.1 GPS 空间检索
```cpp
// 纯函数：GPS 引导的空间索引检索
std::vector<ImagePair> retrieveByGPS(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
) {
    // 1. 过滤有 GNSS 数据的图像
    auto valid_images = filterImagesWithGNSS(images);
    
    // 2. 构建 k-d tree (nanoflann)
    auto kdtree = buildSpatialIndex(valid_images);
    
    // 3. 查询邻近图像
    std::vector<ImagePair> pairs;
    for (size_t i = 0; i < valid_images.size(); ++i) {
        auto neighbors = kdtree.radiusSearch(
            valid_images[i].gnss->position(),
            options.distance_threshold
        );
        
        for (auto j : neighbors) {
            if (i >= j) continue;  // 避免重复
            
            // 可选：检查姿态角度
            if (options.angle_threshold > 0 && 
                !checkAngleSimilarity(valid_images[i], valid_images[j], options)) {
                continue;
            }
            
            pairs.push_back({
                .image1_idx = static_cast<int>(i),
                .image2_idx = static_cast<int>(j),
                .score = computeSpatialScore(valid_images[i], valid_images[j]),
                .method = "gps",
                .spatial_distance = euclideanDistance(
                    valid_images[i].gnss->position(),
                    valid_images[j].gnss->position()
                )
            });
        }
    }
    
    return pairs;
}
```

### 2.2 VLAD 编码检索
```cpp
// 纯函数：VLAD 编码 + FAISS 检索
std::vector<ImagePair> retrieveByVLAD(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
) {
    // 1. 加载或训练 k-means 聚类中心
    auto clusters = loadOrTrainVLADClusters(images, options.vlad_clusters);
    
    // 2. 编码所有图像为 VLAD 向量
    std::vector<VLADDescriptor> vlad_vectors;
    for (const auto& img : images) {
        vlad_vectors.push_back(encodeVLAD(img.feature_file, clusters));
    }
    
    // 3. 构建 FAISS 索引
    auto index = buildFAISSIndex(vlad_vectors);
    
    // 4. 查询 top-K 相似图像
    std::vector<ImagePair> pairs;
    for (size_t i = 0; i < images.size(); ++i) {
        auto neighbors = index.search(vlad_vectors[i], options.top_k);
        
        for (const auto& [j, distance] : neighbors) {
            if (i >= static_cast<size_t>(j)) continue;
            
            pairs.push_back({
                .image1_idx = static_cast<int>(i),
                .image2_idx = j,
                .score = 1.0 / (1.0 + distance),  // 转为相似度
                .method = "vlad",
                .visual_similarity = pairs.back().score
            });
        }
    }
    
    return pairs;
}
```

### 2.3 穷举/序列检索（已有）
```cpp
// 保留现有功能
std::vector<ImagePair> retrieveExhaustive(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
);

std::vector<ImagePair> retrieveSequential(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
);
```

### 2.4 词汇树检索（DBoW3）
```cpp
std::vector<ImagePair> retrieveByVocabTree(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
) {
    // 1. 加载预训练词汇表
    auto vocab = loadVocabulary(options.vocab_file);
    
    // 2. 构建图像数据库
    auto db = buildDBoW3Database(vocab, images);
    
    // 3. 查询相似图像
    std::vector<ImagePair> pairs;
    for (size_t i = 0; i < images.size(); ++i) {
        auto descriptors = loadSIFTDescriptors(images[i].feature_file);
        auto results = db.query(descriptors, options.top_k);
        
        for (const auto& [j, score] : results) {
            if (i >= static_cast<size_t>(j)) continue;
            pairs.push_back({
                .image1_idx = static_cast<int>(i),
                .image2_idx = j,
                .score = score,
                .method = "vocab_tree"
            });
        }
    }
    
    return pairs;
}
```

---

## 3. 混合策略组合

### 3.1 函数组合器
```cpp
// 组合多个检索策略
std::vector<ImagePair> combineStrategies(
    const std::vector<ImageInfo>& images,
    const std::vector<RetrievalFunction>& strategies,
    const std::vector<double>& weights
) {
    std::vector<ImagePair> all_pairs;
    
    for (size_t i = 0; i < strategies.size(); ++i) {
        auto pairs = strategies[i](images, options);
        
        // 加权调整分数
        for (auto& p : pairs) {
            p.score *= weights[i];
        }
        
        all_pairs.insert(all_pairs.end(), pairs.begin(), pairs.end());
    }
    
    // 去重并合并重复对的分数
    return deduplicateAndMerge(all_pairs);
}
```

### 3.2 过滤与排序
```cpp
// 泛型过滤器
template <typename Pred>
std::vector<ImagePair> filterPairs(
    const std::vector<ImagePair>& pairs,
    Pred predicate
) {
    std::vector<ImagePair> result;
    std::copy_if(pairs.begin(), pairs.end(), 
                 std::back_inserter(result), 
                 predicate);
    return result;
}

// 示例：过滤低分数对
auto high_quality_pairs = filterPairs(pairs, 
    [](const ImagePair& p) { return p.score > 0.5; }
);

// 排序
std::vector<ImagePair> sortByScore(std::vector<ImagePair> pairs) {
    std::sort(pairs.begin(), pairs.end(),
              [](const ImagePair& a, const ImagePair& b) {
                  return a.score > b.score;
              });
    return pairs;
}
```

---

## 4. 主流程设计

### 4.1 策略注册表（编译时）
```cpp
// 使用 std::map 存储策略函数
const std::map<std::string, RetrievalFunction> STRATEGIES = {
    {"exhaustive", retrieveExhaustive},
    {"sequential", retrieveSequential},
    {"gps", retrieveByGPS},
    {"vlad", retrieveByVLAD},
    {"vocab_tree", retrieveByVocabTree}
};

// 解析策略字符串 "gps+vlad"
std::vector<std::string> parseStrategyString(const std::string& strategy_str) {
    std::vector<std::string> strategies;
    std::istringstream ss(strategy_str);
    std::string token;
    while (std::getline(ss, token, '+')) {
        strategies.push_back(token);
    }
    return strategies;
}
```

### 4.2 主函数逻辑
```cpp
int main(int argc, char* argv[]) {
    // 1. 解析命令行参数
    RetrievalOptions options = parseCmdLine(argc, argv);
    
    // 2. 加载图像列表（带 GNSS/IMU）
    auto images = loadImagesWithMetadata(options.input_file, options.feature_dir);
    
    // 3. 解析并执行策略
    auto strategy_names = parseStrategyString(options.strategy);
    std::vector<ImagePair> pairs;
    
    if (strategy_names.size() == 1) {
        // 单一策略
        pairs = STRATEGIES.at(strategy_names[0])(images, options);
    } else {
        // 混合策略
        std::vector<RetrievalFunction> funcs;
        for (const auto& name : strategy_names) {
            funcs.push_back(STRATEGIES.at(name));
        }
        pairs = combineStrategies(images, funcs, {0.6, 0.4});  // 权重可配置
    }
    
    // 4. 后处理：过滤 + 排序 + 限制数量
    pairs = filterPairs(pairs, [](const ImagePair& p) { return p.score > 0.01; });
    pairs = sortByScore(pairs);
    if (options.max_pairs > 0 && pairs.size() > options.max_pairs) {
        pairs.resize(options.max_pairs);
    }
    
    // 5. 写入输出 JSON
    writePairsJSON(images, pairs, options.output_file, options);
    
    return 0;
}
```

---

## 5. 模块化设计

### 5.1 目录结构
```
src/algorithm/
├── modules/
│   └── retrieval/
│       ├── retrieval_types.h        # 数据结构定义
│       ├── spatial_retrieval.h      # GPS 空间检索
│       ├── vlad_retrieval.h         # VLAD 编码检索
│       ├── vocab_tree_retrieval.h   # DBoW3 词汇树
│       └── retrieval_utils.h        # 通用工具函数
└── tools/
    ├── isat_retrieve.cpp            # 主 CLI 工具
    └── isat_train_vocab.cpp         # 词汇表训练工具
```

### 5.2 头文件：retrieval_types.h
```cpp
#pragma once

#include <string>
#include <vector>
#include <optional>
#include <functional>

namespace insight::algorithm::retrieval {

// 数据结构定义（如上）

// 策略函数类型
using RetrievalFunction = std::function<
    std::vector<ImagePair>(
        const std::vector<ImageInfo>&,
        const RetrievalOptions&
    )
>;

// 工具函数声明
template <typename Pred>
std::vector<ImagePair> filterPairs(
    const std::vector<ImagePair>& pairs,
    Pred predicate
);

std::vector<ImagePair> sortByScore(std::vector<ImagePair> pairs);
std::vector<ImagePair> deduplicateAndMerge(std::vector<ImagePair> pairs);

}  // namespace insight::algorithm::retrieval
```

---

## 6. 实施计划

### Phase 1: GPS 空间检索 ✅ **COMPLETED** (2026-02-13)
- [x] 下载 nanoflann 到 `third_party/`
- [x] 实现 `spatial_retrieval.h/cpp`
  - [x] `radiusSearchBatch()` - k-d tree radius search
  - [x] `retrieveByGPS()` - GPS 检索主函数
  - [x] `computeSpatialScore()` - 距离转相似度（指数衰减）
- [x] 扩展 `ImageInfo` 支持 GNSS/IMU
- [x] 修改 `loadImagesFromJSON()` 读取坐标数据
- [x] 更新 `writePairsJSON()` 输出空间元数据
- [x] 测试：10 张测试图像 + GPS（网格布局，100m间距）

**成果**：
- 功能通过：GPS检索、混合策略、fallback
- 性能：<1ms for 10 images (k-d tree构建+查询)
- 覆盖率：44% (纯GPS) → 60% (GPS+sequential混合)

### Phase 2: 函数式重构 ✅ **COMPLETED** (2026-02-13)
- [x] 提取 `retrieveExhaustive()` 和 `retrieveSequential()` 为独立函数
- [x] 创建策略注册表 `STRATEGIES` (std::map<std::string, RetrievalFunction>)
- [x] 实现 `combineStrategies()` 组合器（deduplicateAndMerge）
- [x] 主函数改为函数派发模式
- [x] 添加 `--strategy gps+sequential` 混合支持

**成果**：
- 零继承OOP架构：纯函数式策略派发
- 策略组合：支持任意组合（e.g., "gps+sequential+vlad"）
- 自动去重：相同对的score合并，method字段合并为 "gps+sequential"

### Phase 3: VLAD 编码 ✅ **COMPLETED** (2026-02-13)
- [x] 实现 k-means 聚类训练（OpenCV kmeans）
- [x] 实现 VLAD 编码 `encodeVLAD()` (residual aggregation + L2归一化)
- [x] 实现相似度搜索 `findTopKSimilar()` (暴力L2距离)
- [x] 缓存 VLAD 向量到 `.isat_vlad` 文件
- [x] 创建 `isat_train_vlad` 训练工具
- [x] 集成到 `isat_retrieve` (--vlad-codebook, --vlad-cache参数)
- [ ] 测试：无 GPS 数据集（待实际数据测试）

**成果**：
- 功能完成：VLAD编码、k-means训练、缓存机制、检索策略
- 工具：isat_train_vlad（训练视觉词汇）、isat_retrieve --strategy vlad
- 架构：纯函数式实现，无OOP继承，动态策略注册
- 性能：缓存优化，避免重复计算VLAD向量

### Phase 4: DBoW3 集成（可选，5-7 天）🔜 **OPTIONAL**
- [ ] 添加 DBoW3 子模块
- [ ] 创建 `isat_train_vocab` 工具
- [ ] 实现 `retrieveByVocabTree()`
- [ ] 测试：大规模数据集（>10K 图像）

---

## 7. 性能目标

| 场景 | 图像数 | 策略 | 检索时间 | Pairs 数 | Coverage |
|------|--------|------|----------|----------|----------|
| 航拍（有 GPS） | 1K | GPS | <1s | 10K | 95%+ |
| 航拍（有 GPS） | 10K | GPS | <10s | 200K | 95%+ |
| 地面（无 GPS） | 1K | VLAD | <30s | 20K | 80%+ |
| 大规模 | 100K | Vocab Tree | <5min | 2M | 85%+ |

---

## 8. 命令行示例

```bash
# GPS 空间检索
./isat_retrieve -f features/ -i images.json -o pairs_gps.json \
    -s gps --distance-threshold 200 --max-neighbors 50

# 混合策略（GPS + Sequential 保底）
./isat_retrieve -f features/ -i images.json -o pairs_hybrid.json \
    -s gps+sequential --distance-threshold 150

# VLAD 检索（无 GPS）
./isat_retrieve -f features/ -o pairs_vlad.json \
    -s vlad --vlad-clusters 64 --top-k 20

# 词汇树检索（预训练）
./isat_train_vocab -f features/ -o vocab_1M.yml.gz -k 10 -L 6
./isat_retrieve -f features/ -o pairs_vocab.json \
    -s vocab_tree --vocab vocab_1M.yml.gz --top-k 30

# 组合策略（GPS 优先 + VLAD 补充）
./isat_retrieve -f features/ -i images.json -o pairs_combined.json \
    -s gps+vlad --distance-threshold 200 --top-k 15
```

---

## 9. 关键设计决策

1. **函数式 > OOP**: 避免继承层次，使用 std::function + 泛型
2. **纯函数**: 所有检索函数无副作用，便于测试和并行化
3. **组合 > 扩展**: 通过函数组合实现混合策略，而非子类重写
4. **类型安全**: 使用结构化数据（struct）+ std::optional，避免魔法值
5. **image_id 为整数**: `ImageInfo::image_id` 使用 `uint32_t`，与 project/database 导出的 image list 一致；特征/匹配等文件名统一用 `to_string(image_id)` 生成（如 `1.isat_feat`、`1_2.isat_match`），后续 pipeline 使用方便
6. **性能优先**: k-d tree（nanoflann）+ FAISS 优化的向量检索
7. **可扩展**: 新增策略只需添加函数到注册表，无需修改框架

---

## 10. 实施状态

**完成日期**: 2026-02-13  
**完成阶段**: Phase 1 (GPS空间检索) + Phase 2 (函数式重构) + Phase 3 (VLAD视觉编码)

### 已实现功能
- ✅ GPS空间检索（nanoflann k-d tree）
- ✅ 混合策略支持（GPS + Sequential + VLAD）
- ✅ 自动去重与score合并
- ✅ GNSS/IMU数据加载（JSON格式）
- ✅ Graceful fallback（无GPS时使用sequential或vlad）
- ✅ VLAD视觉编码（k-means + residual aggregation）
- ✅ VLAD向量缓存机制（.isat_vlad文件）
- ✅ isat_train_vlad训练工具
- ✅ 完整CLI（--help, --strategy, --vlad-codebook等）

### 测试结果
详见: 
- [GPS/Sequential测试](./isat_retrieve_TEST_RESULTS.md)
- [VLAD快速入门](./VLAD_QUICKSTART.md) - 包含使用示例和故障排查

### 文件清单
```
src/algorithm/modules/retrieval/
├── retrieval_types.h          # 数据结构定义（ImageInfo, ImagePair, GNSSData等）
├── retrieval_types.cpp        # 工具函数（deduplicateAndMerge, combinePairs）
├── spatial_retrieval.h        # GPS检索接口（纯函数）
├── spatial_retrieval.cpp      # nanoflann k-d tree实现
├── vlad_encoding.h            # VLAD编码接口（k-means, encodeVLAD）
├── vlad_encoding.cpp          # OpenCV kmeans + residual aggregation
├── vlad_retrieval.h           # VLAD检索策略接口
└── vlad_retrieval.cpp         # VLAD相似度搜索（top-k L2距离）

src/algorithm/tools/
├── isat_retrieve.cpp          # 主程序（策略注册表 + 函数派发）
└── isat_train_vlad.cpp        # VLAD codebook训练工具

third_party/nanoflann/
└── nanoflann.hpp              # Header-only k-d tree库

doc/tools/
├── isat_retrieve_DESIGN.md         # 本设计文档
├── isat_retrieve_TEST_RESULTS.md   # GPS测试报告
└── VLAD_QUICKSTART.md              # VLAD使用指南（新）
```

---

**下一步**: Phase 4 - DBoW3词汇树集成（可选）或在真实数据集上测试VLAD性能
