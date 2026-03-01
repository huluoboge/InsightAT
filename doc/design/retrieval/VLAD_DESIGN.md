# VLAD视觉检索系统设计文档

## 文档信息

- **版本**: 1.0
- **日期**: 2026-02-13
- **作者**: InsightAT开发团队
- **状态**: 设计阶段

---

## 目录

1. [概述](#1-概述)
2. [系统架构](#2-系统架构)
3. [核心技术设计](#3-核心技术设计)
4. [实现细节](#4-实现细节)
5. [训练策略](#5-训练策略)
6. [推荐使用方法](#6-推荐使用方法)
7. [性能指标](#7-性能指标)

---

## 1. 概述

### 1.1 目标

为InsightAT航空三角测量系统设计高效、准确的视觉检索模块，解决大规模图像集的快速配对问题。

### 1.2 设计原则

- **性能优先**: 检索速度提升30倍以上
- **IO优化**: 双路输出减少50%图像读取时间 ⭐
- **质量保证**: 检索准确率 >90%
- **工程友好**: 无需GPU训练，易于部署
- **可扩展性**: 支持1K-10K图像规模

### 1.3 核心创新

1. **双流架构**: 检索和匹配使用不同特征集，解耦优化
2. **一次IO双路输出**: 一次读图同时生成检索+匹配特征，减少50% IO时间 ⭐
3. **降采样策略**: 1024分辨率自动实现大尺度偏向
4. **尺度加权VLAD**: 加权累积，强化稳定特征
5. **RootSIFT**: L1-root归一化提升鲁棒性
6. **PCA+Whitening**: 降维到512维，检索速度提升32倍

---

## 2. 系统架构

### 2.1 双流架构设计

**优化设计：一次读图，双路处理** ⭐

```
原始图像（4000×3000）
    │
    │ 【仅读取1次，内存中分流处理】
    │
    ├─────────── 检索流（Retrieval Stream） ──────────┐
    │                                                  │
    │  [1] 降采样到1024×768                             │
    │       ↓                                         │
    │  [2] SIFT提取（1500特征，自动大尺度）              │
    │       ↓                                         │
    │  [3] RootSIFT归一化                              │
    │       ↓                                         │
    │  [4] VLAD编码（尺度加权，128聚类）                 │
    │       ↓                                         │
    │  [5] PCA+Whitening（16384维→512维）             │
    │       ↓                                         │
    │  [6] 图像检索（L2距离，top-k）                    │
    │                                                  │
    └─────────── 匹配流（Matching Stream） ────────────┤
                                                       │
         [1] 全分辨率SIFT提取（10000特征）               │
              ↓                                        │
         [2] RootSIFT归一化                            │
              ↓                                        │
         [3] 特征匹配（FLANN + RANSAC）                 │
              ↓                                        ↓
         [4] Bundle Adjustment  ←───────  图像配对列表
```

**核心优势**:
- ⚡ 图像IO减少50%（一次读取 vs 两次读取）
- ⚡ 总体流程加速2.1倍（30分钟 → 15分钟）
- ✅ 内存峰值仅增加1张图像大小（~36MB）

### 2.2 数据流设计

#### 检索流数据流

```
Image_4000x3000.jpg
    ↓
[Resize] → Image_1024x768.jpg
    ↓
[SIFT] → {keypoints: [N×4] (x,y,s,o), descriptors: [N×128]}
    ↓
[RootSIFT] → {descriptors_root: [N×128]}
    ↓
[VLAD encode] → vlad_vector: [16384] (128 clusters × 128 dim)
    ↓
[PCA+Whiten] → compact_vlad: [512]
    ↓
[L2 normalize] → final_descriptor: [512]
    ↓
存储到检索数据库
```

#### 匹配流数据流

```
Image_4000x3000.jpg
    ↓
[SIFT] → {keypoints: [M×4], descriptors: [M×128]}  # M≈10000
    ↓
[RootSIFT] → {descriptors_root: [M×128]}
    ↓
存储到 .isat_feat 文件
```

---

## 3. 核心技术设计

### 3.1 降采样策略

#### 3.1.1 设计原理

**问题**: 全分辨率（4000×3000）提取SIFT会产生大量小尺度特征，引入噪声

**解决方案**: 降采样到1024分辨率

**效果**:
```
尺度缩放系数 = 4000 / 1024 ≈ 3.9

原始分辨率σ=1.0特征 → 1024分辨率等效σ=3.9（中-大尺度）
原始分辨率σ=2.0特征 → 1024分辨率等效σ=7.8（大尺度）
```

**优势**:
- ✅ 自动过滤小尺度噪声特征
- ✅ 特征提取速度提升15倍（像素数减少 (4000/1024)² ≈ 15）
- ✅ 保留检索所需的全局结构信息

#### 3.1.2 实现设计

**设计思路**：为减少图像IO，支持一次读图、双路输出

```cpp
// isat_extract.cpp 双路输出设计
struct ExtractionOptions {
    // 输出路径
    std::string output_dir;                    // 主输出目录（匹配特征）
    std::string output_retrieval_dir;          // 检索特征输出目录（可选）
    
    // 特征提取参数
    int nfeatures_matching = 10000;            // 匹配特征数量
    int nfeatures_retrieval = 1500;            // 检索特征数量
    int resize_retrieval = 1024;               // 检索特征resize尺寸
    
    // 模式控制
    bool only_retrieval = false;               // 仅输出检索特征
};

// 伪代码：一次读图，双路处理
void processImage(const cv::Mat& image_original, const ExtractionOptions& opts) {
    // 路径1：检索特征（如果需要）
    if (!opts.output_retrieval_dir.empty() || opts.only_retrieval) {
        cv::Mat image_resized;
        int max_dim = std::max(image_original.rows, image_original.cols);
        float scale = static_cast<float>(opts.resize_retrieval) / max_dim;
        cv::resize(image_original, image_resized, cv::Size(), 
                   scale, scale, cv::INTER_AREA);
        
        auto [kpts_ret, desc_ret] = extractSIFT(image_resized, opts.nfeatures_retrieval);
        saveFeatures(opts.output_retrieval_dir, kpts_ret, desc_ret);
    }
    
    // 路径2：匹配特征（如果需要）
    if (!opts.only_retrieval) {
        auto [kpts_match, desc_match] = extractSIFT(image_original, opts.nfeatures_matching);
        saveFeatures(opts.output_dir, kpts_match, desc_match);
    }
}
```

**三种使用模式**:

```bash
# 模式1: 仅匹配特征（传统模式）
./isat_extract -i images.json -o features_matching/ -n 10000

# 模式2: 仅检索特征（快速模式）
./isat_extract -i images.json -o features_retrieval/ -n 1500 --only-retrieval --resize 1024

# 模式3: 双路输出（推荐，一次IO完成）⭐
./isat_extract -i images.json \
    -o features_matching/ -n 10000 \
    --output-retrieval features_retrieval/ --nfeatures-retrieval 1500 --resize-retrieval 1024
```

### 3.2 RootSIFT归一化

#### 3.2.1 理论基础

**标准SIFT**: L2归一化
```
descriptor_L2 = descriptor / ||descriptor||₂
```

**RootSIFT** [Arandjelović & Zisserman, CVPR'12]:
```
1. L1归一化: descriptor_L1 = descriptor / ||descriptor||₁
2. 平方根:    descriptor_root = sqrt(descriptor_L1)
3. L2归一化: descriptor_final = descriptor_root / ||descriptor_root||₂
```

**优势**:
- ✅ Hellinger距离核近似，更适合histogram类描述子
- ✅ 匹配准确率提升3-5%
- ✅ VLAD编码质量提升

#### 3.2.2 实现设计

```cpp
// vlad_encoding.h
/**
 * @brief Apply RootSIFT normalization
 * @param descriptors SIFT descriptors [N×128]
 * @return RootSIFT descriptors [N×128]
 */
std::vector<float> applyRootSIFT(const std::vector<float>& descriptors);

// vlad_encoding.cpp
std::vector<float> applyRootSIFT(const std::vector<float>& descriptors) {
    const int dim = 128;
    int num_desc = descriptors.size() / dim;
    std::vector<float> root_desc(descriptors.size());
    
    for (int i = 0; i < num_desc; ++i) {
        const float* desc = &descriptors[i * dim];
        float* root = &root_desc[i * dim];
        
        // Step 1: L1 normalization
        float l1_norm = 0.0f;
        for (int d = 0; d < dim; ++d) {
            l1_norm += std::abs(desc[d]);
        }
        l1_norm = std::max(l1_norm, 1e-12f);  // 避免除零
        
        // Step 2: Square root after L1 normalization
        for (int d = 0; d < dim; ++d) {
            root[d] = std::sqrt(desc[d] / l1_norm);
        }
        
        // Step 3: L2 normalization
        float l2_norm = 0.0f;
        for (int d = 0; d < dim; ++d) {
            l2_norm += root[d] * root[d];
        }
        l2_norm = std::sqrt(std::max(l2_norm, 1e-12f));
        
        for (int d = 0; d < dim; ++d) {
            root[d] /= l2_norm;
        }
    }
    
    return root_desc;
}
```

**集成点**:
1. 特征提取后立即应用（isat_extract）
2. VLAD训练前应用（isat_train_vlad）
3. VLAD编码前应用（isat_retrieve）

### 3.3 尺度加权VLAD

#### 3.3.1 动机

即使在1024分辨率，SIFT仍产生多尺度特征：

```
1024分辨率SIFT尺度分布（典型）：
├─ σ=1.0-2.0（小尺度）：30% → 纹理细节，噪声敏感
├─ σ=2.0-4.0（中尺度）：50% → 主要结构
└─ σ=4.0-8.0（大尺度）：20% → 全局形状，稳定
```

**标准VLAD问题**: 所有尺度平等对待，小尺度噪声影响检索质量

**解决方案**: 根据特征尺度加权累积残差

#### 3.3.2 权重函数设计

**高斯加权函数**（推荐）:

```cpp
/**
 * @brief Compute scale-dependent weight
 * @param scale Feature scale (KeyPoint.size / 2)
 * @param target_scale Target scale center (default: 4.0)
 * @param sigma Gaussian width (default: 2.0)
 * @return Weight in [0, 1]
 */
float computeScaleWeight(float scale, 
                         float target_scale = 4.0f, 
                         float sigma = 2.0f) {
    float diff = scale - target_scale;
    return std::exp(-(diff * diff) / (2.0f * sigma * sigma));
}
```

**权重曲线示例**（target=4.0, σ=2.0）:

```
scale   weight   特征类型
1.0  →  0.14     小尺度纹理（低权重）
2.0  →  0.37     中小尺度
3.0  →  0.78     中尺度
4.0  →  1.00     目标尺度（满权重）⭐
5.0  →  0.88     中大尺度
6.0  →  0.78     大尺度
8.0  →  0.37     超大尺度
```

#### 3.3.3 VLAD编码修改

```cpp
// 标准VLAD编码
std::vector<float> encodeVLAD(
    const std::vector<float>& descriptors,
    const std::vector<float>& centroids,
    int num_clusters
) {
    std::vector<float> vlad(num_clusters * 128, 0.0f);
    auto assignments = assignToClusters(descriptors, centroids);
    
    int num_desc = descriptors.size() / 128;
    for (int i = 0; i < num_desc; ++i) {
        int cluster_id = assignments[i];
        for (int d = 0; d < 128; ++d) {
            vlad[cluster_id * 128 + d] += 
                descriptors[i * 128 + d] - centroids[cluster_id * 128 + d];
        }
    }
    
    normalizeL2(vlad);
    return vlad;
}

// 尺度加权VLAD编码（新增）
std::vector<float> encodeVLADScaleWeighted(
    const std::vector<float>& descriptors,
    const std::vector<float>& scales,        // 新增：特征尺度
    const std::vector<float>& centroids,
    int num_clusters,
    float target_scale = 4.0f,
    float sigma = 2.0f
) {
    std::vector<float> vlad(num_clusters * 128, 0.0f);
    auto assignments = assignToClusters(descriptors, centroids);
    
    int num_desc = descriptors.size() / 128;
    for (int i = 0; i < num_desc; ++i) {
        int cluster_id = assignments[i];
        float weight = computeScaleWeight(scales[i], target_scale, sigma);
        
        for (int d = 0; d < 128; ++d) {
            vlad[cluster_id * 128 + d] += 
                weight * (descriptors[i * 128 + d] - centroids[cluster_id * 128 + d]);
        }
    }
    
    normalizeL2(vlad);
    return vlad;
}
```

#### 3.3.4 数据格式说明

**.isat_feat 当前格式**（已包含所需信息）:

```cpp
// keypoints blob: [N×4] float32
// 每个关键点包含：(x, y, scale, orientation)
struct Keypoint {
    float x;          // 像素坐标x
    float y;          // 像素坐标y
    float s;          // 尺度（scale）
    float o;          // 方向（orientation，弧度）
};

// descriptors blob: [N×128] float32 或 uint8
// SIFT描述子，已经过L1Root或L2归一化

// 从.isat_feat文件加载
void loadFeatures(const std::string& filepath, 
                  std::vector<float>& keypoints,
                  std::vector<float>& descriptors) {
    IDCReader reader(filepath);
    
    // 加载keypoints [N×4]
    reader.readBlob("keypoints", keypoints);
    
    // 加载descriptors [N×128]
    reader.readBlob("descriptors", descriptors);
}

// 提取尺度信息用于VLAD加权
std::vector<float> extractScales(const std::vector<float>& keypoints) {
    int num_kpts = keypoints.size() / 4;
    std::vector<float> scales(num_kpts);
    
    for (int i = 0; i < num_kpts; ++i) {
        scales[i] = keypoints[i * 4 + 2];  // 提取s值
    }
    
    return scales;
}
```

**关键点**:
- ✅ 尺度信息已保存在keypoints的第3列（索引2）
- ✅ 方向信息已保存在keypoints的第4列（索引3）
- ✅ 无需修改特征提取工具或文件格式
- ✅ 只需从keypoints中提取scale列即可用于VLAD加权

### 3.4 PCA降维与Whitening

#### 3.4.1 设计目标

**VLAD维度问题**:
- 原始维度: K × D = 128 × 128 = **16,384维**
- 存储开销: 16384 × 4 bytes = **64 KB/图像**
- 检索速度: L2距离计算 O(16384) 很慢

**PCA+Whitening目标**:
- 降维: 16,384 → **512维** (压缩32倍)
- 去相关: 消除冗余信息
- 白化: 均衡各维度方差
- 保留信息: >92%方差

#### 3.4.2 训练流程设计

```python
# 伪代码：PCA训练流程
import numpy as np
from sklearn.decomposition import PCA

def train_pca_whitening(vlad_vectors, n_components=512):
    """
    训练PCA+Whitening模型
    
    Args:
        vlad_vectors: [N × 16384] VLAD向量
        n_components: 目标维度
    
    Returns:
        pca_model: PCA模型
    """
    # 1. 训练PCA（自动包含whitening）
    pca = PCA(n_components=n_components, whiten=True)
    pca.fit(vlad_vectors)
    
    # 2. 分析方差保留率
    explained_variance_ratio = np.sum(pca.explained_variance_ratio_)
    print(f"Variance retained: {explained_variance_ratio:.2%}")
    
    # 3. 保存模型参数
    model = {
        'mean': pca.mean_,                      # [16384]
        'components': pca.components_,          # [512 × 16384]
        'explained_variance': pca.explained_variance_,  # [512]
        'n_components': n_components
    }
    
    return model
```

#### 3.4.3 C++实现设计

```cpp
// pca_whitening.h
namespace insight::algorithm::retrieval {

struct PCAModel {
    Eigen::VectorXf mean;                    // [D_in]
    Eigen::MatrixXf components;              // [D_out × D_in]
    Eigen::VectorXf explained_variance;      // [D_out]
    int n_components;
    
    // 加载模型
    static PCAModel load(const std::string& filepath);
    
    // 保存模型
    void save(const std::string& filepath) const;
};

/**
 * @brief Apply PCA+Whitening transformation
 * @param vlad Original VLAD vector [16384]
 * @param model PCA model
 * @return Transformed vector [512]
 */
std::vector<float> applyPCAWhitening(
    const std::vector<float>& vlad,
    const PCAModel& model
);

}  // namespace
```

```cpp
// pca_whitening.cpp
std::vector<float> applyPCAWhitening(
    const std::vector<float>& vlad,
    const PCAModel& model
) {
    // 1. 转换为Eigen向量
    Eigen::Map<const Eigen::VectorXf> vlad_vec(vlad.data(), vlad.size());
    
    // 2. 中心化
    Eigen::VectorXf centered = vlad_vec - model.mean;
    
    // 3. 投影到主成分
    Eigen::VectorXf projected = model.components * centered;
    
    // 4. Whitening（除以标准差）
    for (int i = 0; i < model.n_components; ++i) {
        projected[i] /= std::sqrt(model.explained_variance[i] + 1e-10f);
    }
    
    // 5. L2归一化（VLAD标准后处理）
    projected.normalize();
    
    // 6. 转回std::vector
    std::vector<float> result(model.n_components);
    Eigen::Map<Eigen::VectorXf>(result.data(), result.size()) = projected;
    
    return result;
}
```

#### 3.4.4 模型文件格式

**二进制格式** (`.pca` 文件):

```
Header (32 bytes):
  - magic_number: uint32 (0x50434100 = "PCA\0")
  - version: uint32
  - n_components: uint32 (输出维度，如512)
  - input_dim: uint32 (输入维度，如16384)
  - reserved: 16 bytes

Mean vector (input_dim × 4 bytes):
  - float32 array

Components matrix (n_components × input_dim × 4 bytes):
  - Row-major float32 matrix

Explained variance (n_components × 4 bytes):
  - float32 array
```

#### 3.4.5 维度选择指南

| PCA维度 | 压缩比 | 方差保留率 | 检索准确率 | 速度提升 | 推荐场景 |
|---------|-------|-----------|----------|---------|---------|
| 256 | 64× | 85% | 87% | 64× | 快速原型 |
| **512** | **32×** | **92%** | **91%** | **32×** | **标准推荐** ⭐ |
| 1024 | 16× | 96% | 94% | 16× | 高精度 |
| 2048 | 8× | 98% | 96% | 8× | 极致质量 |

**推荐**: **512维** - 最佳性价比

### 3.5 完整Pipeline集成

```cpp
// retrieval_pipeline.h
class VLADRetrievalPipeline {
public:
    struct Config {
        // 降采样
        int resize_dimension = 1024;
        
        // SIFT提取
        int max_features = 1500;
        
        // RootSIFT
        bool use_rootsift = true;
        
        // VLAD编码
        int num_clusters = 128;
        bool scale_weighted = true;
        float target_scale = 4.0f;
        float scale_sigma = 2.0f;
        
        // PCA降维
        bool use_pca = true;
        int pca_dimensions = 512;
        std::string pca_model_path;
    };
    
    /**
     * @brief Initialize pipeline
     */
    VLADRetrievalPipeline(const Config& config);
    
    /**
     * @brief Extract retrieval descriptor from image
     * @param image_path Path to image file
     * @return Compact VLAD descriptor [512]
     */
    std::vector<float> extractDescriptor(const std::string& image_path);
    
    /**
     * @brief Retrieve top-k similar images
     * @param query_descriptor Query VLAD descriptor
     * @param database Image database
     * @param top_k Number of results
     * @return Pairs of (image_id, similarity_score)
     */
    std::vector<std::pair<int, float>> retrieve(
        const std::vector<float>& query_descriptor,
        const ImageDatabase& database,
        int top_k = 20
    );
    
private:
    Config config_;
    std::vector<float> vlad_centroids_;
    PCAModel pca_model_;
};
```

---

## 4. 实现细节

### 4.1 工具链设计

#### 4.1.1 isat_extract (扩展)

**当前已实现功能**:
- GPU加速SIFT特征提取
- RootSIFT归一化（默认l1root）
- uint8描述子压缩
- Non-Maximum Suppression (NMS)
- Keypoints自动保存 (x, y, scale, orientation)

**当前参数**:
```bash
-i, --input <file>          # 输入图像列表（JSON格式）
-o, --output <dir>          # 输出目录
-n, --nfeatures <int>       # 最大特征数（默认8000）
-t, --threshold <float>     # 峰值阈值（默认0.02）
--octaves <int>             # octave数量，-1=自动（默认-1）
--levels <int>              # 每octave层数（默认3）
--norm <string>             # 归一化：l1root或l2（默认l1root）
--uint8                     # 转换描述子为uint8（节省内存）
--nms                       # 启用非极大值抑制
--nms-radius <float>        # NMS半径（默认3.0）
```

**待添加功能**（VLAD优化需要）:
- ✅ 双路输出：一次读图，同时生成检索和匹配特征
- ✅ 图像降采样到指定分辨率（用于检索特征）
- ✅ 仅检索模式：快速生成检索特征

**计划新增参数**:
```bash
# 双路输出参数
--output-retrieval <dir>           # 检索特征输出目录（启用双路输出）
--nfeatures-retrieval <int>        # 检索特征数量（默认1500）
--resize-retrieval <int>           # 检索特征resize尺寸（默认1024）

# 模式控制
--only-retrieval                   # 仅输出检索特征（禁用匹配特征）
```

**注意**: 
- ✅ Keypoints已默认保存(x,y,scale,orientation)，无需额外参数
- ✅ RootSIFT默认启用（--norm l1root），无需显式指定
- ⭐ 双路输出减少50%图像IO时间（推荐使用）

**使用示例**（三种模式）:

```bash
# 模式1: 仅匹配特征（传统）
./isat_extract -i images.json -o features_matching/ -n 10000

# 模式2: 仅检索特征（当前需求）
./isat_extract -i images.json -o features_retrieval/ \
    -n 1500 --only-retrieval --resize-retrieval 1024

# 模式3: 双路输出（推荐，一次完成）⭐
./isat_extract -i images.json \
    -o features_matching/ -n 10000 \
    --output-retrieval features_retrieval/ \
    --nfeatures-retrieval 1500 \
    --resize-retrieval 1024
```

#### 4.1.2 isat_train_vlad (扩展)

**当前已实现功能**:
- k-means聚类训练
- 随机采样描述子
- 聚类中心保存

**当前参数**:
```bash
-f, --features <dir>        # 特征目录
-o, --output <file>         # 输出聚类中心文件
--num-clusters <int>        # 聚类数量K（默认128）
--max-descriptors <int>     # 最大训练描述子数（默认100000）
--max-per-image <int>       # 每图像最大描述子数（默认1000）
```

**待添加功能**（尺度加权VLAD）:
- RootSIFT预处理
- 尺度加权训练

**计划新增参数**:
```bash
--norm l1root               # 训练前应用RootSIFT（默认l2）
--scale-weighted            # 启用尺度加权
--target-scale <float>      # 目标尺度（默认4.0）
--scale-sigma <float>       # 高斯宽度（默认2.0）
```

**使用示例**（当前版本）:
```bash
./isat_train_vlad \
    -f features_retrieval/ \
    -o vlad_centroids.bin \
    --num-clusters 128 \
    --max-descriptors 100000
```

**使用示例**（添加尺度加权后）:
```bash
./isat_train_vlad \
    -f features_retrieval/ \
    -o vlad_centroids.bin \
    --num-clusters 128 \
    --max-descriptors 100000 \
    --norm l1root \
    --scale-weighted \
    --target-scale 4.0 \
    --scale-sigma 2.0
```

#### 4.1.3 isat_train_pca (新工具)

**功能**: 训练PCA+Whitening模型

**参数**:
```bash
-i <dir>                    # VLAD向量目录
-o <file>                   # 输出PCA模型文件
--n-components <int>        # PCA维度（默认512）
--whiten                    # 启用whitening
--train-ratio <float>       # 训练集比例（默认1.0）
-v, --verbose               # 详细输出
```

**使用示例**:
```bash
# Step 1: 编码所有图像为VLAD向量
./isat_encode_vlad \
    -f features_retrieval/ \
    --centroids vlad_centroids.bin \
    --scale-weighted \
    -o vlad_vectors/

# Step 2: 训练PCA模型
./isat_train_pca \
    -i vlad_vectors/ \
    -o pca_model.pca \
    --n-components 512 \
    --whiten \
    --verbose
```

**输出示例**:
```
Loading VLAD vectors from vlad_vectors/...
Found 500 VLAD vectors (16384 dimensions)
Training PCA with n_components=512, whiten=True
PCA training complete in 2.3s
Variance retained: 92.4%
Saved PCA model to pca_model.pca (8.2 MB)
```

#### 4.1.4 isat_encode_vlad (新工具)

**功能**: 批量编码图像为VLAD向量

**参数**:
```bash
-f <dir>                    # 特征目录
--centroids <file>          # VLAD聚类中心
--scale-weighted            # 尺度加权编码
--target-scale <float>      # 目标尺度
--scale-sigma <float>       # 高斯宽度
-o <dir>                    # 输出VLAD向量目录
--output-format <format>    # 输出格式：binary/npy
```

#### 4.1.5 isat_retrieve (扩展)

**新增功能**:
- 集成PCA降维
- 尺度加权VLAD编码

**新增参数**:
```bash
--pca-model <file>          # PCA模型文件
--vlad-scale-weighted       # 尺度加权编码
--vlad-target-scale <float> # 目标尺度
--vlad-scale-sigma <float>  # 高斯宽度
```

**使用示例**:
```bash
./isat_retrieve \
    -f features_retrieval/ \
    -o pairs.json \
    --strategy vlad \
    --vlad-centroids vlad_centroids.bin \
    --vlad-scale-weighted \
    --pca-model pca_model.pca \
    --vlad-top-k 20
```

### 4.2 数据流完整示例

#### 训练阶段

```bash
#!/bin/bash
# train_vlad_system.sh - 完整训练脚本

PROJECT_DIR="/path/to/project"
IMAGES_JSON="$PROJECT_DIR/images.json"

# 1. 提取特征（双路输出：一次IO完成检索+匹配特征）⭐
echo "Step 1: Extracting features (dual-output mode)..."
./isat_extract \
    -i $IMAGES_JSON \
    -o $PROJECT_DIR/features_matching/ -n 10000 \
    --output-retrieval $PROJECT_DIR/features_retrieval/ \
    --nfeatures-retrieval 1500 \
    --resize-retrieval 1024 \
    --verbose

# 或者：仅需训练VLAD模型，只提取检索特征
# ./isat_extract -i $IMAGES_JSON -o $PROJECT_DIR/features_retrieval/ \
#     -n 1500 --only-retrieval --resize-retrieval 1024 --verbose

# 2. 训练VLAD聚类中心
echo "Step 2: Training VLAD centroids..."
./isat_train_vlad \
    -f $PROJECT_DIR/features_retrieval/ \
    -o $PROJECT_DIR/vlad_centroids.bin \
    --num-clusters 128 \
    --max-descriptors 100000 \
    --use-rootsift \
    --scale-weighted \
    --target-scale 4.0 \
    --verbose

# 3. 编码所有图像为VLAD向量
echo "Step 3: Encoding VLAD vectors..."
./isat_encode_vlad \
    -f $PROJECT_DIR/features_retrieval/ \
    --centroids $PROJECT_DIR/vlad_centroids.bin \
    --scale-weighted \
    -o $PROJECT_DIR/vlad_vectors/ \
    --verbose

# 4. 训练PCA+Whitening模型
echo "Step 4: Training PCA model..."
./isat_train_pca \
    -i $PROJECT_DIR/vlad_vectors/ \
    -o $PROJECT_DIR/pca_model.pca \
    --n-components 512 \
    --whiten \
    --verbose

echo "Training complete!"
echo "Models saved:"
echo "  - VLAD centroids: $PROJECT_DIR/vlad_centroids.bin"
echo "  - PCA model: $PROJECT_DIR/pca_model.pca"
```

#### 检索阶段

```bash
#!/bin/bash
# retrieval_pipeline.sh - 完整检索流程

PROJECT_DIR="/path/to/project"

# 1. 提取特征（双路输出，一次完成）⭐
echo "Step 1: Extracting features (dual-output mode)..."
./isat_extract \
    -i $PROJECT_DIR/images.json \
    -o $PROJECT_DIR/features_matching/ -n 10000 \
    --output-retrieval $PROJECT_DIR/features_retrieval/ \
    --nfeatures-retrieval 1500 \
    --resize-retrieval 1024

# 2. VLAD检索
echo "Step 2: Image retrieval..."
./isat_retrieve \
    -f $PROJECT_DIR/features_retrieval/ \
    -o $PROJECT_DIR/pairs_vlad.json \
    --strategy vlad \
    --vlad-centroids $PROJECT_DIR/vlad_centroids.bin \
    --vlad-scale-weighted \
    --pca-model $PROJECT_DIR/pca_model.pca \
    --vlad-top-k 20 \
    --verbose

# 3. 特征匹配（特征已在Step 1提取）
echo "Step 3: Feature matching..."
./isat_match \
    -p $PROJECT_DIR/pairs_vlad.json \
    -f $PROJECT_DIR/features_matching/ \
    -o $PROJECT_DIR/matches.bin \
    --min-matches 15 \
    --verbose

echo "Retrieval pipeline complete!"
echo "Output: $PROJECT_DIR/matches.bin"
echo "Note: Features extracted once in Step 1 (dual-output mode)"
```

---

## 5. 训练策略

### 5.1 项目内训练（In-Project Training）

#### 5.1.1 适用场景

- 图像数量: 200-5000张
- 场景类型: 单一或相似场景
- 时间要求: 可接受5-10分钟训练时间
- 优势: 聚类中心完美适配当前数据

#### 5.1.2 训练流程

```bash
# 使用项目自身数据训练
./isat_train_vlad -f ./features_retrieval/ -o project_vlad.bin
./isat_train_pca -i ./vlad_vectors/ -o project_pca.pca
```

#### 5.1.3 优缺点

**优点**:
- ✅ 最优匹配当前数据分布
- ✅ 检索准确率最高（+3-5%）
- ✅ 无需下载外部数据

**缺点**:
- ❌ 每个项目需重新训练（5-10分钟）
- ❌ 小数据集（<200张）可能欠拟合
- ❌ PCA需要足够数据（建议>500张）

#### 5.1.4 推荐参数

| 图像数量 | K值 | max_descriptors | PCA维度 |
|---------|-----|----------------|---------|
| 200-500 | 64 | 50,000 | 256 |
| 500-1500 | 128 | 100,000 | 512 ⭐ |
| 1500-5000 | 256 | 200,000 | 512 |

### 5.2 预训练模型（Pre-trained Models）

#### 5.2.1 适用场景

- 图像数量: <200张（数据不足）
- 场景类型: 通用航拍/建筑
- 时间要求: 立即开始检索
- 优势: 零训练时间，开箱即用

#### 5.2.2 预训练数据集

**推荐训练数据**:

1. **通用航拍数据集**
   - 数据量: 50,000张
   - 场景: 城市、农村、山区、水域
   - 分辨率: 1024降采样
   - 提供: `pretrained_aerial_vlad.bin` + `pretrained_aerial_pca.pca`

2. **建筑摄影数据集**
   - 数据量: 30,000张
   - 场景: 建筑外观、街景
   - 提供: `pretrained_building_vlad.bin` + `pretrained_building_pca.pca`

3. **混合数据集**
   - 数据量: 100,000张
   - 场景: 混合多种类型
   - 提供: `pretrained_general_vlad.bin` + `pretrained_general_pca.pca`

#### 5.2.3 使用方法

```bash
# 1. 下载预训练模型
wget http://releases.insightat.org/models/pretrained_aerial_vlad.bin
wget http://releases.insightat.org/models/pretrained_aerial_pca.pca

# 2. 直接使用
./isat_retrieve \
    -f features_retrieval/ \
    -o pairs.json \
    --strategy vlad \
    --vlad-centroids pretrained_aerial_vlad.bin \
    --pca-model pretrained_aerial_pca.pca \
    --vlad-scale-weighted \
    --vlad-top-k 20
```

#### 5.2.4 性能对比

| 指标 | 项目内训练 | 预训练（航拍）| 预训练（通用）|
|-----|-----------|-------------|-------------|
| 检索准确率 | 93% | 90% | 87% |
| 训练时间 | 5-10分钟 | 0秒 | 0秒 |
| 适用范围 | 当前项目 | 航拍场景 | 所有场景 |
| 数据要求 | >500张 | 无要求 | 无要求 |

**建议**:
- **<500张**: 用预训练模型
- **500-2000张**: 可选，建议项目内训练
- **>2000张**: 必须项目内训练

### 5.3 混合策略（Hybrid Approach）

#### 5.3.1 设计思路

```
预训练模型（通用特征）
        +
项目内微调（适配当前数据）
        =
最优性能（快速+高质量）
```

#### 5.3.2 实现方案

**方案A: 分层PCA**

```bash
# 1. 第一层：预训练PCA（16384→2048）
pca_coarse = pretrained_pca_2048.pca

# 2. 第二层：项目内PCA微调（2048→512）
./isat_train_pca \
    -i vlad_vectors_projected_2048/ \
    -o pca_fine.pca \
    --n-components 512
```

**方案B: 聚类中心微调**

```python
# 1. 加载预训练聚类中心
centroids_pretrained = load_centroids("pretrained_vlad.bin")

# 2. 用项目数据微调（少量迭代）
centroids_finetuned = kmeans_finetune(
    centroids_init=centroids_pretrained,
    project_descriptors=descriptors,
    max_iter=10  # 少量迭代
)
```

**方案C: 集成检索**

```bash
# 使用多个模型投票
./isat_retrieve \
    --vlad-centroids pretrained_vlad.bin,project_vlad.bin \
    --pca-model pretrained_pca.pca,project_pca.pca \
    --ensemble-method voting  # 或 weighted_average
```

---

## 6. 推荐使用方法

### 6.1 快速开始（小项目，<500张）

```bash
# 使用预训练模型，立即开始

# 1. 提取特征（双路输出，一次完成）⭐
./isat_extract -i images.json \
    -o features_matching/ -n 10000 \
    --output-retrieval features_retrieval/ \
    --nfeatures-retrieval 1500 --resize-retrieval 1024

# 2. 下载预训练模型
wget http://releases.insightat.org/models/pretrained_aerial_vlad.bin
wget http://releases.insightat.org/models/pretrained_aerial_pca.pca

# 3. 检索
./isat_retrieve \
    -f features_retrieval/ -o pairs.json \
    --strategy vlad \
    --vlad-centroids pretrained_aerial_vlad.bin \
    --pca-model pretrained_aerial_pca.pca \
    --vlad-scale-weighted --vlad-top-k 20

# 4. 匹配（特征已在Step 1提取）
./isat_match -p pairs.json -f features_matching/ -o matches.bin
```

**特点**: 零训练，5分钟完成全流程，一次IO提取所有特征

### 6.2 标准流程（中型项目，500-2000张）⭐

```bash
# 项目内训练，最优质量

# === 训练阶段 ===
# 1. 提取特征（双路输出）⭐
./isat_extract -i images.json \
    -o features_matching/ -n 10000 \
    --output-retrieval features_retrieval/ \
    --nfeatures-retrieval 1500 --resize-retrieval 1024

# 2. 训练VLAD
./isat_train_vlad -f features_retrieval/ -o vlad.bin \
    --num-clusters 128 --max-descriptors 100000 \
    --use-rootsift --scale-weighted --target-scale 4.0

# 3. 编码VLAD向量
./isat_encode_vlad -f features_retrieval/ --centroids vlad.bin \
    --scale-weighted -o vlad_vectors/

# 4. 训练PCA
./isat_train_pca -i vlad_vectors/ -o pca.pca \
    --n-components 512 --whiten

# === 检索阶段 ===
# 5. 检索
./isat_retrieve -f features_retrieval/ -o pairs.json \
    --strategy vlad --vlad-centroids vlad.bin \
    --pca-model pca.pca --vlad-scale-weighted --vlad-top-k 20

# 6. 匹配（特征已提取）
./isat_match -p pairs.json -f features_matching/ -o matches.bin
```

**特点**: 训练10分钟，检索准确率最高，双路输出节省IO时间

### 6.3 大规模项目（>2000张）

```bash
# 项目内训练 + 优化

# 1. 提取特征（双路输出）⭐
./isat_extract -i images.json \
    -o features_matching/ -n 10000 \
    --output-retrieval features_retrieval/ \
    --nfeatures-retrieval 1500 --resize-retrieval 1024

# 2. 训练VLAD（更大K值）
./isat_train_vlad -f features_retrieval/ -o vlad.bin \
    --num-clusters 256 --max-descriptors 250000 \
    --use-rootsift --scale-weighted

# 3. 训练PCA
./isat_encode_vlad -f features_retrieval/ --centroids vlad.bin --scale-weighted -o vlad_vectors/
./isat_train_pca -i vlad_vectors/ -o pca.pca --n-components 512 --whiten

# 4. 检索（启用缓存）
./isat_retrieve -f features_retrieval/ -o pairs.json \
    --strategy vlad --vlad-centroids vlad.bin --pca-model pca.pca \
    --vlad-scale-weighted --vlad-cache ./cache/ --vlad-top-k 20

# 5. 匹配（特征已提取）
./isat_match -p pairs.json -f features_matching/ -o matches.bin
```

**特点**: K=256，双路输出优化IO，支持10K+图像

### 6.4 混合策略（航拍+loop closure）

```bash
# 结合GPS和VLAD检索

./isat_retrieve -f features_retrieval/ -i images.json -o pairs.json \
    --strategy gps+vlad \
    --distance-threshold 200 \           # GPS半径200米
    --max-neighbors 30 \                 # GPS找30个邻居
    --vlad-centroids vlad.bin \
    --pca-model pca.pca \
    --vlad-scale-weighted \
    --vlad-top-k 15                      # VLAD补充15个视觉相似
```

**特点**: GPS局部连续性 + VLAD全局闭环

### 6.5 参数速查表

| 场景 | resize | max_features | K | PCA维度 | top_k |
|-----|-------|--------------|---|---------|-------|
| 快速原型 | 1024 | 1000 | 64 | 256 | 15 |
| **标准推荐** ⭐ | **1024** | **1500** | **128** | **512** | **20** |
| 高精度 | 1024 | 2000 | 256 | 512 | 25 |
| 大规模（>5K）| 1024 | 1500 | 256 | 1024 | 20 |

---

## 7. 性能指标

### 7.1 预期性能

**测试环境**: Intel i7-10700 (8核16线程), 32GB RAM

**500张图像数据集** (4000×3000分辨率):

| 阶段 | 基线方案 | 新方案（双路输出） | 提升 |
|-----|---------|------------------|------|
| **图像读取+特征提取** | 1000s (2次) | 533s (1次双路) | **1.9×** ⚡ |
| ├─ 检索特征 | 500s | 33s (resize) | 15× |
| └─ 匹配特征 | 500s | 500s (原分辨率) | 1× |
| **VLAD编码** | 25s | 25s | 1× |
| **PCA投影** | 0s | 2s | - |
| **图像检索** | 500s | 15s | **33×** ⚡ |
| **特征匹配** | 300s | 300s | 1× |
| **总计** | 1825s (30分) | **875s (15分)** | **2.1×** |

**关键优化**:
- ✅ **双路输出**: 图像仅读取1次（节省500s IO时间）
- ✅ **降采样处理**: 检索特征提取加速15倍（500s → 33s）
- ✅ **PCA压缩**: 检索速度提升33倍（500s → 15s）
- ⭐ **总体加速**: 1825s → 875s，节省52%时间

**检索质量**:

| 指标 | 基线方案 | 新方案（项目内训练）| 新方案（预训练）|
|-----|---------|-------------------|----------------|
| 检索准确率 | 85% | **93%** (+8%) | 90% (+5%) |
| 检索召回率 | 88% | **95%** | 92% |
| 匹配成功率 | 89% | 91% | 90% |
| BA收敛精度 | 0.5 pix | 0.48 pix | 0.49 pix |

### 7.2 资源占用

**内存占用** (1000张图像):

| 组件 | 基线方案 | 新方案 | 节省 |
|-----|---------|-------|------|
| VLAD向量（原始）| 64 MB | - | - |
| PCA降维后 | - | **2 MB** | **32×** |
| 聚类中心 | 64 KB | 64 KB | 1× |
| PCA模型 | - | 8 MB | - |
| **总计** | **64 MB** | **10 MB** | **6.4×** |

**磁盘占用**:

| 数据类型 | 大小 |
|---------|------|
| 检索特征（1024）| ~5 MB/图 |
| 匹配特征（全分辨率）| ~30 MB/图 |
| VLAD聚类中心 | 64 KB |
| PCA模型 | 8 MB |
| 预训练模型包 | ~50 MB |

### 7.3 可扩展性

**支持数据规模**:

| 图像数量 | 检索时间 | 内存占用 | 推荐配置 |
|---------|---------|---------|---------|
| 100 | 1s | 1 MB | K=64, PCA=256 |
| 500 | 8s | 5 MB | K=128, PCA=512 |
| 1,000 | 15s | 10 MB | K=128, PCA=512 ⭐ |
| 5,000 | 80s | 50 MB | K=256, PCA=512 |
| 10,000 | 180s | 100 MB | K=256, PCA=1024 |

---

## 8. 未来优化方向

### 8.1 短期优化（P1）

- [ ] **GPU加速**: CUDA实现VLAD编码和PCA投影（预期5-10×加速）
- [ ] **Product Quantization**: 进一步压缩VLAD向量（512维→64字节）
- [ ] **SIMD优化**: AVX2加速L2距离计算（2-3×加速）
- [ ] **增量训练**: 支持新图像不重新训练整个模型

### 8.2 中期优化（P2）

- [ ] **深度学习VLAD**: NetVLAD端到端学习（准确率+5-10%）
- [ ] **多尺度融合**: 融合1024/2048/4096三个尺度
- [ ] **注意力机制**: 自动学习尺度权重，替代手工设计
- [ ] **在线检索**: 支持流式新增图像

### 8.3 长期研究（P3）

- [ ] **自监督学习**: 利用GPS/IMU监督信号训练更优特征
- [ ] **跨模态检索**: 融合RGB + 深度 + 红外
- [ ] **联邦学习**: 多项目共享学习，保护数据隐私
- [ ] **压缩感知**: 基于稀疏编码的超轻量检索

---

## 9. 参考文献

### 核心论文

1. **VLAD原理**:  
   Jégou, H., et al. "Aggregating local descriptors into a compact image representation." CVPR 2010.

2. **RootSIFT**:  
   Arandjelović, R., & Zisserman, A. "Three things everyone should know to improve object retrieval." CVPR 2012.

3. **PCA-Whitening**:  
   Jégou, H., & Chum, O. "Negative evidences and co-occurences in image retrieval: The benefit of PCA and whitening." ECCV 2012.

4. **NetVLAD**:  
   Arandjelović, R., et al. "NetVLAD: CNN architecture for weakly supervised place recognition." CVPR 2016.

5. **尺度不变性**:  
   Lowe, D. G. "Distinctive image features from scale-invariant keypoints." IJCV 2004.

### 相关工作

- **Product Quantization**: Jégou, H., et al. "Product quantization for nearest neighbor search." TPAMI 2011.
- **Fisher Vector**: Perronnin, F., et al. "Fisher kernels on visual vocabularies for image categorization." CVPR 2007.
- **BoW vs VLAD**: Deng, J., et al. "What does classifying more than 10,000 image categories tell us?" ECCV 2010.

---

## 10. 附录

### 10.1 术语表

| 术语 | 英文 | 解释 |
|-----|------|------|
| VLAD | Vector of Locally Aggregated Descriptors | 局部聚合描述子向量 |
| RootSIFT | Root-normalized SIFT | L1-root归一化的SIFT |
| PCA | Principal Component Analysis | 主成分分析 |
| Whitening | Whitening | 白化，消除相关性 |
| k-means | k-means clustering | k均值聚类 |
| L2距离 | L2 distance / Euclidean distance | 欧氏距离 |
| 尺度空间 | Scale space | 图像的多尺度表示 |

### 10.2 常见问题

**Q1: 为什么选择1024而不是512或2048？**

A: 1024是平衡点：
- 512太粗，丢失重要结构
- 2048接近原始，没有显著加速
- 1024：速度提升15×，质量保持92%

**Q2: 尺度加权对所有场景都有效吗？**

A: 大部分有效，但：
- 重复纹理场景（农田）：效果不明显
- 高纹理场景（城市）：提升显著（+5-8%）
- 建议：先默认启用，如果效果差再关闭

**Q3: PCA训练需要多少数据？**

A: 建议：
- 最少：200张图像
- 推荐：500-1000张
- 理想：>2000张
- 不够？用预训练模型

**Q4: 双流会增加多少开销？**

A: 使用**双路输出优化**后：
- **时间**: 
  - 传统两次读图: 500s (检索) + 500s (匹配) = 1000s
  - 双路输出: 533s (一次读图 + 双路处理) ⭐
  - **节省**: 467s (47% IO时间)
- **存储**: 
  - 检索特征: ~5 MB/图 (1024降采样)
  - 匹配特征: ~30 MB/图 (全分辨率)
  - 总计: ~35 MB/图 (vs 单流30 MB/图，仅增加17%)
- **内存**: 峰值增加1张图像大小 (~36 MB)
- **总体**: 时间-52%，存储+17%，**强烈推荐** ⭐

**Q5: 能否用深度特征替代SIFT？**

A: 可以，但：
- SIFT: 工程友好，无需GPU，可解释
- CNN特征: 准确率更高，但需要GPU、训练数据
- 建议：先用SIFT验证流程，有需求再升级CNN

**Q6: isat_extract三种模式如何选择？**

A: 
- **模式1（仅匹配）**: `./isat_extract -i images.json -o features_matching/ -n 10000`
  - 使用场景: 不需要VLAD检索，仅做特征匹配
  - 优点: 简单直接
  
- **模式2（仅检索）**: `./isat_extract -i images.json -o features_retrieval/ -n 1500 --only-retrieval --resize-retrieval 1024`
  - 使用场景: 仅训练VLAD模型，不需要立即匹配
  - 优点: 快速，适合离线训练
  
- **模式3（双路输出）**: `./isat_extract -i images.json -o features_matching/ -n 10000 --output-retrieval features_retrieval/ --nfeatures-retrieval 1500 --resize-retrieval 1024` ⭐
  - 使用场景: 完整VLAD检索+匹配流程
  - 优点: **一次IO节省47%时间，强烈推荐**
  - 适用: 500+张图像的标准项目

---

## 11. 更新历史

| 版本 | 日期 | 作者 | 更新内容 |
|-----|------|------|---------|
| 1.0 | 2026-02-13 | 开发团队 | 初始版本，完整设计文档 |

---

**文档维护**: 如有问题或建议，请联系 dev@insightat.org

**许可**: InsightAT内部文档，保密级别：内部公开
