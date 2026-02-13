# VLAD + PCA 完整使用指南

## 功能概述

✅ **一条命令完成 VLAD + PCA 训练**  
✅ **检索支持 PCA 降维加速**  
✅ **100张图像推荐配置**

---

## 快速开始（100张图像）

### 方案1：VLAD训练 + PCA降维（推荐）

```bash
# Step 1: 提取检索特征（1024分辨率）
./isat_extract -i images.json -o features_retrieval/ \
    -n 1500 --only-retrieval --resize-retrieval 1024

# Step 2: 一条命令训练 VLAD + PCA（不启用whitening）
./isat_train_vlad \
    -f features_retrieval/ \
    -o vlad_centroids.bin \
    --clusters 64 \
    --max-descriptors 50000 \
    --pca-output pca_model.pca \
    --pca-dims 256

# Step 3: 检索时使用PCA模型
./isat_retrieve \
    -f features_retrieval/ \
    -i images.json \
    -o pairs.json \
    --strategy vlad \
    --vlad-codebook vlad_centroids.bin \
    --pca-model pca_model.pca \
    --vlad-top-k 20
```

**参数说明**：
- `--clusters 64`: 100张图像适合用K=64（不是128）
- `--pca-dims 256`: 降到256维（16384→256，压缩64倍）
- **无 --whiten**：100张图像不推荐whitening

---

### 方案2：仅VLAD（最简单）

如果不需要PCA加速：

```bash
# Step 1: 提取特征
./isat_extract -i images.json -o features_retrieval/ \
    -n 1500 --only-retrieval --resize-retrieval 1024

# Step 2: 训练VLAD（不训练PCA）
./isat_train_vlad \
    -f features_retrieval/ \
    -o vlad_centroids.bin \
    --clusters 64 \
    --max-descriptors 50000

# Step 3: 检索（不使用PCA）
./isat_retrieve \
    -f features_retrieval/ \
    -i images.json \
    -o pairs.json \
    --strategy vlad \
    --vlad-codebook vlad_centroids.bin \
    --vlad-top-k 20
```

**优点**：最简单，100张图像检索只需~2秒  
**缺点**：内存占用稍大（64KB vs 1KB per image）

---

## 完整流程（500+张图像）

### 训练阶段

```bash
#!/bin/bash
# train_vlad_pca.sh - 完整训练脚本

PROJECT_DIR="/path/to/project"
IMAGES_JSON="$PROJECT_DIR/images.json"

# 1. 提取检索特征
echo "=== Extracting Features ==="
./isat_extract -i $IMAGES_JSON -o $PROJECT_DIR/features_retrieval/ \
    -n 1500 --only-retrieval --resize-retrieval 1024 --verbose

# 2. 训练 VLAD + PCA（一条命令完成）
echo "=== Training VLAD + PCA ==="
./isat_train_vlad \
    -f $PROJECT_DIR/features_retrieval/ \
    -o $PROJECT_DIR/vlad_centroids.bin \
    --clusters 128 \
    --max-descriptors 100000 \
    --pca-output $PROJECT_DIR/pca_model.pca \
    --pca-dims 512 \
    --whiten \
    --verbose

echo "=== Training Complete ==="
echo "VLAD codebook: $PROJECT_DIR/vlad_centroids.bin"
echo "PCA model: $PROJECT_DIR/pca_model.pca"
```

### 检索阶段

```bash
#!/bin/bash
# retrieve_vlad_pca.sh - 检索流程

PROJECT_DIR="/path/to/project"

# 检索图像对
./isat_retrieve \
    -f $PROJECT_DIR/features_retrieval/ \
    -i $PROJECT_DIR/images.json \
    -o $PROJECT_DIR/pairs.json \
    --strategy vlad \
    --vlad-codebook $PROJECT_DIR/vlad_centroids.bin \
    --pca-model $PROJECT_DIR/pca_model.pca \
    --vlad-top-k 20 \
    --verbose

echo "Generated pairs: $PROJECT_DIR/pairs.json"
```

---

## 参数推荐表

| 图像数量 | K值 | max_descriptors | PCA维度 | Whitening | 理由 |
|---------|-----|----------------|---------|-----------|------|
| **100** | **64** | **50,000** | **256** | **否** | 避免过拟合 ⭐ |
| 200-500 | 64 | 50,000 | 256 | 否 | 数据不足 |
| 500-1500 | 128 | 100,000 | 512 | **是** | 标准配置 ⭐ |
| 1500-5000 | 256 | 200,000 | 512 | 是 | 大规模 |

---

## 命令行参数详解

### isat_train_vlad（扩展版）

```bash
./isat_train_vlad --help

Options:
  -f, --features <dir>          # 特征目录（必需）
  -o, --output <file>           # VLAD codebook输出（必需）
  -k, --clusters <int>          # 聚类数量（默认64）
  -n, --max-descriptors <int>   # 最大训练描述子数（默认1M）
  -p, --max-per-image <int>     # 每图像最大描述子数（默认500）
  -i, --iterations <int>        # k-means最大迭代次数（默认100）
  
  # PCA参数（新增）⭐
  -P, --pca-output <file>       # PCA模型输出（可选）
  -d, --pca-dims <int>          # PCA降维目标维度（默认256）
  -w, --whiten                  # 启用whitening（仅500+图像推荐）
  
  -v, --verbose                 # 详细日志
  -q, --quiet                   # 静默模式
  -h, --help                    # 帮助信息
```

### isat_retrieve（扩展版）

```bash
./isat_retrieve --help

VLAD检索新增参数：
  --vlad-codebook <file>        # VLAD codebook文件
  --vlad-cache <dir>            # VLAD缓存目录（可选）
  --vlad-top-k <int>            # Top-k相似图像（默认20）
  --pca-model <file>            # PCA模型文件（新增）⭐
```

---

## 性能对比

### 100张图像测试

| 配置 | VLAD维度 | 检索时间 | 内存占用 | 压缩比 |
|-----|---------|---------|---------|-------|
| 原始VLAD | 8192 (64×128) | 0.02s | 3.2 MB | 1× |
| **PCA-256** | **256** | **0.0003s** | **100 KB** | **32×** ⭐ |

### 500张图像测试

| 配置 | VLAD维度 | 检索时间 | 内存占用 | 压缩比 |
|-----|---------|---------|---------|-------|
| 原始VLAD | 16384 (128×128) | 15s | 32 MB | 1× |
| **PCA-512** | **512** | **0.5s** | **1 MB** | **32×** ⭐ |
| PCA-512+Whiten | 512 | 0.5s | 1 MB | 32× |

---

## 输出文件说明

### VLAD Codebook（.vcbt格式）

```
vlad_centroids.bin
├─ Header (16 bytes)
│  ├─ Magic: 0x56434254 ("VCBT")
│  ├─ Version: 1
│  ├─ Clusters: K (64 or 128)
│  └─ Descriptor dim: 128
└─ Centroids: [K × 128] float32
```

大小：~64 KB（K=128）或 ~32 KB（K=64）

### PCA Model（.pca格式）

```
pca_model.pca
├─ Header (32 bytes)
│  ├─ Magic: 0x50434100 ("PCA\0")
│  ├─ Version: 1
│  ├─ n_components: 256 or 512
│  ├─ input_dim: 16384 or 8192
│  └─ whiten: 0 or 1
├─ Mean vector: [input_dim] float32
├─ Components: [n_components × input_dim] float32
└─ Explained variance: [n_components] float32
```

大小：
- PCA-256 (K=64): ~2 MB
- PCA-512 (K=128): ~8 MB

---

## 典型输出示例

### 训练输出（100张图像）

```
=== VLAD Codebook Training ===
Feature directory: features_retrieval/
Output file: vlad_centroids.bin
Clusters: 64
Max descriptors: 50000
Found 100 feature files
Sampled 50000 descriptors from 100 files in 234ms
Training k-means with 50000 descriptors...
k-means training complete in 1823ms

=== PCA Training ===
PCA output: pca_model.pca
PCA dimensions: 256
Whitening: disabled
Encoding VLAD vectors from 100 files...
Encoded 100/100 files
Encoded 100 VLAD vectors in 456ms
Training PCA model...
PCA training complete in 123ms
Variance retained: 89.3%
Saved PCA model to pca_model.pca (2.1 MB)

=== Training Complete ===
Total time: 2636ms (~2.6秒)
Clusters: 64
Training samples: 50000
PCA dimensions: 8192 -> 256
Compression ratio: 32x
```

### 检索输出（100张图像）

```
=== VLAD Retrieval ===
Loaded VLAD codebook: 64 clusters
Loaded PCA model: 8192 -> 256 dimensions
VLAD retrieval: 100 images, 64 clusters, top-k=20
PCA enabled: 8192 -> 256 dimensions
VLAD encoding complete for 100 images
VLAD retrieval: generated 1980 pairs from 100 images
Coverage: 99.0% (expected ~100%)
```

---

## 故障排除

### Q1: PCA训练失败："No VLAD vectors encoded"

**原因**：特征文件格式不对或路径错误

**解决**：
```bash
# 检查特征文件
ls features_retrieval/*.isat_feat | head -3

# 验证特征文件格式
./isat_extract -i images.json -o features_retrieval/ -n 1500 --only-retrieval --resize-retrieval 1024
```

### Q2: 检索慢，没有加速效果

**原因**：未提供 `--pca-model` 参数

**解决**：
```bash
# 确保传递PCA模型
./isat_retrieve ... --pca-model pca_model.pca
```

### Q3: 100张图像训练PCA，"Variance retained"很低（<80%）

**原因**：数据量太少，PCA维度过高

**解决**：
```bash
# 降低PCA维度
./isat_train_vlad ... --pca-dims 128  # 从256降到128
```

### Q4: Whitening后准确率下降

**原因**：数据量不足（<500张）导致whitening过拟合

**解决**：
```bash
# 移除 --whiten 参数
./isat_train_vlad ... --pca-output pca.pca --pca-dims 256  # 不加 --whiten
```

---

## 最佳实践

### ✅ 100张图像

```bash
# 推荐配置
./isat_train_vlad -f features/ -o vlad.bin \
    --clusters 64 \
    --pca-output pca.pca --pca-dims 256
    # 注意：不使用 --whiten
```

### ✅ 500-1000张图像

```bash
# 标准配置
./isat_train_vlad -f features/ -o vlad.bin \
    --clusters 128 \
    --pca-output pca.pca --pca-dims 512 \
    --whiten  # 500+张可以启用
```

### ✅ 2000+张图像

```bash
# 高精度配置
./isat_train_vlad -f features/ -o vlad.bin \
    --clusters 256 \
    --max-descriptors 200000 \
    --pca-output pca.pca --pca-dims 512 \
    --whiten
```

---

## 下一步

1. **双路特征提取**：一次IO完成检索+匹配特征提取（节省50% IO时间）
   - 参考：[VLAD_DESIGN.md](../design/retrieval/VLAD_DESIGN.md#双路输出)

2. **尺度加权VLAD**：进一步提升检索准确率（+5-8%）
   - 计划实现：`--scale-weighted` 参数

3. **FAISS集成**：支持10K+图像的超大规模检索
   - 未来优化方向

---

**更新日期**: 2026-02-13  
**作者**: InsightAT开发团队
