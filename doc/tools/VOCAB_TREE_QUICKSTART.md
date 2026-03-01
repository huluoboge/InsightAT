# DBoW3 Vocabulary Tree Retrieval - Quick Start Guide

## Overview

The vocabulary tree retrieval strategy uses **DBoW3** (an improved implementation of bag-of-words) for efficient large-scale image retrieval. It's ideal for datasets with **>10K images** where VLAD becomes computationally expensive.

**Key Advantages:**
- ⚡ **Fast queries**: O(log N) search time using hierarchical vocabulary tree
- 📊 **Scalable**: Handles 100K+ images efficiently
- 🎯 **Better discrimination**: Hierarchical quantization preserves more visual information than flat k-means
- 💾 **Compact representation**: BowVector is more memory-efficient than VLAD for large datasets

## Workflow

### 1. Train Vocabulary Tree

Train a DBoW3 vocabulary from your image descriptors:

```bash
./isat_train_vocab \
    -f ./test_features \
    -o my_vocabulary.dbow3 \
    --branching 10 \
    --depth 6 \
    --max-descriptors 1000000 \
    --max-per-image 500
```

**Parameters:**
- `-f`: Feature directory containing `.isat_feat` files
- `-o`: Output vocabulary file (`.dbow3` format)
- `-k, --branching`: Branching factor for k-means (default: 10)
  - Higher values (15-20) = more discriminative but slower training
  - Lower values (6-10) = faster training, less discriminative
- `-L, --depth`: Tree depth (default: 6)
  - Vocabulary size = k^L (e.g., 10^6 = 1M words)
  - Typical: L=4-6 for photogrammetry (10K-1M words)
- `-n, --max-descriptors`: Maximum descriptors for training (default: 1M)
  - Use 100K-1M for best results
  - More descriptors = better coverage but slower training
- `-p, --max-per-image`: Max descriptors per image (default: 500)
  - Prevents a single image from dominating the vocabulary

**Training Time Estimates** (8-core CPU):
- 100K descriptors, k=10, L=5: ~2-3 minutes
- 1M descriptors, k=10, L=6: ~10-15 minutes
- 1M descriptors, k=15, L=6: ~30-40 minutes

**Vocabulary Size Recommendations:**
- <1K images: k=10, L=4 (10K words)
- 1K-10K images: k=10, L=5 (100K words)
- 10K-100K images: k=10, L=6 (1M words) ⭐ **Recommended**
- >100K images: k=15, L=6 (11M words)

### 2. Run Vocabulary Tree Retrieval

Use the trained vocabulary for image pair retrieval:

```bash
./isat_retrieve \
    -f ./test_features \
    -o vocab_pairs.json \
    --strategy vocab \
    --vocab-file my_vocabulary.dbow3 \
    --vocab-top-k 20
```

**Parameters:**
- `--strategy vocab`: Use vocabulary tree retrieval
- `--vocab-file`: Path to `.dbow3` vocabulary file
- `--vocab-top-k`: Number of most similar images per query (default: 20)
- `--vocab-cache`: (Optional) Cache directory for query optimization

**Expected Output:**
```
I0211 12:34:56 isat_retrieve.cpp:530] Using DBoW3 vocabulary: my_vocabulary.dbow3
I0211 12:35:01 vocab_tree_retrieval.cpp:140] Building vocabulary tree database...
I0211 12:35:15 vocab_tree_retrieval.cpp:152] Querying 1234 images...
I0211 12:35:45 vocab_tree_retrieval.cpp:173] Retrieved 24680 pairs (avg 20.0 per image)
I0211 12:35:46 isat_retrieve.cpp:585] Saved 24680 pairs to vocab_pairs.json
```

### 3. Hybrid Strategies

Combine vocabulary tree with GPS for optimal results:

```bash
./isat_retrieve \
    -f ./test_features \
    -o hybrid_pairs.json \
    --strategy gps+vocab \
    --distance-threshold 200 \
    --max-neighbors 30 \
    --vocab-file my_vocabulary.dbow3 \
    --vocab-top-k 10
```

**Hybrid Strategy Recommendations:**
- `gps+vocab`: **Best for aerial photogrammetry** ⭐
  - GPS provides spatial constraints
  - Vocab tree finds visually similar images beyond GPS radius
  - Combines geometric consistency with visual similarity
- `gps+sequential+vocab`: For strip-based acquisition
  - Sequential ensures temporal continuity
  - GPS adds spatial neighbors
  - Vocab tree fills gaps in coverage

## Comparison: Vocabulary Tree vs VLAD

| Metric | Vocabulary Tree (DBoW3) | VLAD |
|--------|------------------------|------|
| **Training Time** | 10-15 min (1M desc) | 2-3 min (1M desc) |
| **Query Speed** | 🚀 **0.01-0.05s/image** | 0.5-2s/image |
| **Scalability** | ✅ 100K+ images | ⚠️ <10K images |
| **Memory Usage** | 💾 Low (sparse BowVector) | 🔥 High (dense vector) |
| **Retrieval Quality** | 🎯 **Excellent** | Good |
| **Best For** | Large datasets, real-time | Small-medium datasets |

## File Formats

### .dbow3 (Vocabulary Binary)
DBoW3 native format with hierarchical k-means tree:
- **Size**: 50-500 MB depending on vocabulary size
- **Structure**: Compressed binary with QuickLZ
- **Portability**: Platform-independent

Example vocabulary files provided in `third_party/DBow3/`:
- `orbvoc.dbow3`: Sample ORB vocabulary (879K words, ~90MB)

### Vocabulary Tree Database (In-Memory)
Built from descriptors at runtime:
- **BowVector**: Sparse word ID → weight mapping
- **Database**: Inverted index for fast retrieval
- **Not cached** by default (can be added with `--vocab-cache`)

## Performance Tips

### Training Optimization

1. **Sample representative images**: Use diverse scenes from different altitudes/angles
2. **Balance descriptor sampling**: Limit `--max-per-image` to prevent feature-rich images from dominating
3. **Tune vocabulary size**: Match k^L to dataset size
   - Too small: Poor discrimination
   - Too large: Overfitting, slow queries

### Query Optimization

1. **Adjust top-k**: Lower values (10-15) for precise matching, higher (30-50) for recall
2. **Use hybrid strategies**: Combine with GPS to reduce search space
3. **Enable caching** (if implemented): Reuse BowVectors across multiple retrieval runs

## Troubleshooting

### Training Fails with "Insufficient descriptors"
**Solution**: Reduce `--branching` or `--depth`, or increase sampling with higher `--max-descriptors`

### Queries too slow (>1s per image)
**Possible causes**:
- Vocabulary too large (k^L > 10M words)
- Too many features per image (>5K)

**Solutions**:
- Use smaller vocabulary (k=10, L=5)
- Limit features during extraction: `isat_extract --max-features 2000`

### Low retrieval quality
**Diagnosis**:
- Check vocabulary training data: Should cover full dataset diversity
- Try larger vocabulary (increase k or L)
- Increase top-k value (20 → 50)

## Advanced Usage

### Custom Vocabulary from External Sources

DBoW3 supports loading vocabularies trained with ORB-SLAM or other DBoW3-compatible tools:

```bash
# Use pre-trained ORB vocabulary
./isat_retrieve \
    -f ./features \
    -o pairs.json \
    --strategy vocab \
    --vocab-file /path/to/ORBvoc.dbow3
```

**Note**: Ensure descriptor type matches (SIFT vs ORB). SIFT descriptors work best with vocabularies trained on SIFT features.

## References

- **DBoW3**: https://github.com/rmsalinas/DBow3
- **Original DBoW2 Paper**: D. Gálvez-López and J. D. Tardós, "Bags of Binary Words for Fast Place Recognition in Image Sequences," IEEE TRO 2012
- **Vocabulary Tree**: D. Nister and H. Stewenius, "Scalable Recognition with a Vocabulary Tree," CVPR 2006

## Next Steps

After generating pairs with vocabulary tree retrieval:
1. **Feature Matching**: `isat_match -p vocab_pairs.json -f ./test_features -o matches.bin`
2. **Bundle Adjustment**: Load matches into InsightAT for space resection
3. **Quality Analysis**: Compare vocab retrieval quality vs GPS-only or VLAD

For dataset-specific tuning, experiment with:
- Vocabulary size (k, L)
- Top-k value
- Hybrid strategy combinations
