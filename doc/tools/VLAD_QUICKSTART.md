# VLAD Visual Retrieval - Quick Start Guide

## Overview

VLAD (Vector of Locally Aggregated Descriptors) provides visual similarity-based image retrieval when GPS data is unavailable. This guide shows how to train a visual vocabulary and use VLAD for image pair generation.

---

## Workflow

```
[Feature Files]
     ↓
  isat_train_vlad → [Visual Codebook (.vcbt)]
     ↓
  isat_retrieve --strategy vlad → [Image Pairs]
```

---

## Step 1: Train Visual Vocabulary

Train a k-means visual codebook from your feature files:

```bash
./isat_train_vlad \
  -f features/ \
  -o codebook_64.vcbt \
  -k 64 \
  -n 500000 \
  -p 500 \
  -v
```

**Parameters:**
- `-f features/`: Directory containing `.isat_feat` files
- `-o codebook_64.vcbt`: Output codebook file
- `-k 64`: Number of k-means clusters (typical: 64-128)
- `-n 500000`: Max total descriptors for training (default: 1M)
- `-p 500`: Max descriptors per image (default: 500)
- `-v`: Verbose logging

**Training Time:**
- 100 images: ~10-30 seconds
- 1000 images: ~2-5 minutes
- 10000 images: ~20-60 minutes

**Recommendations:**
- Use 64-128 clusters for most datasets
- Sample 500-1000 descriptors per image
- Train on representative subset (10-20% of dataset is sufficient)

---

## Step 2: VLAD Retrieval

Use the trained codebook for visual image retrieval:

```bash
./isat_retrieve \
  -f features/ \
  -o pairs_vlad.json \
  -s vlad \
  --vlad-codebook codebook_64.vcbt \
  --vlad-cache vlad_cache/ \
  --vlad-top-k 20 \
  -v
```

**Parameters:**
- `-s vlad`: Use VLAD visual retrieval strategy
- `--vlad-codebook codebook_64.vcbt`: Pre-trained codebook file
- `--vlad-cache vlad_cache/`: Cache directory for `.isat_vlad` files (optional but recommended)
- `--vlad-top-k 20`: Number of most similar images per query

**Output:**
- JSON file with image pairs sorted by visual similarity
- Pairs include `"visual_similarity"` score in [0, 1]

---

## Step 3: Hybrid GPS + VLAD Strategy

Combine GPS spatial proximity with visual similarity:

```bash
./isat_retrieve \
  -f features/ \
  -i images.json \
  -o pairs_hybrid.json \
  -s "gps+vlad" \
  -d 200 \
  --vlad-codebook codebook_64.vcbt \
  --vlad-cache vlad_cache/ \
  --vlad-top-k 15
```

**Benefits:**
- GPS finds nearby images (fast, accurate if available)
- VLAD fills gaps where GPS unavailable or ambiguous
- Score merging: pairs found by both strategies get higher scores

---

## Cache Management

### VLAD Vector Caching

VLAD vectors are cached in `.isat_vlad` files to avoid recomputation:

```
vlad_cache/
├── img_001.isat_vlad
├── img_002.isat_vlad
└── ...
```

**Cache Benefits:**
- 10-100x speedup on repeated retrieval
- Cache size: ~8-16 KB per image (for K=64 clusters)
- Automatically created on first retrieval

**Clear Cache:**
```bash
rm vlad_cache/*.isat_vlad
```

### When to Retrain Codebook

Retrain if:
- Switching to different scene type (urban → nature)
- Adding significantly different images (>50% new data)
- Changing number of clusters

**No need to retrain if:**
- Adding similar images incrementally
- Same sensor/camera used

---

## Performance Tuning

###clusters (K)**

| K | Training Time | VLAD Vector Size | Accuracy | Recommended For |
|---|---------------|------------------|----------|-----------------|
| 32 | Fast | 16 KB | Moderate | Quick prototyping |
| 64 | **Moderate** | **32 KB** | **Good** | **General use** |
| 128 | Slow | 64 KB | Better | Large diverse datasets |
| 256 | Very slow | 128 KB | Best | >10K images |

### Top-K Selection

| Top-K | Coverage | Redundancy | Matching Time |
|-------|----------|------------|---------------|
| 10 | Low | Low | Fast |
| **20** | **Moderate** | **Low** | **Moderate** |
| 50 | High | Medium | Slow |
| 100+ | Very high | High | Very slow |

**Rule of thumb:** Set `top-k = 10-30` for most cases.

---

## Troubleshooting

### Issue: "Failed to load VLAD codebook"

**Cause:** Codebook file not found or corrupted

**Solution:**
1. Check file path: `ls -lh codebook_64.vcbt`
2. Verify file size >0
3. Retrain if needed

### Issue: "No images with valid GNSS data" (in GPS+VLAD mode)

**Expected behavior:** VLAD strategy should still run

**Verify:** Check log for "VLAD retrieval: generated X pairs"

### Issue: VLAD retrieval very slow

**Solutions:**
1. Enable caching: `--vlad-cache vlad_cache/`
2. Reduce top-k: `--vlad-top-k 10`
3. Reduce cluster count (retrain with `-k 32`)

### Issue: Low retrieval quality

**Solutions:**
1. Increase clusters: Retrain with `-k 128`
2. Train on more diverse images
3. Combine with GPS: `-s "gps+vlad"`

---

## Example Workflows

### Workflow 1: Terrestrial Dataset (No GPS)

```bash
# 1. Extract features
./isat_extract -i images.json -o features/ -n 2000

# 2. Train VLAD codebook
./isat_train_vlad -f features/ -o vocab_urban.vcbt -k 64 -v

# 3. Retrieve pairs
./isat_retrieve \
  -f features/ \
  -o pairs.json \
  -s vlad \
  --vlad-codebook vocab_urban.vcbt \
  --vlad-cache vlad_cache/ \
  --vlad-top-k 20

# 4. Match pairs
./isat_match -i pairs.json -o matches/
```

### Workflow 2: Aerial Survey (GPS Available)

```bash
# 1. Extract features
./isat_extract -i images.json -o features/

# 2. Train VLAD for fallback
./isat_train_vlad -f features/ -o vocab_aerial.vcbt -k 64

# 3. Hybrid retrieval (GPS primary, VLAD fallback)
./isat_retrieve \
  -f features/ \
  -i images.json \
  -o pairs.json \
  -s "gps+vlad" \
  -d 200 \
  --vlad-codebook vocab_aerial.vcbt \
  --vlad-top-k 10

# 4. Match
./isat_match -i pairs.json -o matches/
```

### Workflow 3: Large Dataset (10K+ images)

```bash
# 1. Sample subset for training (10%)
ls features/*.isat_feat | head -1000 > train_list.txt
mkdir features_subset
while read f; do cp "$f" features_subset/; done < train_list.txt

# 2. Train on subset
./isat_train_vlad \
  -f features_subset/ \
  -o vocab_large.vcbt \
  -k 128 \
  -n 1000000 \
  -p 1000

# 3. Retrieve on full dataset with caching
./isat_retrieve \
  -f features/ \
  -o pairs.json \
  -s vlad \
  --vlad-codebook vocab_large.vcbt \
  --vlad-cache vlad_cache/ \
  --vlad-top-k 30
```

---

## Technical Notes

### VLAD Encoding Algorithm

1. **Cluster Assignment**: Assign each SIFT descriptor to nearest k-means centroid
2. **Residual Computation**: Compute difference (descriptor - centroid)
3. **Aggregation**: Sum residuals for each cluster
4. **Normalization**: L2-normalize final vector

**Formula:**
```
VLAD(I) = Normalize([∑_{x∈C1}(x-c1), ..., ∑_{x∈CK}(x-cK)])
```

### Similarity Scoring

- **L2 Distance**: `dist = ||VLAD1 - VLAD2||_2`
- **Similarity Score**: `score = exp(-dist)`
- Lower distance → Higher score

### File Formats

**Codebook (.vcbt):**
```
[Header: 16 bytes]
  - Magic: 0x56434254 ("VCBT")
  - Version: 1
  - Num clusters: K
  - Descriptor dim: 128
[Centroids: K × 128 × 4 bytes]
```

**VLAD Cache (.isat_vlad):**
```
[Header: 12 bytes]
  - Magic: 0x44414C56 ("VLAD")
  - Version: 1
  - Vector size: K × 128
[VLAD vector: K × 128 × 4 bytes]
```

---

## Best Practices

1. **Training Data:** Use diverse, high-altitude imagery
2. **Caching:** Always use `--vlad-cache` for production
3. **Codebook Reuse:** Same codebook works across similar datasets
4. **Hybrid Strategy:** Combine GPS+VLAD for robustness
5. **Parameter Tuning:** Start with K=64, top-k=20, adjust based on results

---

## Next Steps

- **Test on Real Data:** Evaluate retrieval quality on your dataset
- **Optimize Parameters:** Tune K and top-k for your use case
- **Integrate Matching:** Use generated pairs with `isat_match`
- **Compare Strategies:** Benchmark GPS vs VLAD vs Hybrid

For more details, see:
- [isat_retrieve Design Document](./isat_retrieve_DESIGN.md)
- [Test Results](./isat_retrieve_TEST_RESULTS.md)
