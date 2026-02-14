# DBoW3 Dataset Download and Training Scripts

This directory contains automated scripts for downloading datasets and training DBoW3 vocabulary trees for InsightAT image retrieval.

## Quick Start

https://zhuanlan.zhihu.com/p/158763102


```bash
# 1. Download datasets (with auto-extraction)
./download_datasets.sh --all --extract

# 2. Train vocabulary tree
./train_vocabulary.sh

# 3. Use vocabulary in InsightAT
../../build/isat_retrieve \
    --database-path scene.db \
    --image-path query.jpg \
    --retrieval-strategy vocab \
    --vocab-file vocabulary_k10_L6.dbow3
```

## Scripts Overview

### `download_datasets.sh`

Downloads public datasets suitable for vocabulary tree training.

**Supported Datasets:**

| Dataset | Size | Images | Type | Auto-Download |
|---------|------|--------|------|---------------|
| EuRoC MAV | ~8GB | ~20K | Drone indoor/outdoor | ✅ |
| COCO val2017 | ~1GB | 5K | Diverse scenes | ✅ |
| **TUM RGB-D** | ~5GB | ~30K | Indoor SLAM | ❌ Manual (Baidu) |
| **KITTI** | ~22GB | ~20K | Street/driving scenes | ❌ Manual (Baidu) |
| **DSO** | ~2GB | ~10K | Monocular sequences | ❌ Manual (Baidu) |
| **Mono** | ~3GB | ~15K | Monocular VO | ❌ Manual (Baidu) |
| UZH-FPV | ~10GB | ~15K | High-speed drone racing | ❌ Manual |
| Mid-Air | ~50GB+ | ~100K+ | Aerial simulation | ❌ Manual |

**Usage:**

```bash
# Download all auto-downloadable datasets
./download_datasets.sh --all --extract

# Download specific datasets
./download_datasets.sh --euroc --extract
./download_datasets.sh --coco --extract

# Keep archives after extraction
./download_datasets.sh --all --extract --keep-zip
```

**Features:**
- ✅ **Incremental download**: Resume interrupted downloads automatically
- ✅ **Auto-extraction**: Optionally extract archives after download
- ✅ **Progress tracking**: Visual progress bars for downloads
- ✅ **Error handling**: Retry on network failures

### `train_vocabulary.sh`

One-click pipeline to train DBoW3 vocabulary trees from downloaded data.

**Usage:**

```bash
# Basic training with defaults (k=10, L=6)
./train_vocabulary.sh

# Custom parameters
./train_vocabulary.sh \
    --branching 16 \
    --depth 8 \
    --max-images 10000 \
    --output vocabulary_k16_L8.dbow3

# Use specific data directory
./train_vocabulary.sh --data-dir /path/to/custom/images
```

**Parameters:**

| Option | Description | Default | Recommendation |
|--------|-------------|---------|----------------|
| `--branching` | Branching factor (k) | 10 | 6-16 depending on dataset size |
| `--depth` | Tree depth (L) | 6 | 5-8 depending on accuracy needs |
| `--threads` | CPU threads | Auto (nproc) | Use all available cores |
| `--max-images` | Limit training images | Unlimited | 5K-10K often sufficient |
| `--output` | Output file path | `vocabulary_k10_L6.dbow3` | Descriptive names |

## Dataset Details

### EuRoC MAV Dataset

**Source**: ETH Zurich - Autonomous Systems Lab  
**URL**: http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset

**Characteristics:**
- Indoor and outdoor flights
- Stereo camera images (monochrome)
- SLAM-ready (synchronized IMU, groundtruth)
- Variety of environments (Machine Hall, Vicon Room)

**Downloaded Sequences:**
- MH_01_easy, MH_02_easy (Machine Hall)
- V1_01_easy, V2_01_easy (Vicon Rooms)

### COCO Validation 2017

**Source**: Microsoft Common Objects in Context  
**URL**: http://cocodataset.org/

**Characteristics:**
- Diverse everyday scenes
- High-quality RGB images
- Object annotations (not used for vocabulary training)
- Good for general-purpose vocabulary

### Manual Download Datasets

#### UZH-FPV Drone Racing Dataset

**Why manual?** Requires acceptance of terms on website  
**URL**: http://rpg.ifi.uzh.ch/uzh-fpv.html

**Installation:**
```bash
# After downloading from website
mkdir -p ../../data/dbow_training/uzh_fpv
unzip indoor_forward_3_snapdragon.zip -d ../../data/dbow_training/uzh_fpv/
```

**Recommended for**: High-speed drone applications, outdoor aerial

#### Mid-Air Dataset

**Why manual?** Registration required, very large size  
**URL**: https://midair.ulg.ac.be/

**Installation:**
```bash
# Download only "Stereo Images" subset (~50GB) or specific sequences
mkdir -p ../../data/dbow_training/midair
# Extract downloaded files to above directory
```

**Recommended for**: Large-scale vocabulary training, aerial simulations

#### TUM RGB-D Dataset (推荐！)

**Why manual?** Large size, available on Baidu Pan  
**Baidu Pan**: https://pan.baidu.com/s/1nwXtGqH (密码: lsgr)

**Characteristics:**
- RGB-D sequences from TU Munich
- Indoor scenes (office, living room, etc.)
- SLAM benchmark standard
- High-quality RGB images suitable for ORB/SIFT features

**Installation:**
```bash
# After downloading from Baidu Pan
mkdir -p ../../data/dbow_training/tum
# Extract all sequences to tum directory
tar -xzf rgbd_dataset_*.tar.gz -C ../../data/dbow_training/tum/
```

**Recommended for**: Indoor SLAM, general vocabulary training, RGB features

#### KITTI Dataset (推荐！)

**Why manual?** Large size, available on Baidu Pan  
**Baidu Pan**: https://pan.baidu.com/s/1htFmXDE (密码: uu20)

**Characteristics:**
- Autonomous driving benchmark
- Street scenes, varied lighting
- High-resolution stereo images
- Outdoor urban environments

**Installation:**
```bash
# After downloading from Baidu Pan
mkdir -p ../../data/dbow_training/kitti
# Extract sequences to kitti directory
unzip data_odometry_color.zip -d ../../data/dbow_training/kitti/
```

**Recommended for**: Outdoor scenes, adding diversity to vocabulary

#### DSO Dataset

**Why manual?** Available on Baidu Pan  
**Baidu Pan**: https://pan.baidu.com/s/1eSRmeZK (密码: 6x5b)

**Characteristics:**
- Direct Sparse Odometry sequences
- Photometric calibration
- Indoor and outdoor scenes

**Installation:**
```bash
mkdir -p ../../data/dbow_training/dso
# Extract to dso directory
```

**Recommended for**: Direct method comparison, monocular sequences

#### Mono Dataset

**Why manual?** Available on Baidu Pan  
**Baidu Pan**: https://pan.baidu.com/s/1jKaNB3C (密码: u57r)

**Characteristics:**
- Monocular visual odometry sequences
- Various indoor/outdoor scenes

**Installation:**
```bash
mkdir -p ../../data/dbow_training/mono
# Extract to mono directory
```

**Recommended for**: Monocular SLAM vocabulary training

## Training Recommendations

### Dataset Selection Strategy

| Application | Recommended Datasets | Rationale |
|-------------|---------------------|-----------|
| **Drone/UAV** | EuRoC + UZH-FPV | Aerial perspective, drone motion |
| **Indoor SLAM** | TUM + EuRoC | Indoor benchmark + drone indoor |
| **Outdoor/Driving** | KITTI + COCO | Street view + diverse scenes |
| **General purpose** | COCO + TUM + EuRoC | Maximum diversity (indoor/outdoor/aerial) |
| **Monocular VO** | Mono + DSO + EuRoC | Monocular sequences, varied motion |
| **Large-scale aerial** | Mid-Air + EuRoC subset | Massive aerial data, variety |
| **快速测试** | EuRoC (1 sequence) + COCO | Auto-download, ~9GB total |

**最佳实践组合（推荐）：**

```bash
# 方案1: 快速自动下载（适合测试）
./download_datasets.sh --all --extract
# 结果: EuRoC + COCO = ~9GB, 25K images

# 方案2: 室内+航空（下载 TUM + EuRoC）
# EuRoC (auto) + TUM (manual)
# 结果: ~13GB, 50K images, 覆盖室内SLAM和航空应用

# 方案3: 全场景覆盖（推荐大规模训练）
# EuRoC + COCO + TUM + KITTI
# 结果: ~36GB, 75K+ images, 最佳多样性

# 方案4: 最大化（所有数据集）
# All datasets
# 结果: ~100GB+, 150K+ images, 生产级词汇树
```

### Parameter Tuning

**For Different Database Sizes:**

```bash
# Small databases (<1K images)
./train_vocabulary.sh --branching 6 --depth 5

# Medium databases (1K-10K images)
./train_vocabulary.sh --branching 10 --depth 6  # Default

# Large databases (>10K images)
./train_vocabulary.sh --branching 16 --depth 8

# Very large databases (>100K images)
./train_vocabulary.sh --branching 20 --depth 10
```

**Training Time vs Accuracy Trade-off:**

- **Faster training**: Use `--max-images 5000`, smaller k/L
- **Better accuracy**: Use all images, larger k/L (more training time)
- **Balanced**: Default settings (k=10, L=6) with 10K-20K images

### Vocabulary File Sizes

Typical vocabulary sizes:

| Parameters | Vocabulary Nodes | File Size | Training Time (est) |
|------------|------------------|-----------|---------------------|
| k=6, L=5 | ~7,776 | ~20-50MB | ~10-30 min |
| k=10, L=6 | ~1,000,000 | ~50-200MB | ~30-90 min |
| k=16, L=8 | ~4,294,967,296 | ~500MB-2GB | ~2-6 hours |

**Note**: Actual sizes depend on descriptor dimensionality (ORB=256 bits)

## Directory Structure After Download

```
InsightAT/
├── scripts/dbow/
│   ├── download_datasets.sh
│   ├── train_vocabulary.sh
│   └── README.md (this file)
│
└── data/dbow_training/
    ├── euroc/
    │   ├── MH_01_easy/
    │   ├── MH_02_easy/
    │   ├── V1_01_easy/
    │   └── V2_01_easy/
    ├── coco/
    │   └── val2017/
    ├── uzh_fpv/ (manual download)
    ├── midair/ (manual download)
    └── image_list.txt (auto-generated during training)
```

## Troubleshooting

### Download Issues

**Problem**: Download fails or times out  
**Solution**: Re-run script - it will resume from where it stopped (wget -c flag)

**Problem**: Extraction fails  
**Solution**: Check disk space, ensure unzip is installed

### Training Issues

**Problem**: "No images found"  
**Solution**: Ensure datasets are extracted (`--extract` flag or manual extraction)

**Problem**: Out of memory during training  
**Solution**: Reduce `--max-images` or use smaller k/L parameters

**Problem**: Training too slow  
**Solution**: 
- Reduce image count: `--max-images 5000`
- Use smaller parameters: `--branching 8 --depth 5`
- Ensure `--threads` matches CPU cores

### Usage Issues

**Problem**: isat_retrieve doesn't recognize vocab file  
**Solution**: Check file path, ensure file extension is `.dbow3`

**Problem**: Vocabulary performance poor  
**Solution**: 
- Train with more diverse images
- Increase k and L parameters
- Mix multiple datasets for training

## Performance Benchmarks

**Query Performance** (approximate, hardware-dependent):

| Database Size | Vocabulary k/L | Query Time | Recall@10 |
|---------------|----------------|------------|-----------|
| 1K images | k=10, L=6 | ~20ms | >90% |
| 10K images | k=10, L=6 | ~100ms | >85% |
| 100K images | k=16, L=8 | ~500ms | >80% |

**Training Performance** (8-core CPU, 16GB RAM):

| Images | k/L | Training Time | Memory Usage |
|--------|-----|---------------|--------------|
| 5K | k=10, L=6 | ~15 min | ~4GB |
| 10K | k=10, L=6 | ~40 min | ~6GB |
| 20K | k=10, L=6 | ~90 min | ~8GB |

## References

- **DBoW3**: https://github.com/rmsalinas/DBow3
- **EuRoC Dataset**: http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- **COCO Dataset**: http://cocodataset.org/
- **ORB-SLAM3 Vocabulary**: https://github.com/UZ-SLAMLab/ORB_SLAM3 (reference implementation)

## License

These scripts are part of InsightAT and follow the project's license. Downloaded datasets have their own licenses - please review each dataset's terms of use.

---

For more information, see:
- `doc/tools/VOCAB_TREE_QUICKSTART.md` - User guide
- `doc/tools/VOCAB_TREE_IMPLEMENTATION.md` - Technical details
- `VOCAB_TREE_COMPLETE.md` - Complete deployment guide
