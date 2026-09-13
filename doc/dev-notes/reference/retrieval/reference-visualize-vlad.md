# VLAD检索结果可视化工具

## 概述

`visualize_vlad_retrieval.py` 是一个Python工具，用于生成VLAD检索系统的可视化HTML报告。它能够：

- 📊 **加载VLAD向量**: 从`.isat_vlad`缓存文件读取预计算的VLAD特征
- 🔍 **执行检索**: 对每个查询图像检索top-k个最相似的图像
- 📝 **生成HTML报告**: 创建交互式可视化报告，显示查询图像和检索结果
- 🎨 **距离可视化**: 用颜色编码表示检索质量（绿色=优秀，橙色=中等，红色=较差）

## 使用场景

- **快速评估**: 在进行完整消融实验之前，直观查看检索系统的质量
- **问题诊断**: 人工检查检索结果，发现潜在问题（如尺度不变性、光照变化等）
- **参数调优**: 比较不同参数（如聚类数量、PCA维度）对检索质量的影响
- **演示展示**: 生成可分享的HTML报告，展示检索系统效果

## 前置条件

### 1. Python依赖

```bash
# 基础依赖
pip install numpy

# 可选：如果需要处理图像路径
pip install pillow
```

### 2. 数据准备

运行此工具前，需要完成以下步骤：

```bash
# Step 1: 提取SIFT特征（检索专用，降采样到1024）
./build/isat_extract \
    -i images.json \
    -o features_retrieval/ \
    -n 1500 \
    --resize 1024 \
    --only-retrieval

# Step 2: 训练VLAD codebook
./build/isat_train_vlad \
    -f features_retrieval/ \
    -o codebook_128.vlad \
    -k 128 \
    --pca-dim 512 \
    --scale-weighted

# Step 3: 运行检索，生成VLAD缓存
./build/isat_retrieve \
    -f features_retrieval/ \
    -i images.json \
    -o pairs.json \
    --strategy vlad \
    --vlad-codebook codebook_128.vlad \
    --vlad-cache vlad_cache/ \
    --top-k 20
```

完成后，你将拥有：
- `features_retrieval/`: SIFT特征文件 (*.isat_feat)
- `codebook_128.vlad`: VLAD聚类中心和PCA模型
- `vlad_cache/`: VLAD向量缓存 (*.isat_vlad)
- `images.json`: 图像列表（包含路径信息）

## 基本用法

### 最简单的用法

```bash
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir ./vlad_cache \
    --images ./images.json \
    --output ./retrieval_report.html
```

在浏览器中打开生成的 `retrieval_report.html` 查看结果。

### 常用选项

```bash
# 只显示top-5检索结果（默认是top-10）
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir ./vlad_cache \
    --images ./images.json \
    --output report_top5.html \
    --top-k 5

# 只处理前20个查询图像（快速预览）
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir ./vlad_cache \
    --images ./images.json \
    --output quick_preview.html \
    --max-queries 20

# 完整报告，显示top-15检索结果
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir ./vlad_cache \
    --images ./images.json \
    --output full_report_top15.html \
    --top-k 15
```

## 命令行参数

| 参数 | 必需 | 默认值 | 说明 |
|------|------|--------|------|
| `--vlad-dir` | ✅ | - | VLAD缓存目录（包含 `*.isat_vlad` 文件） |
| `--images` | ✅ | - | 图像列表JSON文件路径 |
| `--output` | ✅ | - | 输出HTML文件路径 |
| `--top-k` | ❌ | 10 | 每个查询返回的检索结果数量 |
| `--max-queries` | ❌ | 全部 | 处理的最大查询图像数量（用于快速预览） |

## 图像列表JSON格式

`images.json` 文件格式示例：

```json
{
  "images": [
    {
      "id": 1,
      "path": "/path/to/image_001.jpg",
      "camera_id": 1
    },
    {
      "id": 2,
      "path": "/path/to/image_002.jpg",
      "camera_id": 1
    }
  ]
}
```

**字段说明**:
- `id`: 图像唯一标识符（必需，用于匹配VLAD文件）
- `path`: 图像文件绝对路径（必需，用于HTML显示）
- `camera_id`: 相机ID（可选，工具不使用此字段）

## HTML报告功能

生成的HTML报告包含：

### 1. 统计摘要

- 查询图像总数
- Top-K设置
- 总计检索对数

### 2. 距离颜色编码

- 🟢 **绿色边框**: 距离 < 0.5（优秀匹配）
- 🟡 **橙色边框**: 0.5 ≤ 距离 < 1.0（中等匹配）
- 🔴 **红色边框**: 距离 ≥ 1.0（较差匹配）

### 3. 每个查询的检索结果

- **查询图像**: 蓝色边框，较大显示
- **检索结果**: 按距离排序，显示排名和距离值
- **鼠标悬停**: 检索结果图像会放大并显示阴影

### 4. 交互功能

- 自适应布局（网格显示）
- 图像加载失败时显示占位符
- 完整路径信息（悬停查看）

## 典型工作流程

### 场景1: 快速质量检查

在训练VLAD模型后，快速查看检索质量：

```bash
# 1. 运行检索（生成VLAD缓存）
./build/isat_retrieve \
    -f features_retrieval/ \
    -i images.json \
    -o pairs.json \
    --strategy vlad \
    --vlad-codebook codebook_128.vlad \
    --vlad-cache vlad_cache/ \
    --top-k 20

# 2. 快速预览前10个查询
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir vlad_cache/ \
    --images images.json \
    --output quick_check.html \
    --max-queries 10 \
    --top-k 5

# 3. 在浏览器中打开
firefox quick_check.html  # 或 chrome/safari
```

### 场景2: 对比不同参数

比较不同聚类数量的效果：

```bash
# 训练不同的codebook
./build/isat_train_vlad -f features/ -o codebook_64.vlad -k 64 --pca-dim 512
./build/isat_train_vlad -f features/ -o codebook_128.vlad -k 128 --pca-dim 512
./build/isat_train_vlad -f features/ -o codebook_256.vlad -k 256 --pca-dim 512

# 运行检索
./build/isat_retrieve ... --vlad-codebook codebook_64.vlad --vlad-cache vlad_cache_64/
./build/isat_retrieve ... --vlad-codebook codebook_128.vlad --vlad-cache vlad_cache_128/
./build/isat_retrieve ... --vlad-codebook codebook_256.vlad --vlad-cache vlad_cache_256/

# 生成对比报告
python scripts/visualize_vlad_retrieval.py --vlad-dir vlad_cache_64/ ... --output report_k64.html
python scripts/visualize_vlad_retrieval.py --vlad-dir vlad_cache_128/ ... --output report_k128.html
python scripts/visualize_vlad_retrieval.py --vlad-dir vlad_cache_256/ ... --output report_k256.html

# 人工对比三个HTML文件
```

### 场景3: 完整评估报告

生成完整的评估报告（所有图像，top-15结果）：

```bash
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir vlad_cache/ \
    --images images.json \
    --output full_evaluation_report.html \
    --top-k 15

# 报告可能较大，建议分批处理或使用--max-queries限制
```

## 常见问题

### Q1: 为什么有些图像显示"图像未找到"？

**原因**: `images.json`中的`path`字段指向的文件不存在或路径错误。

**解决方法**:
1. 检查`images.json`中的路径是否正确
2. 确保路径是绝对路径
3. 如果图像已移动，更新JSON文件中的路径

### Q2: VLAD文件加载失败

**错误信息**: `Warning: Invalid VLAD file (wrong magic)`

**原因**:
- 文件损坏
- 非VLAD文件（如误将特征文件放入VLAD目录）
- 版本不兼容

**解决方法**:
1. 重新运行 `isat_retrieve` 生成VLAD缓存
2. 检查VLAD目录中是否只包含 `*.isat_vlad` 文件

### Q3: 报告生成很慢

**原因**: 处理大量图像（例如 >1000张）

**解决方法**:
```bash
# 使用 --max-queries 限制查询数量
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir vlad_cache/ \
    --images images.json \
    --output report.html \
    --max-queries 50  # 只处理前50个查询
```

### Q4: HTML文件很大，浏览器打开很慢

**原因**: 每个查询显示过多检索结果（--top-k过大）

**解决方法**:
```bash
# 减少top-k值
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir vlad_cache/ \
    --images images.json \
    --output report.html \
    --top-k 5  # 只显示top-5
```

## 性能提示

### 大规模数据集处理

对于大型数据集（>1000张图像），建议：

1. **分批处理**: 使用 `--max-queries` 分批生成报告
2. **减少top-k**: 只显示top-5或top-3
3. **并行处理**: 手动分割图像列表，并行生成多个报告

示例脚本（分批处理）：

```bash
# 每批100个查询
for i in {0..9}; do
    start=$((i * 100))
    python scripts/visualize_vlad_retrieval.py \
        --vlad-dir vlad_cache/ \
        --images images.json \
        --output report_batch_${i}.html \
        --max-queries 100 \
        --top-k 5
done
```

## 输出示例

```
============================================================
VLAD检索可视化工具
============================================================

📂 Loading images from images.json...
Loaded 500 images from images.json

📊 Loading VLAD vectors from vlad_cache/...
Successfully loaded VLAD vectors for 500/500 images

🔍 Performing retrieval (top-10) for 500 query images...
  Processing query 10/500...
  Processing query 20/500...
  ...
  Processing query 500/500...

📝 Generating HTML report...
✅ HTML report generated: retrieval_report.html

============================================================
✅ 完成!
============================================================
查询图像数: 500
Top-K: 10
输出文件: retrieval_report.html

在浏览器中打开查看结果:
  file:///home/user/InsightAT/retrieval_report.html
============================================================
```

## 下一步

此工具适用于**定性评估**（人工检查）。如需**定量评估**，可考虑：

1. **Recall@K计算**: 如果你有ground truth配对信息
2. **消融实验**: 系统地比较不同参数配置
3. **性能基准测试**: 与其他检索方法（如VocabTree）对比

这些功能可在后续开发中添加，或通过单独的评估脚本实现。

## 文件位置

- **脚本位置**: `scripts/visualize_vlad_retrieval.py`
- **文档位置**: `doc/dev-notes/tools/visualize_vlad_retrieval.md`
- **相关工具**: `isat_retrieve`, `isat_train_vlad`, `isat_extract`

## 更新历史

- **v1.0** (2026-02-14): 初始版本，支持基本的VLAD检索可视化
