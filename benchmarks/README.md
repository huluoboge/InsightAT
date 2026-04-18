# Benchmarks

本目录包含数据集整理脚本，以及 **COLMAP** 与 **InsightAT（`isat_sfm`）** 的批量运行与稀疏模型对比工具。其中 COLMAP 批跑 **只做稀疏 SfM**（特征提取 + 匹配 + `mapper`），不包含稠密重建 / MVS。所有命令均在 **仓库根目录** 下执行（保证 `import benchmarks....` 可用）。

## 依赖

- **Python 3**，**NumPy**（`pip install numpy`）
- **COLMAP**：可执行文件在 `PATH` 中，或通过 `COLMAP_BIN_DIR` 指定安装目录（见下）
- **InsightAT**：已编译，`build/isat_sfm` 存在，或通过 `ISAT_BIN_DIR` 指定

## 数据目录约定

在使用 `benchmarks/eth3d/prepare_datasets.py` 整理 ETH3D 后，根目录（下称 `<dataset>`）形如：

```
<dataset>/
  scenes/
    <scene_name>/
      images/                 # 图像（常为指向 raw 的符号链接）
      gt/                     # 可选：真值标定（ETH3D）
      results/
        colmap/               # COLMAP 批跑输出（脚本创建）
        insightat/            # InsightAT 批跑输出（脚本创建）
```

批跑脚本写入的路径：

| 工具 | 工作目录 / 产物 |
|------|------------------|
| COLMAP | `scenes/<scene>/results/colmap/workspace/`（含 `sparse/0/`） |
| InsightAT | `scenes/<scene>/results/insightat/work/`（含 `incremental_sfm/`、`sfm_timing.json`） |

## 推荐执行顺序

按顺序完成：**准备数据 → 批量 COLMAP → 批量 InsightAT → 全库对比**（前两步可交换，但对比前两边都需已有 `sparse/0`）。

### 1. 准备数据集（ETH3D 示例）

若压缩包在 `<dataset>/zip/`，先解压再整理：

```bash
python3 benchmarks/eth3d/prepare_datasets.py -d /path/to/eth3d_root --extract
```

若已在 `<dataset>/raw/` 下解压完毕，只需整理目录与符号链接：

```bash
python3 benchmarks/eth3d/prepare_datasets.py -d /path/to/eth3d_root
```

### 2. 批量运行 COLMAP（仅稀疏 SfM，无稠密 / MVS）

脚本使用 **`feature_extractor` → `exhaustive_matcher` → `mapper`**，**不**调用 `automatic_reconstructor`（后者会继续做去畸变与 patch-match 稠密重建）。

COLMAP 3.4+ / 4.x 默认把稀疏模型写成 **二进制**（`images.bin` 等）。批跑在 mapper 成功后会自动执行 **`model_converter --output_type TXT`**，把 `sparse/0` 转为 **`images.txt` / `points3D.txt`**，供本仓库的 Python 统计与 `compare_*.py` 使用。若只想保留二进制，可加 **`--skip-text-export`**。

各场景的 `results/colmap/run.json` 含 **`steps`** 分步耗时，以及：
- **`elapsed_s`**：整段墙钟时间（SfM + BIN→TXT，与 `steps` 各步之和一致）
- **`elapsed_sfm_s`**：仅特征提取 + 匹配 + `mapper`（与 InsightAT 对比 SfM 时用）
- **`elapsed_text_export_s`**：仅 `model_converter`（格式导出，不计入「纯重建」对比时可忽略）

```bash
export COLMAP_BIN_DIR=/path/to/colmap/install/bin   # 含 colmap 可执行文件的目录

python3 benchmarks/sfm_compare/run_colmap_batch.py \
  -d /path/to/eth3d_root \
  --summary /path/to/colmap_batch_summary.json
```

可选：`--camera-model OPENCV`（默认），对应 `feature_extractor` 的 `ImageReader.camera_model`。

默认每次会 **删除并重建** 该场景的 `results/colmap/workspace/`，避免沿用失败或旧版跑出的中间文件；仅当需要保留目录时再使用 `--reuse-workspace`。

### 3. 批量运行 InsightAT（`isat_sfm`）

端到端墙钟时间写入 `results/insightat/run.json`；分步耗时见各场景 `work/sfm_timing.json`。

```bash
export ISAT_BIN_DIR=/path/to/InsightAT/build   # 含 isat_sfm 的目录

python3 benchmarks/sfm_compare/run_insightat_batch.py \
  -d /path/to/eth3d_root \
  --summary /path/to/isat_batch_summary.json
```

无 CUDA 时可加：`--extract-backend glsl --match-backend glsl`。环拍/物体扫描可酌情加 `--fix-intrinsics`。

默认每次会 **删除并重建** `results/insightat/work/`；需要保留工作目录时使用 `--reuse-work`。

### 4. 稀疏模型对比（相似变换对齐相机中心）

在 **同一数据集根目录** 下，对每个场景若同时存在 COLMAP 与 InsightAT 的 `images.txt`，则估计 Umeyama 相似变换并统计位姿中心误差（米）：

```bash
python3 benchmarks/sfm_compare/compare_dataset_batch.py \
  -d /path/to/eth3d_root \
  -o /path/to/compare_colmap_insightat.json
```

默认输出文件为 `<dataset>/compare_colmap_insightat.json`（可用 `-o` 覆盖）。

### 单场景手动对比

仅对比一对 `sparse/0` 目录时：

```bash
python3 benchmarks/sfm_compare/compare_colmap_sparse.py \
  --ref  /path/to/.../results/colmap/workspace/sparse/0 \
  --est  /path/to/.../results/insightat/work/incremental_sfm/colmap/sparse/0 \
  -o metrics.json
```

## 环境变量小结

| 变量 | 含义 |
|------|------|
| `COLMAP_BIN_DIR` | 包含 `colmap` 的目录（批跑时也会 prepend 到 `PATH`） |
| `ISAT_BIN_DIR` | 包含 `isat_sfm` 及 sibling 工具的目录 |

## 子目录说明

- **`eth3d/`** — `prepare_datasets.py`：从 ETH3D 压缩包/解压目录生成统一的 `scenes/` 布局。
- **`sfm_compare/`** — COLMAP / InsightAT 批跑与 `compare_*.py` 对比脚本；共享 `colmap_sparse.py`、`align_similarity.py`（解析 COLMAP 文本模型、Umeyama 对齐）。

## 指标说明（对比脚本）

- **匹配**：按 `images.txt` 中图像 **文件名（basename）** 对齐；仅统计两模型均存在的图像。
- **对齐**：将 **参考侧（ref）** 相机中心经相似变换对齐到 **估计侧（est）**，报告 RMSE、中位数误差、最大误差及尺度因子。
- **解释**：这是典型的「两重建互比」；**不是** ETH3D 官方真值评测。若需与 `gt/` 对比，需另行实现真值格式读取与对齐。

## 辅助脚本（仓库其它位置）

- `scripts/isat_report.py` — 从 `work/` 生成 HTML 质量报告  
- `scripts/visualize_vlad_retrieval.py` — VLAD 检索可视化（需单独流程产出 `.isat_vlad`）
