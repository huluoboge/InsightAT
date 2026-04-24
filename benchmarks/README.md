# Benchmarks

Scripts to prepare **ETH3D-style** layouts, batch-run **COLMAP** (sparse SfM only) and **InsightAT** (`isat_sfm`), compare reconstructions to **GT** (and optionally COLMAP vs InsightAT), and plot summary figures.

Run all commands from the **repository root** so `import benchmarks....` works.

## Dependencies

- **Python 3**, **NumPy** (`pip install numpy`)
- **Matplotlib** (optional, for figures: `pip install matplotlib`)
- **COLMAP** on `PATH`, or set **`COLMAP_BIN_DIR`**
- **InsightAT** built with `isat_sfm` (set **`ISAT_BIN_DIR`** to the directory containing it)
- **`wget`** or **`curl`** if you use the optional ETH3D download helper script
- **`p7zip-full`** (or `7z`) to extract `.7z` archives for `prepare_datasets.py --extract`

## Recording your hardware (recommended)

Published timings are **machine-specific**. When you share numbers or regenerate README figures, record at least:

| Field | Example (this repo’s reference run) |
|-------|-------------------------------------|
| GPU | NVIDIA **GTX 1060** (6 GB) — older consumer card; large images / dense SIFT pyramids are VRAM-sensitive |
| Driver / CUDA | (fill in `nvidia-smi` / `nvcc --version`) |
| COLMAP version | (fill in `colmap -h` or build hash) |
| InsightAT flags | e.g. **`--use-sift-gpu`** forwarded from `run_insightat_batch.py` |

### SIFT backend “fairness”

- **COLMAP** uses its **own** SIFT implementation (often CUDA-accelerated when built with CUDA); it is **not** the same third-party library as InsightAT’s optional **SiftGPU** path.
- If your goal is to compare **InsightAT’s SiftGPU path** to other experiments that also used **SiftGPU-class** extraction inside InsightAT, pass **`--use-sift-gpu`** to `run_insightat_batch.py` (requires an InsightAT build with **`INSIGHTAT_ENABLE_SIFTGPU=ON`**).
- The default InsightAT pipeline uses **PopSift**; use **`--use-pop-sift`** only if you need to force it explicitly.

## Dataset layout

After `benchmarks/eth3d/prepare_datasets.py`, the dataset root `<dataset>` looks like:

```
<dataset>/
  zip/                 # optional: *_dslr_jpg.7z (see download script below)
  raw/                 # extracted archives
  scenes/
    <scene_name>/
      images/          # often a symlink to .../dslr_images/
      gt/              # ETH3D calibration in COLMAP text format (symlink to dslr_calibration_jpg/)
      results/
        colmap/        # created by the COLMAP batch script
        insightat/     # created by the InsightAT batch script
```

| Tool | Main outputs |
|------|----------------|
| COLMAP | `scenes/<scene>/results/colmap/workspace/` (includes `sparse/0/`) |
| InsightAT | `scenes/<scene>/results/insightat/work/` (`incremental_sfm/`, `sfm_timing.json`, …) |

## Optional: download ETH3D training archives

Official listing: [https://www.eth3d.net/datasets](https://www.eth3d.net/datasets) (respect the **CC BY-NC-SA 4.0** license).

Helper (subset of high-res multi-view **training** DSLR JPG packs):

```bash
chmod +x benchmarks/eth3d/download_eth3d_training_scenes.sh   # once
./benchmarks/eth3d/download_eth3d_training_scenes.sh /path/to/eth3d_root
```

- Downloads `*_dslr_jpg.7z` into `<dataset>/zip/`.
- Override the scene list: `SCENES="courtyard pipes" ./benchmarks/eth3d/download_eth3d_training_scenes.sh ...`
- Dry run: `DRY_RUN=1 ./benchmarks/eth3d/download_eth3d_training_scenes.sh ...`
- Mirror override: `ETH3D_BASE_URL=https://www.eth3d.net/data ./benchmarks/eth3d/download_eth3d_training_scenes.sh ...`

Then extract + symlink:

```bash
python3 benchmarks/eth3d/prepare_datasets.py -d /path/to/eth3d_root --extract
# or, if you already extracted into raw/:
python3 benchmarks/eth3d/prepare_datasets.py -d /path/to/eth3d_root
```

## Recommended pipeline

Order: **prepare data → batch COLMAP → batch InsightAT → compare → plots** (the first two can be swapped, but both sparse models must exist before GT comparison).

### 1) Batch COLMAP (sparse only; no dense / MVS)

Pipeline per scene: `feature_extractor` → `exhaustive_matcher` → `mapper` → optional `model_converter` to **TEXT** (`images.txt`, …) for the Python tools here.

```bash
export COLMAP_BIN_DIR=/path/to/colmap/install/bin

python3 benchmarks/sfm_compare/run_colmap_batch.py \
  -d /path/to/eth3d_root \
  --summary /path/to/colmap_batch_summary.json
```

`run.json` per scene includes **`elapsed_sfm_s`** (feature + match + mapper) and **`elapsed_s`** (includes BIN→TXT export when enabled).

By default each scene’s `results/colmap/workspace/` is **deleted and recreated**; use **`--reuse-workspace`** only if you know why you need it.

### 2) Batch InsightAT (`isat_sfm`)

```bash
export ISAT_BIN_DIR=/path/to/InsightAT/build

# Default PopSift path (no extra SIFT flags):
python3 benchmarks/sfm_compare/run_insightat_batch.py \
  -d /path/to/eth3d_root \
  --summary /path/to/insightat_batch_summary.json

# Optional: headless GLSL backends
#   --extract-backend glsl --match-backend glsl

# Optional: SiftGPU path (requires SiftGPU-enabled build)
python3 benchmarks/sfm_compare/run_insightat_batch.py \
  -d /path/to/eth3d_root \
  --use-sift-gpu \
  --summary /path/to/insightat_batch_summary.json
```

Use **`--reuse-work`** to keep an existing `work/`. **`--skip-done`** resumes a batch (skips scenes whose `run.json` has `exit_code == 0`).

### 3) Compare camera centers (Umeyama)

**Default (`--ref-source gt`, can be omitted)** — reference is **`scenes/<scene>/gt/images.txt`**. Writes **two** JSON files in `<dataset>/`:

| File | Meaning |
|------|---------|
| **`compare_gt_colmap.json`** | GT vs **COLMAP** sparse `sparse/0/images.txt` |
| **`compare_gt_insightat.json`** | GT vs **InsightAT** export `.../sparse/0/images.txt` |

Each successful row includes `"reference": "gt"` and `"estimate": "colmap"` or `"insightat"`.

`-o/--output` is **ignored** in GT mode (fixed filenames). stderr prints a note if you pass `-o` anyway.

```bash
python3 benchmarks/sfm_compare/compare_dataset_batch.py -d /path/to/eth3d_root
```

**Legacy** (`--ref-source colmap`): COLMAP sparse as reference vs InsightAT only → **`compare_colmap_insightat.json`** (or `-o` path).

```bash
python3 benchmarks/sfm_compare/compare_dataset_batch.py \
  -d /path/to/eth3d_root \
  --ref-source colmap \
  -o /path/to/compare_colmap_insightat.json
```

### 4) Plots (for `doc/images/benchmarks/`)

```bash
pip install matplotlib
python3 benchmarks/sfm_compare/plot_eth3d_benchmark.py -d /path/to/eth3d_root
# optional: --out-dir /other/dir
# optional: --figure-suffix <tag> (append to PNG basenames so default files are not overwritten)
# optional: --only wall,points,gt-colmap,gt-insightat,legacy (subset; default: all)
```

Outputs (typical):

- `eth3d_wall_time_colmap_vs_insightat.png`
- `eth3d_sparse_points_colmap_vs_insightat.png`
- `eth3d_gt_vs_colmap_alignment.png` — if **`compare_gt_colmap.json`** exists
- `eth3d_gt_vs_insightat_alignment.png` — if **`compare_gt_insightat.json`** exists
- `eth3d_camera_center_alignment.png` — only if **neither** GT JSON exists but **`compare_colmap_insightat.json`** does (legacy)

Timing plots only include scenes where **both** COLMAP and InsightAT batch rows report `exit_code == 0`.

### Single-pair sparse compare

```bash
python3 benchmarks/sfm_compare/compare_colmap_sparse.py \
  --ref  /path/to/.../sparse/0 \
  --est  /path/to/.../sparse/0 \
  -o metrics.json
```

## Environment variables

| Variable | Meaning |
|----------|---------|
| `COLMAP_BIN_DIR` | Directory containing the `colmap` executable (prepended to `PATH` in batch scripts) |
| `ISAT_BIN_DIR` | Directory containing `isat_sfm` and sibling tools |

## Layout of this folder

- **`eth3d/`** — `prepare_datasets.py`, optional `download_eth3d_training_scenes.sh`
- **`sfm_compare/`** — batch runners + `compare_*.py` + `colmap_sparse.py`, `align_similarity.py`

## What the metrics mean

- **Image pairing**: matches **image basenames** in `images.txt`; only images present in **both** models are used.
- **Alignment**: fits a **similarity** mapping **reference camera centers → estimate camera centers**, then reports RMSE / median / max error and scale.
- **Not** the official ETH3D leaderboard submission; it is a **practical** GT or cross-model consistency check using the released calibration bundles.
- **Bar charts (RMSE vs median)**: the two colors are **two statistics of the same residual vector**, not “green = software A, red = software B”.

## Other tools in the repo

- `scripts/isat_report.py` — HTML report from a `work/` directory  
- `scripts/visualize_vlad_retrieval.py` — VLAD retrieval visualization (separate pipeline)
