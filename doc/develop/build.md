# InsightAT

**Open-source incremental SfM with GPU-accelerated feature extraction and matching.**

InsightAT is an open-source Structure from Motion system.
Feature extraction and matching are GPU-accelerated (CUDA or GLSL); you can go from a photo folder to a sparse reconstruction in a single pipeline run.

> ⚠️ **v0.1 — early release:** the core pipeline works; APIs and data formats may still change. Feedback welcome.

---

## Features

- **GPU-accelerated front end** — SIFT extraction and matching on GPU (CUDA and GLSL backends)
- **Runs without CUDA** — GLSL backend uses EGL offscreen; no CUDA Toolkit required, wider GPU support
- **Docker-friendly** — EGL headless needs no X11; runs in containers and on cloud servers
- **End-to-end pipeline** — from a photo directory to sparse reconstruction in one command
- **Step-by-step visualization** — incremental SfM writes Bundler after each registration/BA; `at_bundler_viewer` can inspect every stage
- **CLI-first** — each step is a separate C++ CLI tool for easy automation
- **COLMAP-compatible** — can export COLMAP sparse model for dense reconstruction downstream

### GPU scope (v0.1)

| Step | Back end | Note |
|------|----------|------|
| Feature extraction | **GPU** (CUDA or GLSL) | CUDA is faster; GLSL needs no CUDA Toolkit |
| Feature matching | **GPU** (CUDA or GLSL) | Same as above |
| Geometric verification (RANSAC) | **GPU** or PoseLib (CPU) | Configurable; default is GPU |
| Image retrieval | CPU (VLAD / sequence) | VLAD PCA can use GPU (cuBLAS + cuSOLVER) |
| Track building | CPU | |
| Incremental SfM (resection + BA) | CPU (Ceres / PoseLib) | Resection: GPU or PoseLib |
> Longer term, more steps may move to GPU; v0.1 prioritizes correctness and usability.

## Screenshots

<!-- TODO: replace with real captures -->
```
at_bundler_viewer loading a Bundler bundle.out or COLMAP text sparse (cameras.txt, images.txt, points3D.txt under sparse/0)
(capture to be added)
```

---

## Quick start

### 1. Prepare images

Put photos under a root directory (subfolders group cameras automatically):
```
my_images
├── camera1/
│   ├── IMG_0001.jpg
│   ├── IMG_0002.jpg
│   └── ...
└── camera2/
    ├── IMG_0100.jpg
    └── ...
```

### 2. Run the pipeline

```bash
# After building, one command for the full flow
./build/isat_sfm -i /path/to/photos -w work/ -v
```

This runs: create project → extract → retrieve → match → geometry → tracks → incremental SfM.

**No CUDA?** use GLSL:
```bash
./build/isat_sfm -i /path/to/photos -w work/ \
    --extract-backend glsl --match-backend glsl -v
```

### 3. Inspect output

```bash
# Built-in 3D viewer (points + cameras)
./build/at_bundler_viewer 

# Or MeshLab
meshlab work/incremental_sfm/bundle.out
```

Typical output layout:
```
work/incremental_sfm/
├── poses.json      # Camera poses (R, C)
├── bundle.out      # Bundler (points + cameras, viewable)
└── list.txt        # Image list
```

> With debug enabled, each BA in incremental SfM can emit an intermediate `bundle.out` for `at_bundler_viewer`.

---

## Install

> **Recommended:** for most people use the **repo-root `Dockerfile` + [DOCKER_BUILD.md](../../DOCKER_BUILD.md)** to avoid a long `apt` list on the host. The “from source” section below is for developers who `cmake` directly on the host.

### Docker (same as repo root)

```bash
# From repository root
docker build -t insightat:cuda11.8 -f Dockerfile .
```

Run instructions and `docker-test.sh` are in **[DOCKER_BUILD.md](../../DOCKER_BUILD.md)**. Executables (e.g. `isat_sfm`) are on `PATH` inside the image.

### Build from source

**System:** Ubuntu 22.04 / 24.04, NVIDIA GPU (OpenGL 3.3+; CUDA optional but recommended)

**Dependencies:**
```bash
# Optional: CUDA Toolkit (≥ 11.8, 12.x recommended)
# See https://developer.nvidia.com/cuda-downloads

# System packages
sudo apt install -y \
    cmake build-essential git python3 \
    libeigen3-dev libceres-dev libopencv-dev libgdal-dev \
    libgoogle-glog-dev libglew-dev libegl1-mesa-dev \
    qtbase5-dev
```

**Configure & build:**
```bash
git clone https://github.com/your-org/InsightAT.git
cd InsightAT
# Top-level CMake controls whether SiftGPU builds with the CUDA back end via
# the SIFTGPU_ENABLE_CUDA cache variable.
# CUDA back end on:
cmake -S . -B build -DSIFTGPU_ENABLE_CUDA=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# CUDA off (GLSL/EGL only):
cmake -S . -B build -DSIFTGPU_ENABLE_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Artifacts live under `build/`, including all CLIs and `at_bundler_viewer`.

> Without CUDA Toolkit, CMake disables the CUDA back end and builds GLSL only.

### Optional: Ceres with CUDA (advanced, self-built)

`apt install libceres-dev` is usually CPU-only. For **CUDA linear algebra / accelerated solvers** inside Ceres, build Ceres from source with CUDA enabled (match your CUDA major), then point this project to it:

```bash
cmake -S . -B build -DCeres_DIR=/path/to/ceres/lib/cmake/Ceres ...
```

This is **not** in the root `README` to keep the default path simple; the distro Ceres is enough for development and normal runs.

**GUI in Docker (X11 example; full notes in [DOCKER_BUILD.md](../../DOCKER_BUILD.md))**

To show the UI from a container:

```
docker run -d \
  --name cuda-gl \
  --restart always \
  --gpus all \
  -u $(id -u):$(id -g) \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/$USER/.Xauthority \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/home/$USER/.Xauthority \
  --net=host \
  --ipc=host \
  -w /workspace \
  insightat \
  sleep infinity
```

Example interactive session:
```
docker run --gpus all -it --name insightat-dev -u $(id -u):$(id -g) -v /home/jones:/home/jones -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/home/$USER/.Xauthority -e DISPLAY=$DISPLAY -e XAUTHORITY=/home/$USER/.Xauthority  --net=host --ipc=host  838b6ba31c3a /bin/bash
```

---

## Pipeline details

### Full run

```bash
./build/isat_sfm -i <photos_dir> -w <work_dir>
```

### Staged run

`isat_sfm` supports `--steps` to run a subset. Project and work layout: `<work>/project.iat` and intermediates under `<work>/`:

```bash
# Stop after match + geometry (e.g. inspect pairs / epipolar geometry)
./build/isat_sfm -i photos/ -w work/ --steps create,extract,match

# Continue from existing match/geo: tracks + incremental SfM
./build/isat_sfm -i photos/ -w work/ --steps tracks,incremental_sfm
```

For finer control, call each `isat_*` tool (table below).

### Main `isat_sfm` options

| Flag | Meaning | Default |
|------|---------|---------|
| `-i` / `--input` | Photo root | required |
| `-w` / `--work-dir` | Work dir (`project.iat`, features, matches, SfM output) | required |
| `--steps` | Step list | `create,extract,match,tracks,incremental_sfm` |
| `--extract-backend` | Extraction (cuda / glsl) | `cuda` |
| `--match-backend` | Matching (cuda / glsl) | `cuda` |
| `--fix-intrinsics` | Fix intrinsics (turntable / object capture) | `false` |
| `-v` / `--verbose` | Verbose logging | `false` |

### CLI tools

Each algorithm stage is a standalone C++ binary:

| Tool | Role | Acceleration |
|------|------|----------------|
| `isat_sfm` | **End-to-end** pipeline | — |
| `isat_extract` | SIFT extraction | GPU (CUDA / GLSL) |
| `isat_match` | Feature matching | GPU (CUDA / GLSL) |
| `isat_geo` | Geometry + two-view pose | GPU or PoseLib |
| `isat_retrieve` | Retrieval (VLAD / sequence) | CPU |
| `isat_tracks` | Track building | CPU |
| `isat_incremental_sfm` | Incremental SfM | CPU (Ceres) |
| `isat_project` | Project I/O (create/import/export) | — |
| `isat_camera_estimator` | Camera from EXIF + sensor DB | — |
| `isat_train_vlad` | Train VLAD | — |
| `at_bundler_viewer` | 3D viewer | OpenGL |

Use `-h` on each tool for full options.

---

## Output formats

### Bundler (default, viewable out of the box)

Incremental SfM writes `bundle.out` + `list.txt` for `at_bundler_viewer` or MeshLab; compare with COLMAP `sparse/0` if you export there.

### COLMAP export

Can export COLMAP text sparse (`cameras.txt`, `images.txt`, `points3D.txt`) for COLMAP dense or OpenMVS.

### IDC intermediates

InsightAT uses IDC (Insight Data Container) for intermediates (`.isat_feat`, `.isat_match`, `.isat_tracks`):
self-describing JSON header + binary body, 8-byte aligned; see [design/04_functional_at_toolkit.md](design/04_functional_at_toolkit.md).

---

## Positioning vs COLMAP

InsightAT is not a drop-in COLMAP replacement. COLMAP is a mature general SfM + MVS stack.
InsightAT is a lighter incremental SfM path that can work alongside COLMAP.

| | InsightAT v0.1 | COLMAP |
|---|----------------|--------|
| Extraction / matching | GPU (CUDA or GLSL) | GPU |
| SfM core | CPU (Ceres + PoseLib) | CPU |
| Run without CUDA | ✅ (GLSL) | ❌ (needs CUDA) |
| Process visualization | Bundler per step | Usually after run |
| Dense | ❌ (export to COLMAP/OpenMVS) | ✅ built-in |
| Maturity | Early | Production |
| Platform | Linux | Linux / macOS / Windows |

Typical: InsightAT sparse → COLMAP dense.

**Batch evaluation:** ETH3D prep and COLMAP comparison — [benchmarks/README.md](../../benchmarks/README.md).

---

## Roadmap

- [x] **v0.1** — incremental SfM, GPU front end (CUDA + GLSL), `isat_sfm` one-shot, CLI tool chain
- [ ] **v0.2** — more benchmarks, doc polish
- [ ] **v0.3** — prebuilt packages (.deb / Windows); aerial priors, flightline retrieval
- [ ] **v0.4** — cluster + merge + global BA (large scale)
- [ ] **v1.0** — two-level SfM (coarse to fine); more GPU; production hardening

---

## Design documents

- [Design index](design/index.md) — architecture, data model, coordinates, serialization, UI
- [SfM algorithm design](design/01_algorithm_sfm_philosophy.md) — two-level SfM philosophy
- [CLI I/O](design/05_cli_io_conventions.md) — stdout/stderr and machine-readable output
- [Development notes](../dev-notes/) — process notes in Chinese, experiments, drafts

---

## License

InsightAT project code is under the [MIT License](../../LICENSE).

Third-party code keeps its own licenses; see [THIRD_PARTY_LICENSES.md](../../THIRD_PARTY_LICENSES.md). Default SIFT uses PopSift (MPL-2.0); optional SiftGPU has additional terms.

---

## Contributing

Issues and PRs are welcome. Read the [coding style](design/02-coding_style.md) before large changes.
