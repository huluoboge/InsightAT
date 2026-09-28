# 04 - Functional AT toolkit architecture

> **Status:** describes the **CLI-first toolkit as implemented** (baseline `305bbd1`), plus the
> parts that are still intent only. Every "intent" item is marked as such; the concrete layout and
> stage names are the ones in the tree. See [11_architecture_overview.md](11_architecture_overview.md)
> for the system view and [14_roadmap.md](14_roadmap.md) for everything proposed-but-unbuilt.

**Scope:** how InsightAT’s algorithm tools work — decoupled from the project database, file-based,
CLI-first, and self-describing on disk (IDC).

---

## 1. Design principles

### 1.1 Tenets

**Algorithm independence** — *implemented*
- Every stage depends only on plain data types, not on `Project` / `ProjectDocument` (the Qt-superseded controller); project data reaches a stage as JSON exported from a task snapshot.
- Stages exchange **self-described files**, not a shared in-process database.
- Stage processes are effectively **stateless** across runs: same inputs ⇒ same outputs. (State is
  carried on disk between stages, so a re-run from `tracks` reproduces the same model.)

**CLI first**
- Every major step ships as a **CLI** you can run and test in isolation.
- Each tool implements `-h` and documents file formats.
- **stderr** — logs and progress; **stdout** may carry parseable or piped data (see [05_cli_io_conventions.md](05_cli_io_conventions.md)).

**Distributed-ready (intent only)** — *not implemented*
- All IO is **filesystem**-based, which is a precondition for Docker/NFS/object-store deployment,
  and there is no central app server (orchestration is `isat_sfm` or your own shell).
- But nothing in the tree is distributed: there is no cluster partitioning, no cross-node
  scheduling, and no artifact service. Deployment today is single-machine. See
  [14_roadmap.md](14_roadmap.md).

**Low-level control**
- Prefer **Eigen3**, **Ceres**, and explicit, auditable geometry kernels.
- Avoid “black box” OpenCV high-level entry points for core geometry; keep the math auditable and swappable.

**Strategies and adapters** — *partially implemented*
- Feature backends: **PopSift** (default) and **SiftGPU**; distribution can be grid-NMS or
  quadtree (ORB-SLAM-style *distribution*, not ORB features).
- Matchers: CPU cascade-hash and GPU cascade-hash, selected per build.
- **Not** implemented: pluggable priors. GPS/IMU/GCP guidance does not enter the solve; the only
  retrieval prior is the VLAD path used by `isat_retrieve`.

---

## 2. Data formats

### 2.1 IDC (Insight Data Container)

**Goals**
- Fast binary body + a human/AI-readable **JSON** header
- **Self-describing** — the header is enough to interpret the payload
- **Versioned** schema
- **8-byte alignment** of the binary payload for SIMD, GPU upload, and mmap safety

**Layout**

```
┌─────────────────────────────────────┐
│ Magic: "ISAT" (4 bytes)             │
├─────────────────────────────────────┤
│ Format version: uint32_t (4 bytes)  │
├─────────────────────────────────────┤
│ JSON size: uint64_t (8 bytes)       │
├─────────────────────────────────────┤
│ JSON descriptor (UTF-8, variable)   │
├─────────────────────────────────────┤
│ Padding: 0–7 bytes (align to 8)   │
├─────────────────────────────────────┤
│ Binary payload (LE, 8-byte aligned) │
└─────────────────────────────────────┘
```

**Why 8-byte alignment for the payload**

| Need        | Alignment | Note |
|-------------|-----------|------|
| SIMD        | 8/16/32 B | 8 B is a safe minimum |
| GPU buffers | 4–16 B    | Reduces copy/repack |
| Cross-arch  | 8 B       | ARM64 / x86_64 |
| mmap        | 8 B       | Casting to `float*` is defined |

**Padding**

```cpp
header_size = 4 + 4 + 8 + json_size;
padding = (8 - (header_size % 8)) % 8;
payload_offset = header_size + padding;  // multiple of 8
```

**Comparison (informal)**

| Format   | Alignment | Note |
|----------|-----------|------|
| IDC      | 8 B       | one padding after JSON |
| glTF/GLB | 4 B       | per-chunk |
| HDF5     | 8 B       | per dataset |
| NumPy    | 64 B      | header padding |

**Example JSON header**

```json
{
  "schema_version": "1.1",
  "task_type": "feature_extraction",
  "algorithm": {
    "name": "POP_SIFT",
    "version": "1.3",
    "parameters": {
      "nfeatures": 8000,
      "threshold": 0.0133,
      "octaves": 8,
      "feature_type": "matching",
      "extractor_impl": "popsift"
    }
  },
  "descriptor_schema": {
    "feature_type": "sift",
    "descriptor_dim": 128,
    "descriptor_dtype": "uint8"
  },
  "blobs": [
    {
      "name": "keypoints",
      "dtype": "float32",
      "shape": [8000, 2],
      "offset": 0,
      "size": 64000
    },
    {
      "name": "descriptors",
      "dtype": "uint8",
      "shape": [8000, 128],
      "offset": 64000,
      "size": 1024000
    }
  ],
  "metadata": {
    "image_path": "/data/images/IMG_0001.jpg",
    "timestamp": "2026-02-11T10:30:00Z",
    "execution_time_ms": 1250
  }
}
```

**Binary rules**
- **Little-endian** for all numeric blobs
- Scalar types: `uint8` … `uint64`, `float32`, `float64`
- `shape` describes N×M tensors; **`offset`** is relative to the **start of the binary payload** (after padding)

**Sketch: writer**

```cpp
class IDCWriter {
    static constexpr uint32_t MAGIC_NUMBER = 0x54415349; // "ISAT"
    static constexpr uint32_t FORMAT_VERSION = 1;
    static constexpr size_t ALIGNMENT = 8;

    static size_t calculatePadding(size_t offset) {
        return (ALIGNMENT - (offset % ALIGNMENT)) % ALIGNMENT;
    }
    // write header → JSON → padding → payload
};
```

**Performance (rule of thumb)**

| Op              | Unaligned | 8 B aligned |
|-----------------|-----------|-------------|
| float scans     | slower    | faster      |
| SIMD            | may fault | safe        |
| GL buffer upload| extra pack| direct      |

### 2.2 Lightweight JSON

Smaller knobs (camera JSON, small task JSON) can be **plain JSON** on disk (see [05](05_cli_io_conventions.md) for event lines).

**Example: intrinsics JSON**

```json
{
  "camera_id": 1,
  "model": "PINHOLE",
  "width": 3840,
  "height": 2160,
  "fx": 3600.0,
  "fy": 3600.0,
  "cx": 1920.0,
  "cy": 1080.0,
  "distortion": {
    "model": "RADIAL_TANGENTIAL",
    "k1": -0.12,
    "k2": 0.05,
    "p1": 0.001,
    "p2": -0.002
  }
}
```

---

## 3. Actual layout

Stage entry points are **not** under `src/algorithm/`. Algorithm code is a library
(`src/algorithm/`), and every stage is an executable in `src/cli/`:

```
src/
├── algorithm/            # Library code — no Qt, no database dependency
│   ├── modules/
│   │   ├── camera/       # Minimal Intrinsics + undistortion
│   │   ├── extraction/   # SiftGPU / PopSift + keypoint distribution
│   │   ├── retrieval/    # VLAD + PCA whitening (CPU/CUDA)
│   │   ├── matching/     # SIFT matcher + cascade-hash matching
│   │   ├── cpu_cascade_hash/
│   │   ├── gpu_cascade_hash/
│   │   ├── geometry/     # F/E/H RANSAC (GLSL + CUDA)
│   │   └── sfm/          # tracks, view graph, resection, triangulation, BA
│   ├── export/           # COLMAP exporter, point color
│   └── io/               # IDC reader/writer, track-store IDC, EXIF, geopack
├── cli/                  # isat_*.cpp — one executable per stage
├── database/             # Project/ATTask/ImageGroup model + Cereal (not on the SfM path)
├── ui/, render/, tools/  # Legacy opt-in Qt GUI (INSIGHTAT_BUILD_QT_UI=OFF)
└── util/
```

See [03_directory_organization.md](03_directory_organization.md) for the authoritative tree.

---

## 4. Pipeline (conceptual)

### 4.1 End-to-end

The composition that `isat_sfm` actually runs (default steps
`create,extract,match,tracks,seed_eval,incremental_sfm`, plus opt-in `undistort`):

```
isat_project      →  project.iat + images_all.json + intrinsics
        ↓
  isat_extract    →  feat/ (+ feat_retrieval/ for the VLAD path)
        ↓
  isat_retrieve   →  match/pairs_retrieve.json
        ↓
  isat_match      →  match/pairs_matched.json + match/*.isat_match
        ↓
  isat_geo        →  geo/pairs.json + geo/*.isat_geo   (F/E/H RANSAC)
        ↓
  isat_tracks     →  tracks/tracks.isat_tracks
        ↓
  isat_seed_eval  →  seed_eval_all/  (initial-pair strategy selection)
        ↓
  isat_incremental_sfm →  incremental_sfm/{poses.json, bundler/, colmap/}
        ↓
  isat_undistort (optional)  →  undistorted images + COLMAP sparse
```

`isat_sfm -s/--steps ...` runs any subset; `--existing-task` resumes in an existing work dir.
Other standalone CLIs (`isat_calibrate`, `isat_camera_estimator`, `isat_focal_from_geo`,
`isat_retrieval_match`, `isat_train_vlad`, `isat_seed_eval`, …) exist for isolated tasks.
The appended [appendix in the technical report](../../report/insightat-technical-report.md) lists them all.

### 4.2 Two-level features, single-pass SfM

There **is** a two-level feature extraction today, but **not** the staged "coarse SfM → refine"
design that earlier drafts described:

- **Implemented:** `isat_extract` can emit matching features *and* retrieval features
  (`--output-retrieval`, default 1500 features at 1024 px) in one pass. Retrieval features drive
  VLAD pair selection; matching features drive the actual matching.
- **Implemented:** one incremental SfM pass with resection, triangulation, and BA
  (`isat_incremental_sfm`).
- **Not implemented:** separate coarse/high-accuracy SfM stages, pose-guided full-resolution
  matching, and **tiling** (splitting space into tiles and aligning them). See
  [14_roadmap.md](14_roadmap.md).

---

## 5. CLI interface norms

**Invocation**

```bash
isat_<command> [OPTIONS] <input> <output>
```

**Common flags**
- `-h` / `--help`, `-v` / `--verbose`, `-q` / `--quiet`, `--version`
- **stderr** — glog, `PROGRESS: 0.0–1.0` when used
- **stdout** — reserved for data / `ISAT_EVENT` lines per [05](05_cli_io_conventions.md)

**Example: extraction help (illustrative)**

```
USAGE: isat_extract -i <image_list.json> -o <output_dir>
...
```

---

## 6. UI / orchestration integration

**Implemented — CLI driver.** `isat_sfm` shells out to the sibling `isat_*` binaries with the same
flags you would type in a shell, streams `ISAT_EVENT` lines to stdout, and writes per-run logs under
`logs/run_<timestamp>/`.

**Implemented — Electron GUI.** `sfm-gui/` is the product front-end. It does not reimplement
anything; it runs the same CLIs and reads their `ISAT_EVENT` stream:

```
isat_project create-at-task -p <project.iat> -n <task>     # freeze an ATTask snapshot
isat_project extract -t <task_id> -o <work>/images_all.json -a   # image list from the snapshot
isat_sfm --existing-task -w <work> [--undistort] --steps ...     # reconstruct
sfm-viewer <colmap sparse>                                 # inspect results
```

It also detects the CLI binaries (packaged `resources/bin` → repo `build/` → `ISAT_BIN_DIR`),
tracks per-stage status for Continue/Rebuild, and tails run logs.

**Superseded:** the old "`ProjectDocument` drives the tools via `QProcess`" design belonged to the
dropped Qt GUI. The snapshot it was meant to reload (`ATTask::InputSnapshot`) *is* real and is the
pipeline's input contract — see [11_architecture_overview.md](11_architecture_overview.md).

---

## 7. Third-party stack (indicative)

| Library | Role | Integration |
|---------|------|-------------|
| Eigen3 | Linear algebra | `find_package` |
| Ceres | Bundle adjustment (CPU) | `find_package`, required for the SfM path |
| OpenCV | Image I/O, resizing, some kernels | `find_package`; not used as a high-level SfM entry point |
| Glog | Logging | `find_package` |
| GDAL | Image dimensions/metadata | linked by the legacy render/viewer targets |
| GLEW / OpenGL / EGL | GLSL compute paths (SiftGPU, geometry RANSAC) | `find_package` |
| CUDA / cuDSS | PopSift, cascade matching, CUDA RANSAC, `CUDA_SPARSE` BA | `find_package(CUDAToolkit)` |
| PoseLib | Absolute/relative pose kernels | `third_party/PoseLib` |
| PopSift, SiftGPU | Feature extraction | `third_party/popsift`, `third_party/SiftGPU` |
| nanoflann | Nearest-neighbour search | `third_party/nanoflann` |
| cereal, nlohmann/json | Serialization | `third_party/` |
| cmdLine, progress, task_queue, stlplus3 | CLI/plumbing | `third_party/` |

There is **no RansacLib** in this tree — robust estimation is the hand-written GLSL/CUDA
`geometry` module — and there is **no FLANN** (nanoflann is used instead).

**Avoid in `algorithm/`**
- Qt
- `src/database/` (intrinsics enter as `camera::Intrinsics`)
- PCL (too heavy for core AT)

---

## 8. Roadmap

Everything that is proposed but not built — cluster-parallel SfM and merge, distributed execution,
pose-guided two-level SfM, tiling, CRS-driven reconstruction/export, GNSS/IMU/GCP priors,
vocabulary-tree retrieval, and the ATTask snapshot/task-tree workflow — is collected in
[14_roadmap.md](14_roadmap.md). It is a wish list, not a description of the system.

---

## Summary

1. **Algorithms are decoupled** from the project DB.
2. **CLIs** are the stable integration surface.
3. **IDC** balances speed and inspectability.
4. **Eigen / Ceres / explicit geometry** keep behavior explainable.
5. **Strategies** let you swap the extractor / matcher / BA backend without rewriting the graph.

The toolkit is single-machine today, but the file-driven, CLI-first contract is what would make a
future distributed deployment possible.
