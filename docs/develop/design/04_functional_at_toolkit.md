# 04 - Functional AT toolkit architecture

> **Version:** 1.0  
> **Created:** 2026-02-11  
> **Scope:** How InsightAT’s algorithm tools are meant to work — decoupled from the project database, file-based, CLI-first, and self-describing on disk (IDC).

---

## 1. Design principles

### 1.1 Tenets

**Algorithm independence**
- Every stage depends only on plain data types, not on `Project` or `ProjectDocument`.
- Stages exchange **self-described files**, not a shared in-process database.
- Pipelines are **stateless** pure functions: same inputs ⇒ same outputs.

**CLI first**
- Every major step ships as a **CLI** you can run and test in isolation.
- Each tool implements `-h` and documents file formats.
- **stderr** — logs and progress; **stdout** may carry parseable or piped data (see [05_cli_io_conventions.md](05_cli_io_conventions.md)).

**Distributed-ready**
- **Filesystem** IO fits Docker, NFS, and object stores (mount or sync as paths).
- No central app server; orchestration is your scheduler / shell.

**Low-level control**
- Prefer **Eigen3**, **Ceres**, **RansacLib**-style control.
- Avoid “black box” OpenCV high-level entry points for core geometry; keep the math auditable and swappable.

**Strategies and adapters**
- Multiple backends per step (SIFT/ORB, matchers, priors) behind stable interfaces.
- Optional GPS/IMU guidance, exhaustive or guided matching, etc.

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
  "schema_version": "1.0",
  "container_id": "550e8400-e29b-41d4-a716-446655440000",
  "task_type": "feature_extraction",
  "algorithm": {
    "name": "SIFT_OpenCV",
    "version": "4.8.0",
    "parameters": {
      "nfeatures": 8000,
      "contrastThreshold": 0.04
    }
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

## 3. Suggested `src/algorithm` layout (reference)

The repository may not match this tree exactly; it illustrates intent:

```
src/algorithm/
├── base/                 # Math / geometry headers
│   ├── geometry/         # cameras, epipolar, triangulation, PnP
│   ├── math/             # rotations, numerics, robust front-ends
│   └── types.h
├── modules/              # Stateless cores
│   ├── extraction/
│   ├── matching/
│   ├── retrieval/
│   ├── geometry/
│   └── optimization/
├── io/                   # IDC, JSON helpers, endian
└── tools/                # isat_*.cpp
```

---

## 4. Pipeline (conceptual)

### 4.1 End-to-end

```
Image list (JSON: paths, optional GNSS/IMU)
        ↓
  isat_extract  →  .isat_feat per image
        ↓
  isat_retrieve  →  pair candidates
        ↓
  isat_match  →  .isat_match per pair
        ↓
  isat_geo  →  verified geometry / relative poses
        ↓
  isat_tracks + isat_incremental_sfm  →  sparse model / bundles
        ↓
  optional global BA / refinement
```

(Exact tool names in tree may differ; use the built binaries as source of truth.)

### 4.2 Hybrid reconstruction (two-level SfM)

**Phase A — coarse / fast**
- Downsampled images, conservative matching, strong geometry checks
- Goal: a stable initial pose graph

**Phase B — high accuracy**
- Full-resolution features, pose-guided matching, global BA

**Phase C — tiling (optional)**
- Split space, align tiles, then global pose/structure polish

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

## 6. UI integration

`ProjectDocument` can orchestrate tools without embedding solvers:
1. Export `InputSnapshot` to JSON paths on disk
2. Run `QProcess` with the same flags you would use in a shell
3. Parse stderr for progress; load IDC/JSON products back into the task

---

## 7. Third-party stack (indicative)

| Library   | Role              | Typical integration   |
|-----------|-------------------|------------------------|
| Eigen3    | LA                | Header-only            |
| Ceres     | NLLS              | `find_package`         |
| RansacLib | robust models     | header / submodule     |
| OpenCV    | I/O, some kernels | not for opaque high-level SfM |
| FLANN     | ANN search        | optional                |

**Avoid in `algorithm/`**
- Qt
- PCL (too heavy for core AT)

---

## 8–10. Roadmap, scaling, and extensions (brief)

- **Phased rollout** — IDC IO → base lib → `isat_extract` → matching + geometry → incremental SfM + BA → optional UI wiring
- **Distributed** — one container per job stage; shared filesystem or object storage for artifacts
- **Evolving backends** — global solvers, learned matchers, auto strategy selection (future work)

---

## Summary

1. **Algorithms are decoupled** from the project DB.  
2. **CLIs** are the stable integration surface.  
3. **IDC** balances speed and inspectability.  
4. **Eigen / Ceres / explicit geometry** keep behavior explainable.  
5. **Strategies** let you swap SIFT/Matcher/BA without rewriting the whole graph.

This is the base for a cloud-friendly, testable, file-driven photogrammetry tool chain.
