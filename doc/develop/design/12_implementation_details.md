# 12 - Implementation details and development norms

This document captures important implementation decisions and code-level rules to follow in parallel with [02-coding_style.md](02-coding_style.md).

## 1. Development norms

### 1.1 Dependency rules (critical)
To keep the solver portable and testable, layer boundaries are strict:
- **`src/algorithm/`** — **No Qt includes or links.** Use `std::string`, STL containers, Eigen for math. **The algorithm layer does not depend on `src/database/`**: intrinsics and distortion are described by minimal types (e.g. `insight::camera::Intrinsics` with `fx, fy, cx, cy` and five distortion terms). The CLI or UI loads JSON or exports from `database` and passes these structs into the solver.
- **`src/database/`** — **No Qt.** Types must be plain C++ structs/classes for headless (de)serialization.
- **`src/ui/`** — Qt is allowed. Bridges `std::string` ↔ `QString`, wraps models, and calls algorithm entry points.

### 1.2 Namespaces
- Core: `insight`
- Database: `insight::database`
- UI: `insight::ui`
- Render: `insight::render`

### 1.3 Exceptions and errors
- Low-level (algorithm, IO) — prefer exceptions or `std::optional` / `Expected<T>`.
- UI — catch and convert to `QMessageBox`.
- Logging — use **Glog** consistently.

### 1.4 Resource ownership
- Prefer `std::unique_ptr` / `std::shared_ptr` for non-Qt owned objects.
- Qt `QObject` trees use parent/child lifetimes; non-`QObject` resources are explicit.

## 2. Key implementations

### 2.1 Spatial references
- Local copies of `PROJCS_Database.csv` and `GEOGCS_Database.csv` for lookup.
- `SpatialReferenceTool` bridges these tables to GDAL/PROJ.
- **Local origin / bias** — for large projected CRS (e.g. UTM), the app may subtract a project-wide origin to reduce numerical noise in double precision.

### 2.2 Distortion models
- **Pinhole**, **Brown–Conrady**, **Fisheye**, … as needed.
- Distortion stored as `k1, k2, k3, p1, p2` (aligned with common ContextCapture-style five-parameter models).
- Normalized coordinates before undistortion.
- **Algorithm** — `insight::camera::Intrinsics` in `src/algorithm/modules/camera/camera_types.h` (no `database` link). Resection, undistortion, outlier rejection take this type. **Database** — `CameraModel` holds full metadata; CLI/UI materializes `Intrinsics` from JSON or `CameraModel` when calling the solver.

### 2.3 Task snapshot
- Large `Project` data: snapshots use a **shallow / mid** copy.
- No duplication of raw image bytes — metadata and measurements only.
- `ATTask` instances share `ImageGroup` references but own separate pose arrays.

### 2.4 SfM ID remapping and vectorized cameras

#### Motivation
Image IDs (`image_id`) and camera IDs from the project DB are sparse positive integers. Using them as dense array indices wastes memory and complicates GPU paths.

#### Design
Build an **`IdMapping`** at AT export time; the solver uses **dense** indices `0..N-1` and maps back to original IDs at the end.

```
Original image_id: 1001, 1005, 2008, 2012   (sparse)
Internal index:       0,    1,    2,    3   (dense)

Original camera_id: 1, 3           (sparse)
Internal index:      0, 1          (dense)
```

**`IdMapping`** (`src/algorithm/modules/sfm/id_mapping.h`):
- `original_image_ids[i]` — internal `i` → original `image_id`
- `original_camera_ids[j]` — internal `j` → original `camera_id`
- `image_to_camera[i]` — internal image index → internal camera index
- `original_to_internal_*` — reverse maps for boundary I/O

`isat_intrinsics.h` provides `build_id_mapping_from_image_list(path)`; order follows the image-list JSON order.

#### `MultiCameraSetup` (vectorized)

`incremental_sfm_engine.h` holds dense vectors instead of `unordered_map` maps:

```cpp
struct MultiCameraSetup {
  std::vector<int>                image_to_camera; // image idx → camera idx
  std::vector<camera::Intrinsics> cameras;         // camera idx → intrinsics
};
```

Benefits: contiguous memory, cache-friendly, easy `memcpy` to accelerators for future GPU BA.

#### Pipeline contract

```
Build IdMapping (build_id_mapping_from_image_list)
       ↓
Build vectorized MultiCameraSetup
       ↓
SfM (view graph, tracks, BA — all dense indices)
       ↓
Map outputs back with original_image_id(i)
```

**Important:** for one AT run, all stages (`isat_geo`, `isat_match`, `isat_tracks`, `isat_incremental_sfm`) must share the **same** image list so indices line up. On-disk files (`.isat_geo`, `.isat_match`, …) may still use original IDs in names; the solver always uses dense indices internally.

#### `isat_tracks` two phases and geo inliers

`isat_tracks` is two-pass: phase 1 union–find, phase 2 fills `TrackStore`. **Both phases use only F/E inliers** from `isat_geo` (prefer `F_inliers`, else `E_inliers`) so track quality matches downstream SfM. Phase 1 merges only inliers; phase 2 adds observations for inliers only. Two-view 3D points are **not** used to fill XYZ (inconsistent per-pair frames); 3D comes from global triangulation in incremental SfM.

#### `.isat_tracks` IDC schema history

Written by `save_track_store_to_idc`, read by `load_track_store_from_idc`. Readers ignore unknown flag bits (only `kAlive` is required for “alive”).

| `schema_version` | Writer | Notes |
|------------------|--------|-------|
| `1.0` | `isat_tracks` | Base: observation blobs + metadata |
| `1.1` | `isat_tracks` (with view graph) | JSON header may embed `view_graph_pairs` |
| `1.2` | `isat_incremental_sfm` | `is_sfm_result=true`, SfM stats, `kHasTriangulated` in `track_flags` |

**`track_flags` bits** (`uint8_t`, from `track_store.h`):

| Bit | Constant | Meaning |
|-----|----------|---------|
| 0 (`0x01`) | `kAlive` | Only bit readers must interpret for load |
| 1 (`0x02`) | `kNeedsRetriangulation` | Transient in SfM; no meaning after save |
| 2 (`0x04`) | `kHasTriangulated` | Valid XYZ; schema 1.2+ when written by SfM |
| 3 (`0x08`) | `kSkipFromBA` | Excluded from BA subset; transient |

**Schema 1.2 JSON header example** (from `isat_incremental_sfm`):

```json
{
  "schema_version": "1.2",
  "is_sfm_result": true,
  "num_registered_images": 36,
  "num_triangulated": 38602,
  "num_inlier": 37800,
  "num_outlier": 802,
  "num_not_triangulated": 6719
}
```

- `num_inlier` — alive and triangulated inliers after BA
- `num_outlier` — was triangulated but later marked not alive
- `num_not_triangulated` — never got a 3D point

#### Relationship to GPU BA

Dense `cameras` maps cleanly to Ceres or GPU parameter blocks; `camera_index_for_image[i]` selects the block, similar in spirit to COLMAP’s `camera_id` mapping.

#### Related files

| File | Role |
|------|------|
| `src/algorithm/modules/sfm/id_mapping.h` | `IdMapping` |
| `src/algorithm/tools/isat_intrinsics.h` | `build_id_mapping_from_image_list()` |
| `src/algorithm/modules/sfm/incremental_sfm_engine.h` | Vectorized `MultiCameraSetup` |
| `src/algorithm/modules/sfm/view_graph_loader.cpp` | Pair ID → internal index |
| `src/algorithm/modules/sfm/incremental_sfm.cpp` | `id_mapping` in `run_initial_pair_loop` |
| `src/algorithm/tools/isat_incremental_sfm.cpp` | Build mapping, restore IDs on output |

## 3. Tooling

Standalone tools under `src/algorithm/tools/` should:
1. Expose **`--help`** with a clear IO contract for automation.
2. Support **two input styles** where applicable — JSON file/stdin, and simple CSV line lists.
3. **Silence** — results on **stdout** (JSON or `ISAT_EVENT`); everything else on **stderr** (see [05_cli_io_conventions.md](05_cli_io_conventions.md)).

## 4. Future work
- **v2 data backend** — optional SQLite for very large block matching.
- **Standard exports** — COLMAP, Agisoft-style XML, industry POS outputs.
