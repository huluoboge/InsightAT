# 12 - Implementation details and development norms

This document captures important implementation decisions and code-level rules to follow in parallel with [02-coding_style.md](02-coding_style.md).

## 1. Development norms

### 1.1 Dependency rules (critical)
To keep the solver portable and testable, layer boundaries are strict:
- **`src/algorithm/`** — **No Qt includes or links.** Use `std::string`, STL containers, Eigen for math. **The algorithm layer does not depend on `src/database/`**: intrinsics and distortion are described by minimal types (`insight::camera::Intrinsics`, `src/algorithm/modules/camera/camera_types.h`: `fx, fy, cx, cy, width, height, k1, k2, k3, p1, p2`). The CLI loads JSON or exports from `database` and passes these structs into the solver.
- **`src/cli/`** — **No Qt.** One `isat_*` executable per stage; the only integration surface for the pipeline.
- **`src/database/`** — **No Qt.** Types must be plain C++ structs/classes for headless (de)serialization. Used by `isat_project` / `isat_camera_estimator`, not by the SfM stages.
- **`src/ui/`, `src/render/`, `src/tools/at_bundler_viewer/`, `src/main.cpp`** — **Legacy, opt-in Qt GUI.** Not built by default (`INSIGHTAT_BUILD_QT_UI=OFF`). Qt is allowed here only; do not add new production features to these directories.

### 1.2 Namespaces
- Core: `insight`
- Database: `insight::database`
- CLI: none (each `isat_*` binary is its own `main`)
- UI: `insight::ui` — legacy, opt-in
- Render: `insight::render` — legacy, opt-in

### 1.3 Exceptions and errors
- Low-level (algorithm, IO) — prefer exceptions or `std::optional` / `Expected<T>`.
- Legacy UI — catch and convert to `QMessageBox` (opt-in build only).
- Logging — use **Glog** consistently.

### 1.4 Resource ownership
- Prefer `std::unique_ptr` / `std::shared_ptr` for non-Qt owned objects.
- In the legacy Qt GUI, `QObject` trees use parent/child lifetimes; non-`QObject` resources are explicit.

## 2. Key implementations

### 2.1 Spatial references (**legacy / not on the reconstruction path**)
- Local copies of `data/config/PROJCS_Database.csv` and `data/config/GEOGCS_Database.csv` exist for lookup.
- They are consumed **only by the legacy Qt UI** (`src/ui/system_config.cpp`; the `SpatialReferenceTool` dialog is `src/ui/widgets/spatial_reference_tool.ui`). GDAL is linked only by the legacy render/viewer targets.
- **Not used by the SfM pipeline.** `isat_project set-cs --type local|enu|epsg|wkt` writes `CoordinateSystem` metadata into the project file, but no stage applies a CRS transform. See [11_architecture_overview.md](11_architecture_overview.md) §4 and [archived 08](../../archive/design/08_coordinate_and_rotation.md).

### 2.2 Distortion models
- **Pinhole**, **Brown–Conrady**, **Fisheye**, … as needed.
- Distortion stored as `k1, k2, k3, p1, p2` (aligned with common ContextCapture-style five-parameter models).
- Normalized coordinates before undistortion.
- **Algorithm** — `insight::camera::Intrinsics` in `src/algorithm/modules/camera/camera_types.h` (no `database` link). Resection, undistortion, outlier rejection take this type. **Database** — `CameraModel` holds full metadata; CLI/UI materializes `Intrinsics` from JSON or `CameraModel` when calling the solver.

### 2.3 Task snapshot (the pipeline's input contract)
- `ATTask::InputSnapshot` and `ATTask::Initialization` live in `src/database/database_types.h`.
- `isat_project create-at-task` copies `image_groups`, `measurements`, and the input CRS into a new `InputSnapshot`; `--parent-task-id` records `initialization.prev_task_id`. `input_snapshot.image_groups` is copied by value, not shared by pointer.
- **The snapshot is what the pipeline reads.** `isat_project extract -t <task_id>` and `intrinsics -t <task_id>` export `images_all.json` / cameras **from the snapshot**, never from the live project. `isat_sfm`'s `create` step runs `… → create-at-task → extract -t 0`; the Electron GUI runs `create-at-task → extract -t <id> → isat_sfm --existing-task`.
- Consequence: editing a project after `create-at-task` cannot change that run — the frozen snapshot wins.
- Nesting is a `prev_task_id` link rendered as a task tree; there is **no** `child_tasks` field (`Project.at_tasks` is a flat vector). Seeding a child's poses from the parent (`initial_poses`) has no writer.
- `optimization_config.enable_gnss_constraint` is defaulted to `true` by `isat_project create-at-task`, but nothing in `src/algorithm/` consumes it.

### 2.4 Index-only image and camera identity

**The earlier `IdMapping` design is not implemented.** There is no
`src/algorithm/modules/sfm/id_mapping.h`, no `MultiCameraSetup`, and no
`build_id_mapping_from_image_list()` in the tree. The note in the headers is explicit:
*"index-only, cameras + image_to_camera_index, no IdMapping"*
(`src/cli/project_loader.h`, `src/algorithm/modules/sfm/incremental_sfm_pipeline.h`).

#### How identity actually works

`isat_project` exports a single JSON with an `images[]` array and a `cameras[]` array.
The solver treats the **array position** as the identity:

- image identity — `image_index ∈ [0, num_images)` is the position in `images[]`
- camera identity — `image_to_camera_index[i]` gives the camera index for image `i`
- intrinsics lookup — `cameras[image_to_camera_index[i]]`

```cpp
// src/cli/project_loader.h
struct ProjectData {
  std::vector<camera::Intrinsics> cameras;
  std::vector<int> image_to_camera_index;   // image_index -> camera_index
  std::vector<std::string> image_paths;
};
```

There is no sparse→dense remap because there are no sparse ids in the solver: the JSON order
*is* the index space.

**Important:** for one AT run, all stages (`isat_geo`, `isat_match`, `isat_tracks`,
`isat_incremental_sfm`) must share the **same** image list so indices line up.

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
| `1.3` | `isat_incremental_sfm` (with `sfm_pose`) | Adds `pose_R` / `pose_C` / `registered` / `cam_idx` / `intrinsics` blobs; `has_pose_data=true` |

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

`image_to_camera_index[i]` selects the camera block, similar in spirit to COLMAP’s `camera_id` mapping, and maps cleanly onto Ceres (or future GPU) parameter blocks.

#### Related files

| File | Role |
|------|------|
| `src/cli/project_loader.h` | `ProjectData`: `cameras` + `image_to_camera_index` |
| `src/algorithm/modules/sfm/incremental_sfm_pipeline.h` | `run_initial_pair_loop(..., cameras, image_to_camera_index, ...)` |
| `src/algorithm/modules/sfm/view_graph_loader.cpp` | Pair entries → internal indices |
| `src/algorithm/io/track_store_idc.cpp` | `save_track_store_to_idc` / `load_track_store_from_idc` (schema 1.0–1.3) |
| `src/cli/isat_incremental_sfm.cpp` | Loads `ProjectData`, writes poses + IDC |

## 3. Tooling

Standalone stage tools under `src/cli/` should:
1. Expose **`--help`** with a clear IO contract for automation.
2. Support **two input styles** where applicable — JSON file/stdin, and simple CSV line lists.
3. **Silence** — results on **stdout** (JSON or `ISAT_EVENT`); everything else on **stderr** (see [05_cli_io_conventions.md](05_cli_io_conventions.md)).

## 4. Future work
- **v2 data backend** — optional SQLite for very large block matching.
- **Standard exports** — COLMAP, Agisoft-style XML, industry POS outputs.
