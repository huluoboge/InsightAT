# 09 - Core data model

> **Read this first: the model is larger than the pipeline.** Most types below exist, compile, and (de)serialize — but the SfM stages never touch them. `isat_sfm` works from an image list plus per-image camera intrinsics; `Project` / `ATTask` / `Measurement` / `CoordinateSystem` are used by the project and camera-estimation CLIs and by the (legacy, opt-in) Qt UI.
>
> | Model area | Types exist? | Used by the reconstruction pipeline? |
> |------------|--------------|--------------------------------------|
> | `Project`, `ImageGroup`, `Image`, `CameraModel`, `CameraRig` | Yes | Only via `isat_project` / `isat_camera_estimator` (producing the image list + intrinsics) |
> | `ATTask`, `ATTask::InputSnapshot`, `ATTask::Initialization` | Yes | **Yes, via `isat_project`** — `create-at-task` freezes the snapshot and `extract` / `intrinsics -t` export `images_all.json` + cameras from it. See §5. |
> | `Measurement` (GNSS / IMU / GCP / SLAM) | Yes | **No** — priors do not enter the solve |
> | `CoordinateSystem` (local / ENU / EPSG / WKT) | Yes | **No** — stored as project metadata; no CRS transform is applied |
>
> See [11_architecture_overview.md](11_architecture_overview.md) for what the pipeline really consumes.

Types live in `insight::database`. The style is “structs first”, with Cereal for persistence.

## 1. `Project`
Root object for a job.
- `ProjectInformation` — name, author, paths, …
- `input_coordinate_system` — global input CRS
- `std::vector<CameraModel>` — camera library
- `std::vector<ImageGroup>` — image groupings
- `std::vector<ATTask>` — flat list of AT tasks; **there is no nesting/tree field on `ATTask`**

## 2. `ImageGroup`
Links images to camera models. Modes:
- **Group level** — one intrinsics set for the whole group (typical block flights)
- **Image level** — per-image `CameraModel` (multi-sensor or self-calibration)

## 3. `Image`
- `image_id` — unique key
- `path` — absolute or project-relative
- `InputPose` — prior exterior orientation from imports
- `std::optional<CameraModel>` — only meaningful in **image-level** mode

## 4. `Measurement`
Unified measurement records; each carries a **covariance** where applicable.
- `kGNSS` — position (x, y, z)
- `kIMU` — attitude, accelerations, rates (as modeled)
- `kGCP` — 3D ground point + image observations
- `kSLAM` — relative inter-frame constraints (when used)

## 5. `ATTask`

The fields that exist in `src/database/database_types.h` (verified):

- `id`, `task_id`, `task_name` — identity
- `working_directory` — CLI/reconstruction work dir (the Viewer/Export/Optimization directory contract)
- `input_snapshot` (`ATTask::InputSnapshot`) — `input_coordinate_system` + `measurements` + `image_groups`. A frozen copy of the inputs: `isat_project extract` / `intrinsics` read it, so later edits to the live project cannot change a frozen run
- `initialization` (`std::optional<ATTask::Initialization>`) — `prev_task_id` + `initial_poses`, i.e. where starting poses come from
- `output_coordinate_system` — the output CRS field (this is what the design docs called `OutputConfig`)
- `optimized_poses` — refined exterior parameters from the solve
- `optimization_config` — BA/optimization parameters (added in schema v3)

Fields that **do not exist** despite being described in older design text:

- `child_tasks` — nesting is represented by `initialization.prev_task_id`; `Project` holds a flat
  `std::vector<ATTask>`
- `OutputConfig` — no such type; the output CRS lives in `output_coordinate_system`

**Status: implemented, and load-bearing.** This is the pipeline's input contract:

1. `isat_project create-at-task` freezes the project (groups, images, estimated intrinsics,
   measurements, input CRS) into `ATTask::InputSnapshot`.
2. `isat_project extract -t <task_id>` and `isat_project intrinsics -t <task_id>` export
   `images_all.json` and the camera parameters **from that snapshot only**.
3. `isat_sfm` consumes them. Its `create` step is exactly this sequence
   (`… → create-at-task → extract -t 0`), and the Electron GUI uses
   `create-at-task → extract -t <id> → isat_sfm --existing-task` for continue/rebuild.

`--parent-task-id` writes `initialization.prev_task_id`, which the GUI renders as a task tree.
The one unbuilt piece is pose seeding from the parent: `initialization.initial_poses` has no
writer (see [14_roadmap.md](14_roadmap.md)).

## 6. Integrity
- **`KeyType`** — consistent `uint32_t` (or project-wide ID type) for references
- **`std::optional`** — for incomplete fields; Cereal can omit unset values

## 7. What the pipeline actually reads

The SfM stages consume a much thinner subset than the model above:

- `images_all.json` — the image list written by `isat_project` (paths, group ids, per-image camera index)
- per-camera intrinsics (`fx, fy, cx, cy`, five distortion terms) passed as plain structs

Everything else on this page is available in memory for tools and future work, but is not an input to triangulation, resection, or BA today.
