# 09 - Core data model

Types live in `insight::database`. The style is “structs first”, with Cereal for persistence.

## 1. `Project`
Root object for a job.
- `ProjectInformation` — name, author, paths, …
- `input_coordinate_system` — global input CRS
- `std::vector<CameraModel>` — camera library
- `std::vector<ImageGroup>` — image groupings
- `std::vector<ATTask>` — task tree

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
- `InputSnapshot` — frozen copy of relevant measurements so BA does not see later user edits to raw priors
- `Initialization` — where starting poses come from (e.g. parent task)
- `OutputConfig` — output CRS and rotation export rules
- `optimized_poses` — refined exterior parameters from the solve
- `child_tasks` — nested experiments or iterations

## 6. Integrity
- **`KeyType`** — consistent `uint32_t` (or project-wide ID type) for references
- **`std::optional`** — for incomplete fields; Cereal can omit unset values
