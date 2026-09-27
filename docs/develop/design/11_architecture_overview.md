# 11 - System architecture

InsightAT uses a clear layered design that separates raw data from the solving pipeline.

## 1. Three layers

### Layer 1: Project (global config and raw inputs)
- **Role** — Project metadata, input CRS, camera library, and raw measurements (GNSS/IMU/GCP).
- **Rule** — **No Qt** in this layer. Standard C++ only, plus Eigen and Cereal.

### Layer 2: AT task layer
- **Role** — Run triangulation and adjustment.
- **Mechanism**
  - **Input snapshot** — When a task is created, the system captures and freezes the current Project measurement state.
  - **Tree of tasks** — Tasks can nest; children can seed from parent poses but still reference the same frozen GNSS snapshot.
  - **Isolation** — BA only updates pose copies inside the task; the original Project is not modified in place.
- **Rule** — Core algorithms live under `src/algorithm/`; **no Qt** so the solver runs headless and in batch jobs.

### Layer 3: Output
- **Role** — Export `DBPose` (and related products) in the chosen output CRS and rotation convention.
- **Rule** — Export logic should stay decoupled from the GUI and be callable from CLIs.

## 2. Stack
- **Core** — C++17, Eigen3, OpenCV (no Qt in algorithm)
- **UI** — Qt 5.15 (Qt Widgets), **only** under `src/ui/`
- **Persistence** — Cereal (binary; JSON in current builds for some objects)
- **Rendering** — OpenGL
- **Geodesy** — GDAL / PROJ

## 3. Module map
See [03_directory_organization.md](03_directory_organization.md). Summary:

```bash
src/
├── database/    # Core types, serialization
├── algorithm/   # AT math, geometry, export helpers (no Qt)
├── util/        # Cross-cutting helpers (strings, numerics, …)
└── ui/          # Qt session, dialogs, widgets
    ├── dialogs/
    ├── models/  # Qt model/view adapters
    ├── utils/   # UI helpers (e.g. coordinates)
    └── widgets/ # Reusable widgets (CameraModel, CoordinateSystem, …)
```

## 4. Data flow
1. **Ingest** — User defines CRS and cameras, imports images and measurements into `Project` (UI + database).
2. **Task start** — User creates an `ATTask`; the system builds an `InputSnapshot`.
3. **Solve** — Algorithm reads the snapshot, runs BA / resection, and produces optimized poses.
4. **Save** — The whole `Project` (including all tasks) is serialized to the project file (e.g. `.db` or current JSON-backed format).
