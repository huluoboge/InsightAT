# InsightAT Copilot Instructions

InsightAT is an **Aerial Triangulation (space resection & bundle adjustment) application** for photogrammetry. It manages camera poses, coordinate systems, and measurement data (GNSS/IMU/GCP) through a layered architecture: Project → ImageGroup → Images → Measurements, with tree-structured AT tasks for iterative refinement.

## Architecture Overview

**Three-Layer System:**
1. **Project Layer**: Global configuration, coordinate systems, camera models, images, and raw measurement data
2. **ImageGroup Layer**: Logical grouping of images (e.g., strip, block) with shared parameters
3. **AT Task Layer**: Space resection/bundle adjustment tasks with frozen input snapshots and tree hierarchy for iterative optimization

**Key Pattern**: Cereal binary serialization (`*.db` format) for all data persistence. Version-controlled structs support schema evolution.

## Core Components

### Database Types (`src/database/database_types.h`)
- **`CoordinateSystem`**: EPSG/WKT/ENU/Local support + rotation convention (OmegaPhiKappa/YawPitchRoll)
- **`InputPose`**: Lightweight pose (position + rotation, both optional)
- **`Measurement`**: GNSS/IMU/GCP/SLAM with covariance
- **`Project`**: Top-level container (metadata, cameras, image groups, GCPs, coordinate systems)
- **`ImageGroup`**: Image collection with camera rig references
- **`ATTask`**: Space resection task with InputSnapshot (frozen data) and tree nesting

**Always use versioned serialization** for backward compatibility:
```cpp
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    if (version == 0) { /* legacy */ } 
    else { /* v1+ */ ar(...); }
}
CEREAL_CLASS_VERSION(MyType, 1);
```

### UI Framework (`src/ui/`)
- **`MainWindow`** (Qt5): Central application entry point with menu, panels, and dialogs
- **`ProjectDocument`** (models): Mediates Project ↔ UI synchronization; emits signals on data changes
- **`WorkspaceTreeModel`** (models): Qt tree model for Project hierarchy display

**Signal-Slot Pattern**: UI changes emit signals → ProjectDocument processes → broadcasts updates to views.

### Rendering (`src/render/`)
- OpenGL-based 3D visualization of camera positions, image footprints, and coordinate systems
- **Pan/Zoom/Rotation tools**: Encapsulate interaction; replace deprecated Qt APIs (e.g., `QWheelEvent::delta()` → `angleDelta()`, `Qt::MidButton` → `Qt::MiddleButton`)

## Build & Test

**Build** (from workspace root):
```bash
mkdir -p build && cd build
cmake ..
make -j10
```

**Run Main Application**: `./build/InsightAT_New`

**Unit Tests**: 
- `test_project_serialization` – Project load/save cycles
- `test_serialization_comprehensive` – CoordinateSystem, Measurement, ATTask serialization

Run tests: `./build/test_project_serialization`

## Key Patterns

1. **Immutable Input Snapshots**: ATTask freezes Project measurements at creation; BA optimizes poses *while keeping GNSS constraints unchanged*.
2. **Struct-Based Data Model**: No inheritance hierarchies; composition + enums for type safety (e.g., `Measurement::Type::kGNSS`).
3. **Optional Fields**: Use `std::optional<T>` with Cereal support for sparse data (see cereal helpers in database_types.h).
4. **Namespace**: All code in `namespace insight` + subnamespaces (e.g., `insight::database`, `insight::ui`, `insight::render`).
5. **Qt Integration**: MOC auto-generation enabled; use `Q_OBJECT` in classes with signals/slots.

## Common Tasks

- **Add a Measurement Type**: Extend `Measurement::Type` enum and implement variant struct + serialization in `database_types.h`
- **UI Dialog**: Create `.ui` file via Qt Designer, auto-generated headers appear in `*_autogen/` folder
- **Fix Qt Deprecations**: Search warnings in build output (e.g., `Qt::MidButton` → `Qt::MiddleButton`), update call sites
- **Save/Load Project**: Use `cereal::JSONOutputArchive` / `cereal::JSONInputArchive` for JSON, or binary via `cereal::BinaryOutputArchive`

## External Dependencies
- **Qt5** (Core, Gui, Widgets, OpenGL) – UI framework
- **Eigen3** – Linear algebra
- **OpenCV** – Image processing
- **Ceres Solver** – Bundle adjustment optimization (referenced but not directly in core types)
- **GDAL** – Coordinate system transformations
- **Glog** – Structured logging

## Gotchas

- Serialization format is **binary** (`*.db`); JSON not currently used in production
- **Coordinate system origin matters**: For projected coords (EPSG), origin shifts reduce precision loss in float/double math
- **Rotation convention must match**: OmegaPhiKappa (Z-Y-X extrinsic, photogrammetry) vs. YawPitchRoll (Z-Y-X intrinsic, aerospace)
- **Project immutability on task creation**: Measurements added after ATTask creation won't affect its frozen input
