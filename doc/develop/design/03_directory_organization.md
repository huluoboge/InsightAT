# Source tree layout

How InsightAT source is organized and how it maps to the three-layer design.

---

## 1. `src/` layout

```
src/
├── algorithm/           # Core algorithms (no Qt)
│   ├── modules/         # Pipelines: extraction, retrieval, matching, geometry, …
│   ├── io/              # IDC, EXIF, …
│   └── tools/           # CLI entry points (isat_*.cpp)
├── database/            # Data model and serialization (Cereal; see 07_serialization)
├── util/                # Small shared utilities (string_utils, numeric, …; no Qt)
└── ui/                  # Qt UI (only layer that may depend on Qt)
    └── utils/           # UI helpers (Coordinates, QStringConvert, …)
```

- **algorithm** — No Qt; runs headless and in containers. Aligns with [04_functional_at_toolkit.md](04_functional_at_toolkit.md) and [01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md).
- **database** — Core types (`Project`, `ATTask`, `CoordinateSystem`, …) and (de)serialization. See [09_data_model.md](09_data_model.md) and [07_serialization.md](07_serialization.md).
- **util** — Cross-cutting helpers; no business logic, no Qt.
- **ui** — Main window, dialogs, widgets; depends on `database`. See [06_ui_framework.md](06_ui_framework.md).

---

## 2. Mapping to the three layers

| Layer | Role | Primary directories |
|-------|------|------------------------|
| **Project** | Global config and raw inputs (CRS, cameras, images, measurements) | `database/` (types), `ui/` (editing) |
| **AT task** | Triangulation, snapshots, optimization | `algorithm/`, `database/` (`ATTask`, …) |
| **Output** | Export in chosen frame | `algorithm/` (writers), CLIs, files |

---

## 3. Binaries

- **InsightAT** — Qt app (links `ui`, `database`, `util`, …) with the main project/task UI.
- **isat_*** — Standalone CLIs (extraction, matching, retrieval, geometry, calibration, …); depend on `algorithm` / `database` / `util` only, no Qt.
- **CameraEstimator** — Separate process for intrinsics from EXIF + sensor DB (when built).
