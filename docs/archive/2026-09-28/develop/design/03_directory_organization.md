# 03 - Source tree layout

How the InsightAT repository is organized **as it exists today**. For the architecture those directories implement, see [11_architecture_overview.md](11_architecture_overview.md).

---

## 1. Repository root

```text
InsightAT/
├── src/                 # C++ sources (section 2)
├── third_party/         # Vendored code: SiftGPU, cereal, ImageIO, nanoflann, cmdLine, progress
├── benchmarks/          # ETH3D preparation + COLMAP/InsightAT batch runners and comparisons
├── docs/                # Documentation (GitHub Pages source; index.html + logos at its root)
├── packaging/           # linux/ docker/ appimage/ deb/ windows/ legacy/
├── sfm-gui/             # Electron shell that drives the CLI pipeline
├── sfm-viewer/          # Electron + Three.js viewer for COLMAP sparse results
├── data/                # Sensor DB, GDAL/PROJ data files, sample image lists
├── scripts/             # Python helpers (log analysis, plots, reports)
├── CMakeLists.txt       # Top level; CLI-only by default
├── VERSION              # Single source of the project version
├── vcpkg.json           # Windows dependency manifest
└── DOCKER_BUILD.md      # Container/release build entry point
```

## 2. `src/`

```text
src/
├── cli/                       # ALL isat_* executables — pipeline driver + stage tools
│   ├── isat_sfm.cpp           #   end-to-end driver (sequences the stages)
│   ├── isat_incremental_sfm.cpp
│   └── ...
├── algorithm/
│   ├── modules/               # Stateless algorithmic cores
│   │   ├── camera/            #   intrinsics + undistortion helpers
│   │   ├── extraction/        #   SIFT extraction, feature distribution
│   │   ├── retrieval/         #   VLAD, PCA whitening, spatial/GNSS retrieval
│   │   ├── matching/          #   matcher types and SIFT matcher
│   │   ├── cpu_cascade_hash/  #   cascade-hash matching (CPU)
│   │   ├── gpu_cascade_hash/  #   cascade-hash matching (CUDA)
│   │   ├── geometry/          #   F/E/H RANSAC (CUDA + GLSL/EGL)
│   │   └── sfm/               #   track store, triangulation, resection, BA, pipeline
│   ├── io/                    # IDC reader/writer, geopack, EXIF
│   └── export/                # COLMAP exporter
├── database/                  # Data-model types + Cereal serialization
├── util/                      # string_utils, numeric, insight_global
├── render/                    # OpenGL view + Bundler/COLMAP loaders  [legacy]
├── ui/                        # Qt main window, dialogs, widgets       [legacy]
├── tools/at_bundler_viewer/   # Qt Bundler/point-cloud viewer          [legacy]
└── main.cpp                   # Qt application entry point             [legacy]
```

## 3. Dependency rules

Enforced by convention and build layout (details in [12_implementation_details.md](12_implementation_details.md)):

| Directory | May use Qt? | May include `src/database/`? |
|-----------|-------------|------------------------------|
| `src/algorithm/` | **No** | **No** |
| `src/database/` | **No** | — |
| `src/cli/` | No | Only in the project/camera tools (`isat_project`, `isat_camera_estimator`) |
| `src/util/` | No | No |
| `src/ui/`, `src/render/`, `src/tools/at_bundler_viewer/`, `src/main.cpp` | Yes | Yes |

The first two rows are the load-bearing ones: they keep the solver headless and testable, and let the CLI load JSON/`database` objects and pass minimal structs into the algorithms.

## 4. Binaries

| Binary | Role | Built by default |
|--------|------|------------------|
| `isat_*` | The CLI: pipeline driver + one executable per stage | **Yes** |
| `sfm-gui` (Electron) | The product front-end; drives the CLI and opens the viewer | npm (`sfm-gui/package.json`), not CMake |
| `InsightAT` | Legacy Qt application (project/task UI) | No — `INSIGHTAT_BUILD_QT_UI=ON` |
| `at_bundler_viewer` | Legacy viewer for Bundler `bundle.out` | No — Qt UI build |
| `test_*` | Unit tests, colocated with the modules they cover | Yes (unless tests are disabled) |

## 5. Documentation

See [../README.md](../README.md) for the documentation map, [index.md](index.md) for the current-state design set, and [12_implementation_details.md](12_implementation_details.md) for code-level rules.
