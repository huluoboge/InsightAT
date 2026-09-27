# InsightAT coding standards

> Derived from current `src/algorithm/` practice. New code must follow this document.  
> **Formatting** is enforced by `.clang-format`; this file covers **naming, structure, comments, and design** rules.

---

## Contents

1. [File organization](#1-file-organization)  
2. [Naming](#2-naming)  
3. [Comments](#3-comments)  
4. [Code style (manual)](#4-code-style-supplements)  
5. [Headers](#5-headers)  
6. [Namespaces](#6-namespaces)  
7. [Classes and structs](#7-classes-and-structs)  
8. [Functions](#8-functions)  
9. [Error handling](#9-error-handling)  
10. [GPU code](#10-gpu-code)  
11. [Serialization](#11-serialization)  
12. [CLI tools](#12-cli-tools)  
13. [Do not](#13-do-not)  

---

## 1. File organization

### 1.1 Layout

```
src/
├── algorithm/               # Core solvers: no Qt, no database dep
│   ├── modules/
│   │   ├── camera/          # Minimal intrinsics + undistortion
│   │   ├── extraction/
│   │   ├── retrieval/
│   │   ├── matching/
│   │   ├── geometry/
│   │   └── sfm/
│   ├── io/                  # IDC
│   └── tools/               # isat_*.cpp
├── database/                # Model + Cereal
├── util/                    # Shared helpers: no Qt, no CUDA
└── ui/                      # Only layer that may use Qt
```

### 1.2 Names

| Kind | Rule | Example |
|------|------|---------|
| C++ source | `snake_case.cpp` | `two_view_reconstruction.cpp` |
| C++ header | `snake_case.h` | `gpu_geo_ransac.h` |
| CLI | `isat_<verb>.cpp` | `isat_geo.cpp` |
| GLSL | `<feature>_<kind>.glsl` | `triangulate_comp.glsl` |

### 1.3 File banner

Start every `.cpp` / `.h` with a Doxygen file block:

```cpp
/**
 * @file  gpu_geo_ransac.h
 * @brief GPU two-view geometry (H/F/E) via OpenGL 4.3 compute + EGL.
 *
 * Architecture
 * ────────────
 *  Describe internal layering ...
 *
 * Usage
 * ─────
 *   ...
 */
```

For `isat_*.cpp`, a shorter banner is fine: one-line purpose, IO contract, example command.

---

## 2. Naming

### 2.1 Cheat sheet

| Item | Rule | Example |
|------|------|---------|
| Locals / globals | `snake_case` | `num_matches`, `input_dir` |
| Functions / methods | `snake_case` (incl. Qt slots) | `gpu_ransac_F()`, `on_project_changed()` |
| Types | `PascalCase` | `TwoViewTask`, `GeoRansacConfig` |
| Enum type | `PascalCase` | `Measurement::Type` |
| Enum values | `kPascalCase` | `kGNSS`, `kOmegaPhiKappa` |
| `constexpr` / const | `kCamelCase` | `kEventPrefix`, `kMaxIterations` |
| Macros | `UPPER_SNAKE` | `GPU_GEO_RANSAC_H` |
| Namespaces | `snake_case` | `insight::sfm` |
| Template params | `PascalCase` or single cap | `T`, `Archive` |
| Non-static members | trailing `_` | `context_`, `shader_id_` |

### 2.2 Meaning

- Booleans: `is_`, `has_`, `can_`, or `ok` — e.g. `is_valid`, `has_gnss`, `F_ok`
- Counts: `n_` prefix or `_count` — `n_inliers`, `num_matches`
- Indices: `idx`, `i`, `j`, `k` in tight loops
- Out pointers: `*_out` — `X_out`, `residuals_out`
- For multi-stage structs, use section comments (§7.2)

---

## 3. Comments

### 3.1 Public API (headers)

```cpp
/**
 * Triangulate @p n correspondences in parallel on GPU.
 *
 * @param pts_n   Normalized coords, row-major float[4n] — x1 y1 x2 y2 ...
 * @param n       Count (>= 1).
 * @param R       Camera-2 rotation, row-major 3×3.
 * @param t       Camera-2 translation, length 3.
 * @param X_out   3D points, float[3n].
 * @return        Count of valid (finite, positive depth) points.
 */
int gpu_triangulate(const float* pts_n, int n,
                    const float  R[9],
                    const float  t[3],
                    float*       X_out);
```

Struct fields: `///<` after the member:

```cpp
struct TwoViewTask {
    std::string geo_file;     ///< .isat_geo path
    uint32_t camera1_id = 1;  ///< group id for image 1
    bool     F_ok = false;    ///< F estimated successfully
};
```

### 3.2 Section rules

Use U+2500 lines `─` to separate ideas (78 `─` after `// `, total line width 81).

```cpp
// ─────────────────────────────────────────────────────────────────────────────
// Per-pair task
// ─────────────────────────────────────────────────────────────────────────────
```

Steps: `// ── Step 4: normalize pixels ──`

### 3.3 Ban useless comments

```cpp
// bad
int count = 0;  // count is zero

// good
int n_inliers = 0;  // reset before GPU readback
```

---

## 4. Code style (supplements)

Run `.clang-format`; additionally:

### 4.1 `using` and aliases

At top of `.cpp` (outside an inner namespace):

```cpp
namespace fs = std::filesystem;
using json = nlohmann::json;
```

`using namespace` **only** in `.cpp`, **never** in headers.

### 4.2 C++17

```cpp
for (const auto& [id, rig] : project.camera_rigs) { }
auto [R, t] = decomposeEssential(E, pts1, pts2);
Image img{ .image_id = project.next_image_id++, .filename = abs_path.string() };
```

### 4.3 Lambdas

Short → one line. Long → break and indent 4 spaces; capture explicitly.

### 4.4 Multiple returns

Prefer `std::optional` or a struct; raw out-pointers only on hot paths.

```cpp
std::optional<TwoViewResult> reconstructPair(...);
int gpu_ba_residuals(..., float* wrss_out, int* ninliers_out);
```

---

## 5. Headers

### 5.1 Include guards

```cpp
#pragma once
#ifndef GPU_GEO_RANSAC_H
#define GPU_GEO_RANSAC_H
// ...
#endif  // GPU_GEO_RANSAC_H
```

### 5.2 Order

```cpp
#include "gpu_geo_ransac.h"  // matching .h first in .cpp

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "../io/idc_reader.h"
```

Blank line between groups; alphabetical within a group.

---

## 6. Namespaces

```cpp
namespace insight {
namespace sfm {

// no extra indent for body
struct TwoViewResult { ... };

}  // namespace sfm
}  // namespace insight
```

- Do **not** indent namespace bodies
- Close with `}  // namespace name`
- Use anonymous `namespace { }` in `.cpp` for file-local helpers instead of `static` where appropriate

---

## 7. Classes and structs

### 7.1 Data-only: `struct`

Group members by stage; default-initialize everything; `///<` for non-obvious fields.

### 7.2 Behavior: `class`

Order: `public` → `signals` → `public slots` → `protected` → `private`.  
Private data: `name_` suffix.

---

## 8. Functions

### 8.1 One responsibility; split beyond ~80 lines

### 8.2 Parameter order

`[const inputs] [in-out by pointer last]`

### 8.3 File-local helpers in `.cpp`

```cpp
static std::vector<TwoViewTask> loadPairs(const std::string& json_path, ...);
```

---

## 9. Error handling

### 9.1 Algorithm (no Qt)

| Case | Action |
|------|--------|
| Bug | `LOG(FATAL) <<` |
| Recoverable | `LOG(ERROR) <<` + `false` / `std::nullopt` |
| Debug | `LOG` / `VLOG(1)` |
| Debug assert | `DCHECK` |

### 9.2 CLI

```cpp
if (!fs::is_directory(input_dir)) {
    printEvent({{"type", "project.add_images"}, {"ok", false},
                {"error", "input directory not found"}});
    return 1;
}
```

- **stdout** — `ISAT_EVENT` only when emitting machine output ([05](05_cli_io_conventions.md))  
- **stderr** — glog

### 9.3 `main` must not leak exceptions

```cpp
try { cmd.process(argc, argv); }
catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
}
```

---

## 10. GPU code

### 10.1 Stages

```
Stage 1  [threads]   disk → task inputs
Stage 2  [EGL thread] upload SSBO → dispatch → readback
Stage 3  [threads]   write disk
```

### 10.2 Init / shutdown

```cpp
gpu_geo_init(nullptr);
// ... all pairs on one context ...
gpu_geo_shutdown();
```

GPU calls are **not** thread-safe with EGL; one owning thread or a mutex.

### 10.3 Layouts

| Data | Layout |
|------|--------|
| Corres. | `float[4N]` x1y1x2y2... |
| R | `float[9]` row major |
| 3D | `float[3N]` |
| Residuals | `float[4N]` per correspondence |

Invalid points: **NaN**, length fixed.

---

## 11. Serialization

Cereal with explicit versions:

```cpp
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    ar(CEREAL_NVP(image_id), CEREAL_NVP(filename));
    if (version >= 1) ar(CEREAL_NVP(pose));
}
CEREAL_CLASS_VERSION(Image, 1);
```

- Bump version when adding fields; never remove old fields silently  
- `std::optional` for absent optional columns  
- Production may use **JSON** `.iat` while tests may use binary

---

## 12. CLI tools

```cpp
CmdLine cmd("Short description");
cmd.add(make_option('p', project_file, "project")
    .doc("Path to .iat"));
```

Validate required args after `checkHelp`, print help and return `2` on user error, `1` on runtime failure (team convention; align with [05](05_cli_io_conventions.md)).

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Runtime / IO / solver failure |
| 2 | Bad CLI |

---

## 13. Do not

| Do not | Why |
|--------|-----|
| Qt in `algorithm/` | Headless + library reuse |
| `#define` instead of `constexpr` | Type safety |
| `using namespace std` in headers | ODR / pollution |
| Raw `new`/`delete` without reason | `unique_ptr` / `shared_ptr` |
| Magic numbers | `constexpr` / enums |
| `printf` in new code | glog + streams |
| Disk IO inside GPU “Stage 2” | Keep GPU thread clean |
| Remove Cereal fields | Breaks old projects |
| Non-`ISAT_EVENT` noise on stdout for machine-oriented CLIs | Breaks scripts |

---

*Keep this file in sync with code review; avoid manual “field list” patches without a matching PR.*
