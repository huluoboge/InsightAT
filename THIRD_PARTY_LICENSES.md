# Third-Party Licenses

InsightAT includes or links against the following third-party libraries.
Each retains its original license; see the referenced files for full text.

---

## Bundled in `third_party/`

### SiftGPU
- **Author**: Changchang Wu, University of North Carolina at Chapel Hill
- **License**: Custom (educational, research and non-profit use only)
- **Notice**: Permission to use, copy, modify and distribute this software and
  its documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following paragraph appear in all copies.
- **⚠️ Restriction**: Commercial use is NOT permitted under this license.
- **Files**: `third_party/SiftGPU/`

### PoseLib
- **Author**: Viktor Larsson
- **License**: BSD 3-Clause
- **Files**: `third_party/PoseLib/`

### stlplus3
- **License**: BSD
- **Files**: `third_party/stlplus3/`

### cereal
- **License**: BSD 3-Clause
- **Files**: `third_party/cereal/`

### cmdLine
- **License**: MPL2
- **Files**: `third_party/cmdLine/`

---

## System Dependencies (linked, not bundled)

| Library | License | Notes |
|---------|---------|-------|
| Eigen3 | MPL2 / BSD 3-Clause | Header-only linear algebra |
| Ceres Solver | BSD 3-Clause | Nonlinear least squares (BA) |
| OpenCV | Apache 2.0 | Image I/O, basic CV |
| GDAL | MIT/X | Geospatial coordinate transforms |
| glog | BSD 3-Clause | Logging |
| GLEW | BSD / MIT | OpenGL extension loading |
| Qt5/Qt6 | LGPL 3.0 | UI only (not required for CLI tools) |
| CUDA Toolkit | NVIDIA EULA | GPU compute |
| nlohmann/json | MIT | JSON parsing |
