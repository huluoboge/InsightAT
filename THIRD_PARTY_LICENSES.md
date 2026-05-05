# Third-Party Licenses

Copyright (c) 2026 Yang Hu. InsightAT project code is licensed under MIT (`LICENSE` at repository root).
Third-party components keep their own original licenses.

## Bundled under `third_party/`

### PopSift (default SIFT backend)
- **License**: MPL-2.0
- **Path**: `third_party/popsift/`
- **License file**: `third_party/popsift/COPYING.md`

### SiftGPU (optional backend; disabled by default)
- **License**: Custom (educational, research and non-profit purposes only)
- **Path**: `third_party/SiftGPU/`
- **License file**: `third_party/SiftGPU/LICENSE`
- **Restriction**: Not permitted for general commercial use.

### PoseLib
- **License**: BSD-3-Clause
- **Path**: `third_party/PoseLib/`
- **License file**: `third_party/PoseLib/LICENSE`

### cereal
- **License**: BSD-3-Clause
- **Path**: `third_party/cereal/`

### stlplus3
- **License**: BSD-style (see upstream files)
- **Path**: `third_party/stlplus3/`

### cmdLine
- **License**: MPL-2.0
- **Path**: `third_party/cmdLine/`

### nanoflann
- **License**: BSD-2-Clause
- **Path**: `third_party/nanoflann/`
- **License file**: `third_party/nanoflann/LICENSE`

### task_queue
- **License**: MIT
- **Path**: `third_party/task_queue/`
- **License file**: `third_party/task_queue/LICENSE`

## System Dependencies (linked, not bundled)

- Eigen3: MPL-2.0 / BSD-3-Clause
- Ceres Solver: BSD-3-Clause
- OpenCV: Apache-2.0
- GDAL: MIT/X-style
- glog: BSD-3-Clause
- GLEW: BSD/MIT
- Qt5/Qt6: LGPL-3.0
- CUDA Toolkit: NVIDIA EULA
- nlohmann/json: MIT

## Algorithmic References (independent reimplementation)

### Cascade Hash matching (`src/algorithm/modules/cpu_cascade_hash/`)
- **Reference source**: openMVG cascade-hash matching implementation and related papers
- **Code status**: reimplemented in InsightAT coding style; no third-party source files are bundled in this path
- **Upstream project license (openMVG)**: MPL-2.0
- **Notice**: if future commits copy or adapt upstream code text from openMVG, keep upstream file headers and satisfy MPL-2.0 source-level obligations for those files
- **Patent / commercial note**: openMVG marks Cascade Hashing as a non-free module and notes the original source was released for research usage; for commercial usage, contact the original paper authors.
- **Paper reference**: *Fast and Accurate Image Matching with Cascade Hashing for 3D Reconstruction* (CVPR 2014), Jian Cheng, Cong Leng, Jiaxiang Wu, Hainan Cui, Hanqing Lu.

## Compliance Note

When redistributing binaries or source, keep all required copyright notices
and license texts for third-party components. The repository MIT license does
not replace third-party license obligations.
