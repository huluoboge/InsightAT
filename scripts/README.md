# Build And Packaging Scripts

Build-related shell scripts live under `scripts/` so the repository root stays focused on source code and standard build definitions.

## Local builds

| Purpose | Entry point |
|---------|-------------|
| CUDA 11.8 + SiftGPU | `scripts/build/compile-11.8.sh` |
| CUDA 12.8 + PopSift/Ceres CUDA | `scripts/build/compile-12.8.sh` |
| GUI-only build | `scripts/build/compile-gui-only.sh` |

These scripts resolve the repository root from their own location and can be called from any working directory. Override the output directory with `INSIGHTAT_BUILD_DIR` or `INSIGHTAT_GUI_BUILD_DIR`; override CUDA discovery with `INSIGHTAT_CUDA_ROOT` or `INSIGHTAT_NVCC`; append additional CMake arguments as needed.

## AppImage packaging

| Purpose | Entry point |
|---------|-------------|
| Generic packaging | `scripts/package/compile_appimage.sh` |
| CUDA 11.8 defaults | `scripts/package/compile_appimage-11.8.sh` |
| CUDA 12.8 defaults | `scripts/package/compile_appimage-12.8.sh` |

Build the binaries first, then invoke the matching CUDA wrapper. The generic script accepts `INSIGHTAT_BUILD_DIR`, `CUDA_LIBS_DIR`, `VERSION`, `APPIMAGE_OUT_DIR`, `BUNDLE_PYTHON`, and `INSIGHTAT_QMAKE`.

## Container packages

The CUDA 12.8 Docker pipeline uses Ubuntu's `libceres-dev`, builds InsightAT, and produces both AppImage and DEB artifacts inside the Ubuntu 22.04 container. The host only needs Docker; it does not need CUDA, Qt, Ceres, `dpkg-deb`, or `patchelf`. The DEB package declares the CUDA 12.8 runtime packages and still requires a compatible NVIDIA driver on the target system.

```bash
scripts/docker/docker-build-cuda12.8.sh run
```

Artifacts are exported to `build-appimage-cuda12.8/` and `build-deb-cuda12.8/`. The DEB builder is also available as `scripts/package/build_deb.sh` for an existing build tree.

## Docker builds

| Purpose | Entry point |
|---------|-------------|
| CUDA 11.8 + GCC 11 | `scripts/docker/docker-build.sh` |
| CUDA 12.8 + system Ceres | `scripts/docker/docker-build-cuda12.8.sh` |

The Docker scripts resolve `cuda11.8.dockerfile` and `cuda12.8.dockerfile` from the repository root and use the repository root as the build context. The CUDA 12.8 script supports `build`, `shell`, `extract`, `run`, `clean`, and `help`.

The root `*.dockerfile` files and `CMakeLists.txt` remain at the repository root because Docker build context and CMake discover them there by convention.
