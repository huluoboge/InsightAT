# InsightAT Docker Build Guide (CUDA 11.8 + GCC 11)

**Recommended** way to get a reproducible build: the image installs all heavy native dependencies and compiles the project. Use this if you do not want to install Eigen, Ceres, OpenCV, GDAL, Qt, and related dev packages on the host.

This guide explains how to build InsightAT using Docker with CUDA 11.8 and GCC 11.

## Quick Start

```bash
# Navigate to InsightAT root
cd /path/to/InsightAT

# Build Docker image
./scripts/docker/docker-build.sh

# The image is built as `insightat:cuda11.8`; use the extraction commands below
# to copy binaries to `./build-cuda11.8/`.
```

## Available Commands

```bash
# Build Docker image (one-time, ~30-45 minutes)
./scripts/docker/docker-build.sh

# Start interactive shell in the built image
docker run --gpus all -it --rm insightat:cuda11.8 bash

# Extract compiled binaries from a temporary container
docker create --name insightat-build insightat:cuda11.8
docker cp insightat-build:/workspace/insightat/build ./build-cuda11.8
docker rm insightat-build

# CUDA 12.8 + system Ceres build, AppImage, and DEB extraction
./scripts/docker/docker-build-cuda12.8.sh run

# Show the available CUDA 12.8 script commands
./scripts/docker/docker-build-cuda12.8.sh help
```

## Troubleshooting

**`CMake 3.24 or higher is required` (configure fails in `third_party/popsift`)**  
The CUDA 11.8 Dockerfile installs a new enough CMake with `pip3 install "cmake>=3.24"`. Do not rely only on `apt install cmake` on Ubuntu 22.04 (3.22.x).

## Docker Image Details

**Base Image**: `nvidia/cuda:11.8.0-devel-ubuntu22.04`

**Installed Dependencies**:
- **Compiler**: GCC 11, G++ 11 (default with Ubuntu 22.04)
- **Build Tools**: **CMake 3.24+** (installed via `pip3`; PopSift requires CMake ≥ 3.24, which Ubuntu 22.04’s `apt` `cmake` does not satisfy)
- **CUDA**: 11.8 with full development toolkit
- **Computer Vision**: OpenCV 4.x (with Qt5 GUI support)
- **Geo Data**: GDAL + PROJ + GEOS
- **Linear Algebra**: Eigen3
- **Optimization**: Ceres Solver
- **Logging**: Google Glog
- **Graphics**: OpenGL, GLEW
- **Other**: Boost, SSL, compression libraries

## Building from Source Inside Container

If you want to build manually inside the container:

```bash
# Start interactive shell
docker run --gpus all -it --rm insightat:cuda11.8 bash

# Inside container:
cd /workspace/insightat/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run tests or tools
./isat_project --help
./isat_extract --help
```

## Testing Extracted Binaries

After extraction, binaries are in `./build-cuda11.8/`:

```bash
# Set path to extracted binaries
export ISAT_BIN_DIR=$(pwd)/build-cuda11.8

# Run a tool
$ISAT_BIN_DIR/isat_project --help
$ISAT_BIN_DIR/isat_extract --help
```

## Dockerfile Customization

The CUDA 11.8 Dockerfile is located at `./cuda11.8.dockerfile` and can be customized:

- **Base CUDA Image**: Change `nvidia/cuda:11.8.0-devel-ubuntu22.04` to a different CUDA version
- **Compiler**: Modify GCC version installation (currently GCC 11)
- **Build Type**: Change `-DCMAKE_BUILD_TYPE=Release` if needed

### Example: Different CUDA Version

To build with CUDA 12.1:

```dockerfile
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04
```

Then rebuild:

```bash
docker rmi insightat:cuda11.8
./scripts/docker/docker-build.sh
```

## Docker GPU Support

The Docker setup requires NVIDIA Docker runtime for GPU access:

```bash
# Check if nvidia-docker is installed
docker run --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# If not installed, install NVIDIA Docker:
# https://github.com/NVIDIA/nvidia-docker
```

## Using Docker Compose (Optional)

You can also create a `docker-compose.yml` for development:

```yaml
version: '3.8'

services:
  insightat:
    build:
      context: .
      dockerfile: cuda11.8.dockerfile
    image: insightat:cuda11.8
    container_name: insightat-dev
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - ISAT_BIN_DIR=/workspace/insightat/build
    volumes:
      - .:/workspace/insightat
      - /data:/data:rw
    working_dir: /workspace/insightat/build
    command: /bin/bash
```

Then run:

```bash
docker-compose up -d
docker-compose exec insightat bash
```

## Troubleshooting

### CUDA Version Mismatch

If you see CUDA errors, ensure your host NVIDIA driver supports CUDA 11.8:

```bash
nvidia-smi  # Check driver version
```

### Out of Memory During Build

If the build fails due to memory, reduce parallelism:

```bash
# Modify scripts/docker/docker-build-cuda11.8.sh or cuda11.8.dockerfile:
# Change: make -j$(nproc)
# To:     make -j4
```

### Missing Dependencies in Container

If a build fails missing a library:

1. Check CMakeLists.txt for the required package
2. Add the apt package to the appropriate `.dockerfile`
3. Rebuild: `docker rmi insightat:cuda11.8 && ./scripts/docker/docker-build.sh`

## Comparison: CUDA 11.8 vs Host Build

**Advantages of Docker**:
- ✓ Reproducible environment (exact CUDA/GCC versions)
- ✓ No host dependencies pollution
- ✓ Easy version testing
- ✓ Portable across machines

**Disadvantages**:
- ✗ ~30-45 min first build
- ✗ Requires Docker + GPU runtime
- ✗ Slightly slower than native in some cases

## Next Steps

1. **Test texture object fix**: Run CUDA extraction and verify `.isat_feat` files are valid
2. **Compare with GLSL**: Extract GLSL version separately for comparison
3. **Profile performance**: Use `nvidia-smi` and flamegraph inside container
4. **Create release builds**: Use `scripts/docker/docker-build-cuda12.8.sh` for the CUDA 12.8 AppImage + DEB pipeline

## References

- NVIDIA CUDA Official Image: https://hub.docker.com/r/nvidia/cuda
- CUDA 11.8 Documentation: https://docs.nvidia.com/cuda/archive/cuda-11-8/
- CMake Modern Practice: https://cliutils.gitlab.io/modern-cmake/
