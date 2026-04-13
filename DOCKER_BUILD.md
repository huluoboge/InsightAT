# InsightAT Docker Build Guide (CUDA 11.8 + GCC 11)

This guide explains how to build InsightAT using Docker with CUDA 11.8 and GCC 11.

## Quick Start

```bash
# Navigate to InsightAT root
cd /path/to/InsightAT

# Build Docker image and extract binaries
chmod +x docker-test.sh
./docker-test.sh run

# Binaries will be extracted to: ./build-cuda11.8-docker/build/
```

## Available Commands

```bash
# Build Docker image (one-time, ~30-45 minutes)
./docker-test.sh build

# Start interactive shell in container
./docker-test.sh shell

# Extract compiled binaries from container
./docker-test.sh extract

# Full build + extract
./docker-test.sh run

# Clean up Docker image
./docker-test.sh clean

# Show help
./docker-test.sh help
```

## Docker Image Details

**Base Image**: `nvidia/cuda:11.8.0-devel-ubuntu22.04`

**Installed Dependencies**:
- **Compiler**: GCC 11, G++ 11 (default with Ubuntu 22.04)
- **Build Tools**: CMake, Make, pkg-config
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
./docker-test.sh shell

# Inside container:
cd /workspace/insightat/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run tests or tools
./isat_project --help
./isat_extract --help
```

## Testing Extracted Binaries

After extraction, binaries are in `./build-cuda11.8-docker/build/`:

```bash
# Set path to extracted binaries
export ISAT_BIN_DIR=$(pwd)/build-cuda11.8-docker/build

# Run a tool
$ISAT_BIN_DIR/isat_project --help
$ISAT_BIN_DIR/isat_extract --help
```

## Dockerfile Customization

The Dockerfile is located at `./Dockerfile` and can be customized:

- **Base CUDA Image**: Change `nvidia/cuda:11.8.0-devel-ubuntu22.04` to a different CUDA version
- **Compiler**: Modify GCC version installation (currently GCC 11)
- **CMake Options**: Update in `cmake` command (e.g., add `-DINSIGHTAT_BUILD_SOFTWARES=OFF` to skip GUI)
- **Build Type**: Change `-DCMAKE_BUILD_TYPE=Release` if needed

### Example: Different CUDA Version

To build with CUDA 12.1:

```dockerfile
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04
```

Then rebuild:

```bash
./docker-test.sh clean
./docker-test.sh build
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
      dockerfile: Dockerfile
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
# Modify docker-test.sh or Dockerfile:
# Change: make -j$(nproc)
# To:     make -j4
```

### Missing Dependencies in Container

If a build fails missing a library:

1. Check CMakeLists.txt for the required package
2. Add the apt package to the Dockerfile
3. Rebuild: `./docker-test.sh clean && ./docker-test.sh build`

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
4. **Create release builds**: Use `docker-test.sh` for CI/CD pipelines

## References

- NVIDIA CUDA Official Image: https://hub.docker.com/r/nvidia/cuda
- CUDA 11.8 Documentation: https://docs.nvidia.com/cuda/archive/cuda-11-8/
- CMake Modern Practice: https://cliutils.gitlab.io/modern-cmake/
