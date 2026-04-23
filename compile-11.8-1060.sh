#!/usr/bin/env bash
# compile-11.8-1060.sh
# Build InsightAT with CUDA 11.8 on GTX 1060 (sm_61).
#
# Fixes addressed:
#   1. Force CUDA 11.8 nvcc (/usr/local/cuda-11.8/bin/nvcc) to avoid the
#      system-installed /usr/bin/nvcc wrapper (CUDA 11.5) being picked up
#      by CMake, which does not support compute_89/90.
#   2. CUDA architectures: native binaries for Pascal→Ada + PTX for forward
#      compat (89-virtual covers Ada and newer via JIT).
#   3. SIFTGPU_ENABLE_CUDA=ON explicitly passed so SiftGPU CUDA backend builds.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-ceres-11.8"
NVCC="/usr/local/cuda-11.8/bin/nvcc"

if [[ ! -x "${NVCC}" ]]; then
    echo "ERROR: nvcc not found at ${NVCC}" >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${SCRIPT_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_COMPILER="${NVCC}" \
    -DCUDAToolkit_ROOT=/usr/local/cuda-11.8 \
    -DCMAKE_CUDA_ARCHITECTURES="60;61;70;75;80;86;89-virtual" \
    -DCeres_DIR="$HOME/.local/ceres-cuda118/lib/cmake/Ceres" \
    -DSIFTGPU_ENABLE_CUDA=ON \
    "$@"

make -j"$(nproc)"
