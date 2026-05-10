#!/usr/bin/env bash
# compile-12.8-1060.sh
# Build InsightAT with CUDA 12.8 on GTX 1060 (sm_61) + Ceres linked against CUDA 12.x.
#
# Notes:
#   1. Force CUDA 12.8 nvcc so CMake does not pick a stale /usr/bin/nvcc.
#   2. CUDA architectures: native SMs for common generations + PTX for forward compat.
#   3. SIFTGPU_ENABLE_CUDA=OFF — SiftGPU upstream CUDA API breaks on CUDA 12.8; build
#      without SiftGPU CUDA. Default SfM path uses PopSift (CUDA) + GPU cascade hash
#      (see isat_sfm defaults and README).
#   4. Ceres: default is CUDA 12.8 + cuDSS build under ~/.local/ceres-cuda128/ (matches
#      -DCeres_DIR=$HOME/.local/ceres-cuda128/lib/cmake/Ceres). Override with env
#      INSIGHTAT_CERES_DIR=... or pass a later -DCeres_DIR=... in "$@".
#      We intentionally do NOT read Ceres_DIR from the environment: many dev shells still
#      export Ceres_DIR to an older Ceres (e.g. cuda11), which would silently override cuda128.
#   5. cuDSS: Ceres (cuda128 build) calls find_dependency(cudss). Debian/Ubuntu package
#      installs CMake files under libcudss/12/cmake/cudss — pass -Dcudss_DIR=... below.
#   6. LD_LIBRARY_PATH: prepend cuDSS + CUDA 12.8 lib64 **before** any existing path (e.g.
#      conda/py39 often prepends cuda-11.8; glibc resolves libcusolver.so.11 from the first
#      matching directory, which would wrongly pick 11.8 even when the binary RUNPATH lists 12.8).
#      Same order matters when you run isat_* from a conda shell: export similarly or fix env.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-ceres-12.8"
NVCC="/usr/local/cuda-12.8/bin/nvcc"

if [[ ! -x "${NVCC}" ]]; then
    echo "ERROR: nvcc not found at ${NVCC}" >&2
    exit 1
fi

# cuDSS + CUDA 12.8 libs first (see note 6 above).
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/libcudss/12:/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH:-}"

# CeresConfig find_dependency(CUDAToolkit): if the shell exports CUDAToolkit_ROOT to an
# older CUDA (e.g. 11.8), CMake may ignore the -D and warn (CMP0074). Force 12.8 only.
unset CUDAToolkit_ROOT 2>/dev/null || true

# Default Ceres (CUDA 12.8 + cuDSS install layout you specified). Do not inherit Ceres_DIR
# from the parent shell — it often points at an older prefix and poisons the cache.
unset Ceres_DIR 2>/dev/null || true
INSIGHTAT_CERES_DIR="${INSIGHTAT_CERES_DIR:-$HOME/.local/ceres-cuda128/lib/cmake/Ceres}"
if [[ ! -f "${INSIGHTAT_CERES_DIR}/CeresConfig.cmake" && ! -f "${INSIGHTAT_CERES_DIR}/ceres-config.cmake" ]]; then
    echo "ERROR: Ceres CMake package not found under INSIGHTAT_CERES_DIR=${INSIGHTAT_CERES_DIR}" >&2
    echo "       (expected CeresConfig.cmake or ceres-config.cmake)" >&2
    exit 1
fi

# NVIDIA cuDSS CMake package (system libcudss package on Ubuntu).
INSIGHTAT_CUDSS_DIR="${INSIGHTAT_CUDSS_DIR:-/usr/lib/x86_64-linux-gnu/libcudss/12/cmake/cudss}"
if [[ ! -f "${INSIGHTAT_CUDSS_DIR}/cudss-config.cmake" ]]; then
    echo "ERROR: cuDSS CMake package not found: ${INSIGHTAT_CUDSS_DIR}/cudss-config.cmake" >&2
    echo "       Install libcudss / NVIDIA cuDSS dev files, or set INSIGHTAT_CUDSS_DIR." >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${SCRIPT_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_COMPILER="${NVCC}" \
    -DCUDAToolkit_ROOT=/usr/local/cuda-12.8 \
    -DCMAKE_CUDA_ARCHITECTURES="60;61;70;75;80;86;89;90-virtual" \
    -DPopSift_BUILD_EXAMPLES=OFF \
    -Dcudss_DIR="${INSIGHTAT_CUDSS_DIR}" \
    -DCeres_DIR="${INSIGHTAT_CERES_DIR}" \
    -DSIFTGPU_ENABLE_CUDA=OFF \
    "$@"

make -j"$(nproc)"
