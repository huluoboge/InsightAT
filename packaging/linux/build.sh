#!/usr/bin/env bash
# Local Linux build (Ubuntu 22.04 + CUDA 12.8 recommended).
#
# Default: friendly clone path — uses system/vcpkg Ceres via find_package(Ceres).
# Does NOT require custom CUDA Ceres or cuDSS.
#
# Optional GPU BA (custom Ceres hy + cuDSS):
#   INSIGHTAT_USE_CUDA_CERES=1 ./packaging/linux/build.sh
#
# Extra cmake args can be appended: ./packaging/linux/build.sh -DFOO=ON

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${REPO_ROOT}/build}"
NVCC="${INSIGHTAT_NVCC:-/usr/local/cuda-12.8/bin/nvcc}"
CUDA_ROOT="${CUDAToolkit_ROOT:-/usr/local/cuda-12.8}"

cmake_args=(
  -S "${REPO_ROOT}"
  -B "${BUILD_DIR}"
  -DCMAKE_BUILD_TYPE=Release
  -DINSIGHTAT_BUILD_QT_UI=OFF
  -DPopSift_BUILD_EXAMPLES=OFF
)

if [[ -x "${NVCC}" ]]; then
  unset CUDAToolkit_ROOT 2>/dev/null || true
  export LD_LIBRARY_PATH="${CUDA_ROOT}/lib64:${LD_LIBRARY_PATH:-}"
  cmake_args+=(
    -DCMAKE_CUDA_COMPILER="${NVCC}"
    -DCUDAToolkit_ROOT="${CUDA_ROOT}"
    -DCMAKE_CUDA_ARCHITECTURES="${CMAKE_CUDA_ARCHITECTURES:-60-virtual;61-virtual;70-virtual;75-virtual;80-virtual;86-virtual;89-virtual;90-virtual;120-virtual}"
    -DINSIGHTAT_ENABLE_SIFTGPU=ON
    -DSIFTGPU_ENABLE_CUDA=ON
  )
  echo "[InsightAT] CUDA toolkit: ${CUDA_ROOT}"
else
  echo "[InsightAT] CUDA 12.8 nvcc not found at ${NVCC}; configuring without CUDA toolkit path"
  cmake_args+=(
    -DINSIGHTAT_ENABLE_SIFTGPU=ON
    -DSIFTGPU_ENABLE_CUDA=OFF
  )
fi

if [[ "${INSIGHTAT_USE_CUDA_CERES:-0}" == "1" ]]; then
  unset Ceres_DIR 2>/dev/null || true
  INSIGHTAT_CERES_DIR="${INSIGHTAT_CERES_DIR:-$HOME/.local/ceres-cuda128/lib/cmake/Ceres}"
  INSIGHTAT_CUDSS_DIR="${INSIGHTAT_CUDSS_DIR:-/usr/lib/x86_64-linux-gnu/libcudss/12/cmake/cudss}"
  if [[ ! -f "${INSIGHTAT_CERES_DIR}/CeresConfig.cmake" && ! -f "${INSIGHTAT_CERES_DIR}/ceres-config.cmake" ]]; then
    echo "ERROR: INSIGHTAT_USE_CUDA_CERES=1 but Ceres not found under ${INSIGHTAT_CERES_DIR}" >&2
    exit 1
  fi
  if [[ ! -f "${INSIGHTAT_CUDSS_DIR}/cudss-config.cmake" ]]; then
    echo "ERROR: INSIGHTAT_USE_CUDA_CERES=1 but cuDSS not found under ${INSIGHTAT_CUDSS_DIR}" >&2
    exit 1
  fi
  export LD_LIBRARY_PATH="$(dirname "$(dirname "${INSIGHTAT_CUDSS_DIR}")"):${LD_LIBRARY_PATH:-}"
  cmake_args+=(
    -DCeres_DIR="${INSIGHTAT_CERES_DIR}"
    -Dcudss_DIR="${INSIGHTAT_CUDSS_DIR}"
  )
  echo "[InsightAT] Using custom CUDA Ceres + cuDSS"
else
  echo "[InsightAT] Using system/find_package Ceres (no cuDSS required)"
fi

echo "[InsightAT] Configuring in ${BUILD_DIR}"
cmake "${cmake_args[@]}" "$@"

echo "[InsightAT] Building"
cmake --build "${BUILD_DIR}" -j"$(nproc)"
echo "[InsightAT] Done. Binaries in ${BUILD_DIR}/isat_*"
