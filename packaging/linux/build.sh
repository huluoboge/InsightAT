#!/usr/bin/env bash
# Local Linux build (Ubuntu 22.04 + CUDA 12.8 recommended).
#
# Prefer custom CUDA Ceres (~/.local/ceres-cuda128) when present, with explicit
# cuDSS **12** (NOT the top-level cmake/cudss which resolves to libcudss/13 and
# needs cublas.so.13). Fallback: apt libceres-dev (no cuDSS).
#
# Override:
#   INSIGHTAT_USE_SYSTEM_CERES=1   — force apt Ceres
#   INSIGHTAT_CERES_DIR / INSIGHTAT_CUDSS_DIR — custom paths
#
# CI/Docker release still builds its own Ceres+cuDSS; this script is local-only.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${REPO_ROOT}/build}"
NVCC="${INSIGHTAT_NVCC:-/usr/local/cuda-12.8/bin/nvcc}"
CUDA_ROOT="${CUDAToolkit_ROOT:-/usr/local/cuda-12.8}"

# Stale shell exports poison CMake (often point at wrong CUDA / Ceres).
unset Ceres_DIR 2>/dev/null || true
unset cudss_DIR 2>/dev/null || true
unset CUDAToolkit_ROOT 2>/dev/null || true

DEFAULT_CERES_DIR="${HOME}/.local/ceres-cuda128/lib/cmake/Ceres"
# Must be the CUDA-12 package tree — /usr/lib/.../cmake/cudss picks libcudss/13.
DEFAULT_CUDSS_DIR="/usr/lib/x86_64-linux-gnu/libcudss/12/cmake/cudss"

cmake_args=(
  -S "${REPO_ROOT}"
  -B "${BUILD_DIR}"
  -DCMAKE_BUILD_TYPE=Release
  -DINSIGHTAT_BUILD_QT_UI=OFF
  -DPopSift_BUILD_EXAMPLES=OFF
)

if [[ -x "${NVCC}" ]]; then
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

use_cuda_ceres=0
if [[ "${INSIGHTAT_USE_SYSTEM_CERES:-0}" == "1" ]]; then
  use_cuda_ceres=0
elif [[ -n "${INSIGHTAT_CERES_DIR:-}" ]]; then
  use_cuda_ceres=1
elif [[ -f "${DEFAULT_CERES_DIR}/CeresConfig.cmake" || -f "${DEFAULT_CERES_DIR}/ceres-config.cmake" ]]; then
  use_cuda_ceres=1
  INSIGHTAT_CERES_DIR="${DEFAULT_CERES_DIR}"
fi

if [[ "${use_cuda_ceres}" == "1" ]]; then
  INSIGHTAT_CERES_DIR="${INSIGHTAT_CERES_DIR:-${DEFAULT_CERES_DIR}}"
  INSIGHTAT_CUDSS_DIR="${INSIGHTAT_CUDSS_DIR:-${DEFAULT_CUDSS_DIR}}"
  if [[ ! -f "${INSIGHTAT_CERES_DIR}/CeresConfig.cmake" && ! -f "${INSIGHTAT_CERES_DIR}/ceres-config.cmake" ]]; then
    echo "ERROR: Ceres CMake package not found under ${INSIGHTAT_CERES_DIR}" >&2
    exit 1
  fi
  if [[ ! -f "${INSIGHTAT_CUDSS_DIR}/cudss-config.cmake" ]]; then
    echo "ERROR: cuDSS (CUDA 12) not found: ${INSIGHTAT_CUDSS_DIR}/cudss-config.cmake" >&2
    echo "       Do not use /usr/lib/x86_64-linux-gnu/cmake/cudss (that resolves to libcudss/13)." >&2
    exit 1
  fi
  # cuDSS 12 + CUDA 12.8 first (avoids picking cudss/13 → cublas.so.13).
  export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/libcudss/12:${CUDA_ROOT}/lib64:${LD_LIBRARY_PATH:-}"
  cmake_args+=(
    -DCeres_DIR="${INSIGHTAT_CERES_DIR}"
    -Dcudss_DIR="${INSIGHTAT_CUDSS_DIR}"
  )
  echo "[InsightAT] Using CUDA Ceres: ${INSIGHTAT_CERES_DIR}"
  echo "[InsightAT] Using cuDSS 12:   ${INSIGHTAT_CUDSS_DIR}"
else
  SYSTEM_CERES_DIR=""
  for cand in \
    /usr/lib/x86_64-linux-gnu/cmake/Ceres \
    /usr/lib/cmake/Ceres \
    /usr/local/lib/cmake/Ceres
  do
    if [[ -f "${cand}/CeresConfig.cmake" || -f "${cand}/ceres-config.cmake" ]]; then
      SYSTEM_CERES_DIR="${cand}"
      break
    fi
  done
  if [[ -z "${SYSTEM_CERES_DIR}" ]]; then
    echo "ERROR: no Ceres found. Install libceres-dev, or build CUDA Ceres to ${DEFAULT_CERES_DIR}" >&2
    exit 1
  fi
  cmake_args+=(-DCeres_DIR="${SYSTEM_CERES_DIR}" -Ucudss_DIR)
  echo "[InsightAT] Using system Ceres: ${SYSTEM_CERES_DIR}"
fi

echo "[InsightAT] Configuring in ${BUILD_DIR}"
echo "[InsightAT] Note: Qt GUI is OFF by default (CLI-only)."
cmake "${cmake_args[@]}" "$@"

echo "[InsightAT] Building"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo "[InsightAT] CLI binaries:"
shopt -s nullglob
bins=( "${BUILD_DIR}"/isat_* )
shopt -u nullglob
if [[ ${#bins[@]} -eq 0 ]]; then
  echo "  (none found — build may have failed)" >&2
  exit 1
fi
for b in "${bins[@]}"; do
  echo "  $(basename "$b")"
done
[[ -x "${BUILD_DIR}/CameraEstimator" ]] && echo "  CameraEstimator"
echo "[InsightAT] Done (${#bins[@]} isat_* tools)."
