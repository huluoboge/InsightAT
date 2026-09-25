#!/usr/bin/env bash
# Local Linux build (Ubuntu 22.04 + CUDA 12.8 recommended).
#
# Prefer custom CUDA Ceres (~/.local/ceres-cuda128) when present; otherwise apt
# libceres-dev. cuDSS is selected automatically from the CUDA major version
# (CMake + this script): CUDA 12 → libcudss/12, not the unversioned cmake/cudss
# package which often resolves to libcudss/13.
#
# Override:
#   INSIGHTAT_USE_SYSTEM_CERES=1
#   INSIGHTAT_CERES_DIR / INSIGHTAT_CUDSS_DIR / CUDSS_ROOT
#
# CI/Docker is unchanged (sets its own Ceres + cuDSS).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${REPO_ROOT}/build}"
NVCC="${INSIGHTAT_NVCC:-/usr/local/cuda-12.8/bin/nvcc}"
CUDA_ROOT="${CUDAToolkit_ROOT:-/usr/local/cuda-12.8}"

unset Ceres_DIR 2>/dev/null || true
unset cudss_DIR 2>/dev/null || true
unset CUDAToolkit_ROOT 2>/dev/null || true

DEFAULT_CERES_DIR="${HOME}/.local/ceres-cuda128/lib/cmake/Ceres"

# Detect CUDA major (nvcc "release 12.8" → 12; fallback from path cuda-12.8).
detect_cuda_major() {
  local maj=""
  if [[ -x "${NVCC}" ]]; then
    maj="$("${NVCC}" --version 2>/dev/null | sed -n 's/.*release \([0-9][0-9]*\)\..*/\1/p' | head -1)"
  fi
  if [[ -z "${maj}" && "${CUDA_ROOT}" =~ cuda-([0-9]+) ]]; then
    maj="${BASH_REMATCH[1]}"
  fi
  echo "${maj}"
}

# Resolve libcudss/<major>/cmake/cudss (or CUDSS_ROOT).
resolve_cudss_dir() {
  local maj="$1"
  if [[ -n "${INSIGHTAT_CUDSS_DIR:-}" && -f "${INSIGHTAT_CUDSS_DIR}/cudss-config.cmake" ]]; then
    echo "${INSIGHTAT_CUDSS_DIR}"
    return 0
  fi
  local cand
  for cand in \
    "${CUDSS_ROOT:-}/lib/cmake/cudss" \
    "${CUDSS_ROOT:-}/lib64/cmake/cudss" \
    "/usr/lib/x86_64-linux-gnu/libcudss/${maj}/cmake/cudss" \
    "/usr/lib64/libcudss/${maj}/cmake/cudss" \
    "/usr/local/libcudss/${maj}/cmake/cudss"
  do
    [[ -n "${cand}" && -f "${cand}/cudss-config.cmake" ]] || continue
    echo "${cand}"
    return 0
  done
  return 1
}

CUDA_MAJOR="$(detect_cuda_major)"

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
  echo "[InsightAT] CUDA toolkit: ${CUDA_ROOT} (major=${CUDA_MAJOR:-unknown})"
else
  echo "[InsightAT] CUDA nvcc not found at ${NVCC}; configuring without CUDA toolkit path"
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
  if [[ ! -f "${INSIGHTAT_CERES_DIR}/CeresConfig.cmake" && ! -f "${INSIGHTAT_CERES_DIR}/ceres-config.cmake" ]]; then
    echo "ERROR: Ceres CMake package not found under ${INSIGHTAT_CERES_DIR}" >&2
    exit 1
  fi
  cmake_args+=(-DCeres_DIR="${INSIGHTAT_CERES_DIR}")
  echo "[InsightAT] Using CUDA Ceres: ${INSIGHTAT_CERES_DIR}"

  if [[ -n "${CUDA_MAJOR}" ]] && CUDSS_DIR="$(resolve_cudss_dir "${CUDA_MAJOR}")"; then
    CUDSS_LIB_DIR="$(cd "${CUDSS_DIR}/../.." && pwd)"
    export LD_LIBRARY_PATH="${CUDSS_LIB_DIR}:${CUDA_ROOT}/lib64:${LD_LIBRARY_PATH:-}"
    cmake_args+=(-Dcudss_DIR="${CUDSS_DIR}")
    echo "[InsightAT] Using cuDSS for CUDA ${CUDA_MAJOR}: ${CUDSS_DIR}"
  else
    echo "[InsightAT] WARNING: no libcudss/${CUDA_MAJOR:-?} found; CMake may auto-select or Ceres may fail find_dependency(cudss)" >&2
  fi
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
echo "[InsightAT] Done (${#bins[@]} isat_* tools)."
