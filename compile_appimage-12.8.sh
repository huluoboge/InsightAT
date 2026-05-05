#!/usr/bin/env bash
# Wrapper for AppImage packaging with CUDA 12.8 + cuDSS defaults.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export INSIGHTAT_BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${SCRIPT_DIR}/build-ceres-12.8}"
export CUDA_LIBS_DIR="${CUDA_LIBS_DIR:-/usr/local/cuda-12.8/lib64}"
# Let compile_appimage.sh pick up cuDSS from ldd/system paths; this keeps runtime resolution stable.
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/libcudss/12:/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH:-}"

exec "${SCRIPT_DIR}/compile_appimage.sh" "$@"
