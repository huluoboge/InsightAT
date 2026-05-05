#!/usr/bin/env bash
# Wrapper for AppImage packaging with CUDA 11.8 defaults.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export INSIGHTAT_BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${SCRIPT_DIR}/build-ceres-11.8}"
export CUDA_LIBS_DIR="${CUDA_LIBS_DIR:-/usr/local/cuda-11.8/lib64}"

exec "${SCRIPT_DIR}/compile_appimage.sh" "$@"
