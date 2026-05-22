#!/bin/bash
# Build an InsightAT AppImage for CUDA 12.8
# This script sets the required environment variables and then calls the generic compile_appimage.sh script.

set -euo pipefail

# Set environment variables for CUDA 12.8
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export INSIGHTAT_BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${PWD}/build-ceres-12.8}"
export CUDA_LIBS_DIR="${CUDA_LIBS_DIR:-/usr/local/cuda-12.8/lib64}"
# Version: if VERSION env var is not set, read from VERSION file + append CUDA suffix.
INSIGHTAT_BASE_VERSION=$(cat "${REPO_ROOT}/VERSION" 2>/dev/null || echo "0.1.0")
export VERSION="${VERSION:-${INSIGHTAT_BASE_VERSION}-cuda12.8}"
export APPIMAGE_OUT_DIR="${APPIMAGE_OUT_DIR:-${PWD}/build-appimage-cuda12.8}"

echo "Configuring for CUDA 12.8:"
echo "  INSIGHTAT_BUILD_DIR: $INSIGHTAT_BUILD_DIR"
echo "  CUDA_LIBS_DIR: $CUDA_LIBS_DIR"
echo "  VERSION: $VERSION"
echo "  APPIMAGE_OUT_DIR: $APPIMAGE_OUT_DIR"

# Optional environment variables (can be overridden by user)
export BUNDLE_PYTHON="${BUNDLE_PYTHON:-1}"
export BUNDLE_PYTHON_DIST="${BUNDLE_PYTHON_DIST:-0}"
export INSIGHTAT_QMAKE="${INSIGHTAT_QMAKE:-}"

# Call the generic compile script
exec "${BASH_SOURCE%/*}/compile_appimage.sh"