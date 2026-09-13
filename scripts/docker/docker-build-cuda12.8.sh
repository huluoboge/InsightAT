#!/bin/bash
# Build InsightAT in Docker with CUDA 12.8 and Ubuntu's Ceres package.
# Output: AppImage and Debian package under the repository root.
#
# Usage:
#   scripts/docker/docker-build-cuda12.8.sh build         # Build Docker image (first time)
#   scripts/docker/docker-build-cuda12.8.sh shell         # Interactive shell
#   scripts/docker/docker-build-cuda12.8.sh extract       # Extract AppImage and DEB from built image
#   scripts/docker/docker-build-cuda12.8.sh run           # build + extract (full pipeline)
#   scripts/docker/docker-build-cuda12.8.sh clean         # Remove Docker image

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"

IMAGE_NAME="insightat:cuda12.8"
CONTAINER_NAME="insightat-build-cuda12.8"
DOCKERFILE="${REPO_ROOT}/cuda12.8.dockerfile"
APPIMAGE_OUT_DIR="${REPO_ROOT}/build-appimage-cuda12.8"
DEB_OUTPUT_DIR="${REPO_ROOT}/build-deb-cuda12.8"

help() {
    sed -n '2,10p' "$0"
    echo ""
    echo "Commands:"
    echo "  build       Build Docker image (one-time, ~1-2 hours)"
    echo "  shell       Start interactive shell in container"
    echo "  extract     Extract AppImage and DEB artifacts from the container"
    echo "  run         build + extract (full pipeline)"
    echo "  clean       Remove Docker image + extracted artifacts"
    echo "  help        Show this help"
}

cmd_build() {
    echo "=========================================="
    echo "Building InsightAT Docker image (CUDA 12.8 + system Ceres)"
    echo "=========================================="
    echo ""
    echo "This will take ~1-2 hours depending on your machine."
    echo "Steps:"
    echo "  1. Install system dependencies and Ceres"
    echo "  2. Build InsightAT with CUDA 12.8"
    echo "  3. Package AppImage and DEB"
    echo ""
    docker build --progress=plain -t "${IMAGE_NAME}" -f "${DOCKERFILE}" "${REPO_ROOT}"
    echo ""
    echo "Build complete: ${IMAGE_NAME}"
}

cmd_shell() {
    docker run --rm --gpus all -it "${IMAGE_NAME}" bash
}

cmd_extract() {
    echo "Extracting AppImage from ${IMAGE_NAME} ..."
    docker create --name "${CONTAINER_NAME}" "${IMAGE_NAME}"
    # Remove host output dir first to avoid stale files
    rm -rf "${APPIMAGE_OUT_DIR}" "${DEB_OUTPUT_DIR}"
    docker cp "${CONTAINER_NAME}:/workspace/insightat/build-appimage-cuda12.8" "${APPIMAGE_OUT_DIR}"
    docker cp "${CONTAINER_NAME}:/workspace/insightat/build-deb-cuda12.8" "${DEB_OUTPUT_DIR}"
    docker rm "${CONTAINER_NAME}"
    echo ""
    echo "AppImage at: ${APPIMAGE_OUT_DIR}/"
    ls -lh "${APPIMAGE_OUT_DIR}"/*.AppImage 2>/dev/null || echo "(no .AppImage found)"
    echo "DEB package at: ${DEB_OUTPUT_DIR}/"
    ls -lh "${DEB_OUTPUT_DIR}"/*.deb 2>/dev/null || echo "(no .deb found)"
}

cmd_run() {
    cmd_build
    cmd_extract
}

cmd_clean() {
    echo "Cleaning up..."
    docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
    docker rmi "${IMAGE_NAME}" 2>/dev/null || true
    rm -rf "${APPIMAGE_OUT_DIR}" "${DEB_OUTPUT_DIR}"
    echo "Done."
}

# ── Dispatch ─────────────────────────────────────────────────────────────────
case "${1:-help}" in
    build)   cmd_build ;;
    shell)   cmd_shell ;;
    extract) cmd_extract ;;
    run)     cmd_run ;;
    clean)   cmd_clean ;;
    help|*)  help ;;
esac
