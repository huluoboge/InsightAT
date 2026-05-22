#!/bin/bash
# Build InsightAT in Docker with CUDA 12.8 + cuDSS + custom Ceres (GPU BA).
# Output: AppImage at ./build-appimage-cuda12.8/*.AppImage
#
# Usage:
#   ./docker-build-cuda12.8.sh build         # Build Docker image (first time)
#   ./docker-build-cuda12.8.sh shell         # Interactive shell
#   ./docker-build-cuda12.8.sh extract       # Extract AppImage from built image
#   ./docker-build-cuda12.8.sh run           # build + extract (full pipeline)
#   ./docker-build-cuda12.8.sh clean         # Remove Docker image

set -euo pipefail

IMAGE_NAME="insightat:cuda12.8"
CONTAINER_NAME="insightat-build-cuda12.8"
DOCKERFILE="Dockerfile.cuda12.8"

help() {
    sed -n '2,10p' "$0"
    echo ""
    echo "Commands:"
    echo "  build       Build Docker image (one-time, ~1-2 hours)"
    echo "  shell       Start interactive shell in container"
    echo "  extract     Extract AppImage from container to ./build-appimage-cuda12.8/"
    echo "  run         build + extract (full pipeline)"
    echo "  clean       Remove Docker image + extracted artifacts"
    echo "  help        Show this help"
}

cmd_build() {
    echo "=========================================="
    echo "Building InsightAT Docker image (CUDA 12.8 + cuDSS + Ceres GPU)"
    echo "=========================================="
    echo ""
    echo "This will take ~1-2 hours depending on your machine."
    echo "Steps:"
    echo "  1. Install system deps + cuDSS"
    echo "  2. Build custom Ceres (huluoboge/ceres-solver hy branch w/ CUDA)"
    echo "  3. Build InsightAT"
    echo "  4. Package AppImage"
    echo ""
    docker build --progress=plain -t "${IMAGE_NAME}" -f "${DOCKERFILE}" .
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
    rm -rf "./build-appimage-cuda12.8"
    docker cp "${CONTAINER_NAME}:/workspace/insightat/build-appimage-cuda12.8" "./build-appimage-cuda12.8"
    docker rm "${CONTAINER_NAME}"
    echo ""
    echo "AppImage at: ./build-appimage-cuda12.8/"
    ls -lh ./build-appimage-cuda12.8/*.AppImage 2>/dev/null || echo "(no .AppImage found)"
}

cmd_run() {
    cmd_build
    cmd_extract
}

cmd_clean() {
    echo "Cleaning up..."
    docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
    docker rmi "${IMAGE_NAME}" 2>/dev/null || true
    rm -rf "./build-appimage-cuda12.8"
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
