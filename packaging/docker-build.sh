#!/usr/bin/env bash
# Build / extract InsightAT release packages via Docker (Ubuntu 22.04 + CUDA 12.8).
#
# Usage:
#   ./packaging/docker-build.sh build     # Build image (~1–2h first time)
#   ./packaging/docker-build.sh extract   # Copy AppImage + deb to ./build-appimage|build-deb
#   ./packaging/docker-build.sh run       # build + extract
#   ./packaging/docker-build.sh shell     # Interactive shell
#   ./packaging/docker-build.sh clean

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-insightat:cuda12.8}"
CONTAINER_NAME="${CONTAINER_NAME:-insightat-build-cuda12.8}"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

help() {
  sed -n '2,12p' "$0"
  echo ""
  echo "Commands: build | extract | run | shell | clean | help"
}

cmd_build() {
  echo "Building ${IMAGE_NAME} from ${DOCKERFILE}"
  docker build --progress=plain -t "${IMAGE_NAME}" -f "${DOCKERFILE}" "${REPO_ROOT}"
  echo "Build complete: ${IMAGE_NAME}"
}

cmd_shell() {
  docker run --rm --gpus all -it "${IMAGE_NAME}" bash
}

cmd_extract() {
  echo "Extracting packages from ${IMAGE_NAME} ..."
  docker create --name "${CONTAINER_NAME}" "${IMAGE_NAME}"
  rm -rf "${REPO_ROOT}/build-appimage" "${REPO_ROOT}/build-deb"
  docker cp "${CONTAINER_NAME}:/workspace/insightat/build-appimage" "${REPO_ROOT}/build-appimage"
  docker cp "${CONTAINER_NAME}:/workspace/insightat/build-deb" "${REPO_ROOT}/build-deb"
  docker rm "${CONTAINER_NAME}"
  echo "AppImage: ${REPO_ROOT}/build-appimage/"
  echo "deb:      ${REPO_ROOT}/build-deb/"
  ls -lh "${REPO_ROOT}/build-appimage"/*.AppImage 2>/dev/null || true
  ls -lh "${REPO_ROOT}/build-deb"/*.deb 2>/dev/null || true
}

cmd_run() {
  cmd_build
  cmd_extract
}

cmd_clean() {
  docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
  docker rmi "${IMAGE_NAME}" 2>/dev/null || true
  rm -rf "${REPO_ROOT}/build-appimage" "${REPO_ROOT}/build-deb"
  echo "Done."
}

case "${1:-help}" in
  build)   cmd_build ;;
  shell)   cmd_shell ;;
  extract) cmd_extract ;;
  run)     cmd_run ;;
  clean)   cmd_clean ;;
  help|*)  help ;;
esac
