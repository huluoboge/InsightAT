#!/bin/bash
# Build and test InsightAT in Docker with CUDA 11.8 + GCC 11

set -e

REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"

# Configuration
IMAGE_NAME="insightat:cuda11.8"
CONTAINER_NAME="insightat-build"

echo "=========================================="
echo "Building InsightAT Docker image (CUDA 11.8 + GCC 11)"
echo "=========================================="

# Build image
docker build --progress=plain -t "${IMAGE_NAME}" -f "${REPO_ROOT}/cuda11.8.dockerfile" "${REPO_ROOT}"

echo ""
echo "=========================================="
echo "Build successful: ${IMAGE_NAME}"
echo "=========================================="
echo ""
echo "Run tests with:"
echo "  docker run --gpus all -it --rm ${IMAGE_NAME} bash"
echo ""
echo "To extract and test binaries:"
echo "  docker create --name ${CONTAINER_NAME} ${IMAGE_NAME}"
echo "  docker cp ${CONTAINER_NAME}:/workspace/insightat/build ${REPO_ROOT}/build-cuda11.8"
echo "  docker rm ${CONTAINER_NAME}"
echo ""
