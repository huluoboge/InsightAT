#!/bin/bash
# Docker build, test and extraction script for InsightAT CUDA 11.8
# Usage: ./docker-test.sh [build|run|extract|shell]

set -e

IMAGE_NAME="insightat:cuda11.8"
CONTAINER_NAME="insightat-cuda11.8-temp"
BUILD_EXTRACT_DIR="./build-cuda11.8-docker"

show_help() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build       - Build Docker image"
    echo "  shell       - Run interactive shell in container"
    echo "  extract     - Extract compiled binaries from container to ./build-cuda11.8-docker"
    echo "  run         - Build, extract, and list binaries"
    echo "  clean       - Remove Docker image"
    echo "  help        - Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./docker-test.sh build     # Build image (~30-45 min)"
    echo "  ./docker-test.sh shell     # Interactive bash in container"
    echo "  ./docker-test.sh extract   # Copy build/ from container"
    echo ""
}

build_image() {
    echo "======================================"
    echo "Building Docker image: ${IMAGE_NAME}"
    echo "======================================"
    docker build -t "${IMAGE_NAME}" -f Dockerfile . || {
        echo "ERROR: Docker build failed!"
        exit 1
    }
    echo ""
    echo "✓ Build successful!"
    echo ""
}

run_shell() {
    echo "======================================"
    echo "Starting interactive shell in ${IMAGE_NAME}"
    echo "======================================"
    echo ""
    echo "Available tools in /workspace/insightat/build:"
    echo "  - isat_project"
    echo "  - isat_extract"
    echo "  - isat_retrieve"
    echo "  - isat_match"
    echo "  - isat_geo"
    echo "  - isat_twoview"
    echo "  - isat_calibrate"
    echo "  - isat_camera_estimator"
    echo ""
    echo "Exit with: exit"
    echo ""
    docker run --gpus all -it --rm \
        -v /data:/data:rw \
        "${IMAGE_NAME}" /bin/bash
}

extract_binaries() {
    echo "======================================"
    echo "Extracting binaries from container to:"
    echo "  ${BUILD_EXTRACT_DIR}"
    echo "======================================"
    
    # Check if image exists
    if ! docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
        echo "ERROR: Image ${IMAGE_NAME} not found!"
        echo "Run './docker-test.sh build' first"
        exit 1
    fi
    
    # Create temporary container
    docker create --name "${CONTAINER_NAME}" "${IMAGE_NAME}" /bin/bash > /dev/null
    
    # Extract build directory
    mkdir -p "${BUILD_EXTRACT_DIR}"
    docker cp "${CONTAINER_NAME}:/workspace/insightat/build" "${BUILD_EXTRACT_DIR}/" || {
        docker rm "${CONTAINER_NAME}"
        echo "ERROR: Failed to extract binaries!"
        exit 1
    }
    
    # Clean up
    docker rm "${CONTAINER_NAME}" > /dev/null
    
    echo ""
    echo "✓ Extraction successful!"
    echo "Binaries available at: ${BUILD_EXTRACT_DIR}/build/"
    echo ""
    echo "Available tools:"
    ls -1 "${BUILD_EXTRACT_DIR}/build/isat_"* 2>/dev/null | sed 's/.*/  - &/' || echo "  (none found)"
    echo ""
}

clean_image() {
    echo "Removing Docker image: ${IMAGE_NAME}"
    docker rmi -f "${IMAGE_NAME}" || echo "Image not found or removal failed"
    echo "Done."
}

run_full() {
    build_image
    extract_binaries
}

# Main
case "${1:-help}" in
    build)
        build_image
        ;;
    shell)
        run_shell
        ;;
    extract)
        extract_binaries
        ;;
    run)
        run_full
        ;;
    clean)
        clean_image
        ;;
    help)
        show_help
        ;;
    *)
        echo "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
