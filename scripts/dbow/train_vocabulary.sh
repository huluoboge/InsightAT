#!/bin/bash

################################################################################
# DBoW3 Vocabulary Tree Training Pipeline
#
# This script provides a one-click pipeline to train DBoW3 vocabulary trees
# from downloaded datasets.
#
# Prerequisites:
#   - Run download_datasets.sh first to obtain training data
#   - InsightAT compiled with isat_train_vocab tool
#
# Usage:
#   ./train_vocabulary.sh [OPTIONS]
#
# Options:
#   --data-dir DIR     Directory containing training images (default: auto-detect)
#   --output FILE      Output vocabulary file (default: vocabulary_k10_L6.dbow3)
#   --branching K      Branching factor (default: 10)
#   --depth L          Tree depth (default: 6)
#   --threads N        Number of threads (default: auto)
#   --max-images N     Maximum images to use for training (default: unlimited)
#   --help             Show this help message
################################################################################

set -euo pipefail

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}/../.."
DATA_DIR="${SCRIPT_DIR}/../../data/dbow_training"
BUILD_DIR="${PROJECT_ROOT}/build"
TRAINER="${BUILD_DIR}/isat_train_vocab"
OUTPUT_FILE="vocabulary_k10_L6.dbow3"
BRANCHING=10
DEPTH=6
THREADS=$(nproc)
MAX_IMAGES=0  # 0 = unlimited

# Logging
log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $*"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# Help message
show_help() {
    sed -n '2,18p' "$0" | sed 's/^# //' | sed 's/^#//'
    exit 0
}

# Parse arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --data-dir)
                DATA_DIR="$2"
                shift 2
                ;;
            --output)
                OUTPUT_FILE="$2"
                shift 2
                ;;
            --branching)
                BRANCHING="$2"
                shift 2
                ;;
            --depth)
                DEPTH="$2"
                shift 2
                ;;
            --threads)
                THREADS="$2"
                shift 2
                ;;
            --max-images)
                MAX_IMAGES="$2"
                shift 2
                ;;
            --help|-h)
                show_help
                ;;
            *)
                log_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done
}

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."
    
    # Check if trainer exists
    if [[ ! -f "$TRAINER" ]]; then
        log_error "Trainer not found: ${TRAINER}"
        log_info "Please compile InsightAT first: cd ${PROJECT_ROOT}/build && make isat_train_vocab"
        exit 1
    fi
    
    # Check if data directory exists
    if [[ ! -d "$DATA_DIR" ]]; then
        log_error "Data directory not found: ${DATA_DIR}"
        log_info "Please run download_datasets.sh first to obtain training data"
        exit 1
    fi
    
    log_success "Prerequisites OK"
}

# Collect image paths
collect_images() {
    log_info "Collecting training images from ${DATA_DIR}..."
    
    local image_list_file="${DATA_DIR}/image_list.txt"
    
    # Find all images (common formats from datasets)
    find "$DATA_DIR" -type f \( \
        -iname "*.png" -o \
        -iname "*.jpg" -o \
        -iname "*.jpeg" \
    \) > "$image_list_file"
    
    local total_images=$(wc -l < "$image_list_file")
    
    if [[ $total_images -eq 0 ]]; then
        log_error "No images found in ${DATA_DIR}"
        log_info "Please ensure datasets are downloaded and extracted"
        exit 1
    fi
    
    log_success "Found ${total_images} images"
    
    # Limit images if requested
    if [[ $MAX_IMAGES -gt 0 && $total_images -gt $MAX_IMAGES ]]; then
        log_info "Limiting to ${MAX_IMAGES} images (random sampling)"
        shuf -n "$MAX_IMAGES" "$image_list_file" > "${image_list_file}.tmp"
        mv "${image_list_file}.tmp" "$image_list_file"
    fi
    
    echo "$image_list_file"
}

# Estimate training time
estimate_training_time() {
    local num_images=$1
    
    # Rough estimates based on typical hardware (very approximate)
    # Training time depends heavily on CPU, image size, k, L
    local seconds_per_image=0.5
    local total_seconds=$(echo "$num_images * $seconds_per_image" | bc)
    local minutes=$(echo "$total_seconds / 60" | bc)
    
    log_info "Estimated training time: ~${minutes} minutes (rough estimate)"
    log_warning "Actual time varies based on CPU, image resolution, and parameters"
}

# Train vocabulary
train_vocabulary() {
    local image_list=$1
    local num_images=$(wc -l < "$image_list")
    
    log_info "=== Training Vocabulary Tree ==="
    log_info "Configuration:"
    log_info "  - Images: ${num_images}"
    log_info "  - Branching factor (k): ${BRANCHING}"
    log_info "  - Depth (L): ${DEPTH}"
    log_info "  - Threads: ${THREADS}"
    log_info "  - Output: ${OUTPUT_FILE}"
    
    estimate_training_time "$num_images"
    
    # Run trainer
    log_info "Starting training (this may take a while)..."
    
    if "$TRAINER" \
        --image-list "$image_list" \
        --output "$OUTPUT_FILE" \
        --branching "$BRANCHING" \
        --depth "$DEPTH" \
        --threads "$THREADS"; then
        
        log_success "Vocabulary training completed!"
        
        # Show file size
        if [[ -f "$OUTPUT_FILE" ]]; then
            local file_size=$(du -h "$OUTPUT_FILE" | cut -f1)
            log_info "Vocabulary file size: ${file_size}"
            log_info "Vocabulary saved to: $(readlink -f "$OUTPUT_FILE")"
        fi
    else
        log_error "Training failed!"
        exit 1
    fi
}

# Print usage instructions
show_usage_instructions() {
    cat << EOF

================================================================================
                        VOCABULARY TREE USAGE
================================================================================

Your vocabulary tree is ready! Use it with isat_retrieve:

1. Basic Retrieval
   ---------------------------------------------------------------------------
   ${BUILD_DIR}/isat_retrieve \\
       --database-path scene.db \\
       --image-path query.jpg \\
       --retrieval-strategy vocab \\
       --vocab-file $(readlink -f "$OUTPUT_FILE")

2. Vocabulary Statistics
   ---------------------------------------------------------------------------
   Show tree structure and statistics:
   
   ${TRAINER} --vocab-file "$OUTPUT_FILE" --print-stats

3. Optimization Tips
   ---------------------------------------------------------------------------
   - For LARGE image databases (>10K images):
     Increase k and L: --branching 16 --depth 8

   - For SMALL datasets (<1K images):
     Use smaller tree: --branching 6 --depth 5

   - For FASTER training:
     Reduce --max-images (e.g., 5000-10000 images is often sufficient)

   - For BETTER accuracy:
     Use more diverse training data (multiple datasets mixed)

4. Performance Benchmarks
   ---------------------------------------------------------------------------
   Typical query times (depends on database size and hardware):
   - Small (1K images): ~10-50ms per query
   - Medium (10K images): ~50-200ms per query
   - Large (100K+ images): ~200-1000ms per query

================================================================================

EOF
}

# Main
main() {
    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       DBoW3 Vocabulary Training - InsightAT                    ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""
    
    parse_args "$@"
    check_prerequisites
    
    local image_list
    image_list=$(collect_images)
    
    train_vocabulary "$image_list"
    
    show_usage_instructions
    
    log_success "All done! Your vocabulary tree is ready to use."
    echo ""
}

main "$@"
