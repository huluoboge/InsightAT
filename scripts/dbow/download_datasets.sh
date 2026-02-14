#!/bin/bash

################################################################################
# Dataset Downloader for DBoW3 Vocabulary Training
# 
# This script downloads public datasets for training DBoW3 vocabulary trees.
# Supports incremental downloads (resume on network interruption).
#
# Datasets:
#   - EuRoC MAV Dataset (4 sequences, ~8GB total)
#   - COCO Validation 2017 (~1GB, 5000 images)
#   - UZH-FPV Drone Racing (manual download instructions)
#   - Mid-Air Dataset (manual download instructions)
#
# Usage:
#   ./download_datasets.sh [OPTIONS]
#
# Options:
#   --euroc       Download EuRoC MAV dataset only
#   --coco        Download COCO validation dataset only
#   --all         Download all automated datasets (default)
#   --extract     Extract downloaded archives after download
#   --keep-zip    Keep zip/archive files after extraction
#   --help        Show this help message
################################################################################

set -euo pipefail

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="${SCRIPT_DIR}/../../data/dbow_training"
EUROC_DIR="${DATA_DIR}/euroc"
COCO_DIR="${DATA_DIR}/coco"
DOWNLOAD_EUROC=false
DOWNLOAD_COCO=false
AUTO_EXTRACT=false
KEEP_ARCHIVES=false

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*"
}

# Help message
show_help() {
    sed -n '2,24p' "$0" | sed 's/^# //' | sed 's/^#//'
    exit 0
}

# Parse command line arguments
parse_args() {
    if [[ $# -eq 0 ]]; then
        DOWNLOAD_EUROC=true
        DOWNLOAD_COCO=true
        return
    fi

    while [[ $# -gt 0 ]]; do
        case $1 in
            --euroc)
                DOWNLOAD_EUROC=true
                shift
                ;;
            --coco)
                DOWNLOAD_COCO=true
                shift
                ;;
            --all)
                DOWNLOAD_EUROC=true
                DOWNLOAD_COCO=true
                shift
                ;;
            --extract)
                AUTO_EXTRACT=true
                shift
                ;;
            --keep-zip)
                KEEP_ARCHIVES=true
                shift
                ;;
            --help|-h)
                show_help
                ;;
            *)
                log_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done

    # Default to all if nothing specified
    if [[ "$DOWNLOAD_EUROC" == "false" && "$DOWNLOAD_COCO" == "false" ]]; then
        DOWNLOAD_EUROC=true
        DOWNLOAD_COCO=true
    fi
}

# Check dependencies
check_dependencies() {
    log_info "Checking dependencies..."
    
    local missing_deps=()
    
    if ! command -v wget &> /dev/null; then
        missing_deps+=("wget")
    fi
    
    if ! command -v unzip &> /dev/null; then
        missing_deps+=("unzip")
    fi
    
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        log_error "Missing required dependencies: ${missing_deps[*]}"
        log_info "Install with: sudo apt-get install ${missing_deps[*]}"
        exit 1
    fi
    
    log_success "All dependencies found"
}

# Create directory structure
setup_directories() {
    log_info "Setting up directory structure..."
    
    mkdir -p "${DATA_DIR}"
    mkdir -p "${EUROC_DIR}"
    mkdir -p "${COCO_DIR}"
    
    log_success "Directories created at: ${DATA_DIR}"
}

# Download with resume support
download_file() {
    local url=$1
    local output=$2
    local description=$3
    
    log_info "Downloading ${description}..."
    log_info "URL: ${url}"
    log_info "Output: ${output}"
    
    # Use wget with:
    #   -c: continue/resume
    #   -t 5: retry 5 times
    #   -T 30: timeout 30 seconds
    #   --show-progress: progress bar
    if wget -c -t 5 -T 30 --show-progress "${url}" -O "${output}"; then
        log_success "Downloaded ${description}"
        return 0
    else
        log_error "Failed to download ${description}"
        return 1
    fi
}

# Download EuRoC MAV Dataset
download_euroc() {
    log_info "=== Downloading EuRoC MAV Dataset ==="
    
    # EuRoC dataset URLs (ASL ETH Zurich)
    local base_url="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
    
    # Selected sequences (Machine Hall and Vicon Room)
    declare -A sequences=(
        ["MH_01_easy"]="machine_hall/MH_01_easy/MH_01_easy.zip"
        ["MH_02_easy"]="machine_hall/MH_02_easy/MH_02_easy.zip"
        ["V1_01_easy"]="vicon_room1/V1_01_easy/V1_01_easy.zip"
        ["V2_01_easy"]="vicon_room2/V2_01_easy/V2_01_easy.zip"
    )
    
    local success_count=0
    local total_count=${#sequences[@]}
    
    for seq_name in "${!sequences[@]}"; do
        local seq_path="${sequences[$seq_name]}"
        local url="${base_url}/${seq_path}"
        local output="${EUROC_DIR}/${seq_name}.zip"
        
        if [[ -f "${output}" ]]; then
            log_warning "File already exists: ${output}"
            log_info "Remove it to re-download, or it will be resumed if incomplete"
        fi
        
        if download_file "${url}" "${output}" "EuRoC ${seq_name}"; then
            ((success_count++))
        fi
    done
    
    log_info "Downloaded ${success_count}/${total_count} EuRoC sequences"
    
    if [[ "$AUTO_EXTRACT" == "true" ]]; then
        extract_euroc
    fi
}

# Extract EuRoC archives
extract_euroc() {
    log_info "=== Extracting EuRoC MAV Dataset ==="
    
    for zipfile in "${EUROC_DIR}"/*.zip; do
        if [[ -f "$zipfile" ]]; then
            local basename=$(basename "$zipfile" .zip)
            local extract_dir="${EUROC_DIR}/${basename}"
            
            if [[ -d "$extract_dir" ]]; then
                log_warning "Already extracted: ${basename}"
                continue
            fi
            
            log_info "Extracting ${basename}..."
            if unzip -q "$zipfile" -d "${EUROC_DIR}"; then
                log_success "Extracted ${basename}"
                
                if [[ "$KEEP_ARCHIVES" == "false" ]]; then
                    rm "$zipfile"
                    log_info "Removed archive: ${basename}.zip"
                fi
            else
                log_error "Failed to extract ${basename}"
            fi
        fi
    done
}

# Download COCO Dataset
download_coco() {
    log_info "=== Downloading COCO Validation 2017 ==="
    
    local coco_url="http://images.cocodataset.org/zips/val2017.zip"
    local output="${COCO_DIR}/val2017.zip"
    
    if [[ -f "${output}" ]]; then
        log_warning "File already exists: ${output}"
        log_info "Remove it to re-download, or it will be resumed if incomplete"
    fi
    
    if download_file "${coco_url}" "${output}" "COCO Validation 2017"; then
        if [[ "$AUTO_EXTRACT" == "true" ]]; then
            extract_coco
        fi
    fi
}

# Extract COCO archive
extract_coco() {
    log_info "=== Extracting COCO Validation 2017 ==="
    
    local zipfile="${COCO_DIR}/val2017.zip"
    
    if [[ ! -f "$zipfile" ]]; then
        log_error "COCO archive not found: ${zipfile}"
        return 1
    fi
    
    if [[ -d "${COCO_DIR}/val2017" ]]; then
        log_warning "Already extracted: COCO val2017"
        return 0
    fi
    
    log_info "Extracting COCO validation set..."
    if unzip -q "$zipfile" -d "${COCO_DIR}"; then
        log_success "Extracted COCO validation set"
        
        if [[ "$KEEP_ARCHIVES" == "false" ]]; then
            rm "$zipfile"
            log_info "Removed archive: val2017.zip"
        fi
    else
        log_error "Failed to extract COCO"
        return 1
    fi
}

# Print manual download instructions
show_manual_instructions() {
    cat << 'EOF'

================================================================================
                        MANUAL DOWNLOAD INSTRUCTIONS
================================================================================

Some datasets require manual download due to registration/license requirements:

RECOMMENDED: BaiduPan Datasets (中国大陆用户推荐)
================================================================================

1. TUM RGB-D Dataset (推荐！)
   ------------------------------------------------------------------------------
   Baidu Pan: https://pan.baidu.com/s/1nwXtGqH
   Password: lsgr
   Size: ~5GB | Images: ~30K
   
   Extract to: data/dbow_training/tum/
   
   Best for: Indoor SLAM, general vocabulary training

2. KITTI Dataset (推荐！)
   ------------------------------------------------------------------------------
   Baidu Pan: https://pan.baidu.com/s/1htFmXDE
   Password: uu20
   Size: ~22GB | Images: ~20K
   
   Extract to: data/dbow_training/kitti/
   
   Best for: Outdoor scenes, street view, adding diversity

3. DSO Dataset
   ------------------------------------------------------------------------------
   Baidu Pan: https://pan.baidu.com/s/1eSRmeZK
   Password: 6x5b
   Size: ~2GB | Images: ~10K
   
   Extract to: data/dbow_training/dso/
   
   Best for: Direct method sequences, monocular SLAM

4. Mono Dataset
   ------------------------------------------------------------------------------
   Baidu Pan: https://pan.baidu.com/s/1jKaNB3C
   Password: u57r
   Size: ~3GB | Images: ~15K
   
   Extract to: data/dbow_training/mono/
   
   Best for: Monocular visual odometry

International Datasets
================================================================================

5. UZH-FPV Drone Racing Dataset
   ------------------------------------------------------------------------------
   URL: http://rpg.ifi.uzh.ch/uzh-fpv.html
   
   Steps:
   a) Visit the website and accept terms
   b) Download desired sequences (indoor, outdoor)
   c) Extract to: data/dbow_training/uzh_fpv/
   
   Recommended sequences:
   - indoor_forward_3_snapdragon
   - outdoor_forward_1_snapdragon

6. Mid-Air Dataset
   ------------------------------------------------------------------------------
   URL: https://midair.ulg.ac.be/
   
   Steps:
   a) Visit website and register (academic email recommended)
   b) Download "Stereo Images" subset (~50GB) or specific sequences
   c) Extract to: data/dbow_training/midair/
   
   Note: Full dataset is ~1.5TB. Download only what you need.

7. Alternative: COLMAP's sample datasets
   ------------------------------------------------------------------------------
   COLMAP provides smaller test datasets:
   URL: https://demuc.de/colmap/#download
   
   However, these are NOT suitable for production vocabulary training.
   Use only for testing.

================================================================================

EOF
}

# Print dataset statistics
show_statistics() {
    log_info "=== Dataset Statistics ==="
    
    if [[ -d "$EUROC_DIR" ]]; then
        local euroc_images=$(find "$EUROC_DIR" -name "*.png" 2>/dev/null | wc -l || echo "0")
        log_info "EuRoC images: ${euroc_images}"
    fi
    
    if [[ -d "${COCO_DIR}/val2017" ]]; then
        local coco_images=$(find "${COCO_DIR}/val2017" -name "*.jpg" 2>/dev/null | wc -l || echo "0")
        log_info "COCO images: ${coco_images}"
    fi
    
    local total_size=$(du -sh "${DATA_DIR}" 2>/dev/null | cut -f1 || echo "unknown")
    log_info "Total dataset size: ${total_size}"
}

# Main execution
main() {
    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       DBoW3 Dataset Downloader - InsightAT                     ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""
    
    parse_args "$@"
    check_dependencies
    setup_directories
    
    if [[ "$DOWNLOAD_EUROC" == "true" ]]; then
        download_euroc
    fi
    
    if [[ "$DOWNLOAD_COCO" == "true" ]]; then
        download_coco
    fi
    
    show_manual_instructions
    show_statistics
    
    echo ""
    log_success "Download process completed!"
    log_info "Data location: ${DATA_DIR}"
    log_info "Next step: Run ./train_vocabulary.sh to train vocabulary tree"
    echo ""
}

# Run main
main "$@"
