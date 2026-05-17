#!/usr/bin/env bash
set -euo pipefail

# DTU official dataset downloader.
# Default mode downloads SampleSet only (recommended first step).
#
# Usage:
#   ./benchmarks/dtu/download_dtu.sh [output_dir]
#
# Environment options:
#   DTU_MODE=sample|rectified|full
#     sample    -> SampleSet.zip
#     rectified -> SampleSet.zip + Rectified.zip
#     full      -> SampleSet.zip + Rectified.zip + Points.zip
#
#   KEEP_ARCHIVES=0|1   (default: 1)

BASE_DIR="${1:-./DTU_MVS}"
DTU_MODE="${DTU_MODE:-sample}"
KEEP_ARCHIVES="${KEEP_ARCHIVES:-1}"

mkdir -p "${BASE_DIR}"
cd "${BASE_DIR}"
BASE_DIR_ABS="$(pwd)"

echo "============================================="
echo "DTU MVS 官方数据下载脚本"
echo "保存目录: ${BASE_DIR_ABS}"
echo "下载模式: ${DTU_MODE}"
echo "============================================="

if ! command -v unzip >/dev/null 2>&1; then
    echo "ERROR: unzip 未安装，请先安装后重试。"
    exit 1
fi

if ! command -v wget >/dev/null 2>&1 && ! command -v curl >/dev/null 2>&1; then
    echo "ERROR: 需要 wget 或 curl 之一用于下载。"
    exit 1
fi

DTU_BASE_URL="http://roboimagedata2.compute.dtu.dk/data/MVS"

SAMPLESET_URL="${DTU_BASE_URL}/SampleSet.zip"
RECTIFIED_URL="${DTU_BASE_URL}/Rectified.zip"
POINTS_URL="${DTU_BASE_URL}/Points.zip"

download_file() {
    local url="$1"
    local out="$2"

    if [[ -s "${out}" ]]; then
        echo "检测到本地文件，跳过下载: ${out}"
        return 0
    fi

    echo "下载: ${url}"
    if command -v wget >/dev/null 2>&1; then
        wget -c --tries=5 --timeout=60 "${url}" -O "${out}"
    else
        curl -fL -C - --retry 5 --connect-timeout 60 -o "${out}" "${url}"
    fi

    if [[ ! -s "${out}" ]]; then
        echo "ERROR: 下载失败或文件为空: ${out}"
        exit 1
    fi
}

extract_zip() {
    local archive="$1"
    echo "解压: ${archive}"
    unzip -q -o "${archive}" -d ./
}

case "${DTU_MODE}" in
    sample)
        FILES=("SampleSet.zip")
        URLS=("${SAMPLESET_URL}")
        ;;
    rectified)
        FILES=("SampleSet.zip" "Rectified.zip")
        URLS=("${SAMPLESET_URL}" "${RECTIFIED_URL}")
        ;;
    full)
        FILES=("SampleSet.zip" "Rectified.zip" "Points.zip")
        URLS=("${SAMPLESET_URL}" "${RECTIFIED_URL}" "${POINTS_URL}")
        ;;
    *)
        echo "ERROR: 不支持的 DTU_MODE=${DTU_MODE}，可选: sample | rectified | full"
        exit 1
        ;;
esac

echo "开始下载..."
for i in "${!FILES[@]}"; do
    download_file "${URLS[$i]}" "${FILES[$i]}"
done

echo "开始解压..."
for archive in "${FILES[@]}"; do
    extract_zip "${archive}"
done

if [[ "${KEEP_ARCHIVES}" != "1" ]]; then
    echo "删除压缩包..."
    rm -f "${FILES[@]}"
fi

echo "============================================="
echo "DTU 数据准备完成"
echo "目录: ${BASE_DIR_ABS}"
if command -v tree >/dev/null 2>&1; then
    tree -L 2 .
else
    ls -la
fi
echo "============================================="
echo "建议:"
echo "1) 先用 DTU_MODE=sample 跑通流程"
echo "2) 再用 DTU_MODE=rectified 或 full 做正式 benchmark"
