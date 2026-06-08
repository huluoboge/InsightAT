#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${INSIGHTAT_GUI_BUILD_DIR:-${SCRIPT_DIR}/build-gui}"

if ! command -v cmake >/dev/null 2>&1; then
	echo "ERROR: cmake not found in PATH" >&2
	exit 1
fi

echo "[InsightAT] Configuring GUI-only build in ${BUILD_DIR}"
cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
	-DINSIGHTAT_BUILD_GUI_ONLY=ON \
	"$@"

echo "[InsightAT] Building InsightAT GUI target"
cmake --build "${BUILD_DIR}" --target InsightAT -j"$(nproc)"