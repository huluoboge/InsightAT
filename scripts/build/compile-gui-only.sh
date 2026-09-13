#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
BUILD_DIR="${INSIGHTAT_GUI_BUILD_DIR:-${REPO_ROOT}/build-gui}"

if ! command -v cmake >/dev/null 2>&1; then
	echo "ERROR: cmake not found in PATH" >&2
	exit 1
fi

echo "[InsightAT] Configuring GUI-only build in ${BUILD_DIR}"
cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
	-DINSIGHTAT_BUILD_GUI_ONLY=ON \
	"$@"

echo "[InsightAT] Building InsightAT GUI target"
cmake --build "${BUILD_DIR}" --target InsightAT -j"$(nproc)"
