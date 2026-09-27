#!/usr/bin/env bash
# Optional legacy Qt GUI-only build (not part of default product packages).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${INSIGHTAT_GUI_BUILD_DIR:-${REPO_ROOT}/build-gui}"

if ! command -v cmake >/dev/null 2>&1; then
  echo "ERROR: cmake not found in PATH" >&2
  exit 1
fi

echo "[InsightAT] Configuring legacy Qt GUI-only build in ${BUILD_DIR}"
echo "[InsightAT] Prefer sfm-gui (Node) for product UI."
cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
  -DINSIGHTAT_BUILD_QT_UI=ON \
  -DINSIGHTAT_BUILD_GUI_ONLY=ON \
  "$@"

cmake --build "${BUILD_DIR}" --target InsightAT -j"$(nproc)"
