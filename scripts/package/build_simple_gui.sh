#!/usr/bin/env bash
# Package InsightAT Simple GUI (+ embedded sfm-viewer) for Linux.
# Optional: set ISAT_BIN_DIR to a directory containing isat_* binaries to bundle them.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIMPLE_GUI="$ROOT/simple-gui"
SFM_VIEWER="$ROOT/sfm-viewer"
STAGING_BIN="$ROOT/staging/bin"
MODE="${1:-dir}"

echo "[build_simple_gui] repo=$ROOT"

mkdir -p "$STAGING_BIN"
# Ensure staging/bin exists so electron-builder extraResources does not fail when empty.
# Place a marker if no CLI tools are present.
if [[ -z "$(ls -A "$STAGING_BIN" 2>/dev/null || true)" ]]; then
  echo "CLI tools not staged. Set ISAT_BIN_DIR to bundle isat_* binaries." > "$STAGING_BIN/README.txt"
fi

BIN_SRC="${ISAT_BIN_DIR:-}"
if [[ -z "$BIN_SRC" && -d "$ROOT/build" ]]; then
  BIN_SRC="$ROOT/build"
fi
if [[ -n "$BIN_SRC" && -d "$BIN_SRC" ]]; then
  echo "[build_simple_gui] staging CLI from $BIN_SRC"
  shopt -s nullglob
  for f in "$BIN_SRC"/isat_*; do
    cp -a "$f" "$STAGING_BIN/"
  done
  # Optional shared libs sitting next to binaries
  for f in "$BIN_SRC"/*.so "$BIN_SRC"/*.so.*; do
    [[ -e "$f" ]] || continue
    cp -a "$f" "$STAGING_BIN/" || true
  done
  shopt -u nullglob
  rm -f "$STAGING_BIN/README.txt"
fi

echo "[build_simple_gui] npm install (sfm-viewer)"
(cd "$SFM_VIEWER" && npm install)

echo "[build_simple_gui] npm install (simple-gui)"
(cd "$SIMPLE_GUI" && npm install)

echo "[build_simple_gui] syntax check"
(cd "$SFM_VIEWER" && npm run check)
(cd "$SIMPLE_GUI" && npm run check)

case "$MODE" in
  dir)
    (cd "$SIMPLE_GUI" && npm run pack)
    ;;
  appimage)
    (cd "$SIMPLE_GUI" && npm run pack:appimage)
    ;;
  *)
    echo "Usage: $0 [dir|appimage]" >&2
    exit 2
    ;;
esac

echo "[build_simple_gui] done → $ROOT/dist/simple-gui"
ls -la "$ROOT/dist/simple-gui" || true
