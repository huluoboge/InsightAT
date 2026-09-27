#!/usr/bin/env bash
# Package InsightAT SfM Viewer for Linux (dir / AppImage / deb) or Windows zip.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SFM_VIEWER="$ROOT/sfm-viewer"
MODE="${1:-dir}"

echo "[build_sfm_viewer] repo=$ROOT mode=$MODE"
(cd "$SFM_VIEWER" && npm install)
(cd "$SFM_VIEWER" && npm run check)

case "$MODE" in
  dir)
    (cd "$SFM_VIEWER" && npm run pack)
    ;;
  appimage)
    (cd "$SFM_VIEWER" && npm run pack:appimage)
    ;;
  deb)
    (cd "$SFM_VIEWER" && npm run pack:deb)
    ;;
  win|windows)
    (cd "$SFM_VIEWER" && npm run pack:win)
    ;;
  linux-all)
    (cd "$SFM_VIEWER" && npx electron-builder --linux dir AppImage deb)
    ;;
  *)
    echo "Usage: $0 [dir|appimage|deb|win|linux-all]" >&2
    exit 2
    ;;
esac

echo "[build_sfm_viewer] done → $ROOT/dist/sfm-viewer"
ls -la "$ROOT/dist/sfm-viewer" || true
