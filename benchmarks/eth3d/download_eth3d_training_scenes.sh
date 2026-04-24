#!/usr/bin/env bash
# Download ETH3D *high-res multi-view training* DSLR JPG archives into <dataset>/zip/
# for use with: python3 benchmarks/eth3d/prepare_datasets.py -d <dataset> --extract
#
# License: ETH3D data is CC BY-NC-SA 4.0 — use only under their terms.
# Official page: https://www.eth3d.net/datasets
#
# Usage:
#   ./benchmarks/eth3d/download_eth3d_training_scenes.sh /path/to/dataset_root
#   DRY_RUN=1 ./benchmarks/eth3d/download_eth3d_training_scenes.sh /path/to/dataset_root
#   SCENES="courtyard pipes" ./benchmarks/eth3d/download_eth3d_training_scenes.sh /path/to/dataset_root

set -euo pipefail

BASE_URL="${ETH3D_BASE_URL:-https://www.eth3d.net/data}"

if [[ "${1:-}" == "" || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: $0 <dataset_root>" >&2
  echo "  Downloads <scene>_dslr_jpg.7z into <dataset_root>/zip/ (skipped if already present)." >&2
  exit 2
fi

ROOT="$(realpath "$1")"
ZIP_DIR="$ROOT/zip"
mkdir -p "$ZIP_DIR"

# Default: common high-res multi-view *training* scenes (subset; add more from eth3d.net/datasets).
DEFAULT_SCENES=(
  courtyard delivery_area electro facade kicker meadow office pipes playground
  relief relief_2 terrace terrains
)

if [[ -n "${SCENES:-}" ]]; then
  read -r -a SCENE_LIST <<< "${SCENES}"
else
  SCENE_LIST=("${DEFAULT_SCENES[@]}")
fi

for s in "${SCENE_LIST[@]}"; do
  file="${s}_dslr_jpg.7z"
  dest="$ZIP_DIR/$file"
  if [[ -f "$dest" ]]; then
    echo "[skip] $dest already exists"
    continue
  fi
  url="${BASE_URL}/${file}"
  echo "[get] $url"
  if [[ -n "${DRY_RUN:-}" ]]; then
    continue
  fi
  wget -c -O "$dest" "$url"
done

echo "Done. Next: python3 benchmarks/eth3d/prepare_datasets.py -d \"$ROOT\" --extract"
