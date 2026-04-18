#!/usr/bin/env python3
"""
Compare two COLMAP sparse folders (sparse/0) by similarity alignment of camera centers.

Matches images by filename (basename). Reports position RMSE / median after Umeyama.

Example:
  python3 benchmarks/sfm_compare/compare_colmap_sparse.py \\
    --ref  /data/scenes/courtyard/results/colmap/workspace/sparse/0 \\
    --est  /data/scenes/courtyard/results/insightat/work/incremental_sfm/colmap/sparse/0
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

import numpy as np

from benchmarks.sfm_compare.align_similarity import evaluate_alignment  # noqa: E402
from benchmarks.sfm_compare.colmap_sparse import load_cameras_by_basename  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Compare two COLMAP sparse models (camera centers)")
    ap.add_argument("--ref", required=True, type=Path, help="Reference sparse/0 directory")
    ap.add_argument("--est", required=True, type=Path, help="Estimated sparse/0 directory")
    ap.add_argument("-o", "--output-json", type=Path, default=None, help="Write metrics JSON")
    args = ap.parse_args()

    ref_dir = args.ref.resolve()
    est_dir = args.est.resolve()
    ref_im = ref_dir / "images.txt"
    est_im = est_dir / "images.txt"
    if not ref_im.is_file() or not est_im.is_file():
        print("Error: images.txt missing in ref or est.", file=sys.stderr)
        return 2

    ref_cams = load_cameras_by_basename(ref_im)
    est_cams = load_cameras_by_basename(est_im)
    common = sorted(set(ref_cams.keys()) & set(est_cams.keys()))
    if len(common) < 3:
        print(f"Error: need >=3 common images, got {len(common)}", file=sys.stderr)
        return 2

    src_C = np.stack([ref_cams[k].C for k in common], axis=0)
    dst_C = np.stack([est_cams[k].C for k in common], axis=0)

    metrics = evaluate_alignment(src_C, dst_C)
    metrics["reference"] = str(ref_dir)
    metrics["estimated"] = str(est_dir)
    metrics["matched_basenames"] = len(common)

    print(
        f"matched={len(common)}  rmse={metrics['rmse_m']:.4f}m  median={metrics['median_m']:.4f}m  "
        f"scale={metrics['scale']:.6f}"
    )

    if args.output_json:
        args.output_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
