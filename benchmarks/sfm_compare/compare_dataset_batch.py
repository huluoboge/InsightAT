#!/usr/bin/env python3
"""
For each scene under <dataset>/scenes/, compare COLMAP vs InsightAT sparse models
if both exist:

  results/colmap/workspace/sparse/0
  results/insightat/work/incremental_sfm/colmap/sparse/0

Writes compare.json in dataset root (or --output).
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
    ap = argparse.ArgumentParser(description="Batch compare COLMAP vs InsightAT per scene")
    ap.add_argument("-d", "--dataset-root", required=True, type=Path)
    ap.add_argument("-o", "--output", type=Path, default=None, help="Summary JSON (default: <dataset>/compare_colmap_insightat.json)")
    args = ap.parse_args()

    root = args.dataset_root.resolve()
    scenes_dir = root / "scenes"
    if not scenes_dir.is_dir():
        print(f"Error: {scenes_dir} not found", file=sys.stderr)
        return 2

    out_path = args.output or (root / "compare_colmap_insightat.json")
    rows = []
    for scene in sorted(p.name for p in scenes_dir.iterdir() if p.is_dir()):
        ref = scenes_dir / scene / "results" / "colmap" / "workspace" / "sparse" / "0" / "images.txt"
        est = (
            scenes_dir
            / scene
            / "results"
            / "insightat"
            / "work"
            / "incremental_sfm"
            / "colmap"
            / "sparse"
            / "0"
            / "images.txt"
        )
        if not ref.is_file() or not est.is_file():
            rows.append({"scene": scene, "ok": False, "error": "missing sparse model"})
            print(f"[compare] {scene}: skip (missing ref or est)")
            continue
        ref_cams = load_cameras_by_basename(ref)
        est_cams = load_cameras_by_basename(est)
        common = sorted(set(ref_cams.keys()) & set(est_cams.keys()))
        if len(common) < 3:
            rows.append(
                {
                    "scene": scene,
                    "ok": False,
                    "error": f"too few common images ({len(common)})",
                    "n_common": len(common),
                }
            )
            print(f"[compare] {scene}: skip (common={len(common)})")
            continue
        src_C = np.stack([ref_cams[k].C for k in common], axis=0)
        dst_C = np.stack([est_cams[k].C for k in common], axis=0)
        m = evaluate_alignment(src_C, dst_C)
        m["scene"] = scene
        m["ok"] = True
        m["n_common"] = len(common)
        rows.append(m)
        print(
            f"[compare] {scene}: matched={len(common)} rmse={m['rmse_m']:.4f}m median={m['median_m']:.4f}m"
        )

    out_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
