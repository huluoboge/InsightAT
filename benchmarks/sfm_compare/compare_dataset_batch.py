#!/usr/bin/env python3
"""
For each scene under <dataset>/scenes/, align camera centers and report errors.

Default reference: ETH3D ground truth COLMAP text model at
  scenes/<scene>/gt/images.txt
when present. Estimate: InsightAT export at
  scenes/<scene>/results/insightat/work/incremental_sfm/colmap/sparse/0/images.txt

Optional --ref-source colmap uses COLMAP sparse reconstruction instead of gt/ (legacy A/B).

Writes JSON array to --output (see defaults below).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

import numpy as np

from benchmarks.sfm_compare.align_similarity import evaluate_alignment  # noqa: E402
from benchmarks.sfm_compare.colmap_sparse import load_cameras_by_basename  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Batch compare GT (or COLMAP) vs InsightAT sparse models per scene"
    )
    ap.add_argument("-d", "--dataset-root", required=True, type=Path)
    ap.add_argument(
        "--ref-source",
        choices=("gt", "colmap"),
        default="gt",
        help="Reference poses: gt/images.txt (ETH3D calibration) or COLMAP sparse/0 (default: gt)",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Summary JSON (default: <dataset>/compare_gt_insightat.json or compare_colmap_insightat.json)",
    )
    args = ap.parse_args()

    root = args.dataset_root.resolve()
    scenes_dir = root / "scenes"
    if not scenes_dir.is_dir():
        print(f"Error: {scenes_dir} not found", file=sys.stderr)
        return 2

    if args.output is not None:
        out_path = args.output
    else:
        out_path = root / (
            "compare_gt_insightat.json" if args.ref_source == "gt" else "compare_colmap_insightat.json"
        )

    rows: List[dict] = []
    for scene in sorted(p.name for p in scenes_dir.iterdir() if p.is_dir()):
        base = scenes_dir / scene
        if args.ref_source == "gt":
            ref = base / "gt" / "images.txt"
            ref_tag = "gt"
        else:
            ref = base / "results" / "colmap" / "workspace" / "sparse" / "0" / "images.txt"
            ref_tag = "colmap"

        est = (
            base
            / "results"
            / "insightat"
            / "work"
            / "incremental_sfm"
            / "colmap"
            / "sparse"
            / "0"
            / "images.txt"
        )

        if not ref.is_file():
            rows.append(
                {
                    "scene": scene,
                    "ok": False,
                    "reference": ref_tag,
                    "error": f"missing reference images.txt ({ref.relative_to(root)})",
                }
            )
            print(f"[compare] {scene}: skip (missing ref {ref_tag})")
            continue
        if not est.is_file():
            rows.append(
                {
                    "scene": scene,
                    "ok": False,
                    "reference": ref_tag,
                    "error": "missing InsightAT sparse model",
                }
            )
            print(f"[compare] {scene}: skip (missing est)")
            continue

        ref_cams = load_cameras_by_basename(ref)
        est_cams = load_cameras_by_basename(est)
        common = sorted(set(ref_cams.keys()) & set(est_cams.keys()))
        if len(common) < 3:
            rows.append(
                {
                    "scene": scene,
                    "ok": False,
                    "reference": ref_tag,
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
        m["reference"] = ref_tag
        m["n_common"] = len(common)
        rows.append(m)
        print(
            f"[compare] {scene}: ref={ref_tag} matched={len(common)} "
            f"rmse={m['rmse_m']:.4f}m median={m['median_m']:.4f}m"
        )

    out_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
