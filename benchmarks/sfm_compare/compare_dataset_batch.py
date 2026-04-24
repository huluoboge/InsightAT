#!/usr/bin/env python3
"""
For each scene under <dataset>/scenes/, align camera centers (Umeyama) and report errors.

**Default (--ref-source gt)** — reference is ETH3D ground truth ``gt/images.txt``:

  1) ``compare_gt_colmap.json``  : GT vs COLMAP sparse ``results/colmap/.../sparse/0/images.txt``
  2) ``compare_gt_insightat.json``: GT vs InsightAT ``results/insightat/.../sparse/0/images.txt``

**Legacy (--ref-source colmap)** — reference is COLMAP sparse, estimate is InsightAT only:

  Writes ``compare_colmap_insightat.json`` (or ``-o`` path).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

import numpy as np

from benchmarks.sfm_compare.align_similarity import evaluate_alignment  # noqa: E402
from benchmarks.sfm_compare.colmap_sparse import load_cameras_by_basename  # noqa: E402


def _align_one(
    ref: Path,
    est: Path,
    scene: str,
    reference_tag: str,
    estimate_tag: str,
    root: Path,
) -> Dict[str, Any]:
    if not ref.is_file():
        return {
            "scene": scene,
            "ok": False,
            "reference": reference_tag,
            "estimate": estimate_tag,
            "error": f"missing reference images.txt ({ref.relative_to(root)})",
        }
    if not est.is_file():
        return {
            "scene": scene,
            "ok": False,
            "reference": reference_tag,
            "estimate": estimate_tag,
            "error": f"missing estimate images.txt ({est.relative_to(root)})",
        }

    ref_cams = load_cameras_by_basename(ref)
    est_cams = load_cameras_by_basename(est)
    common = sorted(set(ref_cams.keys()) & set(est_cams.keys()))
    if len(common) < 3:
        return {
            "scene": scene,
            "ok": False,
            "reference": reference_tag,
            "estimate": estimate_tag,
            "error": f"too few common images ({len(common)})",
            "n_common": len(common),
        }

    src_C = np.stack([ref_cams[k].C for k in common], axis=0)
    dst_C = np.stack([est_cams[k].C for k in common], axis=0)
    m = evaluate_alignment(src_C, dst_C)
    m["scene"] = scene
    m["ok"] = True
    m["reference"] = reference_tag
    m["estimate"] = estimate_tag
    m["n_common"] = len(common)
    return m


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Batch compare GT vs COLMAP / InsightAT (or COLMAP ref vs InsightAT legacy)"
    )
    ap.add_argument("-d", "--dataset-root", required=True, type=Path)
    ap.add_argument(
        "--ref-source",
        choices=("gt", "colmap"),
        default="gt",
        help="gt: write compare_gt_colmap.json + compare_gt_insightat.json. "
        "colmap: COLMAP sparse as ref vs InsightAT only (compare_colmap_insightat.json).",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Only for --ref-source colmap: output JSON (default: <dataset>/compare_colmap_insightat.json). "
        "Ignored when --ref-source gt (two fixed filenames are written).",
    )
    args = ap.parse_args()

    root = args.dataset_root.resolve()
    scenes_dir = root / "scenes"
    if not scenes_dir.is_dir():
        print(f"Error: {scenes_dir} not found", file=sys.stderr)
        return 2

    if args.ref_source == "colmap":
        if args.output is not None:
            out_path = args.output
        else:
            out_path = root / "compare_colmap_insightat.json"
        rows: List[Dict[str, Any]] = []
        for scene in sorted(p.name for p in scenes_dir.iterdir() if p.is_dir()):
            base = scenes_dir / scene
            ref = base / "results" / "colmap" / "workspace" / "sparse" / "0" / "images.txt"
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
            row = _align_one(ref, est, scene, "colmap", "insightat", root)
            rows.append(row)
            if row.get("ok"):
                print(
                    f"[compare] {scene}: ref=colmap est=insightat matched={row['n_common']} "
                    f"rmse={row['rmse_m']:.4f}m median={row['median_m']:.4f}m"
                )
            else:
                print(f"[compare] {scene}: skip ({row.get('error', 'unknown')})")
        out_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
        print(f"Wrote {out_path}")
        return 0

    # --ref-source gt : two products
    if args.output is not None:
        print(
            "Note: -o/--output is ignored for --ref-source gt; "
            "writing compare_gt_colmap.json and compare_gt_insightat.json",
            file=sys.stderr,
        )

    rows_colmap: List[Dict[str, Any]] = []
    rows_isat: List[Dict[str, Any]] = []

    for scene in sorted(p.name for p in scenes_dir.iterdir() if p.is_dir()):
        base = scenes_dir / scene
        gt = base / "gt" / "images.txt"
        colmap_est = base / "results" / "colmap" / "workspace" / "sparse" / "0" / "images.txt"
        isat_est = (
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

        rc = _align_one(gt, colmap_est, scene, "gt", "colmap", root)
        rows_colmap.append(rc)
        ri = _align_one(gt, isat_est, scene, "gt", "insightat", root)
        rows_isat.append(ri)

        if rc.get("ok"):
            print(
                f"[compare] {scene}: ref=gt est=colmap matched={rc['n_common']} "
                f"rmse={rc['rmse_m']:.4f}m median={rc['median_m']:.4f}m"
            )
        else:
            print(f"[compare] {scene}: gt vs colmap skip ({rc.get('error', 'unknown')})")

        if ri.get("ok"):
            print(
                f"[compare] {scene}: ref=gt est=insightat matched={ri['n_common']} "
                f"rmse={ri['rmse_m']:.4f}m median={ri['median_m']:.4f}m"
            )
        else:
            print(f"[compare] {scene}: gt vs insightat skip ({ri.get('error', 'unknown')})")

    p_col = root / "compare_gt_colmap.json"
    p_isat = root / "compare_gt_insightat.json"
    p_col.write_text(json.dumps(rows_colmap, indent=2), encoding="utf-8")
    p_isat.write_text(json.dumps(rows_isat, indent=2), encoding="utf-8")
    print(f"Wrote {p_col}")
    print(f"Wrote {p_isat}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
