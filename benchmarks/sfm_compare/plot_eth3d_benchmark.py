#!/usr/bin/env python3
"""
Plot COLMAP vs InsightAT ETH3D-style batch summaries (wall time, sparse points)
and optional GT-vs-InsightAT (or COLMAP-vs-InsightAT) alignment errors.

Reads (under --dataset-root):
  - colmap_batch_summary.json   (from run_colmap_batch.py)
  - insightat_batch_summary.json (from run_insightat_batch.py)
  - compare_gt_insightat.json or compare_colmap_insightat.json (optional; from compare_dataset_batch.py)

Writes PNG figures to --out-dir (default: doc/images/benchmarks).

Requires: matplotlib, numpy
  pip install matplotlib numpy
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:  # pragma: no cover
    print("Error: matplotlib is required. pip install matplotlib", file=sys.stderr)
    raise SystemExit(2) from e


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _by_scene(rows: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {r["scene"]: r for r in rows if isinstance(r, dict) and "scene" in r}


def _collect_timing_pairs(
    root: Path,
) -> Tuple[List[str], List[float], List[float], List[int], List[int]]:
    """Scenes where both COLMAP and InsightAT report exit_code==0."""
    col_path = root / "colmap_batch_summary.json"
    isat_path = root / "insightat_batch_summary.json"
    if not col_path.is_file() or not isat_path.is_file():
        raise FileNotFoundError(f"Need {col_path.name} and {isat_path.name} under {root}")

    col = _by_scene(_load_json(col_path))
    isat = _by_scene(_load_json(isat_path))
    scenes = sorted(set(col) & set(isat))
    names: List[str] = []
    t_col: List[float] = []
    t_isat: List[float] = []
    pts_col: List[int] = []
    pts_isat: List[int] = []
    for s in scenes:
        c, i = col[s], isat[s]
        if c.get("exit_code") != 0 or i.get("exit_code") != 0:
            continue
        names.append(s)
        t_col.append(float(c.get("elapsed_sfm_s", c.get("elapsed_s", 0.0))))
        t_isat.append(float(i.get("elapsed_wall_s", i.get("timing", {}).get("total_s", 0.0))))
        pts_col.append(int(c.get("n_points3d", 0)))
        pts_isat.append(int(i.get("n_points3d", 0)))
    return names, t_col, t_isat, pts_col, pts_isat


def _load_compare_rows(root: Path) -> Tuple[List[Dict[str, Any]], str]:
    """Prefer GT-vs-InsightAT JSON; fall back to legacy COLMAP-vs-InsightAT."""
    for name, ref_label in (
        ("compare_gt_insightat.json", "gt"),
        ("compare_colmap_insightat.json", "colmap"),
    ):
        p = root / name
        if not p.is_file():
            continue
        data = _load_json(p)
        if not isinstance(data, list):
            continue
        rows = [r for r in data if isinstance(r, dict) and r.get("ok")]
        if rows:
            tag = str(rows[0].get("reference", ref_label))
            return rows, tag
    return [], "unknown"


def _bar_grouped(
    scenes: List[str],
    a: List[float],
    b: List[float],
    label_a: str,
    label_b: str,
    ylabel: str,
    title: str,
    out: Path,
) -> None:
    x = np.arange(len(scenes))
    w = 0.38
    fig, ax = plt.subplots(figsize=(max(10.0, len(scenes) * 0.55), 5.2))
    ax.bar(x - w / 2, a, width=w, label=label_a, color="#4C72B0")
    ax.bar(x + w / 2, b, width=w, label=label_b, color="#DD8452")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels(scenes, rotation=40, ha="right")
    ax.legend()
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot ETH3D-style COLMAP vs InsightAT benchmark charts")
    ap.add_argument("-d", "--dataset-root", required=True, type=Path, help="Dataset root (e.g. prepared ETH3D)")
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory for PNG files (default: <repo>/doc/images/benchmarks)",
    )
    args = ap.parse_args()

    root = args.dataset_root.resolve()
    repo = Path(__file__).resolve().parents[2]
    out_dir = (args.out_dir or (repo / "doc" / "images" / "benchmarks")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    names, t_col, t_isat, pts_col, pts_isat = _collect_timing_pairs(root)
    if not names:
        print("No scenes with both COLMAP and InsightAT exit_code==0.", file=sys.stderr)
        return 1

    _bar_grouped(
        names,
        t_col,
        t_isat,
        "COLMAP (sparse SfM only, elapsed_sfm_s)",
        "InsightAT (isat_sfm wall, elapsed_wall_s)",
        "Seconds",
        "ETH3D scenes — wall time (lower is better)",
        out_dir / "eth3d_wall_time_colmap_vs_insightat.png",
    )
    _bar_grouped(
        names,
        [float(v) for v in pts_col],
        [float(v) for v in pts_isat],
        "COLMAP points3D",
        "InsightAT points3D",
        "Count",
        "ETH3D scenes — sparse 3D points (text export)",
        out_dir / "eth3d_sparse_points_colmap_vs_insightat.png",
    )

    compare_rows, ref_tag = _load_compare_rows(root)
    if compare_rows:
        ref_name = "ETH3D GT (dslr_calibration)" if ref_tag == "gt" else "COLMAP sparse"
        scenes_c = [r["scene"] for r in compare_rows]
        rmse = [float(r["rmse_m"]) for r in compare_rows]
        med = [float(r["median_m"]) for r in compare_rows]
        x = np.arange(len(scenes_c))
        w = 0.38
        fig, ax = plt.subplots(figsize=(max(10.0, len(scenes_c) * 0.55), 5.8))
        ax.bar(x - w / 2, rmse, width=w, label="RMSE (all cameras)", color="#55A868")
        ax.bar(x + w / 2, med, width=w, label="Median error", color="#C44E52")
        ax.set_ylabel("Residual (meters)")
        ax.set_title(
            f"Camera centers: {ref_name} = reference, InsightAT = estimate\n"
            "(Umeyama similarity ref→est; green=RMSE, red=median — same alignment per scene)"
        )
        ax.set_xticks(x)
        ax.set_xticklabels(scenes_c, rotation=40, ha="right")
        ax.legend(loc="upper right")
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        fig.tight_layout()
        fig.savefig(out_dir / "eth3d_camera_center_alignment.png", dpi=160)
        plt.close(fig)

    print(f"Wrote figures under {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
