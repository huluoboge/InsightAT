#!/usr/bin/env python3
"""
Plot ETH3D triple comparison: COLMAP (SfM time) vs InsightAT v0.1 vs v0.2 (wall).

Data snapshot is embedded for release notes; regenerate PNG after re-running batches.

  python3 benchmarks/sfm_compare/plot_eth3d_release_triple.py \\
    -o doc/images/benchmarks/eth3d_colmap_vs_insightat_0.1_vs_0.2.png

Requires: matplotlib, numpy
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:  # pragma: no cover
    print("Error: matplotlib required. pip install matplotlib", file=sys.stderr)
    raise SystemExit(2) from e

# ETH3D training subset, 13 scenes — same order as run_*_batch alphabetical scene list.
SCENES = [
    "courtyard",
    "delivery_area",
    "electro",
    "facade",
    "kicker",
    "meadow",
    "office",
    "pipes",
    "playground",
    "relief",
    "relief_2",
    "terrace",
    "terrains",
]

# COLMAP: elapsed_sfm_s (reconstruction only; txt export excluded)
COLMAP_SFM_S = [
    117.1,
    129.1,
    104.0,
    331.7,
    76.5,
    24.8,
    42.2,
    24.0,
    85.6,
    89.9,
    87.8,
    49.5,
    102.5,
]

# InsightAT v0.1.0: CUDA extract/match, wall clock from batch log
ISAT_010_WALL_S = [
    60.6,
    72.7,
    70.6,
    249.6,
    32.0,
    7.9,
    22.5,
    10.6,
    51.0,
    55.2,
    58.1,
    25.4,
    45.1,
]

# InsightAT v0.2.0: PopSift + default gpu_cascade_hash + Ceres CUDA_SPARSE build
ISAT_020_WALL_S = [
    44.3,
    57.0,
    40.7,
    129.7,
    27.1,
    9.8,
    23.4,
    8.3,
    37.9,
    35.6,
    40.7,
    21.3,
    47.3,
]

COLMAP_PTS = [
    22125,
    24251,
    14812,
    52577,
    10794,
    638,
    6140,
    2290,
    8050,
    19234,
    18367,
    7736,
    12174,
]

ISAT_010_PTS = [
    36422,
    45681,
    33654,
    77308,
    16254,
    2159,
    13775,
    7401,
    21538,
    33883,
    34176,
    15923,
    29568,
]

ISAT_020_PTS = [
    30375,
    50007,
    38960,
    83770,
    14002,
    2309,
    9626,
    6384,
    25672,
    38180,
    40066,
    22584,
    41335,
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("doc/images/benchmarks/eth3d_colmap_vs_insightat_0.1_vs_0.2.png"),
    )
    args = ap.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    x = np.arange(len(SCENES))
    w = 0.25

    fig, (ax_t, ax_p) = plt.subplots(
        2, 1, figsize=(14, 10), gridspec_kw={"height_ratios": [1.15, 1.0]}
    )

    ax_t.bar(x - w, COLMAP_SFM_S, width=w, label="COLMAP (SfM time)", color="#4e79a7")
    ax_t.bar(x, ISAT_010_WALL_S, width=w, label="InsightAT v0.1 (wall)", color="#f28e2b")
    ax_t.bar(x + w, ISAT_020_WALL_S, width=w, label="InsightAT v0.2 (wall)", color="#59a14f")

    ax_t.set_ylabel("Seconds")
    ax_t.set_title("ETH3D training subset — reconstruction time (13 scenes, all exit 0)")
    ax_t.set_xticks(x)
    ax_t.set_xticklabels(SCENES, rotation=45, ha="right")
    ax_t.legend(loc="upper left", fontsize=9)
    ax_t.grid(axis="y", alpha=0.3)

    sum_col = float(np.sum(COLMAP_SFM_S))
    sum_01 = float(np.sum(ISAT_010_WALL_S))
    sum_02 = float(np.sum(ISAT_020_WALL_S))
    ax_t.text(
        0.99,
        0.97,
        f"Σ COLMAP SfM: {sum_col:.1f}s\nΣ ISAT v0.1 wall: {sum_01:.1f}s\nΣ ISAT v0.2 wall: {sum_02:.1f}s\n"
        f"v0.2 vs v0.1 wall: {100.0 * sum_02 / sum_01:.1f}%\n"
        f"v0.2 wall vs COLMAP SfM: {100.0 * sum_02 / sum_col:.1f}%",
        transform=ax_t.transAxes,
        fontsize=9,
        verticalalignment="top",
        horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.35),
    )

    ax_p.bar(x - w, COLMAP_PTS, width=w, label="COLMAP n_points3d", color="#4e79a7")
    ax_p.bar(x, ISAT_010_PTS, width=w, label="InsightAT v0.1", color="#f28e2b")
    ax_p.bar(x + w, ISAT_020_PTS, width=w, label="InsightAT v0.2", color="#59a14f")
    ax_p.set_ylabel("Sparse 3D points (txt export stats)")
    ax_p.set_title("Sparse point counts (methodology differs; not directly comparable quality metric)")
    ax_p.set_xticks(x)
    ax_p.set_xticklabels(SCENES, rotation=45, ha="right")
    ax_p.legend(loc="upper left", fontsize=9)
    ax_p.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.output, dpi=160)
    print(f"Wrote {args.output.resolve()}")


if __name__ == "__main__":
    main()
