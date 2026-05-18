#!/usr/bin/env python3
"""Visualize isat_seed_eval report.

Usage:
  python3 scripts/visualize_seed_eval.py -i /path/to/seed_eval_out/report.json
"""

import argparse
import json
from pathlib import Path


def load_report(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def print_table(report):
    results = report.get("results", [])
    if not results:
        print("No results in report")
        return
    print("rank\tstrategy\treg\tpoints\tpts/img\truntime(s)\tscore")
    for r in results:
        m = r.get("metrics", {})
        s = r.get("strategy", {}).get("name", "?")
        print(
            f"{r.get('rank', '?')}\t{s}\t{m.get('registered_images', 0)}\t"
            f"{m.get('triangulated_points', 0)}\t{m.get('points_per_image', 0):.2f}\t"
            f"{m.get('runtime_sec', 0):.2f}\t{m.get('score', 0):.2f}"
        )


def try_plot(report, out_png: Path):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("matplotlib not available; skip plotting")
        return False

    results = report.get("results", [])
    if not results:
        return False

    names = [r.get("strategy", {}).get("name", "?") for r in results]
    scores = [r.get("metrics", {}).get("score", 0.0) for r in results]
    regs = [r.get("metrics", {}).get("registered_images", 0) for r in results]
    pts = [r.get("metrics", {}).get("triangulated_points", 0) for r in results]

    fig, axs = plt.subplots(1, 3, figsize=(12, 4))

    axs[0].bar(names, scores)
    axs[0].set_title("Score")
    axs[0].tick_params(axis="x", rotation=25)

    axs[1].bar(names, regs)
    axs[1].set_title("Registered Images")
    axs[1].tick_params(axis="x", rotation=25)

    axs[2].bar(names, pts)
    axs[2].set_title("Triangulated Points")
    axs[2].tick_params(axis="x", rotation=25)

    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    print(f"wrote plot: {out_png}")
    return True


def main():
    ap = argparse.ArgumentParser(description="Visualize isat_seed_eval report")
    ap.add_argument("-i", "--input", required=True, help="Path to report.json")
    ap.add_argument("-o", "--output", default="", help="Output PNG path (optional)")
    args = ap.parse_args()

    report_path = Path(args.input)
    report = load_report(report_path)
    print_table(report)

    out_png = Path(args.output) if args.output else report_path.parent / "report_plot.png"
    try_plot(report, out_png)


if __name__ == "__main__":
    main()
