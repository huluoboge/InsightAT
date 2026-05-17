#!/usr/bin/env python3
"""Batch-run isat_seed_eval on a scene tree.

Expected layout:
  <scenes_root>/<scene_name>/results/insightat/work/

Required files inside each work dir:
  tracks.isat_tracks
  images_all.json
  geo/pairs.json
  geo/

Per scene outputs:
  <work>/seed_eval_all/
    report.json
    best_seed.json
    report_plot.png
    strategy_*/...

Batch outputs:
  <scenes_root>/seed_eval_batch_summary.json
  <scenes_root>/seed_eval_batch_summary.tsv

Example:
  python3 scripts/run_seed_eval_batch.py \
    -d /home/jones/Data/01-benchmark/03-insightat/scenes \
    --build-dir /home/jones/Git/01jones/InsightAT/build-ceres-12.8
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]


def discover_work_dirs(scenes_root: Path) -> List[Path]:
    work_dirs: List[Path] = []
    for scene_dir in sorted(p for p in scenes_root.iterdir() if p.is_dir()):
        work_dir = scene_dir / "results" / "insightat" / "work"
        if work_dir.is_dir():
            work_dirs.append(work_dir)
    return work_dirs


def is_valid_work_dir(work_dir: Path) -> bool:
    required = [
        work_dir / "tracks.isat_tracks",
        work_dir / "images_all.json",
        work_dir / "geo",
        work_dir / "geo" / "pairs.json",
    ]
    return all(p.exists() for p in required)


def run_cmd(cmd: List[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, capture_output=True)


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def best_result_from_report(report: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    results = report.get("results", [])
    if not results:
        return None
    return results[0]


def scene_name_from_work_dir(work_dir: Path, scenes_root: Path) -> str:
    try:
        return str(work_dir.relative_to(scenes_root).parts[0])
    except Exception:
        return work_dir.parent.parent.parent.name


def write_batch_summary(scenes_root: Path, rows: List[Dict[str, Any]]) -> None:
    summary_json = scenes_root / "seed_eval_batch_summary.json"
    summary_tsv = scenes_root / "seed_eval_batch_summary.tsv"

    with summary_json.open("w", encoding="utf-8") as f:
        json.dump({"rows": rows}, f, indent=2)
        f.write("\n")

    with summary_tsv.open("w", encoding="utf-8") as f:
        f.write(
            "scene\tstatus\tbest_strategy\tregistered_images\ttriangulated_points\t"
            "points_per_image\truntime_sec\tscore\treport\tplot\n"
        )
        for row in rows:
            f.write(
                f"{row.get('scene', '')}\t{row.get('status', '')}\t"
                f"{row.get('best_strategy', '')}\t{row.get('registered_images', '')}\t"
                f"{row.get('triangulated_points', '')}\t{row.get('points_per_image', '')}\t"
                f"{row.get('runtime_sec', '')}\t{row.get('score', '')}\t"
                f"{row.get('report', '')}\t{row.get('plot', '')}\n"
            )


def main() -> int:
    ap = argparse.ArgumentParser(description="Batch-run isat_seed_eval over a scenes directory")
    ap.add_argument("-d", "--dataset-root", required=True, help="Scenes root directory")
    ap.add_argument(
        "--build-dir",
        default=str(REPO_ROOT / "build-ceres-12.8"),
        help="Build directory containing isat_seed_eval and isat_incremental_sfm",
    )
    ap.add_argument(
        "--output-name",
        default="seed_eval_all",
        help="Per-scene output directory name inside each work dir",
    )
    ap.add_argument(
        "--max-eval-images",
        type=int,
        default=6,
        help="Early-stop registration cap for each strategy",
    )
    ap.add_argument(
        "--scene-filter",
        default="",
        help="Only run scenes whose names contain this substring",
    )
    ap.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip scenes whose report.json already exists",
    )
    ap.add_argument(
        "--quiet",
        action="store_true",
        help="Forward -q to isat_seed_eval",
    )
    args = ap.parse_args()

    scenes_root = Path(args.dataset_root).resolve()
    build_dir = Path(args.build_dir).resolve()
    seed_eval_bin = build_dir / "isat_seed_eval"
    incremental_sfm_bin = build_dir / "isat_incremental_sfm"
    visualize_script = REPO_ROOT / "scripts" / "visualize_seed_eval.py"

    if not scenes_root.is_dir():
        print(f"dataset root does not exist: {scenes_root}", file=sys.stderr)
        return 2
    if not seed_eval_bin.is_file():
        print(f"isat_seed_eval not found: {seed_eval_bin}", file=sys.stderr)
        return 2
    if not incremental_sfm_bin.is_file():
        print(f"isat_incremental_sfm not found: {incremental_sfm_bin}", file=sys.stderr)
        return 2
    if not visualize_script.is_file():
        print(f"visualize_seed_eval.py not found: {visualize_script}", file=sys.stderr)
        return 2

    work_dirs = discover_work_dirs(scenes_root)
    if args.scene_filter:
        work_dirs = [
            w for w in work_dirs if args.scene_filter.lower() in scene_name_from_work_dir(w, scenes_root).lower()
        ]

    rows: List[Dict[str, Any]] = []
    print(f"found {len(work_dirs)} work dirs under {scenes_root}")

    for work_dir in work_dirs:
        scene = scene_name_from_work_dir(work_dir, scenes_root)
        out_dir = work_dir / args.output_name
        report_path = out_dir / "report.json"
        plot_path = out_dir / "report_plot.png"

        if not is_valid_work_dir(work_dir):
            print(f"[{scene}] skip: missing required files in {work_dir}")
            rows.append({"scene": scene, "status": "missing_inputs", "report": str(report_path), "plot": str(plot_path)})
            continue

        if args.skip_existing and report_path.is_file():
            print(f"[{scene}] skip: existing report")
            report = load_json(report_path)
            best = best_result_from_report(report)
            row = {
                "scene": scene,
                "status": "existing",
                "report": str(report_path),
                "plot": str(plot_path),
            }
            if best is not None:
                metrics = best.get("metrics", {})
                strategy = best.get("strategy", {})
                row.update(
                    {
                        "best_strategy": strategy.get("name", ""),
                        "registered_images": metrics.get("registered_images", ""),
                        "triangulated_points": metrics.get("triangulated_points", ""),
                        "points_per_image": metrics.get("points_per_image", ""),
                        "runtime_sec": metrics.get("runtime_sec", ""),
                        "score": metrics.get("score", ""),
                    }
                )
            rows.append(row)
            continue

        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"[{scene}] running seed_eval -> {out_dir}")
        t0 = time.perf_counter()
        cmd = [
            str(seed_eval_bin),
            "-t",
            str(work_dir / "tracks.isat_tracks"),
            "-p",
            str(work_dir / "images_all.json"),
            "-m",
            str(work_dir / "geo" / "pairs.json"),
            "-g",
            str(work_dir / "geo"),
            "-o",
            str(out_dir),
            "--max-eval-images",
            str(args.max_eval_images),
            "--incremental-sfm-bin",
            str(incremental_sfm_bin),
        ]
        if args.quiet:
            cmd.append("-q")

        proc = run_cmd(cmd)
        elapsed = time.perf_counter() - t0

        if proc.returncode != 0:
            print(f"[{scene}] seed_eval failed rc={proc.returncode}")
            stderr_path = out_dir / "batch_seed_eval.stderr.txt"
            stderr_path.write_text(proc.stdout + "\n" + proc.stderr, encoding="utf-8")
            rows.append(
                {
                    "scene": scene,
                    "status": "seed_eval_failed",
                    "report": str(report_path),
                    "plot": str(plot_path),
                    "runtime_sec": round(elapsed, 3),
                    "stderr": str(stderr_path),
                }
            )
            continue

        if not report_path.is_file():
            print(f"[{scene}] seed_eval finished but report.json is missing")
            rows.append(
                {
                    "scene": scene,
                    "status": "missing_report",
                    "report": str(report_path),
                    "plot": str(plot_path),
                    "runtime_sec": round(elapsed, 3),
                }
            )
            continue

        viz_cmd = [sys.executable, str(visualize_script), "-i", str(report_path), "-o", str(plot_path)]
        viz = run_cmd(viz_cmd)
        if viz.returncode != 0:
            print(f"[{scene}] visualization failed rc={viz.returncode}")

        report = load_json(report_path)
        best = best_result_from_report(report)
        row: Dict[str, Any] = {
            "scene": scene,
            "status": "ok",
            "report": str(report_path),
            "plot": str(plot_path),
            "runtime_sec": round(elapsed, 3),
        }
        if best is not None:
            metrics = best.get("metrics", {})
            strategy = best.get("strategy", {})
            row.update(
                {
                    "best_strategy": strategy.get("name", ""),
                    "registered_images": metrics.get("registered_images", ""),
                    "triangulated_points": metrics.get("triangulated_points", ""),
                    "points_per_image": metrics.get("points_per_image", ""),
                    "score": metrics.get("score", ""),
                }
            )
            print(
                f"[{scene}] best={row['best_strategy']} reg={row['registered_images']} "
                f"pts={row['triangulated_points']} score={row['score']}"
            )
        rows.append(row)

    write_batch_summary(scenes_root, rows)
    print(f"wrote batch summary: {scenes_root / 'seed_eval_batch_summary.json'}")
    print(f"wrote batch summary: {scenes_root / 'seed_eval_batch_summary.tsv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())