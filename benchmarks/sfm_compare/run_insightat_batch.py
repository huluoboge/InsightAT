#!/usr/bin/env python3
"""
Batch-run InsightAT `isat_sfm` on the same dataset layout as run_colmap_batch.py.

Expected:
  <dataset>/scenes/<scene_name>/images/

Writes per scene:
  <scene>/results/insightat/work/  — project.iat, images_all.json, incremental_sfm/, …

Per-scene ``run.json`` stats: ``n_images_input`` from ``work/images_all.json`` (``images``
array length, same as C++ tools); fallback recursive scan of ``images/`` if missing.
``n_images_registered`` from ``work/incremental_sfm/list.txt`` (Bundler) line count only.

By default each scene **deletes and recreates** `work/` so reruns are full pipelines.
Use `--reuse-work` to keep an existing work directory (expert only).

Use `--skip-done` to resume a batch: scenes that already have `results/insightat/run.json`
with `exit_code == 0` are skipped (work dir untouched); others run as usual (still cleans
`work/` unless `--reuse-work`).

Environment:
  ISAT_BIN_DIR — directory containing isat_sfm (default: <repo>/build if present).

Example:
  ISAT_BIN_DIR=/path/to/InsightAT/build \\
    python3 benchmarks/sfm_compare/run_insightat_batch.py -d /data/eth3d_prepared

Resume (skip scenes that finished successfully last time):
  python3 benchmarks/sfm_compare/run_insightat_batch.py -d /data/eth3d_prepared --skip-done

SiftGPU path (requires INSIGHTAT_ENABLE_SIFTGPU build):
  python3 benchmarks/sfm_compare/run_insightat_batch.py -d /data/eth3d_prepared --use-sift-gpu
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from benchmarks.sfm_compare.colmap_sparse import (  # noqa: E402
    count_bundler_list_paths,
    count_images_in_images_all_json,
    count_images_in_tree,
    count_points3d,
)


def _scene_insightat_run_json(scenes_dir: Path, scene_name: str) -> Path:
    return scenes_dir / scene_name / "results" / "insightat" / "run.json"


def _load_successful_run_row(run_json: Path) -> Optional[Dict[str, Any]]:
    """If run.json records a successful isat_sfm run, return that object; else None."""
    if not run_json.is_file():
        return None
    try:
        data = json.loads(run_json.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict):
        return None
    if data.get("exit_code") != 0:
        return None
    return data


def _isat_sfm_exe() -> str:
    env_dir = os.environ.get("ISAT_BIN_DIR", "")
    if env_dir:
        p = Path(env_dir) / "isat_sfm"
        if p.is_file():
            return str(p)
    cand = _REPO / "build" / "isat_sfm"
    if cand.is_file():
        return str(cand)
    return "isat_sfm"


def main() -> int:
    ap = argparse.ArgumentParser(description="Batch isat_sfm")
    ap.add_argument("-d", "--dataset-root", required=True, help="Root with scenes/<name>/images/")
    ap.add_argument(
        "--insightat-bin-dir",
        default=os.environ.get("ISAT_BIN_DIR", ""),
        help="Directory containing isat_sfm (or set ISAT_BIN_DIR)",
    )
    ap.add_argument("--extract-backend", default="cuda", choices=("cuda", "glsl"))
    ap.add_argument("--match-backend", default="cuda", choices=("cuda", "glsl"))
    ap.add_argument("--fix-intrinsics", action="store_true")
    sift = ap.add_mutually_exclusive_group()
    sift.add_argument(
        "--use-sift-gpu",
        action="store_true",
        help="Forward --use-sift-gpu to isat_sfm (SiftGPU feature path; requires a build with SiftGPU enabled).",
    )
    sift.add_argument(
        "--use-pop-sift",
        action="store_true",
        help="Forward --use-pop-sift to isat_sfm (PopSift; default isat_sfm backend when neither flag is set).",
    )
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument(
        "--summary",
        default="",
        help="Optional path to write batch summary JSON",
    )
    ap.add_argument(
        "--reuse-work",
        action="store_true",
        help="Do not delete results/insightat/work before each scene (default: delete for a clean rerun)",
    )
    ap.add_argument(
        "--skip-done",
        action="store_true",
        help="Skip scenes whose results/insightat/run.json has exit_code==0 (resume batch)",
    )
    args = ap.parse_args()

    root = Path(args.dataset_root).resolve()
    scenes_dir = root / "scenes"
    if not scenes_dir.is_dir():
        print(f"Error: missing {scenes_dir}", file=sys.stderr)
        return 2

    isat = _isat_sfm_exe()
    if args.insightat_bin_dir:
        p = Path(args.insightat_bin_dir) / "isat_sfm"
        if p.is_file():
            isat = str(p)

    env = os.environ.copy()
    if args.insightat_bin_dir:
        env["ISAT_BIN_DIR"] = str(Path(args.insightat_bin_dir).resolve())
        env["PATH"] = str(Path(args.insightat_bin_dir).resolve()) + os.pathsep + env.get("PATH", "")

    results: List[Dict[str, Any]] = []
    scene_names = sorted(p.name for p in scenes_dir.iterdir() if p.is_dir())
    for name in scene_names:
        img_dir = scenes_dir / name / "images"
        if not img_dir.is_dir():
            continue
        run_json = _scene_insightat_run_json(scenes_dir, name)
        if args.skip_done:
            prev = _load_successful_run_row(run_json)
            if prev is not None:
                row = dict(prev)
                row["skipped"] = True
                results.append(row)
                print(f"[isat_sfm] {name}: skip (already done, exit_code=0)")
                continue

        work = scenes_dir / name / "results" / "insightat" / "work"
        if not args.reuse_work and work.exists():
            shutil.rmtree(work)
        work.mkdir(parents=True, exist_ok=True)
        log_path = scenes_dir / name / "results" / "insightat" / "isat_sfm_console.log"

        cmd = [
            isat,
            "-i",
            str(img_dir),
            "-w",
            str(work),
            "--extract-backend",
            args.extract_backend,
            "--match-backend",
            args.match_backend,
        ]
        if args.fix_intrinsics:
            cmd.append("--fix-intrinsics")
        if args.use_sift_gpu:
            cmd.append("--use-sift-gpu")
        elif args.use_pop_sift:
            cmd.append("--use-pop-sift")
        if args.verbose:
            cmd.append("-v")

        log_path.parent.mkdir(parents=True, exist_ok=True)
        t0 = time.perf_counter()
        with open(log_path, "w", encoding="utf-8") as lf:
            lf.write("$ " + " ".join(cmd) + "\n\n")
            lf.flush()
            p = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT, env=env)
        elapsed = time.perf_counter() - t0
        timing_path = work / "sfm_timing.json"
        timing: Optional[Dict[str, Any]] = None
        if timing_path.is_file():
            try:
                timing = json.loads(timing_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                timing = None

        images_all_json = work / "images_all.json"
        bundler_list = work / "incremental_sfm" / "list.txt"
        sparse = work / "incremental_sfm" / "colmap" / "sparse" / "0" / "images.txt"
        # Pipeline truth: project image count (matches isat_retrieval_match / C++ helpers).
        n_input = count_images_in_images_all_json(images_all_json)
        if n_input == 0:
            n_input = count_images_in_tree(img_dir)
        # SfM success: Bundler reconstruction list only (not COLMAP basename heuristics).
        n_reg = count_bundler_list_paths(bundler_list) if bundler_list.is_file() else 0
        n_pts = 0
        if sparse.is_file():
            n_pts = count_points3d(sparse.parent / "points3D.txt")

        row = {
            "scene": name,
            "exit_code": p.returncode,
            "elapsed_wall_s": round(elapsed, 3),
            "clean_work": not args.reuse_work,
            "skipped": False,
            "cmd": cmd,
            "log": str(log_path),
            "sfm_timing_json": str(timing_path) if timing_path.is_file() else "",
            "timing": timing,
            "n_images_input": n_input,
            "n_images_registered": n_reg,
            "n_points3d": n_pts,
        }
        run_json.write_text(json.dumps(row, indent=2), encoding="utf-8")
        print(
            f"[isat_sfm] {name}: code={p.returncode} wall={elapsed:.1f}s "
            f"n_images_input={n_input} n_images_registered={n_reg} n_points3d={n_pts}"
        )
        results.append(row)

    if args.summary:
        Path(args.summary).write_text(json.dumps(results, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
