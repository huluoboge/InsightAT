#!/usr/bin/env python3
"""
Batch-run COLMAP **sparse SfM only** (no dense / MVS).

Pipeline per scene:
  feature_extractor → exhaustive_matcher → mapper → model_converter (BIN→TXT, if needed)

COLMAP 3.4+ / 4.x writes **binary** sparse models (`cameras.bin`, `images.bin`, …) by default.
This script runs `model_converter --output_type TXT` so `sparse/0/` contains `.txt` files for
Python parsers and comparison with InsightAT exports.

This intentionally avoids `automatic_reconstructor`, which also runs undistortion
and patch-match stereo (dense reconstruction).

Expected layout (e.g. after benchmarks/eth3d/prepare_datasets.py):
  <dataset>/scenes/<scene_name>/images/

Writes per scene:
  <scene>/results/colmap/workspace/
    database.db
    sparse/0/   — cameras.txt, images.txt, points3D.txt (after conversion)
  <scene>/results/colmap/run.json  — timing (see below), per-step exit codes, sparse stats

Timing fields:
  - `elapsed_s` — total wall-clock seconds (SfM steps + BIN→TXT when enabled)
  - `elapsed_sfm_s` — sum of feature_extractor + exhaustive_matcher + mapper only
  - `elapsed_text_export_s` — model_converter only (0 if skipped or not run)

By default each scene **deletes and recreates** `workspace/` so reruns never reuse a
failed or stale COLMAP state. Use `--reuse-workspace` to skip deletion (expert only).

Environment:
  COLMAP_BIN_DIR — directory containing colmap executable (default: PATH only).

Example:
  COLMAP_BIN_DIR=/path/to/colmap/install/bin \\
    python3 benchmarks/sfm_compare/run_colmap_batch.py -d /data/eth3d_prepared
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

# repo root = parent of benchmarks/
_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from benchmarks.sfm_compare.colmap_sparse import count_points3d, load_cameras_by_basename  # noqa: E402


def _colmap_exe(bin_dir: Optional[str]) -> str:
    if bin_dir:
        p = Path(bin_dir) / "colmap"
        if p.is_file():
            return str(p)
    return "colmap"


def _sparse_model_dir(workspace: Path) -> Path:
    return workspace / "sparse" / "0"


def _needs_bin_to_txt(sparse_0: Path) -> bool:
    """True if we have a binary model but not yet text images.txt."""
    if not sparse_0.is_dir():
        return False
    if (sparse_0 / "images.txt").is_file():
        return False
    return (sparse_0 / "images.bin").is_file()


def _convert_sparse_bin_to_txt(
    colmap: str,
    workspace: Path,
    env: Dict[str, str],
    logf,
) -> tuple[int, Optional[Dict[str, Any]]]:
    """
    In-place: replace sparse/0 binary model with TXT via model_converter.
    COLMAP 4 requires --output_path to exist as an empty directory.
    """
    s0 = _sparse_model_dir(workspace)
    if not _needs_bin_to_txt(s0):
        return 0, None

    parent = s0.parent
    tmp = parent / "_colmap_txt_out"
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True, exist_ok=True)

    cmd = [
        colmap,
        "model_converter",
        "--input_path",
        str(s0),
        "--output_path",
        str(tmp),
        "--output_type",
        "TXT",
    ]
    logf.write("\n# Convert sparse model BIN -> TXT (for benchmarks / comparison)\n")
    logf.write("$ " + " ".join(cmd) + "\n\n")
    logf.flush()
    t0 = time.perf_counter()
    p = subprocess.run(cmd, stdout=logf, stderr=subprocess.STDOUT, env=env)
    elapsed = time.perf_counter() - t0
    row: Dict[str, Any] = {
        "step": "model_converter",
        "exit_code": p.returncode,
        "elapsed_s": round(elapsed, 3),
        "cmd": cmd,
    }
    if p.returncode != 0:
        return p.returncode, row

    shutil.rmtree(s0)
    tmp.rename(s0)
    return 0, row


def _reset_workspace(workspace: Path, reuse: bool) -> None:
    """
    Fresh COLMAP workspace: remove old database, partial sparse, dense/, etc.
    mapper needs an existing empty `sparse/` directory as --output_path.
    """
    if not reuse and workspace.exists():
        shutil.rmtree(workspace)
    workspace.mkdir(parents=True, exist_ok=True)
    sparse_out = workspace / "sparse"
    sparse_out.mkdir(parents=True, exist_ok=True)


def _run_sparse_sfm_pipeline(
    colmap: str,
    image_path: Path,
    workspace: Path,
    camera_model: str,
    log_file: Path,
    env: Dict[str, str],
    reuse_workspace: bool,
    skip_text_export: bool,
) -> tuple[int, List[Dict[str, Any]], float]:
    """
    Run feature_extractor → exhaustive_matcher → mapper [→ model_converter TXT].
    Returns (exit_code, step_log_rows, total_elapsed_s).
    Stops on first non-zero exit.
    """
    _reset_workspace(workspace, reuse=reuse_workspace)
    db = workspace / "database.db"
    sparse_out = workspace / "sparse"

    steps: List[tuple[str, List[str]]] = [
        (
            "feature_extractor",
            [
                colmap,
                "feature_extractor",
                "--database_path",
                str(db),
                "--image_path",
                str(image_path),
                "--ImageReader.camera_model",
                camera_model,
            ],
        ),
        (
            "exhaustive_matcher",
            [colmap, "exhaustive_matcher", "--database_path", str(db)],
        ),
        (
            "mapper",
            [
                colmap,
                "mapper",
                "--database_path",
                str(db),
                "--image_path",
                str(image_path),
                "--output_path",
                str(sparse_out),
            ],
        ),
    ]

    log_file.parent.mkdir(parents=True, exist_ok=True)
    t_all = time.perf_counter()
    rows: List[Dict[str, Any]] = []
    last_rc = 0

    with open(log_file, "w", encoding="utf-8") as lf:
        for name, cmd in steps:
            lf.write("$ " + " ".join(cmd) + "\n\n")
            lf.flush()
            t0 = time.perf_counter()
            p = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT, env=env)
            elapsed = time.perf_counter() - t0
            rows.append(
                {
                    "step": name,
                    "exit_code": p.returncode,
                    "elapsed_s": round(elapsed, 3),
                    "cmd": cmd,
                }
            )
            if p.returncode != 0:
                last_rc = p.returncode
                break
            last_rc = 0

        if last_rc == 0 and not skip_text_export:
            rc_txt, row_txt = _convert_sparse_bin_to_txt(colmap, workspace, env, lf)
            if row_txt is not None:
                rows.append(row_txt)
            if rc_txt != 0:
                last_rc = rc_txt

    total_elapsed = time.perf_counter() - t_all
    return last_rc, rows, total_elapsed


def _timing_breakdown(step_rows: List[Dict[str, Any]]) -> Dict[str, float]:
    """Split step list into SfM vs model_converter for fair comparison with InsightAT."""
    sfm = 0.0
    export = 0.0
    for r in step_rows:
        sec = float(r.get("elapsed_s") or 0.0)
        if r.get("step") == "model_converter":
            export += sec
        else:
            sfm += sec
    return {
        "elapsed_sfm_s": round(sfm, 3),
        "elapsed_text_export_s": round(export, 3),
    }


def _sparse_stats(workspace: Path) -> Dict[str, Any]:
    sparse = workspace / "sparse" / "0"
    images_txt = sparse / "images.txt"
    pts_txt = sparse / "points3D.txt"
    out: Dict[str, Any] = {"sparse_dir": str(sparse)}
    if not images_txt.is_file():
        out["n_images"] = 0
        out["n_points3d"] = 0
        return out
    out["n_images"] = len(load_cameras_by_basename(images_txt))
    out["n_points3d"] = count_points3d(pts_txt)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Batch COLMAP sparse SfM: feature_extractor + matcher + mapper + BIN→TXT export; no MVS"
        )
    )
    ap.add_argument(
        "-d",
        "--dataset-root",
        required=True,
        help="Root with scenes/<name>/images/ (e.g. ETH3D prepared root)",
    )
    ap.add_argument(
        "--colmap-bin-dir",
        default=os.environ.get("COLMAP_BIN_DIR", ""),
        help="Directory containing `colmap` binary (or set COLMAP_BIN_DIR)",
    )
    ap.add_argument(
        "--camera-model",
        default="OPENCV",
        help="ImageReader.camera_model for feature_extractor (default: OPENCV)",
    )
    ap.add_argument(
        "--summary",
        default="",
        help="Optional path to write batch summary JSON (all scenes)",
    )
    ap.add_argument(
        "--reuse-workspace",
        action="store_true",
        help="Do not delete results/colmap/workspace before each scene (default: delete for a clean rerun)",
    )
    ap.add_argument(
        "--skip-text-export",
        action="store_true",
        help="Keep COLMAP binary sparse model only (no model_converter to TXT; stats in summary will stay 0)",
    )
    args = ap.parse_args()

    root = Path(args.dataset_root).resolve()
    scenes_dir = root / "scenes"
    if not scenes_dir.is_dir():
        print(f"Error: missing {scenes_dir}", file=sys.stderr)
        return 2

    colmap = _colmap_exe(args.colmap_bin_dir or None)
    env = os.environ.copy()
    if args.colmap_bin_dir:
        env["PATH"] = str(Path(args.colmap_bin_dir).resolve()) + os.pathsep + env.get("PATH", "")

    results: List[Dict[str, Any]] = []
    scene_names = sorted(p.name for p in scenes_dir.iterdir() if p.is_dir())
    for name in scene_names:
        img_dir = scenes_dir / name / "images"
        if not img_dir.is_dir():
            continue
        out_base = scenes_dir / name / "results" / "colmap"
        workspace = out_base / "workspace"
        log_path = out_base / "colmap_console.log"
        run_json = out_base / "run.json"

        rc, step_rows, elapsed = _run_sparse_sfm_pipeline(
            colmap,
            img_dir,
            workspace,
            args.camera_model,
            log_path,
            env,
            reuse_workspace=args.reuse_workspace,
            skip_text_export=args.skip_text_export,
        )
        stats = _sparse_stats(workspace)
        bd = _timing_breakdown(step_rows)
        row = {
            "scene": name,
            "exit_code": rc,
            "elapsed_s": round(elapsed, 3),
            **bd,
            "pipeline": "sparse_sfm_only",
            "clean_workspace": not args.reuse_workspace,
            "steps": step_rows,
            "log": str(log_path),
            **stats,
        }
        run_json.write_text(json.dumps(row, indent=2), encoding="utf-8")
        print(
            f"[colmap] {name}: code={rc} total={elapsed:.1f}s sfm={bd['elapsed_sfm_s']:.1f}s "
            f"txt_export={bd['elapsed_text_export_s']:.1f}s "
            f"images={stats.get('n_images', 0)} pts={stats.get('n_points3d', 0)}"
        )
        results.append(row)

    if args.summary:
        Path(args.summary).write_text(json.dumps(results, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
