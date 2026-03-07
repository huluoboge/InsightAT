"""
sfm.py
InsightAT SfM Pipeline – create / extract / match / incremental_sfm.

Usage:
    python -m src.pipeline.pipelines.sfm --input /data/flight --project out.iat --work-dir out_work
    python -m src.pipeline.pipelines.sfm -i /data/flight -p out.iat -w out_work --steps extract,match,incremental_sfm

Steps (controlled by --steps):
  1. create   – Create project, add groups/images, estimate camera (default sensor_db), create AT task
  2. extract  – Export all-group image list, dual feature extraction (retrieval + matching), default CUDA
  3. match    – Train VLAD + PCA, retrieve pairs, match, geo (--twoview, --vis); geo outputs pairs.json + adjacency.json
  4. incremental_sfm – Run isat_incremental_sfm: initial pair + two-view BA; writes initial_poses.json and initial_tracks.isat_tracks to work_dir/incremental_sfm/

Uses geo_dir/pairs.json (geometry-filtered pairs) for incremental_sfm. Image/camera identity is by index in the algorithm; the tool exports original image_ids at the boundary.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path
from typing import Any

from ..runner import (
    ToolError,
    emit_pipeline_event,
    set_cli_log_level,
    set_log_tool_stderr_at_info,
    set_log_tool_stderr_progress_only,
)
from .. import project_utils as pu

log = logging.getLogger(__name__)

DEFAULT_STEPS = ["create", "extract", "match", "incremental_sfm"]


def _scan_subdirs(root: Path, ext: str) -> list[tuple[str, Path]]:
    """Find subdirs that contain at least one image. Returns [(name, path), ...]."""
    exts = {e.strip().lower() for e in ext.split(",") if e.strip()}

    def _has_images(d: Path) -> bool:
        return any(f.suffix.lower() in exts for f in d.iterdir() if f.is_file())

    groups: list[tuple[str, Path]] = []
    for sub in sorted(root.rglob("*")):
        if sub.is_dir() and _has_images(sub):
            rel = sub.relative_to(root)
            groups.append((str(rel).replace(os.sep, "_"), sub))
    if not groups and _has_images(root):
        groups.append((root.name, root))
    return groups


def run_step_create(
    cfg: "SfmConfig",
    groups: list[tuple[str, Path]],
) -> tuple[dict[str, int], int]:
    """
    Create project, add groups/images, estimate camera (default sensor_db), create AT task.
    Returns (group_ids, task_id).
    """
    cfg.work_dir.mkdir(parents=True, exist_ok=True)
    group_ids: dict[str, int] = {}
    skipped: list[str] = []

    if not cfg.dry_run:
        try:
            pu.create_project(cfg.project_path)
            log.info("Created project: %s", cfg.project_path)
        except ToolError as e:
            log.error("Failed to create project: %s", e)
            raise

    emit_pipeline_event({"step": "create_project", "ok": True, "data": {"project": str(cfg.project_path)}})

    for name, path in groups:
        if not cfg.dry_run:
            try:
                evt = pu.add_group(cfg.project_path, name)
                gid = evt.get("data", {}).get("group_id", len(group_ids))
            except ToolError as e:
                log.warning("add-group failed for %s: %s", name, e)
                skipped.append(name)
                continue
        else:
            gid = len(group_ids)

        if not cfg.dry_run:
            try:
                ai_evt = pu.add_images(cfg.project_path, gid, path, ext=cfg.ext)
                added = ai_evt.get("data", {}).get("added", 0)
            except ToolError as e:
                log.warning("add-images failed for %s: %s", name, e)
                skipped.append(name)
                continue
        else:
            added = -1

        if not cfg.dry_run and added < cfg.min_images:
            log.warning("Group %s has %d images (min=%d), skipping", name, added, cfg.min_images)
            skipped.append(name)
            continue

        group_ids[name] = gid
        log.info("Group %d '%s': %d images", gid, name, added)

    if not group_ids:
        raise RuntimeError(f"No groups with >= {cfg.min_images} images (skipped: {skipped})")

    # Estimate camera (sensor_db=None → pipeline uses build/data/config default)
    if not cfg.dry_run:
        try:
            pu.estimate_camera(
                cfg.project_path,
                all_groups=True,
                sensor_db=None,
                max_sample=cfg.max_sample,
            )
            log.info("Camera estimation complete (all groups)")
        except ToolError as e:
            log.warning("Camera estimation failed (continuing): %s", e)
    emit_pipeline_event({"step": "estimate_camera", "ok": True, "data": {}})

    task_id = 0
    if not cfg.dry_run:
        try:
            at_evt = pu.create_at_task(cfg.project_path)
            task_id = at_evt.get("task_id", 0)
            log.info("Created AT task_id=%d", task_id)
        except ToolError as e:
            log.error("Failed to create AT task: %s", e)
            raise
    emit_pipeline_event({"step": "create_at_task", "ok": True, "data": {"task_id": task_id}})

    # Persist task_id for later steps (e.g. --steps extract or --steps match only)
    if not cfg.dry_run:
        (cfg.work_dir / ".sfm_task_id").write_text(str(task_id), encoding="utf-8")

    return group_ids, task_id


def _get_task_id(cfg: "SfmConfig", step_name: str) -> int:
    """Read task_id from work_dir (written by step create). Raises if missing."""
    p = cfg.work_dir / ".sfm_task_id"
    if not p.is_file():
        raise RuntimeError(
            f"Step {step_name} needs task_id; run step create first (or ensure {p} exists)."
        )
    return int(p.read_text(encoding="utf-8").strip())


def run_step_extract(
    cfg: "SfmConfig",
    task_id: int | None = None,
) -> None:
    """Export all-group image list, run dual feature extraction (default CUDA)."""
    if task_id is None:
        task_id = _get_task_id(cfg, "extract")
    images_all = cfg.work_dir / "images_all.json"
    feat_dir = cfg.work_dir / "feat"
    feat_retrieval_dir = cfg.work_dir / "feat_retrieval"

    if not cfg.dry_run:
        feat_dir.mkdir(parents=True, exist_ok=True)
        feat_retrieval_dir.mkdir(parents=True, exist_ok=True)
        pu.export_image_list_all_groups(cfg.project_path, task_id, images_all)
        log.info("Exported image list: %s", images_all)

    emit_pipeline_event({"step": "export_image_list_all", "ok": True, "data": {"output": str(images_all)}})

    if not cfg.dry_run:
        pu.extract_features_dual(
            images_all,
            match_feat_dir=feat_dir,
            retrieval_feat_dir=feat_retrieval_dir,
            extract_backend=cfg.extract_backend,
            extra_args=cfg.extra_extract or None,
        )
    emit_pipeline_event({
        "step": "extract_features",
        "ok": True,
        "data": {"feat_dir": str(feat_dir), "feat_retrieval_dir": str(feat_retrieval_dir)},
    })


def run_step_match(
    cfg: "SfmConfig",
    task_id: int | None = None,
) -> None:
    """Train VLAD + PCA, retrieve, match, geo (--twoview, --vis). Geo writes pairs.json + adjacency.json."""
    if task_id is None:
        task_id = _get_task_id(cfg, "match")
    images_all = cfg.work_dir / "images_all.json"
    feat_dir = cfg.work_dir / "feat"
    feat_retrieval_dir = cfg.work_dir / "feat_retrieval"
    vlad_dir = cfg.work_dir / "vlad"
    match_dir = cfg.work_dir / "match"
    geo_dir = cfg.work_dir / "geo"
    pairs_retrieve = cfg.work_dir / "pairs_retrieve.json"
    codebook_path = vlad_dir / "codebook.vcbt"
    pca_path = vlad_dir / "pca.pca"
    intrinsics_json = cfg.work_dir / "intrinsics.json"

    if not cfg.dry_run:
        vlad_dir.mkdir(parents=True, exist_ok=True)
        match_dir.mkdir(parents=True, exist_ok=True)
        geo_dir.mkdir(parents=True, exist_ok=True)
        vlad_cache = vlad_dir / "cache"
        vlad_cache.mkdir(parents=True, exist_ok=True)

    # Export intrinsics for geo -k -l (multi-camera)
    if not cfg.dry_run:
        pu.export_intrinsics_all_groups(cfg.project_path, task_id, intrinsics_json)
        log.info("Exported intrinsics: %s", intrinsics_json)

    # Train VLAD + PCA
    if not cfg.dry_run:
        pu.train_vlad(
            feat_retrieval_dir,
            codebook_path,
            pca_output=pca_path,
            pca_dims=256,
            whiten=True,
            extra_args=cfg.extra_train_vlad or None,
        )
    emit_pipeline_event({"step": "train_vlad", "ok": True, "data": {"codebook": str(codebook_path), "pca": str(pca_path)}})

    # Retrieve pairs (VLAD + sequential)
    if not cfg.dry_run:
        pu.retrieve_pairs(
            images_all,
            feat_retrieval_dir,
            pairs_retrieve,
            strategy="vlad+sequential",
            window_size=20,
            vlad_codebook=codebook_path,
            vlad_cache_dir=vlad_dir / "cache",
            pca_model_file=pca_path,
            vlad_top_k=20,
            extra_args=cfg.extra_retrieve or None,
        )
    emit_pipeline_event({"step": "retrieve_pairs", "ok": True, "data": {"pairs_json": str(pairs_retrieve)}})

    # Match
    if not cfg.dry_run:
        pu.match_features(
            pairs_retrieve,
            feat_dir,
            match_dir,
            match_backend=cfg.match_backend,
            extra_args=cfg.extra_match or None,
        )
    emit_pipeline_event({"step": "match_features", "ok": True, "data": {"match_dir": str(match_dir)}})

    # Geo (--twoview, --vis); writes geo_dir/pairs.json + geo_dir/adjacency.json
    if not cfg.dry_run:
        pu.geo_verify(
            pairs_retrieve,
            match_dir,
            geo_dir,
            k_json=intrinsics_json,
            image_list=images_all,
            estimate_h=True,
            twoview=True,
            vis=True,
            extra_args=cfg.extra_geo or None,
        )
    emit_pipeline_event({
        "step": "geo_verify",
        "ok": True,
        "data": {
            "geo_dir": str(geo_dir),
            "pairs_json": str(geo_dir / "pairs.json"),
            "adjacency_json": str(geo_dir / "adjacency.json"),
        },
    })


def run_step_incremental_sfm(
    cfg: "SfmConfig",
    task_id: int | None = None,
) -> None:
    """Run isat_incremental_sfm: initial pair + two-view BA; writes poses, tracks, and Bundler (bundle.out, list.txt)."""
    if task_id is None:
        task_id = _get_task_id(cfg, "incremental_sfm")
    images_all = cfg.work_dir / "images_all.json"
    match_dir = cfg.work_dir / "match"
    geo_dir = cfg.work_dir / "geo"
    intrinsics_json = cfg.work_dir / "intrinsics.json"
    pairs_json = geo_dir / "pairs.json"
    sfm_out = cfg.work_dir / "incremental_sfm"
    bundler_out = sfm_out / "bundle.out"
    list_txt = sfm_out / "list.txt"

    if not cfg.dry_run:
        sfm_out.mkdir(parents=True, exist_ok=True)
        events = pu.run_incremental_sfm(
            pairs_json,
            geo_dir,
            match_dir,
            intrinsics_json,
            images_all,
            sfm_out,
            min_tracks=cfg.min_tracks_incremental_sfm,
            optimize_intrinsics=cfg.optimize_intrinsics,
            extra_args=cfg.extra_incremental_sfm or None,
        )
        evt = (events[0] if events else {}).get("data") or {}
        log.info(
            "incremental_sfm output: %s | poses=%s | tracks=%s | Bundler=%s %s",
            sfm_out,
            sfm_out / "initial_poses.json",
            sfm_out / "initial_tracks.isat_tracks",
            bundler_out,
            list_txt,
        )
        # Always print paths so they are visible even with log level > INFO
        print(
            f"SfM output: {sfm_out}\n  Bundler: {bundler_out}\n  list.txt: {list_txt}\n  poses: {sfm_out / 'initial_poses.json'}\n  tracks: {sfm_out / 'initial_tracks.isat_tracks'}",
            file=sys.stderr,
            flush=True,
        )
    else:
        evt = {}

    emit_pipeline_event({
        "step": "incremental_sfm",
        "ok": True,
        "data": {
            "output_dir": str(sfm_out),
            "initial_poses": str(sfm_out / "initial_poses.json"),
            "initial_tracks": str(sfm_out / "initial_tracks.isat_tracks"),
            "bundler_out": str(bundler_out),
            "list_txt": str(list_txt),
            **evt,
        },
    })


def run_pipeline(cfg: "SfmConfig") -> int:
    """Run selected steps. Returns 0 on success, 1 on error."""
    steps_set = set(cfg.steps)
    if not steps_set:
        steps_set = set(DEFAULT_STEPS)

    groups = _scan_subdirs(cfg.input_dir, cfg.ext)
    if not groups:
        log.error("No image-containing subdirectories under %s", cfg.input_dir)
        return 1

    log.info("Discovered %d group(s) under %s", len(groups), cfg.input_dir)
    emit_pipeline_event({
        "step": "scan_input",
        "ok": True,
        "data": {
            "input_dir": str(cfg.input_dir),
            "num_groups": len(groups),
            "groups": [{"name": n, "path": str(p)} for n, p in groups],
        },
    })

    group_ids: dict[str, int] = {}
    task_id = 0

    if "create" in steps_set:
        log.info("=== Step: create ===")
        group_ids, task_id = run_step_create(cfg, groups)

    if "extract" in steps_set:
        log.info("=== Step: extract ===")
        run_step_extract(cfg, task_id if "create" in steps_set else None)

    if "match" in steps_set:
        log.info("=== Step: match ===")
        run_step_match(cfg, task_id if "create" in steps_set else None)

    if "incremental_sfm" in steps_set:
        log.info("=== Step: incremental_sfm ===")
        run_step_incremental_sfm(cfg, task_id if "create" in steps_set else None)

    emit_pipeline_event({
        "step": "pipeline_complete",
        "ok": True,
        "data": {"project": str(cfg.project_path), "work_dir": str(cfg.work_dir)},
    })
    return 0


class SfmConfig:
    def __init__(
        self,
        input_dir: Path,
        project_path: Path,
        work_dir: Path,
        steps: list[str] | None = None,
        ext: str = ".jpg,.tif,.png",
        min_images: int = 5,
        max_sample: int = 5,
        dry_run: bool = False,
        log_level: str = "warn",
        extract_backend: str = "cuda",
        match_backend: str = "cuda",
        extra_extract: list[str] | None = None,
        extra_retrieve: list[str] | None = None,
        extra_train_vlad: list[str] | None = None,
        extra_match: list[str] | None = None,
        extra_geo: list[str] | None = None,
        extra_incremental_sfm: list[str] | None = None,
        min_tracks_incremental_sfm: int = 20,
        optimize_intrinsics: bool = False,
    ) -> None:
        self.input_dir = input_dir
        self.project_path = project_path
        self.work_dir = work_dir
        self.steps = steps or list(DEFAULT_STEPS)
        self.ext = ext
        self.min_images = min_images
        self.max_sample = max_sample
        self.dry_run = dry_run
        self.log_level = log_level
        self.extract_backend = extract_backend
        self.match_backend = match_backend
        self.extra_extract = list(extra_extract) if extra_extract else []
        self.extra_retrieve = list(extra_retrieve) if extra_retrieve else []
        self.extra_train_vlad = list(extra_train_vlad) if extra_train_vlad else []
        self.extra_match = list(extra_match) if extra_match else []
        self.extra_geo = list(extra_geo) if extra_geo else []
        self.extra_incremental_sfm = list(extra_incremental_sfm) if extra_incremental_sfm else []
        self.min_tracks_incremental_sfm = min_tracks_incremental_sfm
        self.optimize_intrinsics = optimize_intrinsics


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="sfm",
        description=(
            "InsightAT SfM pipeline (create / extract / match / incremental_sfm). "
            "Use --steps to run only selected steps."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--input", "-i", required=True, metavar="DIR", help="Root directory to scan for image subdirs.")
    p.add_argument("--project", "-p", required=True, metavar="FILE", help="Output .iat project file.")
    p.add_argument("--work-dir", "-w", required=True, metavar="DIR", help="Working directory for intermediates.")
    p.add_argument(
        "--steps",
        metavar="LIST",
        default="create,extract,match,incremental_sfm",
        help="Comma-separated steps: create, extract, match, incremental_sfm (default: create,extract,match,incremental_sfm).",
    )
    p.add_argument("--ext", default=".jpg,.tif,.png", help="Image extensions (default: .jpg,.tif,.png).")
    p.add_argument("--min-images", type=int, default=5, help="Minimum images per group (default: 5).")
    p.add_argument("--max-sample", type=int, default=5, help="Max images sampled for focal estimation (default: 5).")
    p.add_argument("--extract-backend", choices=("cuda", "glsl"), default="cuda", help="isat_extract backend (default: cuda).")
    p.add_argument("--match-backend", choices=("cuda", "glsl"), default="cuda", help="isat_match backend (default: cuda; use glsl if headless without EGL).")
    p.add_argument("--dry-run", action="store_true", help="Print what would be done without running tools.")
    p.add_argument("--log-level", choices=("error", "warn", "info", "debug"), help="Log level (default: warn).")
    p.add_argument("-v", "--verbose", action="store_true", help="Same as --log-level=info.")
    p.add_argument("-q", "--quiet", action="store_true", help="Same as --log-level=error.")
    p.add_argument("--log-file", metavar="FILE", help="Append logs to file.")
    p.add_argument("--progress", action="store_true", help="Show only progress lines from tools.")
    p.add_argument("--min-tracks-incremental-sfm", type=int, default=20,
                   help="Minimum valid tracks after initial pair (default: 20).")
    p.add_argument("--optimize-intrinsics", action="store_true",
                   help="Optimize intrinsics in incremental SfM global BA (default: fixed).")
    return p


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    # Resolve log level (aligned with 05_cli_io_conventions: --log-level > -q > -v > default warn)
    log_level = args.log_level or ("error" if args.quiet else "info" if args.verbose else "warn")
    level_map = {"error": logging.ERROR, "warn": logging.WARNING, "info": logging.INFO, "debug": logging.DEBUG}
    numeric_level = level_map[log_level]
    log_fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"

    # Pipeline logger: explicit stderr handler so -v/--verbose always shows output (see 05_cli_io_conventions)
    log.setLevel(numeric_level)
    has_stderr = any(getattr(h, "stream", None) is sys.stderr for h in log.handlers)
    if not has_stderr:
        stderr_handler = logging.StreamHandler(sys.stderr)
        stderr_handler.setLevel(numeric_level)
        stderr_handler.setFormatter(logging.Formatter(log_fmt))
        log.addHandler(stderr_handler)
        log.propagate = False  # only our handler, no duplicate from root
    logging.basicConfig(level=numeric_level, format=log_fmt, stream=sys.stderr, force=True)

    if args.log_file:
        fh = logging.FileHandler(args.log_file, encoding="utf-8")
        fh.setFormatter(logging.Formatter(log_fmt))
        log.addHandler(fh)
    set_cli_log_level(log_level)
    set_log_tool_stderr_at_info(log_level in ("info", "debug") or args.progress or bool(args.log_file))
    set_log_tool_stderr_progress_only(args.progress and log_level not in ("info", "debug"))

    steps = [s.strip() for s in args.steps.split(",") if s.strip()]
    for s in steps:
        if s not in ("create", "extract", "match", "incremental_sfm"):
            log.error("Unknown step: %s (allowed: create, extract, match, incremental_sfm)", s)
            return 2

    input_dir = Path(args.input).resolve()
    if not input_dir.is_dir():
        log.error("--input directory does not exist: %s", input_dir)
        return 2

    cfg = SfmConfig(
        input_dir=input_dir,
        project_path=Path(args.project).resolve(),
        work_dir=Path(args.work_dir).resolve(),
        steps=steps,
        ext=args.ext,
        min_images=args.min_images,
        max_sample=args.max_sample,
        dry_run=args.dry_run,
        log_level=log_level,
        extract_backend=args.extract_backend,
        match_backend=args.match_backend,
        min_tracks_incremental_sfm=args.min_tracks_incremental_sfm,
        optimize_intrinsics=args.optimize_intrinsics,
    )

    try:
        return run_pipeline(cfg)
    except (ToolError, RuntimeError) as e:
        log.exception("%s", e)
        return 1


if __name__ == "__main__":
    sys.exit(main())
