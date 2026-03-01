"""
auto_calibrate.py
InsightAT Pipeline – 全自动内参标定流水线

用法（命令行）
--------------
    python -m src.pipeline.pipelines.auto_calibrate \\
        --input /data/flight01 \\
        --project /out/flight01.iat \\
        --work-dir /out/flight01_work \\
        --ext ".jpg,.tif" \\
        --min-images 5 \\
        --max-sample 5

流水线步骤（每个分组独立执行）
--------------------------------
  1. isat_project extract       → image list JSON
  2. isat_extract (dual)        → retrieval + matching .isat_feat
  3. isat_train_vlad            → VLAD codebook + PCA (on retrieval features)
  4. isat_retrieve              → pairs by VLAD + sequential (sequence neighbor 20)
  5. isat_match                 → pair matches (.isat_match) on matching features
  6. isat_project intrinsics    → initial K.json
  7. isat_geo --estimate-h -k   → geometric verification (.isat_geo)
  8. isat_twoview -k            → two-view reconstruction (.isat_twoview)
  9. isat_calibrate             → refined K_refined.json (估计焦距)
 10. isat_project set-camera --from-k  → write refined intrinsics back to .iat

输出
----
  stdout  : PIPELINE_EVENT <json>   (机器可读进度事件)
  stderr  : Python logging           (人类可读日志)
  exit 0  : 成功
  exit 1  : 运行时错误
  exit 2  : 参数错误
"""

from __future__ import annotations

import argparse
import json
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


# ─────────────────────────────────────────────────────────────────────────────
# Pipeline config dataclass (plain dict-style for simplicity)
# ─────────────────────────────────────────────────────────────────────────────

class PipelineConfig:
    def __init__(
        self,
        input_dir: Path,
        project_path: Path,
        work_dir: Path,
        ext: str = ".jpg",
        min_images: int = 5,
        max_sample: int = 5,
        sensor_db: Path | None = None,
        dry_run: bool = False,
        log_level: str = "warn",
        extra_extract: list[str] | None = None,
        extra_retrieve: list[str] | None = None,
        extra_match: list[str] | None = None,
        extra_geo: list[str] | None = None,
        extra_twoview: list[str] | None = None,
        extra_calibrate: list[str] | None = None,
    ) -> None:
        self.input_dir = input_dir
        self.project_path = project_path
        self.work_dir = work_dir
        self.ext = ext
        self.min_images = min_images
        self.max_sample = max_sample
        self.sensor_db = sensor_db
        self.dry_run = dry_run
        self.log_level = log_level
        self.extra_extract = list(extra_extract) if extra_extract else []
        self.extra_retrieve = list(extra_retrieve) if extra_retrieve else []
        self.extra_match = list(extra_match) if extra_match else []
        self.extra_geo = list(extra_geo) if extra_geo else []
        self.extra_twoview = list(extra_twoview) if extra_twoview else []
        self.extra_calibrate = list(extra_calibrate) if extra_calibrate else []


# ─────────────────────────────────────────────────────────────────────────────
# Directory scanner
# ─────────────────────────────────────────────────────────────────────────────

def _scan_subdirs(root: Path, ext: str) -> list[tuple[str, Path]]:
    """
    Recursively find sub-directories that contain at least one image file.

    Returns list of (name, path) sorted by path.
    If root itself contains images (flat layout), it is returned as a single group.
    """
    exts = {e.strip().lower() for e in ext.split(",") if e.strip()}

    def _has_images(d: Path) -> bool:
        return any(f.suffix.lower() in exts for f in d.iterdir() if f.is_file())

    # Collect leaf-like directories with images
    groups: list[tuple[str, Path]] = []
    for sub in sorted(root.rglob("*")):
        if sub.is_dir() and _has_images(sub):
            rel = sub.relative_to(root)
            groups.append((str(rel).replace(os.sep, "_"), sub))

    # If nothing found in subdirs, try root itself
    if not groups and _has_images(root):
        groups.append((root.name, root))

    return groups


# ─────────────────────────────────────────────────────────────────────────────
# Single-group pipeline
# ─────────────────────────────────────────────────────────────────────────────

def _run_group(
    cfg: PipelineConfig,
    group_id: int,
    group_name: str,
    group_dir: Path,
    task_id: int = 0,
) -> bool:
    """
    Run the full calibration pipeline for one image group.
    Returns True on success, False on recoverable error (group skipped).
    """
    # ── Work directories ──────────────────────────────────────────────────────
    gw = cfg.work_dir / f"group_{group_id:04d}"
    feat_dir = gw / "feat"           # matching features (for isat_match)
    feat_retrieval_dir = gw / "feat_retrieval"  # retrieval features (for VLAD train + retrieve)
    vlad_dir = gw / "vlad"
    match_dir = gw / "match"
    geo_dir = gw / "geo"
    tv_dir = gw / "twoview"

    if not cfg.dry_run:
        for d in [gw, feat_dir, feat_retrieval_dir, vlad_dir, match_dir, geo_dir, tv_dir]:
            d.mkdir(parents=True, exist_ok=True)

    codebook_path = vlad_dir / "codebook.vcbt"
    pca_path = vlad_dir / "pca.pca"
    vlad_cache_dir = vlad_dir / "cache"
    if not cfg.dry_run:
        vlad_cache_dir.mkdir(parents=True, exist_ok=True)

    image_list_json = gw / "images.json"
    pairs_json = gw / "pairs.json"
    k_json = gw / "K_initial.json"
    k_refined_json = gw / "K_refined.json"

    def _emit(step: str, ok: bool, data: dict | None = None) -> None:
        emit_pipeline_event({
            "step": step,
            "group_id": group_id,
            "group_name": group_name,
            "ok": ok,
            "data": data or {},
        })

    # ── Step 1: export image list ─────────────────────────────────────────────
    log.info("[group %d/%s] step 1: export image list", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.export_image_list(cfg.project_path, group_id, image_list_json, task_id=task_id)
            _emit("export_image_list", True, {"output": str(image_list_json)})
        except ToolError as e:
            _emit("export_image_list", False, {"error": str(e)})
            return False
    else:
        _emit("export_image_list", True, {"dry_run": True})

    # ── Step 2: dual feature extraction (retrieval + matching) ───────────────
    log.info("[group %d/%s] step 2: extract features (retrieval + matching)", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.extract_features_dual(
                image_list_json,
                match_feat_dir=feat_dir,
                retrieval_feat_dir=feat_retrieval_dir,
                extra_args=cfg.extra_extract or None,
            )
            _emit("extract_features", True, {"feat_dir": str(feat_dir), "feat_retrieval_dir": str(feat_retrieval_dir)})
        except ToolError as e:
            _emit("extract_features", False, {"error": str(e)})
            return False
    else:
        _emit("extract_features", True, {"dry_run": True})

    # ── Step 3: train VLAD (on retrieval features, with PCA) ───────────────────
    log.info("[group %d/%s] step 3: train VLAD + PCA", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.train_vlad(
                feat_retrieval_dir,
                codebook_path,
                pca_output=pca_path,
                pca_dims=256,
                whiten=True,
            )
            _emit("train_vlad", True, {"codebook": str(codebook_path), "pca": str(pca_path)})
        except ToolError as e:
            _emit("train_vlad", False, {"error": str(e)})
            return False
    else:
        _emit("train_vlad", True, {"dry_run": True})

    # ── Step 4: retrieval (VLAD + sequential, sequence neighbor 20) ───────────
    log.info("[group %d/%s] step 4: retrieve pairs (vlad+sequential)", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.retrieve_pairs(
                image_list_json,
                feat_retrieval_dir,
                pairs_json,
                strategy="vlad+sequential",
                window_size=20,
                vlad_codebook=codebook_path,
                vlad_cache_dir=vlad_cache_dir,
                pca_model_file=pca_path,
                vlad_top_k=20,
                extra_args=cfg.extra_retrieve or None,
            )
            _emit("retrieve_pairs", True, {"pairs": str(pairs_json)})
        except ToolError as e:
            _emit("retrieve_pairs", False, {"error": str(e)})
            return False
    else:
        _emit("retrieve_pairs", True, {"dry_run": True})

    # ── Step 5: feature matching (on matching features) ───────────────────────
    log.info("[group %d/%s] step 5: match features", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.match_features(pairs_json, feat_dir, match_dir, cfg.extra_match or None)
            _emit("match_features", True, {"match_dir": str(match_dir)})
        except ToolError as e:
            _emit("match_features", False, {"error": str(e)})
            return False
    else:
        _emit("match_features", True, {"dry_run": True})

    # ── Step 6: export initial intrinsics ─────────────────────────────────────
    log.info("[group %d/%s] step 6: export initial intrinsics", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.export_intrinsics(cfg.project_path, group_id, k_json, task_id=task_id)
            _emit("export_intrinsics", True, {"k_json": str(k_json)})
        except ToolError as e:
            log.warning("[group %d] No intrinsics in project, proceeding without K", group_id)
            _emit("export_intrinsics", False, {"warning": str(e)})
            k_json = None  # type: ignore[assignment]
    else:
        _emit("export_intrinsics", True, {"dry_run": True})

    # ── Step 7: geometric verification ───────────────────────────────────────
    log.info("[group %d/%s] step 7: geometric verification", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.geo_verify(
                pairs_json, match_dir, geo_dir,
                k_json=k_json,
                image_list=image_list_json if k_json else None,
                estimate_h=True,
                extra_args=cfg.extra_geo or None,
            )
            _emit("geo_verify", True, {"geo_dir": str(geo_dir)})
        except ToolError as e:
            _emit("geo_verify", False, {"error": str(e)})
            return False
    else:
        _emit("geo_verify", True, {"dry_run": True})

    # ── Step 8: two-view reconstruction ──────────────────────────────────────
    log.info("[group %d/%s] step 8: two-view reconstruct", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.twoview_reconstruct(
                pairs_json, geo_dir, match_dir, tv_dir,
                k_json=k_json,
                image_list=image_list_json if k_json else None,
                extra_args=cfg.extra_twoview or None,
            )
            _emit("twoview_reconstruct", True, {"tv_dir": str(tv_dir)})
        except ToolError as e:
            _emit("twoview_reconstruct", False, {"error": str(e)})
            return False
    else:
        _emit("twoview_reconstruct", True, {"dry_run": True})

    # ── Step 9: calibrate (refine K / 估计焦距) ──────────────────────────────────────────
    log.info("[group %d/%s] step 9: calibrate", group_id, group_name)
    if not cfg.dry_run:
        try:
            pu.calibrate(tv_dir, k_refined_json, cfg.extra_calibrate or None)
            _emit("calibrate", True, {"k_refined": str(k_refined_json)})
        except ToolError as e:
            _emit("calibrate", False, {"error": str(e)})
            return False
    else:
        _emit("calibrate", True, {"dry_run": True})

    # ── Step 10: write refined K back to project ───────────────────────────────
    log.info("[group %d/%s] step 10: set-camera --from-k", group_id, group_name)
    if not cfg.dry_run:
        try:
            evt = pu.set_camera_from_k(cfg.project_path, group_id, k_refined_json)
            _emit("set_camera", True, evt.get("data", {}))
        except ToolError as e:
            _emit("set_camera", False, {"error": str(e)})
            return False
    else:
        _emit("set_camera", True, {"dry_run": True})

    return True


# ─────────────────────────────────────────────────────────────────────────────
# Main pipeline entry point
# ─────────────────────────────────────────────────────────────────────────────

def run_pipeline(cfg: PipelineConfig) -> int:
    """
    Run the full pipeline for all groups discovered under cfg.input_dir.

    Returns 0 on success, 1 if any group failed.
    """
    cfg.work_dir.mkdir(parents=True, exist_ok=True)

    # ── Discover groups ────────────────────────────────────────────────────────
    groups = _scan_subdirs(cfg.input_dir, cfg.ext)
    if not groups:
        log.error("No image-containing subdirectories found under %s", cfg.input_dir)
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

    # ── Create project ─────────────────────────────────────────────────────────
    if not cfg.dry_run:
        try:
            pu.create_project(cfg.project_path)
            log.info("Created project: %s", cfg.project_path)
        except ToolError as e:
            log.error("Failed to create project: %s", e)
            return 1

    emit_pipeline_event({"step": "create_project", "ok": True,
                         "data": {"project": str(cfg.project_path)}})

    # ── Add groups + images ────────────────────────────────────────────────────
    group_ids: dict[str, int] = {}
    skipped_groups: list[str] = []

    for name, path in groups:
        # add-group
        if not cfg.dry_run:
            try:
                evt = pu.add_group(cfg.project_path, name)
                gid = evt.get("data", {}).get("group_id", len(group_ids))
            except ToolError as e:
                log.warning("add-group failed for %s: %s", name, e)
                skipped_groups.append(name)
                continue
        else:
            gid = len(group_ids)

        # add-images
        if not cfg.dry_run:
            try:
                ai_evt = pu.add_images(cfg.project_path, gid, path, ext=cfg.ext)
                added = ai_evt.get("data", {}).get("added", 0)
            except ToolError as e:
                log.warning("add-images failed for group %s: %s", name, e)
                skipped_groups.append(name)
                continue
        else:
            added = -1  # unknown in dry-run

        if added < cfg.min_images and not cfg.dry_run:
            log.warning(
                "Group %s has only %d images (min=%d), skipping",
                name, added, cfg.min_images,
            )
            skipped_groups.append(name)
            continue

        group_ids[name] = gid
        log.info("Group %d '%s': %d images added", gid, name, added)

    if not group_ids:
        log.error("No groups with sufficient images (min=%d)", cfg.min_images)
        return 1

    # ── Estimate initial camera intrinsics (EXIF/sensor_db) ───────────────────
    if not cfg.dry_run:
        try:
            pu.estimate_camera(
                cfg.project_path,
                all_groups=True,
                sensor_db=cfg.sensor_db,
                max_sample=cfg.max_sample,
            )
            log.info("Camera estimation complete for all groups")
        except ToolError as e:
            log.warning("Camera estimation failed (continuing): %s", e)
    emit_pipeline_event({"step": "estimate_camera", "ok": True, "data": {}})

    # ── Create AT task (snapshot of project for extract/intrinsics) ───────────
    task_id = 0
    if not cfg.dry_run:
        try:
            at_evt = pu.create_at_task(cfg.project_path)
            task_id = at_evt.get("task_id", 0)
            log.info("Created AT task_id=%d for pipeline", task_id)
        except ToolError as e:
            log.error("Failed to create AT task: %s", e)
            return 1
    emit_pipeline_event({"step": "create_at_task", "ok": True, "data": {"task_id": task_id}})

    # ── Per-group pipeline ─────────────────────────────────────────────────────
    failed = 0
    for name, gid in group_ids.items():
        path = dict(groups)[name]
        log.info("=== Processing group %d '%s' ===", gid, name)
        ok = _run_group(cfg, gid, name, path, task_id=task_id)
        if not ok:
            log.error("Group %d '%s' failed", gid, name)
            failed += 1

    emit_pipeline_event({
        "step": "pipeline_complete",
        "ok": failed == 0,
        "data": {
            "total_groups": len(group_ids),
            "failed_groups": failed,
            "skipped_groups": skipped_groups,
            "project": str(cfg.project_path),
        },
    })

    return 0 if failed == 0 else 1


# ─────────────────────────────────────────────────────────────────────────────
# CLI entry point
# ─────────────────────────────────────────────────────────────────────────────

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="auto_calibrate",
        description=(
            "InsightAT auto-calibration pipeline.\n"
            "Scans subdirectories, creates project, estimates + refines "
            "camera intrinsics per group."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--input", "-i", required=True, metavar="DIR",
        help="Root directory to scan for image sub-directories.",
    )
    p.add_argument(
        "--project", "-p", required=True, metavar="FILE",
        help="Output .iat project file path.",
    )
    p.add_argument(
        "--work-dir", "-w", required=True, metavar="DIR",
        help="Working directory for intermediate files.",
    )
    p.add_argument(
        "--ext", default=".jpg,.tif,.png", metavar="EXTS",
        help="Comma-separated image extensions (default: .jpg,.tif,.png).",
    )
    p.add_argument(
        "--min-images", type=int, default=5, metavar="N",
        help="Minimum images per group (groups below are skipped, default: 5).",
    )
    p.add_argument(
        "--max-sample", type=int, default=5, metavar="N",
        help="Max images sampled for focal estimation (default: 5).",
    )
    p.add_argument(
        "--sensor-db", metavar="FILE",
        help="Path to sensor database file for focal estimation.",
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="Print what would be done without executing any tools.",
    )
    p.add_argument(
        "--log-level", choices=("error", "warn", "info", "debug"), metavar="LEVEL",
        help="Log level: error|warn|info|debug (same as isat_* CLI). Default: warn.",
    )
    p.add_argument(
        "-v", "--verbose", action="store_true",
        help="Same as --log-level=info.",
    )
    p.add_argument(
        "-q", "--quiet", action="store_true",
        help="Same as --log-level=error.",
    )
    p.add_argument(
        "-l", "--log-file", metavar="FILE",
        help="Append all logs (Python + tool stderr when info/debug/--progress) to this file.",
    )
    p.add_argument(
        "--progress", action="store_true",
        help="Show only progress-like lines from tools (e.g. 50/550, %%) instead of full stderr.",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    # ── Resolve log level (same priority as CLI: --log-level > -q > -v > default warn)
    log_level = args.log_level or ("error" if args.quiet else "info" if args.verbose else "warn")

    # ── Python logging level (aligned with CLI_IO_CONVENTIONS)
    level_map = {"error": logging.ERROR, "warn": logging.WARNING, "info": logging.INFO, "debug": logging.DEBUG}
    level = level_map[log_level]
    log_fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    logging.basicConfig(level=level, format=log_fmt, stream=sys.stderr)
    if args.log_file:
        log_stream = open(args.log_file, mode="a", encoding="utf-8", buffering=1)
        file_handler = logging.StreamHandler(log_stream)
        file_handler.setLevel(level)
        file_handler.setFormatter(logging.Formatter(log_fmt))
        logging.getLogger().addHandler(file_handler)
        log.info("Logging to file: %s", args.log_file)
    set_cli_log_level(log_level)
    set_log_tool_stderr_at_info(log_level in ("info", "debug") or args.progress or bool(args.log_file))
    set_log_tool_stderr_progress_only(args.progress and log_level not in ("info", "debug"))

    # ── Build config ───────────────────────────────────────────────────────────
    input_dir = Path(args.input).resolve()
    if not input_dir.is_dir():
        print(f"ERROR: --input directory does not exist: {input_dir}", file=sys.stderr)
        return 2

    cfg = PipelineConfig(
        input_dir=input_dir,
        project_path=Path(args.project).resolve(),
        work_dir=Path(args.work_dir).resolve(),
        ext=args.ext,
        min_images=args.min_images,
        max_sample=args.max_sample,
        sensor_db=Path(args.sensor_db).resolve() if args.sensor_db else None,
        dry_run=args.dry_run,
        log_level=log_level,
    )

    return run_pipeline(cfg)


if __name__ == "__main__":
    sys.exit(main())
