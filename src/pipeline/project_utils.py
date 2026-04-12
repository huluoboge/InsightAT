"""
project_utils.py
InsightAT Pipeline – high-level wrappers for isat_project subcommands.

All functions raise ToolError on non-zero exit; see runner.py.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from .runner import find_binary, run_tool, ToolError  # noqa: F401 re-export ToolError

log = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────


def _isat_project() -> str:
    return find_binary("isat_project")


def _isat_camera_estimator() -> str:
    return find_binary("isat_camera_estimator")


def _isat_extract() -> str:
    return find_binary("isat_extract")


def _isat_retrieve() -> str:
    return find_binary("isat_retrieve")


def _isat_match() -> str:
    return find_binary("isat_match")


def _isat_geo() -> str:
    return find_binary("isat_geo")


def _isat_twoview() -> str:
    return find_binary("isat_twoview")


def _isat_calibrate() -> str:
    return find_binary("isat_calibrate")


def _isat_tracks() -> str:
    return find_binary("isat_tracks")


def _isat_incremental_sfm() -> str:
    return find_binary("isat_incremental_sfm")


def get_default_sensor_db_path() -> Path | None:
    """
    Path to camera_sensor_database.txt under build/data/config (CMake copies data/config at build).
    Used when pipeline calls isat_camera_estimator without user-supplied --sensor-db.
    Returns None if the file does not exist.
    """
    try:
        bin_path = Path(find_binary("isat_project")).resolve()
        build_dir = bin_path.parent
        candidate = build_dir / "data" / "config" / "camera_sensor_database.txt"
        if candidate.is_file():
            return candidate
    except (FileNotFoundError, OSError):
        pass
    return None


# ─────────────────────────────────────────────────────────────────────────────
# isat_project wrappers
# ─────────────────────────────────────────────────────────────────────────────


def create_project(project_path: str | Path) -> dict[str, Any]:
    """
    isat_project create -p <project_path>
    Returns the ISAT_EVENT dict (type=project.create).
    """
    events = run_tool([_isat_project(), "create", "-p", str(project_path)])
    return _first_event(events, "create_project")


def add_group(project_path: str | Path, group_name: str) -> dict[str, Any]:
    """
    isat_project add-group -p <project_path> -n <group_name>
    Returns the ISAT_EVENT dict (type=project.add_group).
    """
    events = run_tool(
        [
            _isat_project(),
            "add-group",
            "-p",
            str(project_path),
            "-n",
            group_name,
        ]
    )
    return _first_event(events, "add_group")


def add_images(
    project_path: str | Path,
    group_id: int,
    directory: str | Path,
    ext: str | None = None,
    recursive: bool = False,
    no_dedup: bool = False,
) -> dict[str, Any]:
    """
    isat_project add-images -p <project_path> -g <group_id> -i <directory> [options]
    Returns the ISAT_EVENT dict (type=project.add_images).
    """
    cmd = [
        _isat_project(),
        "add-images",
        "-p",
        str(project_path),
        "-g",
        str(group_id),
        "-i",
        str(directory),
    ]
    if ext:
        cmd += ["--ext", ext]
    if recursive:
        cmd.append("--recursive")
    if no_dedup:
        cmd.append("--no-dedup")
    events = run_tool(cmd)
    return _first_event(events, "add_images")


def set_camera_from_k(
    project_path: str | Path,
    group_id: int,
    k_json: str | Path,
) -> dict[str, Any]:
    """
    isat_project set-camera -p <project_path> -g <group_id> --from-k <k_json>
    Returns the ISAT_EVENT dict (type=project.set_camera).
    """
    events = run_tool(
        [
            _isat_project(),
            "set-camera",
            "-p",
            str(project_path),
            "-g",
            str(group_id),
            "--from-k",
            str(k_json),
        ]
    )
    return _first_event(events, "set_camera_from_k")


def create_at_task(
    project_path: str | Path,
    name: str | None = None,
    parent_task_id: int | None = None,
) -> dict[str, Any]:
    """
    isat_project create-at-task -p <project_path> [-n <name>] [--parent-task-id N]
    Creates an AT task with a snapshot of current project (image groups, etc.).
    Supports nesting: use parent_task_id to create a sub-task under another AT task.
    Returns the event data dict with task_id, uuid, task_name.
    """
    cmd = [
        _isat_project(),
        "create-at-task",
        "-p",
        str(project_path),
    ]
    if name:
        cmd += ["-n", name]
    if parent_task_id is not None:
        cmd += ["--parent-task-id", str(parent_task_id)]
    events = run_tool(cmd)
    ev = _first_event(events, "create_at_task")
    return ev.get("data", {})


def export_image_list(
    project_path: str | Path,
    group_id: int,
    output_json: str | Path,
    task_id: int = 0,
) -> None:
    """
    isat_project extract -p <project_path> -g <group_id> -t <task_id> -o <output_json>
    (No ISAT_EVENT; writes file directly.)
    """
    run_tool(
        [
            _isat_project(),
            "extract",
            "-p",
            str(project_path),
            "-g",
            str(group_id),
            "-t",
            str(task_id),
            "-o",
            str(output_json),
        ]
    )


def export_image_list_all_groups(
    project_path: str | Path,
    task_id: int,
    output_json: str | Path,
) -> None:
    """
    isat_project extract -p <project_path> -t <task_id> -o <output_json> -a
    Export image list for all groups (for pipeline step: extract on all images).
    """
    run_tool(
        [
            _isat_project(),
            "extract",
            "-p",
            str(project_path),
            "-t",
            str(task_id),
            "-o",
            str(output_json),
            "-a",
        ]
    )


def export_intrinsics(
    project_path: str | Path,
    group_id: int,
    output_json: str | Path,
    task_id: int = 0,
) -> None:
    """
    isat_project intrinsics -p <project_path> -g <group_id> -t <task_id> -o <output_json>
    (No ISAT_EVENT; writes file directly.)
    """
    run_tool(
        [
            _isat_project(),
            "intrinsics",
            "-p",
            str(project_path),
            "-g",
            str(group_id),
            "-t",
            str(task_id),
            "-o",
            str(output_json),
        ]
    )


def export_intrinsics_all_groups(
    project_path: str | Path,
    task_id: int,
    output_json: str | Path,
) -> None:
    """
    isat_project intrinsics -p <project_path> -t <task_id> -o <output_json> -a
    Export multi_camera_v1 intrinsics for all groups (for use with isat_geo -k -l).
    """
    run_tool(
        [
            _isat_project(),
            "intrinsics",
            "-p",
            str(project_path),
            "-t",
            str(task_id),
            "-o",
            str(output_json),
            "-a",
        ]
    )


# ─────────────────────────────────────────────────────────────────────────────
# isat_camera_estimator wrapper
# ─────────────────────────────────────────────────────────────────────────────


def estimate_camera(
    project_path: str | Path,
    group_id: int | None = None,
    all_groups: bool = False,
    sensor_db: str | Path | None = None,
    max_sample: int = 5,
    auto_split: bool = True,
    split_threads: int = 0,
) -> list[dict[str, Any]]:
    """
    isat_camera_estimator -p <project_path> [-g <group_id> | -a] [options]
    Returns list of ISAT_EVENT dicts (type=camera_estimator.estimate), one per group.
    When sensor_db is None, uses build/data/config/camera_sensor_database.txt if present.
    auto_split=True (default): scan all images and split groups with mixed camera models.
    """
    if group_id is None and not all_groups:
        raise ValueError("Specify group_id or all_groups=True")
    cmd = [_isat_camera_estimator(), "-p", str(project_path)]
    if all_groups:
        cmd.append("-a")
    else:
        cmd += ["-g", str(group_id)]
    if sensor_db is not None:
        cmd += ["-d", str(sensor_db)]
    else:
        default_db = get_default_sensor_db_path()
        if default_db is not None:
            cmd += ["-d", str(default_db)]
    cmd += ["--max-sample", str(max_sample)]
    if auto_split:
        cmd.append("--auto-split")
        if split_threads > 0:
            cmd += ["--split-threads", str(split_threads)]
    return run_tool(cmd)


# ─────────────────────────────────────────────────────────────────────────────
# Per-step wrappers (feature extraction through calibration)
# ─────────────────────────────────────────────────────────────────────────────


def _isat_train_vlad() -> str:
    return find_binary("isat_train_vlad")


def extract_features(
    image_list_json: str | Path,
    feat_dir: str | Path,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """isat_extract -i <list> -o <feat_dir> [extra_args]"""
    cmd = [_isat_extract(), "-i", str(image_list_json), "-o", str(feat_dir)]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def extract_features_dual(
    image_list_json: str | Path,
    match_feat_dir: str | Path,
    retrieval_feat_dir: str | Path,
    nfeatures_retrieval: int = 1500,
    resize_retrieval: int = 1024,
    extract_backend: str = "cuda",
    extra_args: list[str] | None = None,
) -> list[dict]:
    """
    isat_extract with dual output: matching features (-o) + retrieval features (--output-retrieval).
    Use match_feat_dir for isat_match; use retrieval_feat_dir for isat_train_vlad and isat_retrieve VLAD.
    extract_backend: "cuda" or "glsl" (default cuda).
    """
    cmd = [
        _isat_extract(),
        "-i",
        str(image_list_json),
        "-o",
        str(match_feat_dir),
        "--output-retrieval",
        str(retrieval_feat_dir),
        "--nfeatures-retrieval",
        str(nfeatures_retrieval),
        "--resize-retrieval",
        str(resize_retrieval),
        "--extract-backend",
        str(extract_backend),
        "--nfeatures",
        "40000",
        "--threshold",
        "0.02",
        "--octaves",
        "-1",
        "--levels",
        "3",
        "--norm",
        "l1root",
        "--no-adapt",
        "--nms",
        "--uint8",
    ]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def train_vlad(
    feature_dir: str | Path,
    codebook_path: str | Path,
    pca_output: str | Path,
    num_clusters: int = 64,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """
    isat_train_vlad -f <feature_dir> -o <codebook.vcbt> [-P pca.pca] [-d dims] [-w] [-k clusters]
    Train VLAD codebook on retrieval .isat_feat; optionally output PCA model for dimensionality reduction.
    """
    cmd = [
        _isat_train_vlad(),
        "-f",
        str(feature_dir),
        "-o",
        str(codebook_path),
        "-k",
        str(num_clusters),
        "--pca-output",
        str(pca_output),
        "--whiten",
        "--scale-weighted",
    ]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def retrieve_pairs(
    image_list_json: str | Path,
    feat_dir: str | Path,
    output_json: str | Path,
    strategy: str = "sequential+vlad",
    window_size: int = 10,
    vlad_codebook: str | Path | None = None,
    vlad_cache_dir: str | Path | None = None,
    pca_model_file: str | Path | None = None,
    vlad_top_k: int = 20,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """
    isat_retrieve -i <list> -f <feat_dir> -o <pairs.json> -s <strategy> [VLAD/sequential options].
    strategy e.g. "vlad+sequential" for VLAD + sequence; --window for sequential neighbor size.
    """
    cmd = [
        _isat_retrieve(),
        "-i",
        str(image_list_json),
        "-f",
        str(feat_dir),
        "-o",
        str(output_json),
        "-s",
        strategy,
        "-w",
        str(window_size),
        "--max-neighbors",
        "50",
    ]
    if vlad_codebook:
        cmd += ["--vlad-codebook", str(vlad_codebook)]
        cmd += ["--vlad-top-k", str(vlad_top_k)]
        if vlad_cache_dir:
            cmd += ["--vlad-cache", str(vlad_cache_dir)]
        if pca_model_file:
            cmd += ["--pca-model", str(pca_model_file)]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def match_features(
    pairs_json: str | Path,
    feat_dir: str | Path,
    match_dir: str | Path,
    match_backend: str = "cuda",
    extra_args: list[str] | None = None,
) -> list[dict]:
    """isat_match -i <pairs> -f <feat_dir> -o <match_dir> [--match-backend cuda|glsl] [extra_args]"""
    cmd = [
        _isat_match(),
        "-i",
        str(pairs_json),
        "-f",
        str(feat_dir),
        "-o",
        str(match_dir),
        "--match-backend",
        match_backend,
        "--max-features",
        "-1",
        "--threads",
        "4",
    ]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def geo_verify(
    pairs_json: str | Path,
    match_dir: str | Path,
    geo_dir: str | Path,
    image_list: str | Path,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """isat_geo -i <pairs> -m <match_dir> -o <geo_dir> [--estimate-h] [--twoview] [--vis] [-k -l].
    When image_list has embedded 'cameras' (index-only export), -l alone is enough; -k optional.
    """
    cmd = [
        _isat_geo(),
        "-i",
        str(pairs_json),
        "-m",
        str(match_dir),
        "-o",
        str(geo_dir),
        "--image-list",
        str(image_list),
        "--min-inliers",
        "15",
        "--backend",
        "gpu",
        "--estimate-h",
        "--twoview",
        "--vis",
    ]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def twoview_reconstruct(
    pairs_json: str | Path,
    geo_dir: str | Path,
    match_dir: str | Path,
    twoview_dir: str | Path,
    k_json: str | Path | None = None,
    image_list: str | Path | None = None,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """isat_twoview -i <pairs> -g <geo_dir> -m <match_dir> -o <tv_dir> [-k -l].
    When image_list has embedded 'cameras' (index-only export), -l alone is enough."""
    cmd = [
        _isat_twoview(),
        "-i",
        str(pairs_json),
        "-g",
        str(geo_dir),
        "-m",
        str(match_dir),
        "-o",
        str(twoview_dir),
    ]
    if k_json:
        cmd += ["-k", str(k_json)]
    if image_list:
        cmd += ["-l", str(image_list)]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def calibrate(
    twoview_dir: str | Path,
    output_k_json: str | Path,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """isat_calibrate -t <twoview_dir> -o <K_refined.json>"""
    cmd = [
        _isat_calibrate(),
        "-t",
        str(twoview_dir),
        "-o",
        str(output_k_json),
    ]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def run_tracks(
    pairs_json: str | Path,
    match_dir: str | Path,
    geo_dir: str | Path,
    image_list: str | Path,
    output_path: str | Path,
    *,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """
    isat_tracks -i pairs.json -m match_dir -g geo_dir -l image_list -o tracks.isat_tracks.

    Builds tracks from match + geo inliers; output is a single .isat_tracks IDC for incremental SfM.
    """
    cmd = [
        _isat_tracks(),
        "-i",
        str(pairs_json),
        "-m",
        str(match_dir),
        "-g",
        str(geo_dir),
        "-l",
        str(image_list),
        "-o",
        str(output_path),
        "--min-track-length",
        "2",
    ]
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


def run_incremental_sfm(
    tracks_path: str | Path,
    project_path: str | Path,
    pairs_path: str | Path,
    geo_dir: str | Path,
    output_dir: str | Path,
    fix_intrinsics: bool = False,
    extra_args: list[str] | None = None,
) -> list[dict]:
    """
    isat_incremental_sfm -t tracks -p project -m pairs -g geo -o output_dir.

    Loads tracks from .isat_tracks IDC and project JSON (images + cameras, camera_index).
    Uses pairs JSON for view graph and geo_dir for .isat_geo. Writes poses.json to output_dir.

    Args:
        fix_intrinsics:         Pass ``--fix-intrinsics`` to hold camera calibration constant.
    """
    cmd = [
        _isat_incremental_sfm(),
        "-t",
        str(tracks_path),
        "-p",
        str(project_path),
        "-m",
        str(pairs_path),
        "-g",
        str(geo_dir),
        "-o",
        str(output_dir),
    ]
    if fix_intrinsics:
        cmd.append("--fix-intrinsics")
    if extra_args:
        cmd += extra_args
    return run_tool(cmd)


# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────────────────────────────────────


def _first_event(events: list[dict], context: str) -> dict[str, Any]:
    if not events:
        log.warning("%s: tool produced no ISAT_EVENT", context)
        return {}
    return events[0]
