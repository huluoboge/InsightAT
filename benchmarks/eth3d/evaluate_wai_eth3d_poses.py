#!/usr/bin/env python3
"""Evaluate the pose collection produced by run_wai_eth3d_batch.py.

The estimated SfM coordinate system is arbitrary, so every scene is aligned
independently with a 3D similarity transform before measuring errors:

    C_gt ~= scale * R_align @ C_est + t

The evaluator reports camera-center errors in the ground-truth metric unit and
rotation errors in degrees.  It accepts both the batch-level poses.json and a
single per-scene poses.json.

Example:
    python3 benchmarks/eth3d/evaluate_wai_eth3d_poses.py \
        --poses /path/to/prj/poses.json
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from benchmarks.sfm_compare.align_similarity import apply_similarity, umeyama  # noqa: E402


CSV_FIELDS = [
    "scene",
    "ground_truth_count",
    "estimated_count",
    "common_count",
    "valid_common_count",
    "estimated_coverage",
    "common_coverage",
    "scale_est_to_gt",
    "position_rmse_m",
    "position_median_m",
    "position_mean_m",
    "position_max_m",
    "rotation_rmse_deg",
    "rotation_median_deg",
    "rotation_mean_deg",
    "rotation_max_deg",
    "run_exit_code",
    "ok",
    "error",
]

POSE_CSV_FIELDS = [
    "scene",
    "pose_type",
    "image_name",
    "frame_name",
    "image_index",
    "camera_index",
    "valid",
    "camera_center_x",
    "camera_center_y",
    "camera_center_z",
    "c2w_r00",
    "c2w_r01",
    "c2w_r02",
    "c2w_r10",
    "c2w_r11",
    "c2w_r12",
    "c2w_r20",
    "c2w_r21",
    "c2w_r22",
    "fx",
    "fy",
    "cx",
    "cy",
    "width",
    "height",
    "k1",
    "k2",
    "k3",
    "p1",
    "p2",
    "error",
]

COMPARISON_CSV_FIELDS = [
    "scene",
    "image_name",
    "match_status",
    "valid",
    "ground_truth_cx",
    "ground_truth_cy",
    "ground_truth_cz",
    "estimated_cx",
    "estimated_cy",
    "estimated_cz",
    "aligned_estimated_cx",
    "aligned_estimated_cy",
    "aligned_estimated_cz",
    "position_error_m",
    "rotation_error_deg",
    "scale_est_to_gt",
    "run_exit_code",
    "error",
]


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value


def image_key(pose: Any) -> str:
    if not isinstance(pose, dict):
        return ""
    name = pose.get("image_name")
    return Path(name).name if isinstance(name, str) and name else ""


def index_poses(poses: Any) -> dict[str, dict[str, Any]]:
    if not isinstance(poses, list):
        return {}
    indexed: dict[str, dict[str, Any]] = {}
    for pose in poses:
        key = image_key(pose)
        if key and isinstance(pose, dict):
            indexed[key] = pose
    return indexed


def pose_matrix(pose: dict[str, Any]) -> np.ndarray:
    """Read camera-to-world, with a fallback for the raw estimated fields."""
    value = pose.get("camera_to_world")
    if value is not None:
        matrix = np.asarray(value, dtype=np.float64)
    else:
        rotation = np.asarray(pose.get("R_world_to_camera"), dtype=np.float64)
        center = np.asarray(pose.get("camera_center"), dtype=np.float64)
        if rotation.shape != (3, 3) or center.shape != (3,):
            raise ValueError("missing camera_to_world and invalid raw pose fields")
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rotation.T
        matrix[:3, 3] = center

    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("camera_to_world must be a finite 4x4 matrix")
    return matrix


def intrinsic_value(intrinsics: Any, *keys: str) -> Any:
    if not isinstance(intrinsics, dict):
        return ""
    for key in keys:
        value = intrinsics.get(key)
        if isinstance(value, (int, float)) and np.isfinite(float(value)):
            return value
    return ""


def pose_csv_row(scene: str, pose_type: str, pose: dict[str, Any]) -> dict[str, Any]:
    row = {field: "" for field in POSE_CSV_FIELDS}
    row.update(
        {
            "scene": scene,
            "pose_type": pose_type,
            "image_name": image_key(pose),
            "frame_name": pose.get("frame_name", ""),
            "image_index": pose.get("image_index", ""),
            "camera_index": pose.get("camera_index", ""),
            "valid": False,
        }
    )
    try:
        matrix = pose_matrix(pose)
    except (TypeError, ValueError) as error:
        row["error"] = str(error)
        return row

    row["valid"] = True
    center = matrix[:3, 3]
    row.update(
        {
            "camera_center_x": float(center[0]),
            "camera_center_y": float(center[1]),
            "camera_center_z": float(center[2]),
        }
    )
    for row_index in range(3):
        for column_index in range(3):
            row[f"c2w_r{row_index}{column_index}"] = float(
                matrix[row_index, column_index]
            )

    intrinsics = pose.get("intrinsics")
    row.update(
        {
            "fx": intrinsic_value(intrinsics, "fx", "fl_x"),
            "fy": intrinsic_value(intrinsics, "fy", "fl_y"),
            "cx": intrinsic_value(intrinsics, "cx"),
            "cy": intrinsic_value(intrinsics, "cy"),
            "width": intrinsic_value(intrinsics, "width", "w"),
            "height": intrinsic_value(intrinsics, "height", "h"),
            "k1": intrinsic_value(intrinsics, "k1"),
            "k2": intrinsic_value(intrinsics, "k2"),
            "k3": intrinsic_value(intrinsics, "k3"),
            "p1": intrinsic_value(intrinsics, "p1"),
            "p2": intrinsic_value(intrinsics, "p2"),
        }
    )
    return row


def flatten_pose_rows(scene_bundles: Iterable[dict[str, Any]]) -> tuple[
    list[dict[str, Any]], list[dict[str, Any]]
]:
    ground_truth_rows: list[dict[str, Any]] = []
    estimated_rows: list[dict[str, Any]] = []
    for bundle in scene_bundles:
        scene = str(bundle.get("scene", ""))
        ground_truth = bundle.get("ground_truth")
        estimated = bundle.get("estimated")
        if isinstance(ground_truth, list):
            ground_truth_rows.extend(
                pose_csv_row(scene, "ground_truth", pose)
                for pose in ground_truth
                if isinstance(pose, dict)
            )
        if isinstance(estimated, list):
            estimated_rows.extend(
                pose_csv_row(scene, "estimated", pose)
                for pose in estimated
                if isinstance(pose, dict)
            )
    return ground_truth_rows, estimated_rows


def set_comparison_center(row: dict[str, Any], prefix: str, center: np.ndarray) -> None:
    row[f"{prefix}_cx"] = float(center[0])
    row[f"{prefix}_cy"] = float(center[1])
    row[f"{prefix}_cz"] = float(center[2])


def build_comparison_rows(
    scene: str,
    ground_truth_by_name: dict[str, dict[str, Any]],
    estimated_by_name: dict[str, dict[str, Any]],
    run_exit_code: Any,
) -> tuple[
    list[dict[str, Any]],
    list[tuple[str, np.ndarray, np.ndarray, dict[str, Any]]],
]:
    rows: list[dict[str, Any]] = []
    valid_records: list[tuple[str, np.ndarray, np.ndarray, dict[str, Any]]] = []
    all_names = sorted(set(ground_truth_by_name) | set(estimated_by_name))
    for name in all_names:
        has_ground_truth = name in ground_truth_by_name
        has_estimated = name in estimated_by_name
        status = (
            "common"
            if has_ground_truth and has_estimated
            else "ground_truth_only"
            if has_ground_truth
            else "estimated_only"
        )
        row = {
            "scene": scene,
            "image_name": name,
            "match_status": status,
            "valid": False,
            "ground_truth_cx": "",
            "ground_truth_cy": "",
            "ground_truth_cz": "",
            "estimated_cx": "",
            "estimated_cy": "",
            "estimated_cz": "",
            "aligned_estimated_cx": "",
            "aligned_estimated_cy": "",
            "aligned_estimated_cz": "",
            "position_error_m": "",
            "rotation_error_deg": "",
            "scale_est_to_gt": "",
            "run_exit_code": run_exit_code,
            "error": "",
        }
        ground_truth_matrix = None
        estimated_matrix = None
        if has_ground_truth:
            try:
                ground_truth_matrix = pose_matrix(ground_truth_by_name[name])
                set_comparison_center(row, "ground_truth", ground_truth_matrix[:3, 3])
            except (TypeError, ValueError) as error:
                row["error"] = f"ground_truth: {error}"
        if has_estimated:
            try:
                estimated_matrix = pose_matrix(estimated_by_name[name])
                set_comparison_center(row, "estimated", estimated_matrix[:3, 3])
            except (TypeError, ValueError) as error:
                row["error"] = f"estimated: {error}"
        if not has_ground_truth:
            row["error"] = "missing_ground_truth"
        elif not has_estimated:
            row["error"] = "missing_estimated"
        elif ground_truth_matrix is not None and estimated_matrix is not None:
            valid_records.append((name, estimated_matrix, ground_truth_matrix, row))
        rows.append(row)
    return rows, valid_records


def rotation_error_deg(estimated_c2w: np.ndarray, ground_truth_c2w: np.ndarray,
                       alignment_rotation: np.ndarray) -> float:
    """Return the angle between aligned estimated and ground-truth orientations."""
    estimated_world_rotation = alignment_rotation @ estimated_c2w[:3, :3]
    relative = ground_truth_c2w[:3, :3].T @ estimated_world_rotation
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def metric_values(values: np.ndarray, suffix: str) -> dict[str, float]:
    return {
        f"{suffix}_rmse": float(np.sqrt(np.mean(values * values))),
        f"{suffix}_median": float(np.median(values)),
        f"{suffix}_mean": float(np.mean(values)),
        f"{suffix}_max": float(np.max(values)),
    }


def empty_row(scene: str, gt_count: int, estimated_count: int, common_count: int,
              run_exit_code: Any, error: str) -> dict[str, Any]:
    return {
        "scene": scene,
        "ground_truth_count": gt_count,
        "estimated_count": estimated_count,
        "common_count": common_count,
        "valid_common_count": 0,
        "estimated_coverage": estimated_count / gt_count if gt_count else None,
        "common_coverage": common_count / gt_count if gt_count else None,
        "scale_est_to_gt": None,
        "position_rmse_m": None,
        "position_median_m": None,
        "position_mean_m": None,
        "position_max_m": None,
        "rotation_rmse_deg": None,
        "rotation_median_deg": None,
        "rotation_mean_deg": None,
        "rotation_max_deg": None,
        "run_exit_code": run_exit_code,
        "ok": False,
        "error": error,
    }


def evaluate_scene(scene_bundle: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    scene = str(scene_bundle.get("scene", ""))
    gt_by_name = index_poses(scene_bundle.get("ground_truth"))
    estimated_by_name = index_poses(scene_bundle.get("estimated"))
    common_names = sorted(set(gt_by_name) & set(estimated_by_name))
    run_exit_code = scene_bundle.get("run_exit_code")

    base = {
        "scene": scene,
        "ground_truth_count": len(gt_by_name),
        "estimated_count": len(estimated_by_name),
        "common_count": len(common_names),
        "valid_common_count": 0,
        "estimated_coverage": (
            len(estimated_by_name) / len(gt_by_name) if gt_by_name else None
        ),
        "common_coverage": (
            len(common_names) / len(gt_by_name) if gt_by_name else None
        ),
        "scale_est_to_gt": None,
        "position_rmse_m": None,
        "position_median_m": None,
        "position_mean_m": None,
        "position_max_m": None,
        "rotation_rmse_deg": None,
        "rotation_median_deg": None,
        "rotation_mean_deg": None,
        "rotation_max_deg": None,
        "run_exit_code": run_exit_code,
        "ok": False,
        "error": "",
    }

    comparison_rows, valid_records = build_comparison_rows(
        scene, gt_by_name, estimated_by_name, run_exit_code
    )
    estimated_centers = [record[1][:3, 3] for record in valid_records]
    ground_truth_centers = [record[2][:3, 3] for record in valid_records]
    invalid_names = [
        row["image_name"]
        for row in comparison_rows
        if row["match_status"] == "common" and row["error"]
    ]
    valid_count = len(valid_records)
    base["valid_common_count"] = valid_count
    if valid_count < 3:
        reason = f"需要至少 3 个有效共同位姿，实际为 {valid_count}"
        if invalid_names:
            reason += f"；无效位姿 {len(invalid_names)} 个"
        for row in comparison_rows:
            if row["match_status"] == "common" and not row["error"]:
                row["error"] = reason
        base["error"] = reason
        return base, {
            "position_errors": np.empty(0),
            "rotation_errors": np.empty(0),
            "comparison_rows": comparison_rows,
        }

    source_centers = np.stack(estimated_centers, axis=0)
    target_centers = np.stack(ground_truth_centers, axis=0)
    try:
        similarity = umeyama(source_centers, target_centers)
    except (AssertionError, np.linalg.LinAlgError, ValueError) as error:
        base["error"] = f"Sim(3) 对齐失败: {error}"
        for row in comparison_rows:
            if row["match_status"] == "common" and not row["error"]:
                row["error"] = base["error"]
        return base, {
            "position_errors": np.empty(0),
            "rotation_errors": np.empty(0),
            "comparison_rows": comparison_rows,
        }

    centered_source = source_centers - source_centers.mean(axis=0)
    if not np.isfinite(centered_source).all() or np.sum(centered_source * centered_source) <= 1e-20:
        base["error"] = "估计相机中心退化，无法估计 Sim(3)"
        for row in comparison_rows:
            if row["match_status"] == "common" and not row["error"]:
                row["error"] = base["error"]
        return base, {
            "position_errors": np.empty(0),
            "rotation_errors": np.empty(0),
            "comparison_rows": comparison_rows,
        }

    aligned_centers = apply_similarity(similarity, source_centers)
    position_errors = np.linalg.norm(aligned_centers - target_centers, axis=1)
    rotation_errors = np.asarray(
        [
            rotation_error_deg(
                valid_records[index][1],
                valid_records[index][2],
                similarity.R,
            )
            for index in range(valid_count)
        ],
        dtype=np.float64,
    )
    for index, (_, _, _, comparison_row) in enumerate(valid_records):
        set_comparison_center(comparison_row, "aligned_estimated", aligned_centers[index])
        comparison_row["position_error_m"] = float(position_errors[index])
        comparison_row["rotation_error_deg"] = float(rotation_errors[index])
        comparison_row["scale_est_to_gt"] = float(similarity.scale)
        comparison_row["valid"] = True

    base["scale_est_to_gt"] = float(similarity.scale)
    base.update(
        {
            "position_rmse_m": metric_values(position_errors, "position")["position_rmse"],
            "position_median_m": metric_values(position_errors, "position")["position_median"],
            "position_mean_m": metric_values(position_errors, "position")["position_mean"],
            "position_max_m": metric_values(position_errors, "position")["position_max"],
            "rotation_rmse_deg": metric_values(rotation_errors, "rotation")["rotation_rmse"],
            "rotation_median_deg": metric_values(rotation_errors, "rotation")[
                "rotation_median"
            ],
            "rotation_mean_deg": metric_values(rotation_errors, "rotation")["rotation_mean"],
            "rotation_max_deg": metric_values(rotation_errors, "rotation")["rotation_max"],
        }
    )
    if invalid_names:
        base["error"] = f"跳过 {len(invalid_names)} 个无效共同位姿"
    base["ok"] = True
    return base, {
        "position_errors": position_errors,
        "rotation_errors": rotation_errors,
        "comparison_rows": comparison_rows,
    }


def load_scene_bundles(root: dict[str, Any]) -> list[dict[str, Any]]:
    scenes = root.get("scenes")
    if isinstance(scenes, list):
        bundles = [scene for scene in scenes if isinstance(scene, dict)]
        if bundles:
            return bundles
    if "ground_truth" in root and "estimated" in root:
        return [root]
    raise ValueError("输入 JSON 不是批量或单场景位姿集合")


def quaternion_rotation(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """COLMAP Hamilton quaternion to world-to-camera rotation matrix."""
    return np.asarray(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def read_colmap_cameras(path: Path) -> dict[int, dict[str, Any]]:
    cameras: dict[int, dict[str, Any]] = {}
    if not path.is_file():
        return cameras
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 5:
            continue
        camera_id, model = int(fields[0]), fields[1]
        width, height = int(fields[2]), int(fields[3])
        params = [float(value) for value in fields[4:]]
        intrinsics: dict[str, Any] = {"width": width, "height": height}
        if model in {"SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL"} and len(params) >= 3:
            intrinsics.update({"fx": params[0], "fy": params[0], "cx": params[1], "cy": params[2]})
            if model == "SIMPLE_RADIAL" and len(params) >= 4:
                intrinsics["k1"] = params[3]
            elif model == "RADIAL" and len(params) >= 5:
                intrinsics.update({"k1": params[3], "k2": params[4]})
        elif model in {"PINHOLE", "OPENCV", "OPENCV_FISHEYE", "FULL_OPENCV"} and len(params) >= 4:
            intrinsics.update({"fx": params[0], "fy": params[1], "cx": params[2], "cy": params[3]})
            if model == "OPENCV" and len(params) >= 8:
                intrinsics.update({"k1": params[4], "k2": params[5], "p1": params[6], "p2": params[7]})
            elif model == "OPENCV_FISHEYE" and len(params) >= 8:
                intrinsics.update({"k1": params[4], "k2": params[5], "k3": params[6]})
            elif model == "FULL_OPENCV" and len(params) >= 12:
                intrinsics.update({"k1": params[4], "k2": params[5], "p1": params[8], "p2": params[9], "k3": params[6]})
        cameras[camera_id] = intrinsics
    return cameras


def read_colmap_ground_truth(gt_dir: Path) -> list[dict[str, Any]]:
    """Read ETH3D DSLR ground-truth camera poses from COLMAP text files."""
    image_path = gt_dir / "images.txt"
    if not image_path.is_file():
        raise FileNotFoundError(f"missing COLMAP ground-truth file: {image_path}")
    cameras = read_colmap_cameras(gt_dir / "cameras.txt")
    lines = image_path.read_text(encoding="utf-8").splitlines()
    poses: list[dict[str, Any]] = []
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        if not line or line.startswith("#"):
            index += 1
            continue
        fields = line.split()
        if len(fields) < 10:
            index += 1
            continue
        try:
            image_id = int(fields[0])
            qw, qx, qy, qz = (float(value) for value in fields[1:5])
            translation = np.asarray([float(value) for value in fields[5:8]], dtype=np.float64)
            camera_id = int(fields[8])
        except ValueError:
            index += 1
            continue
        image_name = Path(" ".join(fields[9:])).name
        rotation = quaternion_rotation(qw, qx, qy, qz)
        rotation_camera_to_world = rotation.T
        center = -rotation_camera_to_world @ translation
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rotation_camera_to_world
        matrix[:3, 3] = center
        poses.append(
            {
                "image_name": image_name,
                "image_index": image_id,
                "camera_index": camera_id,
                "R_world_to_camera": rotation.tolist(),
                "camera_center": center.tolist(),
                "camera_to_world": matrix.tolist(),
                "intrinsics": cameras.get(camera_id, {}),
            }
        )
        # COLMAP stores each image's 2D observations on the next line.
        index += 2
    return poses


def image_names_from_work(work_dir: Path) -> dict[int, str]:
    image_list = work_dir / "images_all.json"
    if not image_list.is_file():
        return {}
    try:
        images = read_json(image_list).get("images", [])
    except (OSError, ValueError, json.JSONDecodeError):
        return {}
    names: dict[int, str] = {}
    if isinstance(images, list):
        for fallback_index, image in enumerate(images):
            if not isinstance(image, dict):
                continue
            image_index = image.get("image_index", fallback_index)
            image_path = image.get("path")
            if isinstance(image_index, int) and isinstance(image_path, str):
                names[image_index] = Path(image_path).name
    return names


def estimated_poses_from_file(path: Path, work_dir: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    value = read_json(path)
    raw_poses = value.get("estimated")
    if isinstance(raw_poses, list):
        return raw_poses
    raw_poses = value.get("poses", [])
    if not isinstance(raw_poses, list):
        return []
    names = image_names_from_work(work_dir)
    estimates: list[dict[str, Any]] = []
    for pose in raw_poses:
        if not isinstance(pose, dict):
            continue
        image_index = pose.get("image_index")
        rotation = pose.get("R")
        center = pose.get("C")
        if not isinstance(image_index, int) or not isinstance(rotation, list) or len(rotation) != 9:
            continue
        if not isinstance(center, list) or len(center) != 3:
            continue
        rotation_matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        camera_to_world = np.eye(4, dtype=np.float64)
        camera_to_world[:3, :3] = rotation_matrix.T
        camera_to_world[:3, 3] = np.asarray(center, dtype=np.float64)
        estimates.append(
            {
                "image_name": names.get(image_index, ""),
                "image_index": image_index,
                "camera_index": pose.get("camera_index"),
                "R_world_to_camera": rotation_matrix.tolist(),
                "camera_center": center,
                "camera_to_world": camera_to_world.tolist(),
                "intrinsics": {},
            }
        )
    return estimates


def load_eth3d_bundles(dataset_root: Path, project_root: Path) -> list[dict[str, Any]]:
    """Build evaluator input directly from prepared ETH3D data and SfM work dirs."""
    data_scenes = dataset_root / "scenes"
    bundles: list[dict[str, Any]] = []
    for data_scene in sorted(path for path in data_scenes.iterdir() if path.is_dir()):
        name = data_scene.name
        gt_dir = dataset_root / "raw" / name / "dslr_calibration_jpg"
        if not gt_dir.is_dir():
            gt_dir = data_scene / "gt"

        project_scene = project_root / "prj" / name
        run_path = project_scene / "run.json"
        if not run_path.is_file():
            run_path = data_scene / "results" / "insightat" / "run.json"
        run_exit_code: Any = None
        if run_path.is_file():
            try:
                run_exit_code = read_json(run_path).get("exit_code")
            except (OSError, ValueError, json.JSONDecodeError):
                pass

        pose_candidates = [
            (project_scene / "poses.json", project_scene / "work"),
            (project_scene / "work" / "incremental_sfm" / "poses.json", project_scene / "work"),
            (data_scene / "results" / "insightat" / "work" / "incremental_sfm" / "poses.json",
             data_scene / "results" / "insightat" / "work"),
            (data_scene / "results" / "insightat" / "previous_workspace" / "incremental_sfm" / "poses.json",
             data_scene / "results" / "insightat" / "previous_workspace"),
            (data_scene / "results" / "previous_scenes-00" / "insightat" / "work" / "incremental_sfm" / "poses.json",
             data_scene / "results" / "previous_scenes-00" / "insightat" / "work"),
        ]
        pose_path, work_dir = next(
            ((path, work) for path, work in pose_candidates if path.is_file()),
            (pose_candidates[0][0], pose_candidates[0][1]),
        )
        try:
            ground_truth = read_colmap_ground_truth(gt_dir)
        except (OSError, ValueError) as error:
            ground_truth = []
            gt_error = str(error)
        else:
            gt_error = ""
        try:
            estimated = estimated_poses_from_file(pose_path, work_dir)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            estimated = []
            pose_error = str(error)
        else:
            pose_error = ""
        bundles.append(
            {
                "schema": "insightat_eth3d_pose_collection_v1",
                "scene": name,
                "run_exit_code": run_exit_code,
                "ground_truth": ground_truth,
                "estimated": estimated,
                "source_files": {
                    "ground_truth": str(gt_dir / "images.txt"),
                    "estimated": str(pose_path) if pose_path.is_file() else "",
                },
                "load_error": "; ".join(error for error in (gt_error, pose_error) if error),
            }
        )
    return bundles


def aggregate_rows(rows: list[dict[str, Any]],
                   residuals: list[dict[str, Any]]) -> dict[str, Any]:
    position = [
        values["position_errors"]
        for values in residuals
        if values["position_errors"].size
    ]
    rotation = [
        values["rotation_errors"]
        for values in residuals
        if values["rotation_errors"].size
    ]
    position_errors = np.concatenate(position) if position else np.empty(0)
    rotation_errors = np.concatenate(rotation) if rotation else np.empty(0)

    total_gt = sum(int(row["ground_truth_count"]) for row in rows)
    total_estimated = sum(int(row["estimated_count"]) for row in rows)
    total_common = sum(int(row["common_count"]) for row in rows)
    total_valid = sum(int(row["valid_common_count"]) for row in rows)
    total = {
        "scene": "__TOTAL__",
        "ground_truth_count": total_gt,
        "estimated_count": total_estimated,
        "common_count": total_common,
        "valid_common_count": total_valid,
        "estimated_coverage": total_estimated / total_gt if total_gt else None,
        "common_coverage": total_common / total_gt if total_gt else None,
        "scale_est_to_gt": None,
        "position_rmse_m": None,
        "position_median_m": None,
        "position_mean_m": None,
        "position_max_m": None,
        "rotation_rmse_deg": None,
        "rotation_median_deg": None,
        "rotation_mean_deg": None,
        "rotation_max_deg": None,
        "run_exit_code": "",
        "ok": all(bool(row["ok"]) for row in rows) if rows else False,
        "error": "",
    }
    if position_errors.size:
        total.update(
            {
                "position_rmse_m": float(np.sqrt(np.mean(position_errors**2))),
                "position_median_m": float(np.median(position_errors)),
                "position_mean_m": float(np.mean(position_errors)),
                "position_max_m": float(np.max(position_errors)),
            }
        )
    if rotation_errors.size:
        total.update(
            {
                "rotation_rmse_deg": float(np.sqrt(np.mean(rotation_errors**2))),
                "rotation_median_deg": float(np.median(rotation_errors)),
                "rotation_mean_deg": float(np.mean(rotation_errors)),
                "rotation_max_deg": float(np.max(rotation_errors)),
            }
        )
    if not total["ok"]:
        total["error"] = "存在未完成 Sim(3) 对齐的场景"
    return total


def csv_cell(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if value is None:
        return ""
    return value


def write_csv(path: Path, rows: Iterable[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {field: csv_cell(row.get(field, "")) for field in fieldnames}
            )


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="逐场景 Sim(3) 对齐 ETH3D 相机位姿，生成汇总和逐图像 CSV"
    )
    parser.add_argument("--poses", type=Path, help="wai_eth3d 格式的批量或单场景 poses.json")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        help="prepared ETH3D 数据目录；直接从 scenes/*/gt 和 raw/*/dslr_calibration_jpg/images.txt 读取 GT",
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        help="项目输出目录；读取 prj/<scene>/poses.json 或 work 下的 SfM 位姿。默认与 dataset-root 相同以兼容旧结果",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="CSV 输出路径（默认与输入同目录的 pose_evaluation.csv）",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        help="JSON 输出路径（默认与输入同目录的 pose_evaluation.json）",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="位姿 CSV 输出目录（默认与输入文件或项目目录同目录）",
    )
    parser.add_argument(
        "--ground-truth-csv",
        type=Path,
        default=None,
        help="真值位姿 CSV 路径",
    )
    parser.add_argument(
        "--estimated-csv",
        type=Path,
        default=None,
        help="估计位姿 CSV 路径",
    )
    parser.add_argument(
        "--comparison-csv",
        type=Path,
        default=None,
        help="逐图像对比 CSV 路径",
    )
    parser.add_argument(
        "--csv-only",
        action="store_true",
        help="只写 CSV，不生成 pose_evaluation.json",
    )
    args = parser.parse_args()

    if args.poses and args.dataset_root:
        parser.error("--poses 和 --dataset-root 只能指定一个")
    if not args.poses and not args.dataset_root:
        parser.error("必须指定 --poses 或 --dataset-root")

    poses_path: Path | None = None
    if args.poses:
        poses_path = args.poses.expanduser().resolve()
        if not poses_path.is_file():
            print(f"错误: 位姿文件不存在: {poses_path}", file=sys.stderr)
            return 2
        try:
            bundles = load_scene_bundles(read_json(poses_path))
        except (OSError, ValueError, json.JSONDecodeError) as error:
            print(f"错误: 无法读取位姿集合: {error}", file=sys.stderr)
            return 2
        default_output_dir = poses_path.parent
    else:
        dataset_root = args.dataset_root.expanduser().resolve()
        if not (dataset_root / "scenes").is_dir():
            print(f"错误: 缺少数据场景目录: {dataset_root / 'scenes'}", file=sys.stderr)
            return 2
        project_root = (
            args.project_root.expanduser().resolve() if args.project_root else dataset_root
        )
        try:
            bundles = load_eth3d_bundles(dataset_root, project_root)
        except OSError as error:
            print(f"错误: 无法读取 ETH3D 位姿: {error}", file=sys.stderr)
            return 2
        default_output_dir = project_root / "evaluation"

    rows: list[dict[str, Any]] = []
    residuals: list[dict[str, Any]] = []
    for bundle in bundles:
        row, values = evaluate_scene(bundle)
        rows.append(row)
        residuals.append(values)
        status = "ok" if row["ok"] else "failed"
        print(
            f"[pose-eval] {row['scene']}: {status}, "
            f"common={row['common_count']}/{row['ground_truth_count']}, "
            f"valid={row['valid_common_count']}, "
            f"pos_rmse={row['position_rmse_m'] if row['position_rmse_m'] is not None else 'n/a'}m, "
            f"rot_rmse={row['rotation_rmse_deg'] if row['rotation_rmse_deg'] is not None else 'n/a'}deg"
        )

    total = aggregate_rows(rows, residuals)
    ground_truth_rows, estimated_rows = flatten_pose_rows(bundles)
    comparison_rows = [
        comparison
        for values in residuals
        for comparison in values["comparison_rows"]
    ]
    output_dir = (
        args.output_dir
        or (args.csv.parent if args.csv is not None else default_output_dir)
    ).expanduser().resolve()
    output_csv = (args.csv or output_dir / "pose_evaluation.csv").expanduser().resolve()
    output_json = (args.json or output_dir / "pose_evaluation.json").expanduser().resolve()
    ground_truth_csv = (
        args.ground_truth_csv or output_dir / "ground_truth_poses.csv"
    ).expanduser().resolve()
    estimated_csv = (
        args.estimated_csv or output_dir / "estimated_poses.csv"
    ).expanduser().resolve()
    comparison_csv = (
        args.comparison_csv or output_dir / "pose_comparison.csv"
    ).expanduser().resolve()
    write_csv(output_csv, [*rows, total], CSV_FIELDS)
    write_csv(ground_truth_csv, ground_truth_rows, POSE_CSV_FIELDS)
    write_csv(estimated_csv, estimated_rows, POSE_CSV_FIELDS)
    write_csv(comparison_csv, comparison_rows, COMPARISON_CSV_FIELDS)
    print(f"[pose-eval] CSV: {output_csv}")
    print(f"[pose-eval] GT poses CSV: {ground_truth_csv}")
    print(f"[pose-eval] estimated poses CSV: {estimated_csv}")
    print(f"[pose-eval] comparison CSV: {comparison_csv}")
    if not args.csv_only:
        write_json(
            output_json,
            {
                "schema": "insightat_eth3d_pose_evaluation_v1",
                "generated_at": datetime.now(timezone.utc).isoformat(),
                "input": str(poses_path) if poses_path is not None else "ETH3D dataset/project roots",
                "alignment": {
                    "direction": "estimated_camera_center -> ground_truth_camera_center",
                    "transform": "C_gt ~= scale * R_align @ C_est + t",
                    "position_unit": "ground-truth unit (ETH3D metric when applicable)",
                    "rotation_unit": "degree",
                },
                "csv_outputs": {
                    "summary": str(output_csv),
                    "ground_truth_poses": str(ground_truth_csv),
                    "estimated_poses": str(estimated_csv),
                    "comparison": str(comparison_csv),
                },
                "scenes": rows,
                "overall": total,
            },
        )
        print(f"[pose-eval] JSON: {output_json}")
    return 0 if total["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
