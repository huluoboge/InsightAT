#!/usr/bin/env python3
"""Create the common pose-evaluator input for a DTU scan and InsightAT model.

DTU SampleSet's ``pos_XXX.txt`` files store 3x4 camera projection matrices.
The input image IDs come from ``work/images_all.json``; InsightAT's COLMAP
export renames images to zero-padded IDs, so this maps poses back to the source
image basenames before evaluating them with ``evaluate_wai_eth3d_poses.py``.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import numpy as np


def projection_to_c2w(projection: np.ndarray) -> np.ndarray:
    """RQ-decompose a pinhole 3x4 projection matrix into camera-to-world."""
    if projection.shape != (3, 4):
        raise ValueError(f"expected a 3x4 projection matrix, got {projection.shape}")
    q, r = np.linalg.qr(np.flipud(projection[:, :3]).T)
    intrinsics = np.fliplr(np.flipud(r.T))
    world_to_camera = np.flipud(q.T)
    signs = np.diag(np.where(np.diag(intrinsics) >= 0.0, 1.0, -1.0))
    intrinsics = intrinsics @ signs
    world_to_camera = signs @ world_to_camera
    intrinsics /= intrinsics[2, 2]
    translation = np.linalg.solve(intrinsics, projection[:, 3])
    center_m = -(world_to_camera.T @ translation) / 1000.0  # DTU coordinates are mm.

    c2w = np.eye(4, dtype=np.float64)
    c2w[:3, :3] = world_to_camera.T
    c2w[:3, 3] = center_m
    return c2w


def read_estimated_poses(model_images: Path, image_by_id: dict[int, str]) -> list[dict[str, Any]]:
    """Read COLMAP images.txt, resolving output IDs to original image names."""
    def quaternion_rotation(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
        return np.asarray([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
             2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
             2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
             1 - 2 * (qx * qx + qy * qy)],
        ], dtype=np.float64)

    lines = model_images.read_text(encoding="utf-8", errors="replace").splitlines()
    poses: list[dict[str, Any]] = []
    index = 0
    while index < len(lines):
        fields = lines[index].split()
        index += 1
        if len(fields) < 10:
            continue
        try:
            image_id = int(fields[0])
            qw, qx, qy, qz = map(float, fields[1:5])
            translation = np.asarray(list(map(float, fields[5:8])), dtype=np.float64)
            camera_id = int(fields[8])
        except (ValueError, IndexError):
            continue

        rotation = quaternion_rotation(qw, qx, qy, qz)
        center = -rotation.T @ translation
        c2w = np.eye(4, dtype=np.float64)
        c2w[:3, :3] = rotation.T
        c2w[:3, 3] = center
        poses.append({
            "image_name": image_by_id.get(image_id, Path(fields[9]).name),
            "image_index": image_id,
            "camera_index": camera_id,
            "R_world_to_camera": rotation.tolist(),
            "camera_center": center.tolist(),
            "camera_to_world": c2w.tolist(),
            "intrinsics": {},
        })
        index += 1  # COLMAP stores observations on the next line.
    return poses


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dtu-root", type=Path, required=True,
                        help="SampleSet directory containing MVS Data/")
    parser.add_argument("--work-dir", type=Path, required=True,
                        help="InsightAT work directory containing images_all.json")
    parser.add_argument("--model-images", type=Path, required=True,
                        help="InsightAT COLMAP sparse model images.txt")
    parser.add_argument("--scan", type=int, required=True, help="DTU scan number")
    parser.add_argument("--output", type=Path, required=True, help="Output pose-bundle JSON")
    args = parser.parse_args()

    work_dir = args.work_dir.resolve()
    dataset_root = args.dtu_root.resolve()
    images_all = json.loads((work_dir / "images_all.json").read_text(encoding="utf-8"))
    image_by_id = {
        int(image["id"]): Path(image["path"]).name
        for image in images_all.get("images", [])
        if isinstance(image, dict) and "id" in image and "path" in image
    }

    projection_dir = dataset_root / "MVS Data" / "Calibration" / "cal18"
    gt_poses: list[dict[str, Any]] = []
    for image_id, image_name in sorted(image_by_id.items()):
        match = re.fullmatch(r"rect_(\d+)_3_r5000\.png", image_name)
        if not match:
            continue
        position = int(match.group(1))
        projection_path = projection_dir / f"pos_{position:03d}.txt"
        if not projection_path.is_file():
            continue
        c2w = projection_to_c2w(np.loadtxt(projection_path, dtype=np.float64))
        gt_poses.append({"image_name": image_name, "image_index": image_id,
                         "camera_to_world": c2w.tolist()})

    estimate = read_estimated_poses(args.model_images, image_by_id)
    bundle = {
        "scene": f"DTU_scan{args.scan:03d}",
        "run_exit_code": 0,
        "ground_truth": gt_poses,
        "estimated": estimate,
        "metadata": {"ground_truth_source": "DTU SampleSet Calibration/cal18/pos_XXX.txt",
                     "input_images": len(image_by_id),
                     "gt_poses": len(gt_poses),
                     "estimated_poses": len(estimate)},
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(bundle, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(bundle["metadata"], indent=2))
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
