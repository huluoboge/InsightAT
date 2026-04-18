"""
Parse COLMAP sparse text models (images.txt) for camera centers and names.
Convention matches COLMAP / InsightAT export: X_cam = R * X_world + t, C = -R^T @ t.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator

import numpy as np


@dataclass(frozen=True)
class CameraPose:
    """World-to-camera rotation R_wc and camera center C in world coordinates."""

    name: str
    R_wc: np.ndarray  # 3x3
    C: np.ndarray  # 3,


def _quat_wxyz_to_R(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Unit quaternion (w,x,y,z) to rotation matrix (world to camera)."""
    n = qw * qw + qx * qx + qy * qy + qz * qz
    if n < 1e-20:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * qw * qx, s * qw * qy, s * qw * qz
    xx, xy, xz = s * qx * qx, s * qx * qy, s * qx * qz
    yy, yz, zz = s * qy * qy, s * qy * qz, s * qz * qz
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def iter_cameras_from_images_txt(path: Path) -> Iterator[CameraPose]:
    """
    Yield one CameraPose per image block in COLMAP images.txt.
    NAME may be basename only; caller often matches on Path(name).name.
    """
    text = path.read_text(encoding="utf-8", errors="replace").splitlines()
    i = 0
    while i < len(text):
        line = text[i].strip()
        if not line or line.startswith("#"):
            i += 1
            continue
        parts = line.split()
        if len(parts) < 10:
            i += 1
            continue
        try:
            _img_id = int(parts[0])
            qw, qx, qy, qz = map(float, parts[1:5])
            tx, ty, tz = map(float, parts[5:8])
            _cam_id = int(parts[8])
            name = parts[9]
        except (ValueError, IndexError):
            i += 1
            continue
        R = _quat_wxyz_to_R(qw, qx, qy, qz)
        t = np.array([tx, ty, tz], dtype=np.float64)
        C = -R.T @ t
        yield CameraPose(name=name, R_wc=R, C=C)
        i += 2  # skip POINTS2D line


def load_cameras_by_basename(images_txt: Path) -> Dict[str, CameraPose]:
    """Map basename(NAME) -> CameraPose (last wins if duplicates)."""
    out: Dict[str, CameraPose] = {}
    for cam in iter_cameras_from_images_txt(images_txt):
        key = Path(cam.name).name
        out[key] = cam
    return out


def count_points3d(points3d_txt: Path) -> int:
    """Count non-comment data lines in points3D.txt."""
    if not points3d_txt.is_file():
        return 0
    n = 0
    for line in points3d_txt.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        n += 1
    return n
