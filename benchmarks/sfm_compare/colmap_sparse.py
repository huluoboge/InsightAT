"""
Parse COLMAP sparse text models (images.txt) for camera centers and names.
Convention matches COLMAP / InsightAT export: X_cam = R * X_world + t, C = -R^T @ t.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator

import numpy as np

# Flat `images/` folders (COLMAP / benchmark layout): count only common raster extensions.
_IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp", ".webp"})


def count_images_in_dir(directory: Path) -> int:
    """Count image files directly under ``directory`` (non-recursive)."""
    if not directory.is_dir():
        return 0
    n = 0
    for p in directory.iterdir():
        if p.is_file() and p.suffix.lower() in _IMAGE_SUFFIXES:
            n += 1
    return n


def count_images_in_tree(directory: Path) -> int:
    """Count image files under ``directory`` (recursive), same extensions as flat layout."""
    if not directory.is_dir():
        return 0
    n = 0
    for p in directory.rglob("*"):
        if p.is_file() and p.suffix.lower() in _IMAGE_SUFFIXES:
            n += 1
    return n


_RE_COLMAP_NUM_IMAGES = re.compile(r"^#\s*Number of images:\s*(\d+)\s*$")


def count_bundler_list_paths(list_txt: Path) -> int:
    """Bundler ``list.txt``: one registered image path per non-empty, non-comment line."""
    if not list_txt.is_file():
        return 0
    n = 0
    for line in list_txt.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        n += 1
    return n


def count_images_in_images_all_json(images_all_json: Path) -> int:
    """
    InsightAT ``work/images_all.json``: same rule as ``isat_sfm`` / ``isat_retrieval_match``:
    length of the top-level ``images`` array.
    """
    if not images_all_json.is_file():
        return 0
    try:
        data = json.loads(images_all_json.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return 0
    if not isinstance(data, dict):
        return 0
    images = data.get("images")
    if isinstance(images, list):
        return len(images)
    return 0


def count_colmap_images_txt_cameras(images_txt: Path) -> int:
    """
    Number of cameras in COLMAP ``images.txt`` without loading the whole file.

    Prefer the ``# Number of images: N`` header when present (second line can be huge).
    Otherwise stream-parse image header lines and skip each following POINTS2D line.
    """
    if not images_txt.is_file():
        return 0
    n = 0
    with images_txt.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                m = _RE_COLMAP_NUM_IMAGES.match(line.rstrip("\n"))
                if m:
                    return int(m.group(1))
                continue
            s = line.strip()
            if not s:
                continue
            parts = s.split()
            if len(parts) < 10:
                continue
            try:
                int(parts[0])
                float(parts[1])
                float(parts[2])
                float(parts[3])
                float(parts[4])
            except (ValueError, IndexError):
                continue
            n += 1
            f.readline()
    return n


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
