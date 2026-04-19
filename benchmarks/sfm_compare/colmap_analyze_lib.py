"""
Load COLMAP sparse **text** models and compute benchmark metrics (JSON-serializable).

Requires ``cameras.txt``, ``images.txt``, ``points3D.txt`` under a directory (e.g. ``sparse/0``).
For binary-only models, run ``colmap model_converter --output_type TXT`` first.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# ── data structures ──────────────────────────────────────────────────────────


@dataclass
class ColmapCamera:
    camera_id: int
    model: str
    width: int
    height: int
    params: List[float]


@dataclass
class ColmapImage:
    image_id: int
    qvec: Tuple[float, float, float, float]
    tvec: np.ndarray  # 3,
    camera_id: int
    name: str
    points2d: List[Tuple[float, float, int]]  # x, y, point3d_id (-1 if none)


@dataclass
class ColmapPoint3D:
    point_id: int
    xyz: np.ndarray  # 3,
    rgb: Tuple[int, int, int]
    error: float  # mean reprojection error stored by COLMAP (px)
    track: List[Tuple[int, int]]  # (image_id, point2d_idx)


@dataclass
class ColmapTextModel:
    cameras: Dict[int, ColmapCamera] = field(default_factory=dict)
    images: Dict[int, ColmapImage] = field(default_factory=dict)
    points3d: Dict[int, ColmapPoint3D] = field(default_factory=dict)


# ── quaternion / rotation (COLMAP: world to camera) ──────────────────────────


def _quat_wxyz_to_R(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    n = qw * qw + qx * qx + qy * qy + qz * qz
    if n < 1e-20:
        return np.eye(3, dtype=np.float64)
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


# ── projection (aligned with COLMAP / OpenCV radial-tan) ───────────────────────


def _distort_opencv(xn: float, yn: float, k1: float, k2: float, k3: float, p1: float, p2: float) -> Tuple[float, float]:
    """COLMAP ``OPENCV`` (8 extra params): radial k1,k2 only; k3 in signature kept 0 for OPENCV."""
    r2 = xn * xn + yn * yn
    r4 = r2 * r2
    r6 = r4 * r2
    rad = 1 + k1 * r2 + k2 * r4 + k3 * r6
    xd = xn * rad + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
    yd = yn * rad + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
    return xd, yd


def _distort_full_opencv_colmap(
    xn: float,
    yn: float,
    k1: float,
    k2: float,
    p1: float,
    p2: float,
    k3: float,
    k4: float,
    k5: float,
    k6: float,
) -> Tuple[float, float]:
    """COLMAP ``FULL_OPENCV`` — same rational radial as colmap/sensor/models.h."""
    u2 = xn * xn
    v2 = yn * yn
    uv = xn * yn
    r2 = u2 + v2
    r4 = r2 * r2
    r6 = r4 * r2
    radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) / (1.0 + k4 * r2 + k5 * r4 + k6 * r6)
    xd = xn * radial + 2.0 * p1 * uv + p2 * (r2 + 2.0 * xn * xn)
    yd = yn * radial + 2.0 * p2 * uv + p1 * (r2 + 2.0 * yn * yn)
    return xd, yd


def _distort_simple_radial(xn: float, yn: float, k: float) -> Tuple[float, float]:
    r2 = xn * xn + yn * yn
    s = 1.0 + k * r2
    return xn * s, yn * s


def _distort_radial(xn: float, yn: float, k1: float, k2: float) -> Tuple[float, float]:
    r2 = xn * xn + yn * yn
    r4 = r2 * r2
    s = 1.0 + k1 * r2 + k2 * r4
    return xn * s, yn * s


def project_world_to_pixel(
    Xw: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    cam: ColmapCamera,
) -> Optional[Tuple[float, float]]:
    """Project world point; return pixel (u,v) or None if behind camera."""
    Xc = R @ Xw + t
    z = float(Xc[2])
    if z <= 0:
        return None
    xn = float(Xc[0] / z)
    yn = float(Xc[1] / z)
    model = cam.model.upper()
    p = cam.params

    if model == "SIMPLE_PINHOLE":
        f, cx, cy = p[0], p[1], p[2]
        return f * xn + cx, f * yn + cy
    if model == "PINHOLE":
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        return fx * xn + cx, fy * yn + cy
    if model == "SIMPLE_RADIAL":
        f, cx, cy, k = p[0], p[1], p[2], p[3]
        xd, yd = _distort_simple_radial(xn, yn, k)
        return f * xd + cx, f * yd + cy
    if model == "RADIAL":
        f, cx, cy, k1, k2 = p[0], p[1], p[2], p[3], p[4]
        xd, yd = _distort_radial(xn, yn, k1, k2)
        return f * xd + cx, f * yd + cy
    if model == "OPENCV":
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        k1, k2, p1, p2 = p[4], p[5], p[6], p[7]
        xd, yd = _distort_opencv(xn, yn, k1, k2, 0.0, p1, p2)
        return fx * xd + cx, fy * yd + cy
    if model == "FULL_OPENCV" and len(p) >= 12:
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        k1, k2, p1, p2, k3, k4, k5, k6 = p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11]
        xd, yd = _distort_full_opencv_colmap(xn, yn, k1, k2, p1, p2, k3, k4, k5, k6)
        return fx * xd + cx, fy * yd + cy

    return None


def camera_models_supported_for_reproj() -> Tuple[str, ...]:
    """Camera models implemented for optional per-observation reprojection."""
    return ("SIMPLE_PINHOLE", "PINHOLE", "SIMPLE_RADIAL", "RADIAL", "OPENCV", "FULL_OPENCV")


# ── text parsers ─────────────────────────────────────────────────────────────


def _parse_cameras_txt(path: Path) -> Dict[int, ColmapCamera]:
    out: Dict[int, ColmapCamera] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            cid = int(parts[0])
            model = parts[1]
            w, h = int(parts[2]), int(parts[3])
            params = [float(x) for x in parts[4:]]
        except (ValueError, IndexError):
            continue
        out[cid] = ColmapCamera(camera_id=cid, model=model, width=w, height=h, params=params)
    return out


def _parse_images_txt(path: Path) -> Dict[int, ColmapImage]:
    text = path.read_text(encoding="utf-8", errors="replace").splitlines()
    out: Dict[int, ColmapImage] = {}
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
            img_id = int(parts[0])
            qw, qx, qy, qz = map(float, parts[1:5])
            tx, ty, tz = map(float, parts[5:8])
            cam_id = int(parts[8])
            name = parts[9]
            tvec = np.array([tx, ty, tz], dtype=np.float64)
        except (ValueError, IndexError):
            i += 1
            continue
        i += 1
        pts2d: List[Tuple[float, float, int]] = []
        if i < len(text):
            pl = text[i].strip()
            if pl and not pl.startswith("#"):
                pp = pl.split()
                for k in range(0, len(pp), 3):
                    if k + 2 >= len(pp):
                        break
                    try:
                        x, y = float(pp[k]), float(pp[k + 1])
                        pid = int(pp[k + 2])
                        pts2d.append((x, y, pid))
                    except (ValueError, IndexError):
                        break
                i += 1
        out[img_id] = ColmapImage(
            image_id=img_id,
            qvec=(qw, qx, qy, qz),
            tvec=tvec,
            camera_id=cam_id,
            name=name,
            points2d=pts2d,
        )
    return out


def _parse_points3d_txt(path: Path) -> Dict[int, ColmapPoint3D]:
    out: Dict[int, ColmapPoint3D] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        try:
            pid = int(parts[0])
            xyz = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=np.float64)
            r, g, b = int(parts[4]), int(parts[5]), int(parts[6])
            err = float(parts[7])
            track: List[Tuple[int, int]] = []
            for k in range(8, len(parts), 2):
                if k + 1 >= len(parts):
                    break
                track.append((int(parts[k]), int(parts[k + 1])))
        except (ValueError, IndexError):
            continue
        out[pid] = ColmapPoint3D(point_id=pid, xyz=xyz, rgb=(r, g, b), error=err, track=track)
    return out


def load_colmap_text_model(sparse_dir: Path) -> ColmapTextModel:
    """Load ``sparse_dir`` containing cameras.txt, images.txt, points3D.txt."""
    sparse_dir = sparse_dir.resolve()
    cams = _parse_cameras_txt(sparse_dir / "cameras.txt")
    imgs = _parse_images_txt(sparse_dir / "images.txt")
    pts = _parse_points3d_txt(sparse_dir / "points3D.txt")
    return ColmapTextModel(cameras=cams, images=imgs, points3d=pts)


# ── metrics ──────────────────────────────────────────────────────────────────


def _pct(lst: Sequence[float], p: float) -> float:
    if not lst:
        return float("nan")
    s = sorted(lst)
    idx = max(0, min(int(len(s) * p / 100.0), len(s) - 1))
    return float(s[idx])


def _hist(data: List[float], edges: List[float], labels: List[str]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for i, label in enumerate(labels):
        lo = edges[i]
        hi = edges[i + 1] if i + 1 < len(edges) else float("inf")
        cnt = sum(1 for x in data if lo <= x < hi)
        out.append({"label": label, "lo": lo, "count": cnt})
    return out


def compute_colmap_metrics(
    model: ColmapTextModel,
    sparse_dir: Path,
    *,
    recompute_per_observation: bool = True,
) -> Dict[str, Any]:
    """
    Track statistics, per-image observation counts, and reprojection metrics.

    **Default:** ``recompute_per_observation=True`` — project every track observation using
    ``cameras.txt`` in **OpenCV / COLMAP** tangential order (must match exported COLMAP files).

    **If** ``recompute_per_observation=False`` — skip projection and use only the ``ERROR``
    column in ``points3D.txt`` for aggregate stats (no per-observation residuals).
    """
    sparse_dir_s = str(sparse_dir.resolve())
    cams = model.cameras
    imgs = model.images
    pts3 = model.points3d

    supported = set(camera_models_supported_for_reproj())
    unsupported = {c.model.upper() for c in cams.values() if c.model.upper() not in supported}
    reproj_ok = recompute_per_observation and len(unsupported) == 0 and len(cams) > 0

    track_lengths: List[int] = []
    point_errors: List[float] = []
    # For each image_id: COLMAP ERROR of every 3D point that observes this image (one entry per point)
    point_err_visible_by_image: Dict[int, List[float]] = {im_id: [] for im_id in imgs.keys()}
    n_obs_track_total = 0

    for p in pts3.values():
        point_errors.append(p.error)
        tl = len(p.track)
        n_obs_track_total += tl
        if tl >= 2:
            track_lengths.append(tl)
        vis_im: set[int] = set()
        for im_id, _p2 in p.track:
            vis_im.add(im_id)
        for im_id in vis_im:
            if im_id in point_err_visible_by_image:
                point_err_visible_by_image[im_id].append(p.error)

    errors: List[float] = []
    errors_per_image: Dict[int, List[float]] = {}
    n_track_mismatch = 0

    if reproj_ok:
        for p in pts3.values():
            Xw = p.xyz
            for im_id, p2d_idx in p.track:
                im = imgs.get(im_id)
                if im is None:
                    n_track_mismatch += 1
                    continue
                cam = cams.get(im.camera_id)
                if cam is None:
                    n_track_mismatch += 1
                    continue
                if p2d_idx < 0 or p2d_idx >= len(im.points2d):
                    n_track_mismatch += 1
                    continue
                u_obs, v_obs, pid = im.points2d[p2d_idx]
                if pid != p.point_id:
                    n_track_mismatch += 1
                    continue
                R = _quat_wxyz_to_R(*im.qvec)
                uv = project_world_to_pixel(Xw, R, im.tvec, cam)
                if uv is None:
                    continue
                err = math.hypot(uv[0] - u_obs, uv[1] - v_obs)
                errors.append(err)
                errors_per_image.setdefault(im_id, []).append(err)

    points2d_with_3d: Dict[int, int] = {}
    for im in imgs.values():
        points2d_with_3d[im.image_id] = sum(1 for _, _, pid in im.points2d if pid >= 0)

    mean_tl = sum(track_lengths) / len(track_lengths) if track_lengths else 0.0
    med_tl = _pct([float(x) for x in track_lengths], 50) if track_lengths else float("nan")

    len_hist = _hist(
        [float(x) for x in track_lengths],
        [2, 3, 4, 5, 7, 10, 15, 20, 30, 50, 1e9],
        ["2", "3", "4", "5", "6–7", "8–10", "11–15", "16–20", "21–30", "31–50", "≥50"],
    )

    point_err_hist = _hist(
        point_errors,
        [0, 0.25, 0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 1e9],
        ["0–0.25", "0.25–0.5", "0.5–1", "1–2", "2–3", "3–5", "5–10", "10–20", "≥20"],
    )

    points_all_zero = bool(point_errors) and max(point_errors) <= 0.0 and any(e == 0.0 for e in point_errors)

    reprojection_from_file: Dict[str, Any] = {
        "source": "points3D.txt_ERROR_column",
        "description": "Per-point mean reprojection error (px) as stored by COLMAP / exporters",
        "n_points": len(point_errors),
        "mean": sum(point_errors) / len(point_errors) if point_errors else float("nan"),
        "median": _pct(point_errors, 50) if point_errors else float("nan"),
        "p90": _pct(point_errors, 90) if point_errors else float("nan"),
        "p99": _pct(point_errors, 99) if point_errors else float("nan"),
        "lt1px_pct_points": sum(1 for e in point_errors if e < 1.0) / len(point_errors) * 100.0
        if point_errors
        else float("nan"),
        "hist": point_err_hist,
        "warning_all_zero": points_all_zero,
    }

    reprojection_recomputed: Optional[Dict[str, Any]] = None
    if recompute_per_observation:
        reprojection_recomputed = {
            "available": bool(errors),
            "unsupported_camera_models": sorted(unsupported),
            "n_observations_computed": len(errors),
            "n_track_consistency_warnings": n_track_mismatch,
        }
        if errors:
            reprojection_recomputed.update(
                {
                    "mean": sum(errors) / len(errors),
                    "median": _pct(errors, 50),
                    "p90": _pct(errors, 90),
                    "p99": _pct(errors, 99),
                    "lt1px": sum(1 for e in errors if e < 1.0) / len(errors) * 100.0,
                    "lt2px": sum(1 for e in errors if e < 2.0) / len(errors) * 100.0,
                    "gt10px": sum(1 for e in errors if e > 10.0) / len(errors) * 100.0,
                    "per_image_median": {str(k): _pct(v, 50) for k, v in sorted(errors_per_image.items())},
                    "err_hist": _hist(
                        errors,
                        [0, 0.5, 1, 2, 3, 5, 10, 20, 50, 1e9],
                        ["0–0.5", "0.5–1", "1–2", "2–3", "3–5", "5–10", "10–20", "20–50", "≥50"],
                    ),
                }
            )
        elif unsupported:
            reprojection_recomputed["note"] = (
                "Skipped: unsupported camera model(s) for Python projector — use points3D ERROR only"
            )

    per_image_rows: List[Dict[str, Any]] = []
    for im_id in sorted(imgs.keys()):
        im = imgs[im_id]
        vis_list = point_err_visible_by_image.get(im_id, [])
        med_file = _pct(vis_list, 50) if vis_list else None
        row: Dict[str, Any] = {
            "image_id": im_id,
            "name": Path(im.name).name,
            "n_observations_points2d": points2d_with_3d.get(im_id, 0),
            "median_reproj_from_points3d_px": med_file,
        }
        if errors_per_image.get(im_id):
            row["median_reproj_per_observation_px"] = _pct(errors_per_image[im_id], 50)
        per_image_rows.append(row)

    summary = {
        "n_cameras_intrinsics": len(cams),
        "n_images_registered": len(imgs),
        "n_points3d": len(pts3),
        "n_observations_in_tracks": n_obs_track_total,
        "mean_track_length": mean_tl,
        "median_track_length": med_tl,
        "median_reproj_error_points3d_px": reprojection_from_file["median"],
    }

    tracks_block = {
        "n_points_with_track_len_ge_2": len(track_lengths),
        "mean_length": mean_tl,
        "median_length": med_tl,
        "len_hist": len_hist,
    }

    out: Dict[str, Any] = {
        "schema_version": 2,
        "sparse_dir": sparse_dir_s,
        "summary": summary,
        "tracks": tracks_block,
        "reprojection": reprojection_from_file,
        "per_image": per_image_rows,
    }
    if reprojection_recomputed is not None:
        out["reprojection_recomputed"] = reprojection_recomputed
    return out


def analyze_sparse_directory(
    sparse_dir: Path,
    *,
    recompute_per_observation: bool = True,
) -> Dict[str, Any]:
    """Load model from ``sparse_dir`` and return metrics dict."""
    sparse_dir = sparse_dir.resolve()
    if not (sparse_dir / "cameras.txt").is_file():
        raise FileNotFoundError(f"missing cameras.txt under {sparse_dir}")
    if not (sparse_dir / "images.txt").is_file():
        raise FileNotFoundError(f"missing images.txt under {sparse_dir}")
    if not (sparse_dir / "points3D.txt").is_file():
        raise FileNotFoundError(f"missing points3D.txt under {sparse_dir}")
    model = load_colmap_text_model(sparse_dir)
    return compute_colmap_metrics(model, sparse_dir, recompute_per_observation=recompute_per_observation)
