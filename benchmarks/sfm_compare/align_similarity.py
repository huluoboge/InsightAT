"""
Similarity transform (Umeyama) between two sets of corresponding 3D points.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class Similarity:
    scale: float
    R: np.ndarray  # 3x3
    t: np.ndarray  # 3


def umeyama(src: np.ndarray, dst: np.ndarray) -> Similarity:
    """
    Least-squares similarity mapping src -> dst:
        dst ~ scale * R @ src + t
    src, dst: (N, 3)
    """
    assert src.shape == dst.shape and src.ndim == 2 and src.shape[1] == 3
    n = src.shape[0]
    if n < 3:
        raise ValueError("Need at least 3 correspondences for similarity")

    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    xs = src - mu_s
    xd = dst - mu_d
    # Cross-covariance (Umeyama / Horn)
    H = (xs.T @ xd) / n
    U, D, Vt = np.linalg.svd(H)
    Sfix = np.eye(3)
    if np.linalg.det(Vt.T @ U.T) < 0:
        Sfix[2, 2] = -1.0
    R = Vt.T @ Sfix @ U.T
    var_src = np.sum(xs * xs) / n
    scale = float(np.sum(D * np.diag(Sfix)) / var_src if var_src > 1e-20 else 1.0)
    t = mu_d - scale * R @ mu_s
    return Similarity(scale=scale, R=R, t=t)


def apply_similarity(S: Similarity, pts: np.ndarray) -> np.ndarray:
    return (S.scale * (S.R @ pts.T).T) + S.t


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Angle between two rotation matrices (degrees)."""
    R = R_a.T @ R_b
    tr = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(tr)))


def evaluate_alignment(src_C: np.ndarray, dst_C: np.ndarray) -> dict:
    """
    Fit similarity src -> dst using corresponding camera centers (meters).
    src_C, dst_C: (N,3) same row order.
    """
    S = umeyama(src_C, dst_C)
    pred = apply_similarity(S, src_C)
    err = np.linalg.norm(pred - dst_C, axis=1)
    return {
        "n": int(src_C.shape[0]),
        "scale": S.scale,
        "rmse_m": float(np.sqrt(np.mean(err**2))),
        "median_m": float(np.median(err)),
        "mean_m": float(np.mean(err)),
        "max_m": float(np.max(err)),
    }
