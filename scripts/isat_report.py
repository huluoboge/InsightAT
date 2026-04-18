#!/usr/bin/env python3
from __future__ import annotations
"""
InsightAT Pipeline HTML Report
─────────────────────────────────────────────────────────────────────────────
Reads all pipeline artefacts from a work directory and produces a single
self-contained dark-theme HTML report covering:

  • Summary — registered images, 3D points, median reprojection error
  • Feature extraction — per-image keypoint count histogram
  • Retrieval — pair count statistics
  • Matching — input match count distribution
  • Geometry verification — F/E/H inlier histograms (from report_geo.py style)
  • Track statistics — track length distribution, coverage
  • SfM statistics — reprojection error distribution, per-image quality table

Usage:
    python3 scripts/isat_report.py \\
        --work-dir /path/to/work \\
        --output   report.html

    # With explicit sfm output dir (if different from work-dir):
    python3 scripts/isat_report.py \\
        --work-dir  /work \\
        --sfm-dir   /work/incremental_sfm \\
        --output    report.html

    # With timing JSON (from isat_sfm):
    python3 scripts/isat_report.py \\
        --work-dir  /work \\
        --timing-json /work/timing.json \\
        --output    report.html

Expected layout in --work-dir:
    images.json          (or images_all.json)
    pairs.json
    feat/                *.isat_feat
    match/               *.isat_match
    geo/                 *.isat_geo
    tracks.isat_tracks   (also searched in --sfm-dir)
    poses.json           (also searched in --sfm-dir)

No external Python dependencies — stdlib + struct + json only.
"""

import argparse
import datetime
import json
import math
import re
import struct
import sys
from pathlib import Path


# ─────────────────────────────────────────────────────────────────────────────
# IDC reader
# ─────────────────────────────────────────────────────────────────────────────

_DTYPE_SIZES = {"float32": 4, "float64": 8, "uint8": 1, "uint16": 2, "uint32": 4, "uint64": 8, "int32": 4, "int64": 8}
_STRUCT_FMTS = {"float32": "f", "float64": "d", "uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q", "int32": "i", "int64": "q"}


def _read_idc_header(path: str) -> dict:
    """Read only the IDC JSON header (fast, no payload decode)."""
    with open(path, "rb") as f:
        if f.read(4) != b"ISAT":
            raise ValueError(f"{path}: not an IDC file")
        f.read(4)  # version
        jsz = struct.unpack("<Q", f.read(8))[0]
        return json.loads(f.read(jsz).decode("utf-8"))


def _read_idc_blobs(path: str, want: set[str]) -> dict[str, list]:
    """Read selected blobs from an IDC file. Returns {name: flat_list}."""
    with open(path, "rb") as f:
        if f.read(4) != b"ISAT":
            raise ValueError(f"{path}: not an IDC file")
        f.read(4)
        jsz = struct.unpack("<Q", f.read(8))[0]
        hdr = json.loads(f.read(jsz).decode("utf-8"))
        pos = 16 + jsz
        pad = (8 - pos % 8) % 8
        f.read(pad)
        payload = f.read()

    result: dict[str, list] = {}
    for b in hdr.get("blobs", []):
        if b["name"] not in want:
            continue
        dtype = b["dtype"]
        offset = b["offset"]
        size = b["size"]
        fmt = _STRUCT_FMTS.get(dtype, "f")
        n = size // _DTYPE_SIZES.get(dtype, 4)
        result[b["name"]] = list(struct.unpack_from(f"<{n}{fmt}", payload, offset))
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Data loading helpers
# ─────────────────────────────────────────────────────────────────────────────

def load_image_list(work_dir: Path) -> dict[int, str]:
    for name in ("images.json", "images_all.json", "images_extract.json"):
        p = work_dir / name
        if p.exists():
            data = json.loads(p.read_text())
            return {e["image_index"]: e["path"] for e in data["images"]}
    return {}


def load_feat_stats(feat_dir: Path) -> list[dict]:
    """Return list of {image_index, num_keypoints} sorted by image_index."""
    out = []
    for fp in sorted(feat_dir.glob("*.isat_feat")):
        try:
            hdr = _read_idc_header(str(fp))
        except Exception as e:
            print(f"  [warn] feat {fp.name}: {e}", file=sys.stderr)
            continue
        idx = hdr.get("image_index", int(fp.stem))
        # Keypoint count is in blobs[0]['shape'][0]; no top-level num_keypoints field
        blobs = hdr.get("blobs", [])
        kp_blob = next((b for b in blobs if b["name"] == "keypoints"), None)
        nkp = kp_blob["shape"][0] if kp_blob and kp_blob.get("shape") else hdr.get("num_keypoints", hdr.get("num_features", 0))
        out.append({"image_index": idx, "num_keypoints": nkp})
    return sorted(out, key=lambda x: x["image_index"])


def load_match_stats(match_dir: Path) -> list[dict]:
    """Return list of {i1, i2, num_matches}."""
    out = []
    pat = re.compile(r'^(\d+)_(\d+)\.isat_match$')
    for fp in sorted(match_dir.glob("*.isat_match")):
        m = pat.match(fp.name)
        if not m:
            continue
        try:
            hdr = _read_idc_header(str(fp))
        except Exception as e:
            print(f"  [warn] match {fp.name}: {e}", file=sys.stderr)
            continue
        # num_matches lives in metadata sub-dict
        nm = hdr.get("metadata", {}).get("num_matches", hdr.get("num_matches", 0))
        out.append({"i1": int(m.group(1)), "i2": int(m.group(2)), "num_matches": nm})
    return out


def load_geo_stats(geo_dir: Path) -> list[dict]:
    """Return list of geo pair dicts (same fields as report_geo.py)."""
    out = []
    for fp in sorted(geo_dir.glob("*.isat_geo")):
        try:
            hdr = _read_idc_header(str(fp))
        except Exception as e:
            print(f"  [warn] geo {fp.name}: {e}", file=sys.stderr)
            continue
        geo   = hdr.get("geometry", {})
        ip    = hdr.get("image_pair", {})
        F     = geo.get("F", {})
        E     = geo.get("E", {})
        H     = geo.get("H", {})
        degen = geo.get("degeneracy", {})
        out.append({
            "i1":      ip.get("image1_index", -1),
            "i2":      ip.get("image2_index", -1),
            "n":       hdr.get("num_matches_input", 0),
            "F_est":   F.get("estimated", False),
            "F_in":    F.get("num_inliers", 0),
            "F_ratio": F.get("inlier_ratio", 0.0),
            "E_est":   E.get("estimated", False),
            "E_in":    E.get("num_inliers", 0),
            "H_est":   H.get("estimated", False),
            "H_in":    H.get("num_inliers", 0),
            "degen":   degen.get("is_degenerate", False),
            "hf":      degen.get("h_over_f_ratio", 0.0),
        })
    return out


# ── track_flags bit constants (must match track_store.h) ────────────────────
_FLAG_ALIVE           = 0x01
_FLAG_NEEDS_RETRI     = 0x02
_FLAG_HAS_TRIANGULATED = 0x04
_FLAG_SKIP_FROM_BA    = 0x08


def load_tracks_idc(path: str) -> dict:
    """Load track IDC header + key blobs. Returns dict with stats + arrays."""
    hdr = _read_idc_header(path)
    blobs = _read_idc_blobs(path, {
        "track_xyz", "track_flags", "track_obs_offset",
        "obs_image_index", "obs_u", "obs_v", "obs_flags"
    })
    return {"header": hdr, **blobs}


def _track_is_alive(flag: int) -> bool:
    """True if track is alive (kAlive bit set)."""
    return bool(flag & _FLAG_ALIVE)


def _track_is_triangulated(flag: int, xyz: list, tid: int, is_sfm_result: bool) -> bool:
    """True if track is triangulated.

    Schema 1.2 (is_sfm_result=True): use kHasTriangulated bit.
    Older schemas: fall back to xyz != (0,0,0) check.
    """
    if is_sfm_result:
        return bool(flag & _FLAG_HAS_TRIANGULATED)
    xi = tid * 3
    if xi + 2 >= len(xyz):
        return False
    return not (xyz[xi] == 0.0 and xyz[xi+1] == 0.0 and xyz[xi+2] == 0.0)


def load_poses(path: str) -> dict:
    return json.loads(Path(path).read_text())


def _find_file(primary: Path, fallback, names: list) -> "Path | None":
    for name in names:
        p = primary / name
        if p.exists():
            return p
    if fallback:
        for name in names:
            p = fallback / name
            if p.exists():
                return p
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Reprojection error computation
# ─────────────────────────────────────────────────────────────────────────────

def _project(X, R9, C3, fx, fy, cx, cy, k1, k2, k3, p1, p2):
    """Project world point X through camera. Returns (u, v) or None."""
    Xc0 = R9[0]*X[0] + R9[1]*X[1] + R9[2]*X[2] - (R9[0]*C3[0]+R9[1]*C3[1]+R9[2]*C3[2])
    Xc1 = R9[3]*X[0] + R9[4]*X[1] + R9[5]*X[2] - (R9[3]*C3[0]+R9[4]*C3[1]+R9[5]*C3[2])
    Xc2 = R9[6]*X[0] + R9[7]*X[1] + R9[8]*X[2] - (R9[6]*C3[0]+R9[7]*C3[1]+R9[8]*C3[2])
    if Xc2 <= 0:
        return None
    xn = Xc0 / Xc2
    yn = Xc1 / Xc2
    r2 = xn*xn + yn*yn
    r4 = r2 * r2
    r6 = r4 * r2
    rad = 1 + k1*r2 + k2*r4 + k3*r6
    # C++ convention (resection.cpp): p1 in (r2+2xn²) / p2 in cross term for x;
    # opposite to OpenCV p1/p2 roles.
    xd = xn*rad + 2*p2*xn*yn + p1*(r2 + 2*xn*xn)
    yd = yn*rad + 2*p1*xn*yn + p2*(r2 + 2*yn*yn)
    return fx*xd + cx, fy*yd + cy


def compute_reproj_stats(tracks: dict, poses_data: dict) -> dict:
    """Compute per-observation reprojection errors. Returns stats dict."""
    # Build pose map: image_index -> (R9, C3, cam_intrinsics)
    cameras = poses_data["cameras"]
    img2cam  = poses_data.get("image_to_camera_index", [])
    pose_map: dict[int, tuple] = {}
    for p in poses_data["poses"]:
        ii = p["image_index"]
        # Prefer per-pose camera_index (new format), fall back to top-level array
        if "camera_index" in p:
            ci = p["camera_index"]
        elif img2cam and ii < len(img2cam):
            ci = img2cam[ii]
        else:
            ci = 0
        cam = cameras[ci] if ci < len(cameras) else cameras[0]
        pose_map[ii] = (p["R"], p["C"],
                        cam["fx"], cam["fy"], cam["cx"], cam["cy"],
                        cam.get("k1",0), cam.get("k2",0), cam.get("k3",0),
                        cam.get("p1",0), cam.get("p2",0))

    hdr             = tracks.get("header", {})
    is_sfm_result   = hdr.get("is_sfm_result", False)
    track_xyz       = tracks.get("track_xyz", [])
    track_flags     = tracks.get("track_flags", [])
    track_obs_offset= tracks.get("track_obs_offset", [])
    obs_img_idx     = tracks.get("obs_image_index", [])
    obs_u           = tracks.get("obs_u", [])
    obs_v           = tracks.get("obs_v", [])

    n_tracks = len(track_flags)
    errors: list[float] = []
    errors_per_image: dict[int, list[float]] = {}
    track_lengths: list[int] = []
    n_triangulated = 0

    for tid in range(n_tracks):
        flag = int(track_flags[tid])
        if not _track_is_alive(flag):
            continue
        if not _track_is_triangulated(flag, track_xyz, tid, is_sfm_result):
            continue
        n_triangulated += 1
        xi = tid * 3
        X = track_xyz[xi:xi+3]

        obs_start = int(track_obs_offset[tid])
        obs_end   = int(track_obs_offset[tid + 1]) if tid + 1 < len(track_obs_offset) else len(obs_img_idx)
        track_len = 0
        for oi in range(obs_start, obs_end):
            ii = int(obs_img_idx[oi])
            if ii not in pose_map:
                continue
            R9, C3, fx, fy, cx, cy, k1, k2, k3, p1, p2 = pose_map[ii]
            uv = _project(X, R9, C3, fx, fy, cx, cy, k1, k2, k3, p1, p2)
            if uv is None:
                continue
            err = math.sqrt((uv[0] - obs_u[oi])**2 + (uv[1] - obs_v[oi])**2)
            errors.append(err)
            errors_per_image.setdefault(ii, []).append(err)
            track_len += 1
        if track_len >= 2:
            track_lengths.append(track_len)

    if not errors:
        return {"available": False}

    def _pct(lst, p):
        lst_s = sorted(lst)
        idx = max(0, min(int(len(lst_s) * p / 100), len(lst_s)-1))
        return lst_s[idx]

    per_img_median = {ii: _pct(errs, 50) for ii, errs in errors_per_image.items()}

    return {
        "available":        True,
        "n_triangulated":   n_triangulated,
        "n_obs":            len(errors),
        "mean":             sum(errors) / len(errors),
        "median":           _pct(errors, 50),
        "p90":              _pct(errors, 90),
        "p99":              _pct(errors, 99),
        "lt1px":            sum(1 for e in errors if e < 1.0) / len(errors) * 100,
        "lt2px":            sum(1 for e in errors if e < 2.0) / len(errors) * 100,
        "gt10px":           sum(1 for e in errors if e > 10.0) / len(errors) * 100,
        "per_img_median":   per_img_median,
        "track_lengths":    track_lengths,
        "err_hist":         _hist(errors, [0, 0.5, 1, 2, 3, 5, 10, 20, 50, 1e9],
                                  ['0–0.5','0.5–1','1–2','2–3','3–5','5–10','10–20','20–50','≥50']),
    }


def compute_track_stats(tracks: dict) -> dict:
    hdr             = tracks.get("header", {})
    is_sfm_result   = hdr.get("is_sfm_result", False)
    track_flags     = tracks.get("track_flags", [])
    track_obs_offset= tracks.get("track_obs_offset", [])
    track_xyz       = tracks.get("track_xyz", [])
    obs_img_idx     = tracks.get("obs_image_index", [])

    n_total = len(track_flags)
    n_valid = sum(1 for f in track_flags if _track_is_alive(int(f)))
    n_tri   = 0
    lengths: list[int] = []
    img_coverage: dict[int, int] = {}

    for tid in range(n_total):
        flag  = int(track_flags[tid])
        alive = _track_is_alive(flag)
        tri   = _track_is_triangulated(flag, track_xyz, tid, is_sfm_result)
        # tri implies alive in practice (clear_track_xyz clears kHasTriangulated);
        # n_tri == triangulated 3D points == the only meaningful count.
        if tri:
            n_tri += 1
        if not alive:
            continue

        obs_start = int(track_obs_offset[tid]) if tid < len(track_obs_offset) else 0
        obs_end   = int(track_obs_offset[tid + 1]) if tid + 1 < len(track_obs_offset) else len(obs_img_idx)
        length = obs_end - obs_start
        if length >= 2:
            lengths.append(length)
        for oi in range(obs_start, obs_end):
            ii = int(obs_img_idx[oi])
            img_coverage[ii] = img_coverage.get(ii, 0) + 1

    def _pct(lst, p):
        lst_s = sorted(lst)
        idx = max(0, min(int(len(lst_s) * p / 100), len(lst_s)-1))
        return lst_s[idx]

    # Prefer pre-computed stats from SfM header (faster + accurate)
    if is_sfm_result:
        n_tri = hdr.get("num_triangulated", hdr.get("num_inlier", n_tri))

    tri_rate = n_tri / n_total * 100.0 if n_total > 0 else 0.0

    return {
        "n_total":        n_total,
        "n_valid":        n_valid,
        "n_tri":          n_tri,
        "tri_rate":       tri_rate,
        "is_sfm_result":  is_sfm_result,
        "n_obs":          len(obs_img_idx),
        "mean_len":       sum(lengths) / len(lengths) if lengths else 0,
        "median_len":     _pct(lengths, 50) if lengths else 0,
        "img_coverage":   img_coverage,
        "len_hist":       _hist(lengths, [2,3,4,5,7,10,15,20,30,50,1e9],
                               ['2','3','4','5','6–7','8–10','11–15','16–20','21–30','31–50','≥50']),
    }


# ─────────────────────────────────────────────────────────────────────────────
# Stats helpers
# ─────────────────────────────────────────────────────────────────────────────

def _hist(data: list, edges: list, labels: list) -> list[dict]:
    out = []
    for i, label in enumerate(labels):
        lo = edges[i]
        hi = edges[i + 1] if i + 1 < len(edges) else float("inf")
        cnt = sum(1 for x in data if lo <= x < hi)
        out.append({"label": label, "lo": lo, "count": cnt})
    return out


def _pct_of(lst, p):
    if not lst: return 0
    lst_s = sorted(lst)
    idx = max(0, min(int(len(lst_s) * p / 100), len(lst_s)-1))
    return lst_s[idx]


# ─────────────────────────────────────────────────────────────────────────────
# SVG bar chart (borrowed from report_geo.py, generic version)
# ─────────────────────────────────────────────────────────────────────────────

_SVG_W  = 640
_SVG_H  = 220
_PAD_L  = 52
_PAD_R  = 16
_PAD_T  = 22
_PAD_B  = 60


def _svg_bar(hist: list, title: str, color: str = "#4e9af1",
             color_fn=None) -> str:
    if not hist:
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{_SVG_W}" height="60" '
                f'style="background:#1e2130;border-radius:6px;display:block;margin:0 auto">'
                f'<text x="{_SVG_W//2}" y="35" text-anchor="middle" fill="#556" '
                f'font-family="monospace" font-size="12">{title} — no data</text></svg>')
    counts = [b["count"] for b in hist]
    max_c  = max(counts) if counts and max(counts) > 0 else 1
    w, h   = _SVG_W, _SVG_H
    pl, pr, pt, pb = _PAD_L, _PAD_R, _PAD_T, _PAD_B
    iw = w - pl - pr
    ih = h - pt - pb
    n  = len(hist)
    bw = iw / n
    gap = max(1, bw * 0.12)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'style="background:#1e2130;border-radius:6px;display:block;margin:0 auto">',
        f'<text x="{w//2}" y="{pt-6}" text-anchor="middle" fill="#aab" '
        f'font-size="12" font-family="monospace">{title}</text>',
    ]
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = pt + ih * (1 - frac)
        val = int(max_c * frac)
        lines += [
            f'<line x1="{pl}" y1="{y:.1f}" x2="{w-pr}" y2="{y:.1f}" stroke="#334" stroke-dasharray="3,3"/>',
            f'<text x="{pl-4}" y="{y+4:.1f}" text-anchor="end" fill="#778" font-size="10" font-family="monospace">{val}</text>',
        ]
    for i, b in enumerate(hist):
        c   = b["count"]
        bh  = (c / max_c) * ih
        x   = pl + i * bw + gap / 2
        y   = pt + ih - bh
        bw_ = bw - gap
        fc  = color_fn(b) if color_fn else color
        lines.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw_:.1f}" height="{bh:.1f}" fill="{fc}" rx="2"/>')
        if c > 0:
            lines.append(f'<text x="{x+bw_/2:.1f}" y="{y-2:.1f}" text-anchor="middle" fill="#ccc" font-size="9" font-family="monospace">{c}</text>')
        lx, ly = x + bw_/2, pt + ih + 14
        lines.append(f'<text x="{lx:.1f}" y="{ly:.1f}" text-anchor="end" fill="#778" font-size="9" font-family="monospace" transform="rotate(-40 {lx:.1f} {ly:.1f})">{b["label"]}</text>')
    lines += [
        f'<line x1="{pl}" y1="{pt}" x2="{pl}" y2="{pt+ih}" stroke="#556"/>',
        f'<line x1="{pl}" y1="{pt+ih}" x2="{w-pr}" y2="{pt+ih}" stroke="#556"/>',
        '</svg>',
    ]
    return "\n".join(lines)


def _err_color(b: dict) -> str:
    lo = b["lo"]
    if lo < 0.5:  return "#50d090"   # green
    if lo < 1.0:  return "#7ec850"
    if lo < 2.0:  return "#f0c040"   # yellow
    if lo < 5.0:  return "#f07040"
    return "#e05050"                  # red


def _len_color(_: dict) -> str:
    return "#4e9af1"


def _kp_color(_: dict) -> str:
    return "#7e6af1"


# ─────────────────────────────────────────────────────────────────────────────
# HTML generation
# ─────────────────────────────────────────────────────────────────────────────

_CSS = """
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: #13151f; color: #ccd; font-family: 'Segoe UI', system-ui, sans-serif; font-size: 14px; }
.wrap { max-width: 1320px; margin: 0 auto; padding: 28px 20px; }

/* ── headings ── */
h1 { color: #e8eaf0; font-size: 1.55rem; letter-spacing: .01em; }
h2 { color: #9ab; font-size: 1.05rem; margin: 32px 0 12px;
     border-left: 3px solid #4e9af1; padding-left: 10px; }
.sub { color: #556; font-size: 0.82rem; margin-top: 6px; }

/* ── summary cards ── */
.cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(175px, 1fr));
         gap: 12px; margin-bottom: 12px; }
.card { background: #1e2130; border-radius: 10px; padding: 14px 18px; }
.card .val { font-size: 1.55rem; font-weight: 700; color: #dde; line-height: 1.2; }
.card .lbl { color: #667; font-size: 0.76rem; margin-top: 5px; line-height: 1.4; }
.card.good .val { color: #50d090; }
.card.warn .val { color: #f0a040; }
.card.bad  .val { color: #e05050; }

/* ── chart grid ── */
.charts { display: grid; grid-template-columns: repeat(auto-fill, minmax(680px, 1fr));
          gap: 16px; margin-bottom: 12px; }
.chart-box { background: #1a1d2a; border-radius: 10px; padding: 14px 10px; overflow: hidden; }

/* ── small key-value stats table (centered, auto-width) ── */
.stats-tbl { border-collapse: collapse; font-size: 13px; margin: 0 auto 20px; min-width: 280px; }
.stats-tbl td { padding: 5px 16px; border-bottom: 1px solid #1e2130; color: #bbc; }
.stats-tbl td:first-child { color: #778; white-space: nowrap; padding-right: 24px; }
.stats-tbl td:last-child  { text-align: right; font-variant-numeric: tabular-nums;
                             font-family: 'Consolas', monospace; color: #dde; }
.stats-tbl tr:hover td { background: #1e2130; }

/* ── large sortable data table ── */
.tbl-wrap { overflow-x: auto; margin-bottom: 12px; }
table.sortable { width: 100%; border-collapse: collapse; font-size: 13px; }
table.sortable th { background: #252838; color: #9ab; font-weight: 600;
                    padding: 9px 12px; text-align: center;
                    cursor: pointer; user-select: none; white-space: nowrap; border-bottom: 2px solid #2e3248; }
table.sortable th:hover { background: #2e3248; }
table.sortable td { padding: 6px 12px; border-bottom: 1px solid #1a1d2a; text-align: center; }
table.sortable tr:hover td { background: #1e2130; }
table.sortable td:nth-child(2) { text-align: left; }

/* ── shared ── */
.num { text-align: right; font-variant-numeric: tabular-nums; }
.q-hi  { color: #50d090; }
.q-mid { color: #f0c040; }
.q-lo  { color: #e05050; }
.divider { border: none; border-top: 1px solid #252838; margin: 36px 0 24px; }

/* ── table toolbar ── */
.tbl-toolbar { display: flex; align-items: center; gap: 10px; margin-bottom: 10px; }
.search { background: #1e2130; border: 1px solid #334; border-radius: 5px;
          color: #ccd; padding: 5px 10px; font-size: 13px; width: 240px; }

/* ── timing ── */
.timing-wrap { background: #1a1d2a; border-radius: 10px; padding: 16px 20px; margin-bottom: 12px; }
.timing-row { display: flex; align-items: center; gap: 12px; margin-bottom: 7px; font-size: 12.5px; }
.timing-label { width: 180px; text-align: right; color: #9ab; white-space: nowrap;
                overflow: hidden; text-overflow: ellipsis; flex-shrink: 0; }
.timing-track { flex: 1; height: 20px; background: #252838; border-radius: 4px; overflow: hidden; }
.timing-fill { height: 100%; background: #4e9af1; border-radius: 4px; min-width: 3px; }
.timing-val { width: 70px; color: #ccd; font-variant-numeric: tabular-nums;
              font-family: 'Consolas', monospace; flex-shrink: 0; }

::-webkit-scrollbar { width: 5px; height: 5px; }
::-webkit-scrollbar-track { background: #111; }
::-webkit-scrollbar-thumb { background: #444; border-radius: 3px; }
"""

_JS = r"""
document.querySelectorAll('table.sortable').forEach(tbl => {
  tbl.querySelectorAll('th').forEach((th, ci) => {
    let asc = true;
    th.addEventListener('click', () => {
      const tbody = tbl.querySelector('tbody');
      const rows  = Array.from(tbody.querySelectorAll('tr'));
      rows.sort((a, b) => {
        const av = a.cells[ci]?.dataset.val ?? a.cells[ci]?.textContent ?? '';
        const bv = b.cells[ci]?.dataset.val ?? b.cells[ci]?.textContent ?? '';
        const an = parseFloat(av), bn = parseFloat(bv);
        if (!isNaN(an) && !isNaN(bn)) return asc ? an - bn : bn - an;
        return asc ? av.localeCompare(bv) : bv.localeCompare(av);
      });
      rows.forEach(r => tbody.appendChild(r));
      tbl.querySelectorAll('th').forEach(t => t.textContent = t.textContent.replace(/[▲▼]/g,'').trimEnd());
      th.textContent += asc ? ' ▲' : ' ▼';
      asc = !asc;
    });
  });
});
document.querySelectorAll('input.tbl-filter').forEach(inp => {
  inp.addEventListener('input', () => {
    const q = inp.value.toLowerCase();
    const tbl = document.getElementById(inp.dataset.target);
    tbl.querySelectorAll('tbody tr').forEach(r => {
      r.style.display = r.textContent.toLowerCase().includes(q) ? '' : 'none';
    });
  });
});
"""


def _card(val, lbl, cls=""):
    return f'<div class="card {cls}"><div class="val">{val}</div><div class="lbl">{lbl}</div></div>'


def _qcls(v, lo, hi):
    return "q-hi" if v >= hi else ("q-mid" if v >= lo else "q-lo")


def _timing_html(timing: dict) -> str:
    """Render horizontal bar chart for pipeline timing."""
    if not timing:
        return ""
    steps = list(timing.items())
    max_t = max(t for _, t in steps) if steps else 1
    rows = []
    for name, t_sec in steps:
        pct = t_sec / max_t * 100
        val_str = f"{t_sec/60:.1f} min" if t_sec >= 60 else f"{t_sec:.1f} s"
        rows.append(
            f'<div class="timing-row">'
            f'<span class="timing-label" title="{name}">{name}</span>'
            f'<div class="timing-track"><div class="timing-fill" style="width:{pct:.1f}%"></div></div>'
            f'<span class="timing-val">{val_str}</span>'
            f'</div>'
        )
    return '<div class="timing-wrap">' + "\n".join(rows) + "</div>"


def generate_html(*, scene_name: str, work_dir: str,
                  image_paths: dict[int, str],
                  feat: list, geo: list, match: list,
                  track_stats: dict, reproj: dict,
                  poses_data,
                  timing: dict) -> str:

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    title = f"InsightAT Pipeline Report — {scene_name}"

    # ── counts ─────────────────────────────────────────────────────────────
    n_images_total = len(image_paths) or len(feat)
    n_reg = len(poses_data["poses"]) if poses_data else 0
    n_pts  = track_stats.get("n_tri") or reproj.get("n_triangulated", 0)
    n_geo_pairs = len([p for p in geo if p["F_est"]])
    med_reproj = reproj.get("median", float("nan")) if reproj.get("available") else float("nan")
    kp_counts  = [f["num_keypoints"] for f in feat]
    mean_kp = sum(kp_counts) / len(kp_counts) if kp_counts else 0

    reg_cls = "good" if n_images_total and n_reg / n_images_total > 0.9 else \
              ("warn" if n_images_total and n_reg / n_images_total > 0.7 else "bad")
    rp_cls  = "good" if not math.isnan(med_reproj) and med_reproj < 1.5 else \
              ("warn" if not math.isnan(med_reproj) and med_reproj < 3 else "bad")

    summary_cards = f"""
<div class="cards">
  {_card(f'{n_reg} / {n_images_total}', 'Registered images', reg_cls)}
  {_card(f'{n_pts:,}', '3D points (triangulated)', 'good' if n_pts > 1000 else 'warn')}
  {_card(f'{track_stats.get("tri_rate", 0.0):.1f}%', 'Tri. rate (tri/total)', 'good' if track_stats.get("tri_rate", 0) > 60 else ('warn' if track_stats.get("tri_rate", 0) > 30 else 'bad'))}
  {_card(f'{med_reproj:.2f} px' if not math.isnan(med_reproj) else '—', 'Median reprojection error', rp_cls)}
  {_card(f'{n_geo_pairs}', 'Valid geo pairs (F est.)')}
  {_card(f'{mean_kp:.0f}', 'Mean keypoints / image')}
  {_card(f'{track_stats.get("n_valid", 0):,}', 'Total tracks')}
  {_card(f'{track_stats.get("mean_len", 0):.1f}', 'Mean track length')}
  {_card(f'{len(match)}', 'Matched pairs')}
</div>"""

    # ── timing ─────────────────────────────────────────────────────────────
    timing_section = ""
    if timing:
        timing_section = f"""
<hr class="divider">
<h2>Pipeline Timing</h2>
{_timing_html(timing)}"""

    # ── feature section ────────────────────────────────────────────────────
    kp_hist = _hist(kp_counts,
                    [0, 500, 1000, 2000, 3000, 5000, 7000, 10000, 15000, 1e9],
                    ['<500','500–1k','1k–2k','2k–3k','3k–5k','5k–7k','7k–10k','10k–15k','≥15k'])
    feat_svg = _svg_bar(kp_hist, f"Keypoints per Image  (n={len(kp_counts)}, mean={mean_kp:.0f})",
                        color_fn=_kp_color)

    # ── match section ──────────────────────────────────────────────────────
    nm_vals = [m["num_matches"] for m in match]
    nm_hist = _hist(nm_vals,
                    [0, 20, 50, 100, 200, 500, 1000, 2000, 5000, 1e9],
                    ['<20','20–50','50–100','100–200','200–500','500–1k','1k–2k','2k–5k','≥5k'])
    match_svg = _svg_bar(nm_hist, f"Input Matches per Pair  (n={len(nm_vals)})", color="#9b59b6")

    # ── geo section ────────────────────────────────────────────────────────
    fi_vals = [p["F_in"] for p in geo if p["F_est"]]
    fi_hist = _hist(fi_vals, [0,8,20,50,100,250,500,1000,2000,1e9],
                    ['<8','8–20','20–50','50–100','100–250','250–500','500–1k','1k–2k','≥2k'])
    def _fi_color(b):
        lo = b["lo"]
        if lo < 8:   return "#e05050"
        if lo < 20:  return "#f0a040"
        if lo < 100: return "#f0c040"
        return "#50d090"
    geo_svg = _svg_bar(fi_hist, f"F-Inliers per Pair  (n={len(fi_vals)} with F est.)",
                       color_fn=_fi_color)
    n_F_est = sum(1 for p in geo if p["F_est"])
    F_pct   = n_F_est / len(geo) * 100 if geo else 0

    # ── track section ──────────────────────────────────────────────────────
    _n_tri   = track_stats.get("n_tri", 0)
    _n_total = track_stats.get("n_valid", 0) or track_stats.get("n_total", 1)
    _rate    = track_stats.get("tri_rate", 0.0)
    _tri_label = f"{_n_tri:,} / {_n_total:,} triangulated  ({_rate:.1f}%)"
    track_svg = _svg_bar(track_stats.get("len_hist", []),
                         f"Track Length Distribution  ({_tri_label})",
                         color="#2ecc71")

    # ── reproj section ─────────────────────────────────────────────────────
    reproj_svg = ""
    sfm_per_img_rows = ""
    if reproj.get("available"):
        reproj_svg = _svg_bar(reproj["err_hist"],
                              f"Reprojection Error Distribution  (n_obs={reproj['n_obs']:,})",
                              color_fn=_err_color)
        per_img = sorted(reproj["per_img_median"].items(), key=lambda x: -x[1])
        sfm_rows = []
        img_cov = track_stats.get("img_coverage", {})
        for ii, med in sorted(reproj["per_img_median"].items(), key=lambda x: x[0]):
            name = Path(image_paths.get(ii, "")).name if image_paths else f"img_{ii}"
            n_tr = img_cov.get(ii, 0)
            rd_cls = _qcls(med, 1.5, 3.0) if med < 2 else "q-lo"
            # actually: good < 1.5, warn 1.5–3, bad > 3
            rd_cls = "q-hi" if med < 1.5 else ("q-mid" if med < 3.0 else "q-lo")
            sfm_rows.append(
                f'<tr>'
                f'<td class="num">{ii}</td>'
                f'<td style="color:#89a;font-size:12px;font-family:monospace">{name}</td>'
                f'<td class="num {rd_cls}" data-val="{med:.4f}">{med:.3f}</td>'
                f'<td class="num">{n_tr}</td>'
                f'</tr>'
            )
        sfm_per_img_rows = "\n".join(sfm_rows)

    reproj_section = ""
    if reproj.get("available"):
        err_stat_rows = "".join([
            f'<tr><td>{k}</td><td class="num">{v}</td></tr>'
            for k, v in [
                ("Total observations", f"{reproj['n_obs']:,}"),
                ("Triangulated points", f"{reproj['n_triangulated']:,}"),
                ("Mean error", f"{reproj['mean']:.3f} px"),
                ("Median error", f"{reproj['median']:.3f} px"),
                ("P90", f"{reproj['p90']:.3f} px"),
                ("P99", f"{reproj['p99']:.3f} px"),
                ("< 1 px", f"{reproj['lt1px']:.1f}%"),
                ("< 2 px", f"{reproj['lt2px']:.1f}%"),
                ("> 10 px", f"{reproj['gt10px']:.3f}%"),
            ]
        ])
        reproj_section = f"""
<hr class="divider">
<h2>SfM — Reprojection Error</h2>
<div class="charts"><div class="chart-box">{reproj_svg}</div></div>
<table class="stats-tbl">
<tbody>{err_stat_rows}</tbody>
</table>
<h2>SfM — Per-Image Quality <span style="font-size:12px;color:#556;font-weight:400">(click column to sort)</span></h2>
<div class="tbl-toolbar">
  <label style="color:#778;font-size:12px">Filter:</label>
  <input class="search tbl-filter" data-target="sfm-tbl" placeholder="image index or name…">
</div>
<div class="tbl-wrap">
<table class="sortable" id="sfm-tbl">
<thead><tr>
  <th>Img</th><th style="text-align:left">Filename</th>
  <th title="Median per-image reprojection error (px)">Median reproj (px)</th>
  <th title="Number of track observations for this image">Track obs.</th>
</tr></thead>
<tbody>{sfm_per_img_rows}</tbody>
</table>
</div>"""

    # ── geo stats row ───────────────────────────────────────────────────────
    geo_stat_rows = "".join([
        f'<tr><td>{k}</td><td class="num">{v}</td></tr>'
        for k, v in [
            ("Total pairs", len(geo)),
            ("F estimated", f"{n_F_est} ({F_pct:.1f}%)"),
            ("Median F-inliers", f"{_pct_of(fi_vals, 50):.0f}" if fi_vals else "—"),
            ("Degenerate pairs", sum(1 for p in geo if p["degen"])),
        ]
    ])

    # ── combine ─────────────────────────────────────────────────────────────
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title>
<style>{_CSS}</style>
</head>
<body>
<div class="wrap">

<div style="margin-bottom:32px">
  <h1>{title}</h1>
  <div class="sub">Generated: {now} &nbsp;|&nbsp; Work dir: <code style="color:#89a;font-size:12px">{work_dir}</code></div>
</div>

<h2>Summary</h2>
{summary_cards}
{timing_section}

<hr class="divider">
<h2>Feature Extraction</h2>
<div class="charts"><div class="chart-box">{feat_svg}</div></div>

<hr class="divider">
<h2>Matching</h2>
<div class="charts"><div class="chart-box">{match_svg}</div></div>

<hr class="divider">
<h2>Geometry Verification</h2>
<table class="stats-tbl"><tbody>{geo_stat_rows}</tbody></table>
<div class="charts"><div class="chart-box">{geo_svg}</div></div>

<hr class="divider">
<h2>Tracks</h2>
<div class="charts"><div class="chart-box">{track_svg}</div></div>

{reproj_section}

</div>
<script>{_JS}</script>
</body>
</html>"""
    return html


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="InsightAT pipeline HTML report",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("-w", "--work-dir",   required=True, help="Pipeline work directory")
    ap.add_argument("--sfm-dir",          default=None,  help="SfM output dir (default: same as work-dir)")
    ap.add_argument("-o", "--output",     default=None,  help="Output HTML path (default: <work-dir>/report.html)")
    ap.add_argument("--timing-json",      default=None,  help="JSON file: {step_name: seconds, ...}")
    ap.add_argument("--scene-name",       default=None,  help="Scene name for report title")
    args = ap.parse_args()

    work  = Path(args.work_dir).resolve()
    # Auto-detect sfm output dir: isat_sfm writes to <work>/incremental_sfm/
    if args.sfm_dir:
        sfm = Path(args.sfm_dir).resolve()
    elif (work / "incremental_sfm").is_dir():
        sfm = work / "incremental_sfm"
    else:
        sfm = work
    out   = Path(args.output).resolve() if args.output else work / "report.html"

    if not work.exists():
        print(f"Error: work-dir does not exist: {work}", file=sys.stderr)
        sys.exit(2)

    scene_name = args.scene_name or work.name

    # ── Load data ───────────────────────────────────────────────────────────
    print("Loading image list…", file=sys.stderr)
    image_paths = load_image_list(work)
    print(f"  {len(image_paths)} images", file=sys.stderr)

    print("Loading feature stats…", file=sys.stderr)
    feat_stats = []
    feat_dir = work / "feat"
    if feat_dir.exists():
        feat_stats = load_feat_stats(feat_dir)
        print(f"  {len(feat_stats)} feat files", file=sys.stderr)
    else:
        print("  [warn] feat/ not found", file=sys.stderr)

    print("Loading match stats…", file=sys.stderr)
    match_stats = []
    match_dir = work / "match"
    if match_dir.exists():
        match_stats = load_match_stats(match_dir)
        print(f"  {len(match_stats)} match files", file=sys.stderr)
    else:
        print("  [warn] match/ not found", file=sys.stderr)

    print("Loading geo stats…", file=sys.stderr)
    geo_stats = []
    geo_dir = work / "geo"
    if geo_dir.exists():
        geo_stats = load_geo_stats(geo_dir)
        print(f"  {len(geo_stats)} geo files", file=sys.stderr)
    else:
        print("  [warn] geo/ not found", file=sys.stderr)

    # tracks.isat_tracks lives in work dir; poses.json lives in sfm (incremental_sfm/)
    tracks_path = _find_file(sfm, work, ["tracks.isat_tracks"])
    poses_path  = _find_file(sfm, work, ["poses.json"])

    track_stats = {}
    reproj      = {"available": False}
    poses_data  = None

    if tracks_path:
        print(f"Loading tracks from {tracks_path}…", file=sys.stderr)
        try:
            tracks = load_tracks_idc(str(tracks_path))
            track_stats = compute_track_stats(tracks)
            print(f"  {track_stats.get('n_valid',0)} valid tracks, "
                  f"{track_stats.get('n_tri',0)} triangulated", file=sys.stderr)
        except Exception as e:
            print(f"  [warn] track load failed: {e}", file=sys.stderr)
            tracks = {}
    else:
        tracks = {}
        print("  [warn] tracks.isat_tracks not found", file=sys.stderr)

    if poses_path:
        print(f"Loading poses from {poses_path}…", file=sys.stderr)
        try:
            poses_data = load_poses(str(poses_path))
            print(f"  {len(poses_data['poses'])} registered images", file=sys.stderr)
        except Exception as e:
            print(f"  [warn] poses load failed: {e}", file=sys.stderr)

    if tracks and poses_data:
        print("Computing reprojection errors…", file=sys.stderr)
        try:
            reproj = compute_reproj_stats(tracks, poses_data)
            if reproj.get("available"):
                print(f"  median={reproj['median']:.3f} px  n_obs={reproj['n_obs']:,}", file=sys.stderr)
        except Exception as e:
            print(f"  [warn] reproj computation failed: {e}", file=sys.stderr)
            reproj = {"available": False}

    timing = {}
    timing_path: Optional[Path] = None
    if args.timing_json:
        timing_path = Path(args.timing_json)
    else:
        # Auto-discover timing.json from work dir or sfm dir
        for candidate in [work / "timing.json", sfm / "timing.json"]:
            if candidate.exists():
                timing_path = candidate
                print(f"  Found timing.json: {timing_path}", file=sys.stderr)
                break
    if timing_path:
        try:
            timing = json.loads(timing_path.read_text())
        except Exception as e:
            print(f"  [warn] timing JSON load failed: {e}", file=sys.stderr)

    print("Generating HTML…", file=sys.stderr)
    html = generate_html(
        scene_name=scene_name,
        work_dir=str(work),
        image_paths=image_paths,
        feat=feat_stats,
        geo=geo_stats,
        match=match_stats,
        track_stats=track_stats,
        reproj=reproj,
        poses_data=poses_data,
        timing=timing,
    )

    out.write_text(html, encoding="utf-8")
    print(f"Saved → {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
