#!/usr/bin/env python3
"""
InsightAT Geo-Verification Quality Report
──────────────────────────────────────────
Reads all *.isat_geo files in a geo/ directory and produces a self-contained
HTML quality report.

Usage:
    python3 scripts/report_geo.py \\
        --geo <geo_dir> \\
        --images <images_all.json>  \\  # optional, adds image filenames
        --out report.html

Output: single self-contained HTML (dark theme, SVG charts, sortable tables).
No external dependencies beyond numpy.
"""

import argparse
import json
import struct
import sys
import math
import datetime
from pathlib import Path
from collections import defaultdict

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# IDC header reader (no payload needed)
# ─────────────────────────────────────────────────────────────────────────────

def read_idc_hdr(path: str) -> dict:
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"ISAT":
            raise ValueError(f"{path}: bad magic {magic!r}")
        f.read(4)  # version
        jsz = struct.unpack("<Q", f.read(8))[0]
        return json.loads(f.read(jsz).decode("utf-8"))


# ─────────────────────────────────────────────────────────────────────────────
# Data loading
# ─────────────────────────────────────────────────────────────────────────────

def load_geo_dir(geo_dir: str) -> list[dict]:
    """Return list of pair-dicts sorted by (i1, i2)."""
    pairs = []
    for f in sorted(Path(geo_dir).glob("*.isat_geo")):
        try:
            hdr = read_idc_hdr(str(f))
        except Exception as e:
            print(f"  [warn] {f.name}: {e}", file=sys.stderr)
            continue
        geo  = hdr.get("geometry", {})
        ip   = hdr.get("image_pair", {})
        F    = geo.get("F", {})
        E    = geo.get("E", {})
        H    = geo.get("H", {})
        degen = geo.get("degeneracy", {})
        pairs.append({
            "i1":          ip.get("image1_index", -1),
            "i2":          ip.get("image2_index", -1),
            "n":           hdr.get("num_matches_input", 0),
            "F_est":       F.get("estimated", False),
            "F_in":        F.get("num_inliers", 0),
            "F_ratio":     F.get("inlier_ratio", 0.0),
            "E_est":       E.get("estimated", False),
            "E_in":        E.get("num_inliers", 0),
            "H_est":       H.get("estimated", False),
            "H_in":        H.get("num_inliers", 0),
            "degenerate":  degen.get("is_degenerate", False),
            "hf":          degen.get("h_over_f_ratio", 0.0),
            "score":       geo.get("score_prelim", 0.0),
            "disp":        geo.get("median_pixel_disp", 0.0),
        })
    return pairs


def load_image_names(images_json: str) -> dict[int, str]:
    with open(images_json) as f:
        data = json.load(f)
    return {e["image_index"]: Path(e["path"]).name for e in data["images"]}


# ─────────────────────────────────────────────────────────────────────────────
# Statistics
# ─────────────────────────────────────────────────────────────────────────────

def compute_stats(pairs: list[dict]) -> dict:
    n_pairs = len(pairs)
    n_match = np.array([p["n"] for p in pairs])
    F_pairs = [p for p in pairs if p["F_est"]]
    fi = np.array([p["F_in"] for p in F_pairs]) if F_pairs else np.array([0])
    hf = np.array([p["hf"] for p in pairs])

    # Per-image
    img_data: dict[int, dict] = {}
    for p in pairs:
        for idx in (p["i1"], p["i2"]):
            if idx not in img_data:
                img_data[idx] = {"pairs": [], "degen_cnt": 0}
            img_data[idx]["pairs"].append(p)
            if p["hf"] > 0.95:
                img_data[idx]["degen_cnt"] += 1

    img_stats = []
    for idx, d in sorted(img_data.items()):
        pp = d["pairs"]
        fp = [p for p in pp if p["F_est"]]
        img_stats.append({
            "idx":        idx,
            "n_pairs":    len(pp),
            "n_F_est":    len(fp),
            "mean_F_in":  float(np.mean([p["F_in"] for p in fp])) if fp else 0.0,
            "min_F_in":   int(min(p["F_in"] for p in fp)) if fp else 0,
            "n_matches":  int(np.mean([p["n"] for p in pp])),
            "degen_cnt":  d["degen_cnt"],
            "degen_frac": d["degen_cnt"] / len(pp) if pp else 0.0,
        })

    all_img_ids = set(img_data.keys())
    max_img = max(all_img_ids) if all_img_ids else 0
    expected = set(range(max_img + 1))
    disconnected = sorted(expected - all_img_ids)

    return {
        "n_pairs":        n_pairs,
        "n_images":       len(all_img_ids),
        "n_F_est":        sum(1 for p in pairs if p["F_est"]),
        "n_E_est":        sum(1 for p in pairs if p["E_est"]),
        "n_H_est":        sum(1 for p in pairs if p["H_est"]),
        "n_none_est":     sum(1 for p in pairs
                              if not p["F_est"] and not p["E_est"] and not p["H_est"]),
        "n_degenerate":   sum(1 for p in pairs if p["degenerate"]),
        "n_hf95":         int((hf > 0.95).sum()),
        "n_match_total":  int(n_match.sum()),
        "n_match_mean":   float(n_match.mean()),
        "n_match_median": float(np.median(n_match)),
        "n_match_p5":     float(np.percentile(n_match, 5)),
        "n_match_p95":    float(np.percentile(n_match, 95)),
        "fi_mean":        float(fi.mean()),
        "fi_median":      float(np.median(fi)),
        "fi_p5":          float(np.percentile(fi, 5)),
        "fi_p95":         float(np.percentile(fi, 95)),
        "fi_lt8":         int((fi < 8).sum()),
        "hf_mean":        float(hf.mean()),
        "disconnected":   disconnected,
        "img_stats":      img_stats,
        # histograms (for charts)
        "fi_hist":        _histogram(fi, [0,8,20,50,100,250,500,1000,2000,5000,99999]),
        "nm_hist":        _histogram(n_match, [0,10,50,100,250,500,1000,2000,5000,99999]),
        "hf_hist":        _histogram(hf * 100, [0,10,20,30,40,50,60,70,80,90,100,200,999]),
    }


def _histogram(arr: np.ndarray, edges: list) -> list[dict]:
    out = []
    for i in range(len(edges) - 1):
        lo, hi = edges[i], edges[i + 1]
        cnt = int(((arr >= lo) & (arr < hi)).sum())
        if hi >= 99999:
            label = f"≥{lo}"
        else:
            label = f"{lo}–{hi}"
        out.append({"label": label, "lo": lo, "hi": hi, "count": cnt})
    return out


# ─────────────────────────────────────────────────────────────────────────────
# SVG chart helpers
# ─────────────────────────────────────────────────────────────────────────────

_SVG_W = 680
_SVG_H = 220
_PAD_L = 52
_PAD_R = 16
_PAD_T = 18
_PAD_B = 60


def _bar_chart_svg(hist: list[dict], title: str, color: str = "#4e9af1",
                   highlight_fn=None) -> str:
    """Return an inline <svg> bar chart for a histogram."""
    counts = [b["count"] for b in hist]
    max_c  = max(counts) if counts else 1
    w      = _SVG_W
    h      = _SVG_H
    pl, pr, pt, pb = _PAD_L, _PAD_R, _PAD_T, _PAD_B
    inner_w = w - pl - pr
    inner_h = h - pt - pb
    n       = len(hist)
    bar_w   = inner_w / n
    gap     = max(1, bar_w * 0.12)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'style="background:#1e2130;border-radius:6px;display:block;margin:0 auto">',
        # title
        f'<text x="{w//2}" y="{pt - 4}" text-anchor="middle" '
        f'fill="#aab" font-size="12" font-family="monospace">{title}</text>',
    ]

    # y-axis gridlines & labels
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = pt + inner_h * (1 - frac)
        val = int(max_c * frac)
        lines.append(
            f'<line x1="{pl}" y1="{y:.1f}" x2="{w - pr}" y2="{y:.1f}" '
            f'stroke="#334" stroke-dasharray="3,3"/>'
        )
        lines.append(
            f'<text x="{pl - 4}" y="{y + 4:.1f}" text-anchor="end" '
            f'fill="#778" font-size="10" font-family="monospace">{val}</text>'
        )

    # bars
    for i, b in enumerate(hist):
        c      = b["count"]
        bar_h  = (c / max_c) * inner_h if max_c > 0 else 0
        x      = pl + i * bar_w + gap / 2
        y      = pt + inner_h - bar_h
        bw     = bar_w - gap
        fc     = (highlight_fn(b) if highlight_fn else color)
        lines.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw:.1f}" height="{bar_h:.1f}" '
            f'fill="{fc}" rx="2"/>'
        )
        # value label on top
        if c > 0:
            lines.append(
                f'<text x="{x + bw/2:.1f}" y="{y - 2:.1f}" text-anchor="middle" '
                f'fill="#ccc" font-size="9" font-family="monospace">{c}</text>'
            )
        # x-axis label
        label = b["label"]
        lx    = x + bw / 2
        ly    = pt + inner_h + 14
        lines.append(
            f'<text x="{lx:.1f}" y="{ly:.1f}" text-anchor="end" '
            f'fill="#778" font-size="9" font-family="monospace" '
            f'transform="rotate(-40 {lx:.1f} {ly:.1f})">{label}</text>'
        )

    # axes
    lines.append(
        f'<line x1="{pl}" y1="{pt}" x2="{pl}" y2="{pt + inner_h}" stroke="#556"/>'
    )
    lines.append(
        f'<line x1="{pl}" y1="{pt + inner_h}" x2="{w - pr}" y2="{pt + inner_h}" stroke="#556"/>'
    )
    lines.append("</svg>")
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
# HTML generation
# ─────────────────────────────────────────────────────────────────────────────

_CSS = """
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: #13151f; color: #ccd; font-family: 'Segoe UI', system-ui, sans-serif; font-size: 14px; }
h1 { color: #e8eaf0; font-size: 1.5rem; }
h2 { color: #9ab; font-size: 1.1rem; margin: 28px 0 10px; border-left: 3px solid #4e9af1; padding-left: 10px; }
.wrap { max-width: 1200px; margin: 0 auto; padding: 24px 16px; }
.header { margin-bottom: 28px; }
.header .sub { color: #667; font-size: 0.82rem; margin-top: 4px; }

/* summary cards */
.cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(180px, 1fr)); gap: 12px; margin-bottom: 8px; }
.card { background: #1e2130; border-radius: 8px; padding: 14px 16px; }
.card .val { font-size: 1.6rem; font-weight: 700; color: #fff; }
.card .lbl { color: #778; font-size: 0.78rem; margin-top: 3px; }
.card.warn .val { color: #f0a040; }
.card.good .val { color: #50d090; }
.card.bad  .val { color: #e05050; }

/* charts */
.charts { display: grid; grid-template-columns: repeat(auto-fill, minmax(680px, 1fr)); gap: 16px; margin-bottom: 8px; }
.chart-box { background: #1a1d2a; border-radius: 8px; padding: 12px; }

/* tables */
.tbl-wrap { overflow-x: auto; margin-bottom: 8px; }
table { width: 100%; border-collapse: collapse; font-size: 13px; }
th { background: #252838; color: #9ab; font-weight: 600; padding: 8px 10px;
     text-align: left; cursor: pointer; user-select: none; white-space: nowrap; }
th:hover { background: #2e3248; }
td { padding: 6px 10px; border-bottom: 1px solid #1e2130; }
tr:hover td { background: #1e2130; }
.num { text-align: right; font-variant-numeric: tabular-nums; }

/* quality color cells */
.q-hi  { color: #50d090; }
.q-mid { color: #f0c040; }
.q-lo  { color: #e05050; }
.q-ok  { color: #4e9af1; }

/* section divider */
.divider { border: none; border-top: 1px solid #252838; margin: 32px 0 20px; }

/* badge */
.badge { display:inline-block; padding: 1px 7px; border-radius: 10px; font-size: 11px; font-weight:600; }
.badge-green { background:#1a3d2a; color:#50d090; }
.badge-red   { background:#3d1a1a; color:#e05050; }
.badge-yellow{ background:#3d3210; color:#f0c040; }
.badge-gray  { background:#252838; color:#778; }

/* search box */
.search { background:#1e2130; border:1px solid #334; border-radius:5px; color:#ccd; padding:5px 10px; font-size:13px; width:220px; }
.tbl-toolbar { display:flex; align-items:center; gap:10px; margin-bottom:8px; }
.tbl-toolbar label { color:#778; font-size:12px; }
"""

_JS = r"""
// Sortable tables
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

// Filter inputs
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


def _pct(v: float) -> str:
    return f"{v * 100:.1f}%"


def _flt(v: float, d: int = 1) -> str:
    return f"{v:.{d}f}"


def _q_cls(v: float, lo: float, hi: float) -> str:
    """Color class: good if v>=hi, warn if v>=lo, bad otherwise."""
    if v >= hi:   return "q-hi"
    if v >= lo:   return "q-mid"
    return "q-lo"


def _fi_color(b: dict) -> str:
    lo = b["lo"]
    if lo < 8:    return "#e05050"
    if lo < 20:   return "#f0a040"
    if lo < 100:  return "#f0c040"
    if lo < 500:  return "#4e9af1"
    return "#50d090"


def _hf_color(b: dict) -> str:
    lo = b["lo"]
    if lo >= 80:  return "#e05050"
    if lo >= 60:  return "#f0a040"
    return "#4e9af1"


def generate_html(stats: dict, pairs: list[dict], img_names: dict,
                  geo_dir: str) -> str:
    s = stats
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    title = f"Geo Quality Report — {Path(geo_dir).resolve().name}"

    # ── summary cards ──────────────────────────────────────────────────────
    F_pct = s["n_F_est"] / s["n_pairs"] * 100 if s["n_pairs"] else 0
    E_pct = s["n_E_est"] / s["n_pairs"] * 100 if s["n_pairs"] else 0

    def card(val, lbl, cls=""):
        return f'<div class="card {cls}"><div class="val">{val}</div><div class="lbl">{lbl}</div></div>'

    dc = "" if s["disconnected"] else "good"
    cards_html = f"""
<div class="cards">
  {card(s['n_pairs'], 'Total geo pairs')}
  {card(s['n_images'], 'Connected images')}
  {card(f"{F_pct:.1f}%", 'F estimated', 'good' if F_pct > 95 else 'warn')}
  {card(f"{E_pct:.1f}%", 'E estimated', 'good' if E_pct > 70 else 'warn')}
  {card(f"{s['fi_median']:.0f}", 'Median F-inliers', 'good' if s['fi_median'] > 100 else 'warn')}
  {card(f"{s['n_match_mean']:.0f}", 'Mean input matches')}
  {card(s['n_hf95'], 'Highly planar pairs (H/F>0.95)', 'warn' if s['n_hf95'] > 0 else 'good')}
  {card(len(s['disconnected']), 'Isolated images (0 pairs)', 'bad' if s['disconnected'] else 'good')}
</div>"""

    # ── overview stats block ────────────────────────────────────────────────
    def stat_row(k, v):
        return f'<tr><td>{k}</td><td class="num">{v}</td></tr>'

    overview_html = f"""
<table style="width:auto;min-width:360px">
<tbody>
  {stat_row('Total pairs', s['n_pairs'])}
  {stat_row('F estimated', f"{s['n_F_est']} ({F_pct:.1f}%)")}
  {stat_row('E estimated', f"{s['n_E_est']} ({E_pct:.1f}%)")}
  {stat_row('H estimated', f"{s['n_H_est']} ({s['n_H_est']/s['n_pairs']*100:.1f}%)")}
  {stat_row('Not estimated (all)', s['n_none_est'])}
  {stat_row('Degenerate (flag)', s['n_degenerate'])}
  {stat_row('Highly planar H/F>0.95', s['n_hf95'])}
  {stat_row('Total input matches', f"{s['n_match_total']:,}")}
  {stat_row('Mean / Median matches', f"{s['n_match_mean']:.0f} / {s['n_match_median']:.0f}")}
  {stat_row('P5 / P95 matches', f"{s['n_match_p5']:.0f} / {s['n_match_p95']:.0f}")}
  {stat_row('Mean / Median F-inliers', f"{s['fi_mean']:.1f} / {s['fi_median']:.1f}")}
  {stat_row('P5 / P95 F-inliers', f"{s['fi_p5']:.0f} / {s['fi_p95']:.0f}")}
  {stat_row('F-inliers < 8', s['fi_lt8'])}
  {stat_row('Isolated images', str(s['disconnected']) if s['disconnected'] else '—')}
</tbody>
</table>"""

    # ── charts ──────────────────────────────────────────────────────────────
    fi_svg = _bar_chart_svg(s["fi_hist"], "F-Inlier Count Distribution",
                             highlight_fn=_fi_color)
    nm_svg = _bar_chart_svg(s["nm_hist"], "Input Match Count Distribution",
                             color="#7e6af1")
    hf_svg = _bar_chart_svg(s["hf_hist"], "H/F Ratio × 100 Distribution",
                             highlight_fn=_hf_color)

    # ── per-image table ──────────────────────────────────────────────────────
    img_rows = []
    for r in sorted(s["img_stats"], key=lambda x: x["mean_F_in"]):
        idx    = r["idx"]
        name   = img_names.get(idx, "—")
        np_    = r["n_pairs"]
        nf     = r["n_F_est"]
        mfi    = r["mean_F_in"]
        minfi  = r["min_F_in"]
        dc_    = r["degen_cnt"]
        df_    = r["degen_frac"]

        mfi_cls  = _q_cls(mfi,  50, 200)
        minfi_cls = _q_cls(minfi, 8, 30)
        df_cls   = "q-lo" if df_ > 0.7 else ("q-mid" if df_ > 0.4 else "q-hi")

        img_rows.append(
            f'<tr>'
            f'<td class="num">{idx}</td>'
            f'<td style="font-family:monospace;font-size:12px;color:#89a">{name}</td>'
            f'<td class="num">{np_}</td>'
            f'<td class="num {mfi_cls}" data-val="{mfi:.1f}">{mfi:.0f}</td>'
            f'<td class="num {minfi_cls}" data-val="{minfi}">{minfi}</td>'
            f'<td class="num {df_cls}" data-val="{df_:.3f}">{dc_}/{np_} ({df_*100:.0f}%)</td>'
            f'</tr>'
        )

    img_table = f"""
<div class="tbl-toolbar">
  <label>Filter:</label>
  <input class="search tbl-filter" data-target="img-tbl" placeholder="image index or name…">
</div>
<div class="tbl-wrap">
<table class="sortable" id="img-tbl">
<thead>
<tr>
  <th title="Image index">Img</th>
  <th>Filename</th>
  <th title="Number of geo pairs this image appears in">Pairs</th>
  <th title="Mean F-matrix inliers across all pairs (when F estimated)">Mean F-in</th>
  <th title="Minimum F-inliers in any pair">Min F-in</th>
  <th title="Pairs with H/F > 0.95 (planar/degenerate)">Degenerate</th>
</tr>
</thead>
<tbody>
{''.join(img_rows)}
</tbody>
</table>
</div>"""

    # ── weakest pairs table ──────────────────────────────────────────────────
    F_pairs = [p for p in pairs if p["F_est"]]
    worst   = sorted(F_pairs, key=lambda x: x["F_in"])[:50]

    def geo_badge(est: bool) -> str:
        return '<span class="badge badge-green">✓</span>' if est \
               else '<span class="badge badge-gray">–</span>'

    def degen_badge(p: dict) -> str:
        if p["degenerate"]:
            return '<span class="badge badge-red">degen</span>'
        if p["hf"] > 0.8:
            return '<span class="badge badge-yellow">planar</span>'
        return ''

    weak_rows = []
    for p in worst:
        fi_cls = _q_cls(p["F_in"], 8, 50)
        weak_rows.append(
            f'<tr>'
            f'<td class="num">{p["i1"]}</td>'
            f'<td class="num">{p["i2"]}</td>'
            f'<td class="num">{p["n"]}</td>'
            f'<td class="num {fi_cls}" data-val="{p["F_in"]}">{p["F_in"]}</td>'
            f'<td class="num">{p["F_ratio"]:.3f}</td>'
            f'<td class="num">{geo_badge(p["E_est"])}</td>'
            f'<td class="num">{p["hf"]:.3f}</td>'
            f'<td>{degen_badge(p)}</td>'
            f'</tr>'
        )

    weak_table = f"""
<div class="tbl-wrap">
<table class="sortable" id="weak-tbl">
<thead>
<tr>
  <th>Img1</th><th>Img2</th>
  <th title="Input match count">Matches</th>
  <th title="F-matrix inlier count">F-in</th>
  <th title="F-matrix inlier ratio">F-ratio</th>
  <th title="Essential matrix estimated?">E</th>
  <th title="H/F inlier ratio">H/F</th>
  <th>Flag</th>
</tr>
</thead>
<tbody>
{''.join(weak_rows)}
</tbody>
</table>
</div>"""

    # ── most degenerate images ───────────────────────────────────────────────
    degen_imgs = sorted(s["img_stats"], key=lambda x: -x["degen_frac"])[:20]
    degen_rows = []
    for r in degen_imgs:
        if r["degen_cnt"] == 0:
            break
        df_cls = "q-lo" if r["degen_frac"] > 0.7 else ("q-mid" if r["degen_frac"] > 0.4 else "q-ok")
        degen_rows.append(
            f'<tr>'
            f'<td class="num">{r["idx"]}</td>'
            f'<td style="font-family:monospace;font-size:12px;color:#89a">{img_names.get(r["idx"],"—")}</td>'
            f'<td class="num {df_cls}" data-val="{r["degen_frac"]:.3f}">'
            f'{r["degen_cnt"]}/{r["n_pairs"]} ({r["degen_frac"]*100:.0f}%)</td>'
            f'<td class="num">{r["mean_F_in"]:.0f}</td>'
            f'</tr>'
        )

    degen_table = f"""
<div class="tbl-wrap">
<table class="sortable" id="degen-tbl">
<thead>
<tr>
  <th>Img</th><th>Filename</th>
  <th title="Pairs with H/F>0.95 / total pairs">Degenerate pairs</th>
  <th title="Mean F-inliers">Mean F-in</th>
</tr>
</thead>
<tbody>
{''.join(degen_rows) if degen_rows else '<tr><td colspan="4" style="color:#667;padding:12px">No highly degenerate images.</td></tr>'}
</tbody>
</table>
</div>"""

    # ── full HTML ────────────────────────────────────────────────────────────
    disconnected_note = ""
    if s["disconnected"]:
        disconnected_note = (
            f'<div style="background:#3d1a1a;border-radius:6px;padding:10px 14px;'
            f'margin-bottom:16px;color:#e07070">⚠ Isolated images (no geo pairs): '
            f'{s["disconnected"]}</div>'
        )

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

<div class="header">
  <h1>{title}</h1>
  <div class="sub">Generated: {now} &nbsp;|&nbsp; Source: {Path(geo_dir).resolve()}</div>
</div>

{disconnected_note}

<h2>Summary</h2>
{cards_html}

<h2>Overview Statistics</h2>
{overview_html}

<hr class="divider">
<h2>F-Inlier Distribution</h2>
<div class="charts">
  <div class="chart-box">{fi_svg}</div>
  <div class="chart-box">{nm_svg}</div>
  <div class="chart-box">{hf_svg}</div>
</div>

<hr class="divider">
<h2>Per-Image Quality  <span style="font-size:12px;color:#556;font-weight:400">(click column header to sort)</span></h2>
{img_table}

<hr class="divider">
<h2>50 Weakest Pairs  <span style="font-size:12px;color:#556;font-weight:400">(by F-inliers)</span></h2>
{weak_table}

<hr class="divider">
<h2>Most Degenerate Images  <span style="font-size:12px;color:#556;font-weight:400">(H/F &gt; 0.95 pair fraction)</span></h2>
{degen_table}

</div>
<script>{_JS}</script>
</body>
</html>"""
    return html


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="InsightAT geo quality report")
    ap.add_argument("--geo",    required=True, metavar="DIR",  help="geo/ directory")
    ap.add_argument("--images", default=None,  metavar="JSON", help="images_all.json (optional)")
    ap.add_argument("--out",    default="geo_report.html", metavar="FILE", help="output HTML")
    args = ap.parse_args()

    geo_dir = args.geo
    print(f"Loading geo files from {geo_dir} …", file=sys.stderr)
    pairs = load_geo_dir(geo_dir)
    if not pairs:
        print("No .isat_geo files found.", file=sys.stderr)
        sys.exit(1)
    print(f"  Loaded {len(pairs)} pairs.", file=sys.stderr)

    img_names: dict[int, str] = {}
    if args.images:
        img_names = load_image_names(args.images)

    print("Computing statistics …", file=sys.stderr)
    stats = compute_stats(pairs)

    print("Generating HTML …", file=sys.stderr)
    html = generate_html(stats, pairs, img_names, geo_dir)

    out = args.out
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Saved → {out}  ({len(pairs)} pairs, {stats['n_images']} images)",
          file=sys.stderr)


if __name__ == "__main__":
    main()
