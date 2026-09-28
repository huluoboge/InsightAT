#!/usr/bin/env python3
"""Generate the isat_sfm pipeline diagrams (SVG).

Source of truth: ``src/cli/isat_sfm.cpp`` (the orchestrator that spawns the
sibling ``isat_*`` CLIs). Regenerate with::

    python3 scripts/gen_pipeline_diagrams.py

Outputs (each diagram as SVG plus a 2x PNG fallback, because several
Markdown viewers do not render embedded SVG)::

    docs/images/pipeline/isat_sfm_pipeline.svg      .png  阶段 / 子进程 / 产物 总览
    docs/images/pipeline/isat_sfm_match_detail.svg  .png  match 阶段决策图
    docs/images/pipeline/isat_sfm_process.svg       .png  增量 SfM 内部流程

PNG rasterization needs a Chromium-based browser on PATH; if none is found the
SVG files are still written and the PNG step is skipped with a warning.

Text metrics were calibrated by rendering a probe sheet in Chrome
(mono advance = 0.6013 em, CJK advance = 1.0 em); every string is wrapped and
bounds-checked so nothing overflows its box.
"""
from __future__ import annotations

import html
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "docs" / "images" / "pipeline"

SANS = "Noto Sans CJK SC, Noto Sans, PingFang SC, Microsoft YaHei, DejaVu Sans, sans-serif"
MONO = "DejaVu Sans Mono, Menlo, Consolas, monospace"

INK = "#0f172a"
SLATE = "#334155"
MUTED = "#64748b"
FAINT = "#94a3b8"
LINE = "#cbd5e1"
CARD = "#ffffff"
SOFT = "#f8fafc"

PNG_SCALE = 2         # PNG fallback rasterized at 2x for HiDPI reading

MONO_RATIO = 0.6013   # measured
SANS_RATIO = 0.535    # latin, measured-ish
CJK_RATIO = 1.0       # measured


def esc(s: str) -> str:
    return html.escape(s, quote=True)


def tw(s: str, size: float, mono: bool = True) -> float:
    w = 0.0
    for ch in s:
        if ord(ch) > 0x2E7F:                 # CJK / fullwidth
            w += size * CJK_RATIO
        else:
            w += size * (MONO_RATIO if mono else SANS_RATIO)
    return w


def _chunks(s: str) -> list[str]:
    out: list[str] = []
    cur = ""
    for ch in s:
        if ch == " ":
            if cur:
                out.append(cur)
                cur = ""
            out.append(" ")
        else:
            cur += ch
    if cur:
        out.append(cur)
    return out


def wrap(s: str, size: float, max_w: float, mono: bool = True, indent: str = "") -> list[str]:
    """Greedy wrap; hard-splits tokens that are wider than max_w (long CJK runs)."""
    if not s:
        return []
    lines: list[str] = []
    cur = ""
    for ch in _chunks(s):
        if ch == " " and not cur:
            continue
        if tw(cur + ch, size, mono) <= max_w:
            cur += ch
            continue
        if cur:
            lines.append(cur.rstrip())
            cur = ""
        if ch == " ":
            continue
        if tw(ch, size, mono) <= max_w:
            cur = ch
            continue
        # hard split a single oversized token
        for c in ch:
            if tw(cur + c, size, mono) > max_w and cur:
                lines.append(cur.rstrip())
                cur = ""
            cur += c
    if cur:
        lines.append(cur.rstrip())
    return [indent + ln for ln in lines] if indent else lines


class Svg:
    def __init__(self, width: float, height: float):
        self.w, self.h = width, height
        self.parts: list[str] = []
        self.checks: list[tuple[str, float, float]] = []

    def check(self, s: str, size: float, avail: float, mono: bool = True, where: str = "") -> str:
        w = tw(s, size, mono)
        self.checks.append((where or s[:34], w, avail))
        if w > avail + 0.5:
            raise SystemExit(f"OVERFLOW [{where}] {w:.1f}px > {avail:.1f}px :: {s!r}")
        return s

    def add(self, s: str) -> None:
        self.parts.append(s)

    def rect(self, x, y, w, h, fill=CARD, stroke=LINE, rx=8, sw=1.2, dash=None, opacity=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        o = f' opacity="{opacity}"' if opacity else ""
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" '
                 f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{d}{o}/>')

    def line(self, x1, y1, x2, y2, stroke=LINE, sw=1.4, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
                 f'stroke="{stroke}" stroke-width="{sw}"{d}/>')

    def path(self, d, stroke=LINE, sw=1.4, dash=None, fill="none"):
        dd = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{dd}/>')

    def head(self, x, y, direction="down", color=FAINT, s=5.0):
        if direction == "down":
            d = f"M {x-s:.1f} {y-1.6*s:.1f} L {x:.1f} {y:.1f} L {x+s:.1f} {y-1.6*s:.1f} Z"
        else:
            d = f"M {x-1.6*s:.1f} {y-s:.1f} L {x:.1f} {y:.1f} L {x-1.6*s:.1f} {y+s:.1f} Z"
        self.add(f'<path d="{d}" fill="{color}"/>')

    def arrow_down(self, x, y1, y2, color=FAINT, sw=1.6):
        self.line(x, y1, x, y2 - 8, color, sw)
        self.head(x, y2, "down", color)

    def text(self, x, y, s, size=13, fill=INK, family=MONO, anchor="start", weight="400"):
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-family="{family}" font-size="{size}" '
                 f'fill="{fill}" text-anchor="{anchor}" font-weight="{weight}">{esc(s)}</text>')

    def render(self) -> str:
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w:.0f}" '
                f'height="{self.h:.0f}" viewBox="0 0 {self.w:.0f} {self.h:.0f}">\n'
                f'<rect width="{self.w:.0f}" height="{self.h:.0f}" fill="#ffffff"/>\n'
                + "\n".join(self.parts) + "\n</svg>\n")


# ─────────────────────────────────────────────────────────────────────────────
# Diagram 1 — stage / subprocess / artifact map
# ─────────────────────────────────────────────────────────────────────────────
STAGES = [
    dict(num="1", name="create", zh="项目 → 任务快照 → 输入清单", accent="#2563eb",
         cmds=["isat_project create -p <work>/project.iat",
               "isat_project add-group  -n <group>           × N",
               "isat_project add-images -g <gid> -i <dir>    × N",
               "isat_camera_estimator -a --max-sample 5 --auto-split",
               "isat_project create-at-task    # 冻结任务快照 AT_0",
               "isat_project extract -t 0 -a -o images_all.json"],
         arts=["project.iat", "camera_estimate_meta.json", "images_all.json"],
         note="--existing-task 时整段跳过，直接复用 images_all.json"),
    dict(num="2", name="extract", zh="双路特征提取", accent="#7c3aed",
         cmds=["isat_extract            # 全分辨率，总是执行",
               "  --nfeatures 10000 --threshold 0.0067 --octaves -1 --levels 3",
               "  --image-max-dim 3200 --norm l1root --no-adapt --uint8 --nms",
               "",
               "isat_extract            # 候选对发现级，仅非穷举时",
               "  --output-retrieval --nfeatures-retrieval 1500",
               "  --resize-retrieval 1024 --only-retrieval"],
         arts=["feat/*.isat_feat", "feat/matching_extract_meta.json",
               "feat_retrieval/*.isat_feat"],
         note="提取器 PopSift（默认）或 --use-sift-gpu；后端 --extract-backend cuda|glsl"),
    dict(num="3", name="match", zh="候选对 → 匹配 → 几何验证", accent="#db2777",
         cmds=["# 候选对（分支见 match 决策图）",
               "n_img < 60 或 --exhaustive-match → 穷举 C(n,2) 对",
               "否则 isat_retrieval_match -o match/pairs_retrieve.json",
               "     └ low_peak 图并入其全部穷举对",
               "",
               "isat_gpu_cascade_hashing_match  # --match-impl cascade-gpu",
               "isat_geo_cuda                   # --geo-backend cuda",
               "isat_focal_from_geo             # --focal-from-geo auto"],
         arts=["match/pairs_retrieve.json", "match/pairs_matched.json", "match/*.isat_match",
               "geo/*.isat_geo", "geo/pairs.json", "images_all.json（fx 回写）"],
         note="匹配实现 cascade-gpu|cascade|siftgpu；几何后端有回退链"),
    dict(num="4", name="tracks", zh="构建多视图轨迹", accent="#ea580c",
         cmds=["isat_tracks -i geo/pairs.json -m match/ -g geo/ \\",
               "            -l images_all.json --min-track-length 2"],
         arts=["tracks/tracks.isat_tracks", "（IDC，内嵌 view_graph_pairs）"],
         note=""),
    dict(num="5", name="seed_eval", zh="四种种子策略短窗口评估", accent="#ca8a04",
         cmds=["isat_seed_eval --max-eval-images 6 \\",
               "            --incremental-sfm-bin isat_incremental_sfm",
               "# balanced / wide_baseline / support_first / conservative",
               "# 每种策略实测调用 isat_incremental_sfm 注册前 6 张"],
         arts=["seed_eval_all/report.json", "seed_eval_all/best_seed.json",
               "seed_eval_all/report_plot.png"],
         note="胜出策略回填 incremental_sfm 的 5 个 --init-* / resection 参数"),
    dict(num="6", name="incremental_sfm", zh="增量重建 + BA", accent="#059669",
         cmds=["isat_incremental_sfm -t tracks/tracks.isat_tracks \\",
               "            -p images_all.json -m match/ -g geo/ \\",
               "            -f feat/ -o <work>/incremental_sfm",
               "# 可选：--init-*（来自 seed_eval）、--fix-intrinsics、--ba-threads",
               "# 可选：--debug-dir sfm_interval --debug-interval 1"],
         arts=["incremental_sfm/poses.json", "incremental_sfm/tracks.isat_tracks",
               "incremental_sfm/bundler/bundle.out", "incremental_sfm/colmap/sparse/0",
               "sfm_interval/iter_NNNN/*（可选）"],
         note=""),
    dict(num="7", name="undistort", zh="去畸变 + COLMAP 导出（可选）", accent="#0891b2",
         cmds=["isat_undistort -p images_all.json \\",
               "            -t incremental_sfm/tracks.isat_tracks \\",
               "            -j incremental_sfm/poses.json -f feat/",
               "# --binary（默认）或 --text"],
         arts=["incremental_sfm/colmap/images/", "incremental_sfm/colmap/sparse/0"],
         note="--undistort 才加入阶段；缺 poses.json / tracks 时跳过并告警"),
]


def diagram_overview() -> Svg:
    W = 1300.0
    x_stage, w_stage = 26.0, 196.0
    x_cmd, w_cmd = 238.0, 706.0
    x_art, w_art = 960.0, 314.0
    sub_avail = w_stage - 30
    note_avail = w_stage - 24
    cmd_avail = w_cmd - 12
    art_avail = w_art - 30
    sub_size, note_size, cmd_size, art_size = 11.5, 10.5, 12.5, 12.0
    cmd_lh, art_lh, sub_lh, note_lh = 20.0, 19.0, 15.0, 14.0

    rows = []
    for st in STAGES:
        cmds = []
        for ln in st["cmds"]:
            cmds.extend(wrap(ln, cmd_size, cmd_avail, True, "") if ln else [""])
        arts = []
        for a in st["arts"]:
            arts.extend(wrap(a, art_size, art_avail, True, "    "))
        subs = wrap(st["zh"], sub_size, sub_avail, False)
        notes = wrap(st["note"], note_size, note_avail, False) if st["note"] else []
        card_h = 58 + len(subs) * sub_lh + (10 + len(notes) * note_lh if notes else 0) + 14
        rh = max(len(cmds) * cmd_lh, len(arts) * art_lh, card_h) + 18
        rows.append(dict(st=st, cmds=cmds, arts=arts, subs=subs, notes=notes, h=rh))

    gap = 26.0
    top = 140.0
    y = top
    for r in rows:
        r["y"] = y
        y += r["h"] + gap
    total_h = y - gap + 150

    svg = Svg(W, total_h)
    svg.text(26, 52, "isat_sfm —— 端到端 CLI 流水线", size=25, family=SANS, weight="700")
    svg.text(26, 79, "每个阶段都是一个独立的 isat_* 子进程；阶段之间只通过工作目录里的自描述文件交换数据",
             size=13.5, family=SANS, fill=MUTED)
    svg.text(26, 103, "输入：-i <图像目录>  -w <工作目录>      基线 main@305bbd1      默认阶段 "
                      "create, extract, match, tracks, seed_eval, incremental_sfm",
             size=12, family=MONO, fill=FAINT)

    for label, x in (("阶段", x_stage), ("子进程调用", x_cmd), ("产出", x_art)):
        svg.text(x + 2, 130, label, size=12, family=SANS, fill=MUTED, weight="700")
    svg.line(x_stage, 136, W - 26, 136, stroke=LINE, sw=1.2)

    for i, r in enumerate(rows):
        st, ry, rh = r["st"], r["y"], r["h"]
        accent = st["accent"]
        svg.line(x_stage, ry - gap / 2, W - 26, ry - gap / 2, stroke="#eef2f7", sw=1.0)
        svg.rect(x_stage, ry, w_stage, rh, fill=SOFT, stroke=accent, rx=10, sw=1.5)
        svg.rect(x_stage, ry + 2, 5.0, rh - 4, fill=accent, stroke=accent, rx=2.5, sw=1.0)
        cx = x_stage + 27
        svg.add(f'<circle cx="{cx:.1f}" cy="{ry+31:.1f}" r="13" fill="{accent}"/>')
        svg.text(cx, ry + 36, st["num"], size=14, family=SANS, fill="#ffffff",
                 anchor="middle", weight="700")
        svg.text(x_stage + 49, ry + 36, st["name"], size=15.5, family=MONO, weight="700")
        for k, ln in enumerate(r["subs"]):
            svg.check(ln, sub_size, sub_avail, False, "subtitle")
            svg.text(x_stage + 24, ry + 57 + k * sub_lh, ln, size=sub_size, family=SANS,
                     fill=MUTED)
        if r["notes"]:
            ny = max(ry + 57 + len(r["subs"]) * sub_lh + 10, ry + rh - 14 - len(r["notes"]) * note_lh)
            for k, ln in enumerate(r["notes"]):
                svg.check(ln, note_size, note_avail, False, "note")
                svg.text(x_stage + 14, ny + k * note_lh, ln, size=note_size, family=SANS,
                         fill=FAINT)

        cy = ry + 24
        for ln in r["cmds"]:
            if ln:
                svg.check(ln, cmd_size, cmd_avail, True, "cmd")
                svg.text(x_cmd, cy, ln, size=cmd_size, family=MONO,
                         fill=FAINT if ln.lstrip().startswith("#") else INK)
            cy += cmd_lh

        ay = ry + 24
        for a in r["arts"]:
            svg.check(a, art_size, art_avail, True, "artifact")
            svg.text(x_art, ay, a, size=art_size, family=MONO, fill=SLATE)
            ay += art_lh

        if i < len(rows) - 1:
            svg.arrow_down(x_stage + w_stage / 2, ry + rh + 2, ry + rh + gap - 2)

    fy = rows[-1]["y"] + rows[-1]["h"] + 26
    svg.rect(26, fy, W - 52, 72, fill=SOFT, stroke=LINE, rx=10)
    svg.text(44, fy + 28, "运行收尾", size=13, family=SANS, weight="700")
    tail = "<work>/sfm_timing.json（各阶段耗时） · stdout ISAT_EVENT sfm.pipeline_timing · stderr 阶段耗时表 · logs/run_<ts>/"
    svg.check(tail, 12, W - 52 - 36, True, "footer")
    svg.text(44, fy + 52, tail, size=12, family=MONO, fill=SLATE)
    svg.text(26, total_h - 18, "由 scripts/gen_pipeline_diagrams.py 生成；默认参数取自 src/cli/isat_sfm.cpp",
             size=11, family=SANS, fill=FAINT)
    return svg


# ─────────────────────────────────────────────────────────────────────────────
# Diagram 2 — match step decision tree
# ─────────────────────────────────────────────────────────────────────────────
def diagram_match_detail() -> Svg:
    W, H = 1300.0, 1180.0
    svg = Svg(W, H)
    svg.text(26, 50, "match 阶段决策图", size=24, family=SANS, weight="700")
    svg.text(26, 77, "候选对来源、匹配实现、几何后端与焦距回写的实际分支；默认不是 VLAD / GPS 向量检索",
             size=13.5, family=SANS, fill=MUTED)
    svg.text(26, 100, "基线 main@305bbd1 · 本阶段由 isat_sfm 以子进程方式调度", size=12,
             family=MONO, fill=FAINT)

    def node(x, y, w, h, title, lines, accent, fill=CARD, tsize=13.5, lsize=12.0):
        svg.rect(x, y, w, h, fill=fill, stroke=accent, rx=9, sw=1.5)
        svg.rect(x, y + 1.5, w, 3.4, fill=accent, stroke=accent, rx=1.7, sw=1.0)
        t = svg.check(title, tsize, w - 24, False, "node title")
        svg.text(x + w / 2, y + 27, t, size=tsize, family=SANS, anchor="middle",
                 weight="700", fill=INK)
        ly = y + 27 + 19
        for ln in lines:
            svg.check(ln, lsize, w - 22, True, "node line")
            svg.text(x + w / 2, ly, ln, size=lsize, family=MONO, anchor="middle", fill=SLATE)
            ly += 17

    # 1. entry
    node(500, 116, 300, 64, "读取图像数", ["n_img = |images_all.json.images|"], "#0f172a")
    svg.arrow_down(650, 180, 222)

    # 2. decision
    svg.rect(380, 222, 540, 90, fill="#fffbeb", stroke="#f59e0b", rx=9, sw=1.6)
    svg.text(650, 254, "◇ 走穷举配对？", size=14.5, family=SANS, anchor="middle",
             weight="700", fill="#b45309")
    svg.check("n_img < --auto-exhaustive-max-images（默认 60）", 12, 500, True, "decision1")
    svg.text(650, 278, "n_img < --auto-exhaustive-max-images（默认 60）", size=12,
             family=MONO, anchor="middle", fill=SLATE)
    svg.check("或显式 --exhaustive-match", 12, 500, True, "decision2")
    svg.text(650, 297, "或显式 --exhaustive-match", size=12, family=MONO, anchor="middle",
             fill=SLATE)

    # 3. branches
    svg.path("M 650 312 V 330 H 210 V 348", stroke="#db2777", sw=1.6)
    svg.head(210, 348, "down", "#db2777")
    svg.text(196, 327, "是", size=13, family=SANS, fill="#db2777", weight="700", anchor="end")
    node(60, 348, 300, 84, "本地穷举配对（不调子进程）",
         ["write match/pairs_retrieve.json", "= C(n_img, 2) 全部图像对"], "#db2777",
         fill="#fdf2f8", tsize=13.0)

    svg.path("M 650 312 V 330 H 1090 V 348", stroke="#db2777", sw=1.6)
    svg.head(1090, 348, "down", "#db2777")
    svg.text(1104, 327, "否", size=13, family=SANS, fill="#db2777", weight="700")
    node(940, 348, 300, 100, "isat_retrieval_match（非 VLAD/GPS）",
         ["Wu / VisualSFM 风格", "小图 SIFT 穷举匹配 + F 验证",
          "-o match/pairs_retrieve.json"], "#db2777", fill="#fdf2f8", tsize=12.5)

    # 4. merge
    svg.path("M 210 432 V 460 H 650 V 486", stroke=LINE, sw=1.5)
    svg.head(650, 486, "down", FAINT)
    svg.path("M 1090 448 V 460 H 650", stroke=LINE, sw=1.5)
    node(380, 486, 540, 104, "候选对：match/pairs_retrieve.json",
         ["穷举路径：C(n_img, 2) 全对",
          "默认：小图 SIFT 穷举匹配 + F 验证",
          "再并入 low_peak 图的穷举对"], "#db2777", fill=SOFT, tsize=13.0,
         lsize=11.5)

    # 5. matching
    svg.arrow_down(650, 590, 624)
    node(400, 624, 500, 84, "特征匹配（全分辨率）",
         ["isat_gpu_cascade_hashing_match", "→ match/pairs_matched.json"], "#7c3aed")

    svg.line(650, 708, 650, 718, stroke="#c4b5fd", sw=1.5)
    svg.path("M 230 718 H 1070", stroke="#c4b5fd", sw=1.5)
    impl = [(60, "cascade", "isat_cpu_cascade_hashing_match", "#7c3aed"),
            (480, "cascade-gpu（默认）", "isat_gpu_cascade_hashing_match", "#7c3aed"),
            (900, "siftgpu", "isat_match --use-sift-gpu", "#7c3aed")]
    for x, name, cmd, col in impl:
        svg.rect(x, 736, 340, 66, fill=CARD, stroke="#ddd6fe", rx=9, sw=1.4)
        svg.check(name, 13, 316, False, "impl title")
        svg.text(x + 14, 762, name, size=13, family=SANS, weight="700", fill=col)
        svg.check(cmd, 11.5, 316, True, "impl cmd")
        svg.text(x + 14, 786, cmd, size=11.5, family=MONO, fill=SLATE)
        svg.line(x + 170, 718, x + 170, 732, stroke="#c4b5fd", sw=1.5)
        svg.head(x + 170, 736, "down", "#c4b5fd", 4.2)

    svg.path("M 230 802 H 1070", stroke=LINE, sw=1.4)
    svg.line(650, 802, 650, 812, stroke=LINE, sw=1.4)
    svg.arrow_down(650, 812, 846)

    node(400, 846, 500, 80, "几何验证：F / E / H RANSAC",
         ["--geo-min-inliers 10   --geo-thresh-f 16.0 px",
          "→ geo/*.isat_geo + geo/pairs.json"], "#db2777")

    svg.line(650, 926, 650, 940, stroke=LINE, sw=1.4)
    svg.path("M 232 940 H 1068", stroke=LINE, sw=1.4)
    geo = [(26, "cuda（默认）", "isat_geo_cuda（8 点 E）", "#059669"),
           (444, "gpu-gl", "isat_geo --backend gpu-gl", "#0891b2"),
           (862, "poselib", "isat_geo --backend poselib", "#64748b")]
    for x, name, cmd, col in geo:
        svg.rect(x, 956, 412, 64, fill=CARD, stroke=col, rx=9, sw=1.4)
        svg.check(name, 13, 388, False, "geo title")
        svg.text(x + 14, 980, name, size=13, family=SANS, weight="700", fill=col)
        svg.check(cmd, 11.5, 388, True, "geo cmd")
        svg.text(x + 14, 1003, cmd, size=11.5, family=MONO, fill=SLATE)
        svg.line(x + 206, 940, x + 206, 952, stroke=LINE, sw=1.4)
        svg.head(x + 206, 956, "down", FAINT, 4.2)
    svg.text(232, 1040, "isat_geo_cuda 二进制缺失时自动降级到 gpu-gl", size=11, family=SANS,
             fill=FAINT, anchor="middle")

    # 6. focal-from-geo
    svg.rect(26, 1066, 1248, 92, fill=SOFT, stroke="#0ea5e9", rx=9, sw=1.5)
    svg.text(44, 1094, "几何验证之后：isat_focal_from_geo（--focal-from-geo auto|always|never）",
             size=13, family=SANS, weight="700", fill="#0369a1")
    for k, ln in enumerate([
        "auto  ：camera_estimate_meta.json 的 needs_focal_from_geo / any_fallback 为真，或内参疑似 f35=35 兜底 → 由视图图 F 矩阵估 fx 并回写 images_all.json",
        "always：无条件执行，失败即中止          never：跳过",
    ]):
        svg.check(ln, 11.5, 1212, False, "focal line")
        svg.text(44, 1117 + k * 17, ln, size=11.5, family=MONO, fill=SLATE)
    return svg



# ─────────────────────────────────────────────────────────────────────────────
# Diagram 3 — the incremental SfM process itself
#   Source: src/algorithm/modules/sfm/incremental_sfm_pipeline.{h,cpp} and the
#   option values that src/cli/isat_incremental_sfm.cpp actually sets.
# ─────────────────────────────────────────────────────────────────────────────
INIT_CARDS = [
    dict(name="载入轨迹与视图图", accent="#2563eb",
         desc=["读取 tracks.isat_tracks；若其中没有内嵌 view graph，则由 pairs.json 与 geo/ 的 "
               "F/E/H 结果重建两视图图。"],
         params=["load_track_store_from_idc(...)", "build_view_graph_from_geo(...)"]),
    dict(name="锚定世界坐标系", accent="#2563eb",
         desc=["初始对的第一张图 im0 定义为世界原点，作为所有 global BA 的固定锚点，"
               "防止规范自由度漂移。"],
         params=["C(im0) = 0    R(im0) = I", "anchor_image = im0"]),
    dict(name="预热求解后端", accent="#2563eb",
         desc=["进入主循环前先初始化 resection 后端与 GPU 上下文，"
               "避免第一次 resection 承担 EGL / shader 编译开销。"],
         params=["resection_init_gpu()", "backend: poselib（默认）| cuda"]),
]

LOOP_STEPS = [
    dict(name="选择候选图像", accent="#db2777",
         desc=["按可见性金字塔覆盖率排序未注册图像并做 3D-2D 数量门限；每迭代最多取 40 个候选，"
               "用三角化轨迹计数作缓存 watermark。"],
         params=["min_3d2d_count = 30",
                 "min_visibility_coverage = 0.02（6 层金字塔）"]),
    dict(name="Resection（PnP）", accent="#db2777",
         desc=["对候选逐个 dry-run，在通过硬门限者中挑最优后提交注册（最多试 8 个）；"
               "提交时按需写回外点。"],
         params=["min_inliers = 30    min_inlier_ratio = 0.10",
                 "大场景 ≥ 0.15    期望 50 / 0.20    RANSAC 4 px"]),
    dict(name="三角化新增观测", accent="#db2777",
         desc=["用新注册的相机三角化新轨迹，并更新既有轨迹的观测。"],
         params=["commit_reproj_px = 16.0",
                 "min_angle_deg = 0.5    max_angle_deg = 120"]),
    dict(name="BA 调度（三阶段）", accent="#7c3aed",
         desc=["按已注册图像数决定本轮跑 global BA 还是 local BA，并给出 global BA 的节奏。"],
         params=["n < 41：每次注册都跑 global BA",
                 "41 ≤ n < 100：linear gap = ceil(5 + 0.12·n)",
                 "n ≥ 100：每迭代 local BA（kBatchNeighbor, k=8）",
                 "         + 周期 global，gap = ceil(22 + 0.06·n)"]),
    dict(name="BA + 迭代外点剔除", accent="#7c3aed",
         desc=["Huber 稳健核叠 MAD 迭代剔除，最多 10 轮；内参按相机注册数分相位渐进解锁。"],
         params=["threshold_px 4.0    mad_k 2.5    Huber δ 0.5–3.0 px",
                 "角 0.5°–120°    深度 ≤ 200× 场景中位",
                 "内参：n<3 全固定 → ≥3 fx+k1 → ≥10 +k2 → ≥50 全部"]),
    dict(name="重三角化", accent="#ea580c",
         desc=["按范围与周期恢复未三角化或被剔除的轨迹。"],
         params=["kNewImages（每次 local BA 之后）",
                 "kPendingOnly 每 3 次迭代    kFullScan 每 10 次迭代"]),
    dict(name="观测恢复", accent="#ea580c",
         desc=["global BA 后若某相机焦距变化超过阈值，重新评估标记为 kRestorable 的观测并恢复合格者。"],
         params=["|Δfx/fx| > 0.02 触发 → restore_reproj_px = 4.0"]),
    dict(name="快照与循环判断", accent="#0891b2",
         desc=["按 --debug-interval 写 Bundler 迭代快照；只要还有候选图像就回到第 1 步。"],
         params=["sfm_interval/iter_NNNN/（--debug-dir）",
                 "无候选连续 2 次 → global BA + kFullScan 救援"]),
]

FINISH_CARDS = [
    dict(name="最终重三角化 + 全局 BA", accent="#059669",
         desc=["收尾做一次 kPendingOnly 重三角化与最终 global BA，按紧门限剔除后不再做 kFullScan，"
               "避免把刚剔除的轨迹又三角化回来。"],
         params=["final: kPendingOnly → global BA"]),
    dict(name="导出成果", accent="#059669",
         desc=["写出位姿、Bundler 与 COLMAP 稀疏模型，并回写更新后的轨迹存储。"],
         params=["poses.json / bundler/bundle.out",
                 "colmap/sparse/0 / tracks.isat_tracks"]),
]


def diagram_sfm_process() -> Svg:
    W = 1300.0
    L, R = 26.0, 1274.0
    svg = Svg(W, 10)
    svg.text(26, 50, "增量 SfM（incremental_sfm 阶段）内部流程", size=24, family=SANS, weight="700")
    svg.text(26, 77, "由 isat_incremental_sfm 调用 run_incremental_sfm_pipeline；"
                     "每次迭代注册一张新图像，循环直到没有可注册的候选",
             size=13.5, family=SANS, fill=MUTED)
    svg.text(26, 100, "基线 main@305bbd1 · 参数取值来自 src/cli/isat_incremental_sfm.cpp 实际写入的 "
                      "IncrementalSfMOptions", size=12, family=MONO, fill=FAINT)

    y = 132.0
    # ── 阶段 0
    svg.add(f'<circle cx="34" cy="{y-5:.1f}" r="5" fill="#2563eb"/>')
    svg.text(48, y, "阶段 0 · 载入与初始化", size=14, family=SANS, weight="700", fill="#2563eb")
    svg.line(230, y - 5, R, y - 5, stroke="#e2e8f0", sw=1.2)
    y += 16
    gap = 22.0
    cw = (R - L - 2 * gap) / 3.0
    dh, ph, dlh, plh = 27.0, 0.0, 16.0, 16.0
    heights = []
    for c in INIT_CARDS:
        d = wrap(c["desc"][0], 11.5, cw - 30, False)
        heights.append(max(27 + len(d) * dlh + 10 + len(c["params"]) * plh + 16, 120.0))
    ch = max(heights)
    for i, c in enumerate(INIT_CARDS):
        x = L + i * (cw + gap)
        svg.rect(x, y, cw, ch, fill=SOFT, stroke=c["accent"], rx=9, sw=1.4)
        svg.check(c["name"], 13.5, cw - 28, False, "init title")
        svg.text(x + 15, y + 26, c["name"], size=13.5, family=SANS, weight="700", fill=c["accent"])
        dy = y + 26 + 19
        for ln in wrap(c["desc"][0], 11.5, cw - 30, False):
            svg.check(ln, 11.5, cw - 30, False, "init desc")
            svg.text(x + 15, dy, ln, size=11.5, family=SANS, fill=MUTED)
            dy += dlh
        dy += 8
        for ln in c["params"]:
            svg.check(ln, 11.5, cw - 30, True, "init param")
            svg.text(x + 15, dy, ln, size=11.5, family=MONO, fill=SLATE)
            dy += plh
    y += ch

    # ── 阶段 1
    y += 34
    svg.add(f'<circle cx="34" cy="{y-5:.1f}" r="5" fill="#ca8a04"/>')
    svg.text(48, y, "阶段 1 · 初始对搜索与两视图初始化", size=14, family=SANS, weight="700", fill="#b45309")
    svg.line(330, y - 5, R, y - 5, stroke="#e2e8f0", sw=1.2)
    y += 16
    desc = ("在视图图上按得分顺序枚举初始对，逐对做两视图重建并跑 BA，全部门限通过才接受；"
            "失败则继续枚举，直到找到可用初始对（run_initial_pair_loop）。")
    dlines = wrap(desc, 12, 620, False)
    params = ["min_tracks_for_intital_pair = 50",
              "min_num_inliers（E-RANSAC）= 100",
              "max_forward_motion = 0.95（|tz| / ‖t‖）",
              "min_angle_deg = 2.0    min_median_angle_deg = 30.0",
              "BA RMSE ≤ ba_rmse_max = 10.0 px",
              "搜索范围 100 × 50（max_first / max_second images）"]
    ih = max(27 + len(dlines) * 16 + 18, 27 + len(params) * 16 + 18, 110.0)
    svg.rect(L, y, R - L, ih, fill="#fffbeb", stroke="#f59e0b", rx=9, sw=1.5)
    svg.text(L + 16, y + 27, "run_initial_pair_loop", size=13.5, family=MONO, weight="700", fill="#b45309")
    dy = y + 27 + 20
    for ln in dlines:
        svg.text(L + 16, dy, ln, size=12, family=SANS, fill=SLATE)
        dy += 16
    py = y + 27 + 20
    for ln in params:
        svg.check(ln, 11.5, 600, True, "initpair param")
        svg.text(L + 680, py, ln, size=11.5, family=MONO, fill=SLATE)
        py += 16
    y += ih

    # ── 阶段 2 · 主循环
    y += 34
    svg.add(f'<circle cx="34" cy="{y-5:.1f}" r="5" fill="#db2777"/>')
    svg.text(48, y, "阶段 2 · 增量主循环 —— 每次迭代注册 1 张新图像", size=14, family=SANS,
             weight="700", fill="#be185d")
    svg.line(430, y - 5, R, y - 5, stroke="#e2e8f0", sw=1.2)
    y += 18

    c1, c2, c3 = 90.0, 300.0, 840.0
    row_h = []
    for s in LOOP_STEPS:
        d = wrap(s["desc"][0], 11.5, c2 + 520 - c2 - 20, False)
        row_h.append(max(46.0, len(d) * 16 + 20, len(s["params"]) * 16 + 20))
    inner = sum(row_h) + 12 * (len(LOOP_STEPS) - 1) + 34
    ctop = y
    svg.rect(L, ctop, R - L, inner, fill="#fff7fb", stroke="#f9a8d4", rx=14, sw=1.6, dash="6 5")
    svg.text(L + 18, ctop + 22, "循环体：① 选择候选 → ② Resection → ③ 三角化 → ④ BA → ⑤ 剔除 → ⑥ 重三角化 "
                                "→ ⑦ 观测恢复 → ⑧ 快照判据", size=11.5, family=SANS, fill="#be185d",
             weight="700")

    ry = ctop + 36
    centers = []
    for i, (s, h) in enumerate(zip(LOOP_STEPS, row_h)):
        accent = s["accent"]
        svg.rect(c1, ry, R - 26 - c1, h, fill=CARD, stroke="#f1f5f9", rx=8, sw=1.2)
        svg.rect(c1, ry + 2, 4.0, h - 4, fill=accent, stroke=accent, rx=2.0, sw=1.0)
        svg.add(f'<circle cx="{c1+22:.1f}" cy="{ry+h/2:.1f}" r="12" fill="{accent}"/>')
        svg.text(c1 + 22, ry + h / 2 + 4.5, str(i + 1), size=12.5, family=SANS, fill="#ffffff",
                 anchor="middle", weight="700")
        svg.check(s["name"], 13, 170, False, "loop name")
        svg.text(c1 + 42, ry + h / 2 + 4.5, s["name"], size=13, family=SANS, weight="700", fill=INK)
        dy = ry + 20
        for ln in wrap(s["desc"][0], 11.5, c2 + 520 - c2 - 20, False):
            svg.check(ln, 11.5, c2 + 520 - c2 - 20, False, "loop desc")
            svg.text(c2, dy, ln, size=11.5, family=SANS, fill=MUTED)
            dy += 16
        py = ry + 20
        for ln in s["params"]:
            svg.check(ln, 11.5, R - 26 - c3 - 14, True, "loop param")
            svg.text(c3, py, ln, size=11.5, family=MONO, fill=SLATE)
            py += 16
        centers.append(ry + h / 2)
        ry += h + 12

    # loop-back rail
    rail_x = 62.0
    svg.path(f"M {c1} {centers[-1]:.1f} H {rail_x:.1f} V {centers[0]:.1f} H {c1 - 7:.1f}",
             stroke="#be185d", sw=1.6)
    svg.head(c1, centers[0], "right", "#be185d", 5.0)
    svg.add(f'<text x="{rail_x-8:.1f}" y="{(centers[0]+centers[-1])/2:.1f}" font-family="{SANS}" '
            f'font-size="11.5" fill="#be185d" font-weight="700" text-anchor="middle" '
            f'transform="rotate(-90 {rail_x-8:.1f} {(centers[0]+centers[-1])/2:.1f})">'
            f'仍有候选图像 → 下一轮迭代</text>')
    y = ctop + inner

    # ── 阶段 3
    y += 34
    svg.add(f'<circle cx="34" cy="{y-5:.1f}" r="5" fill="#059669"/>')
    svg.text(48, y, "阶段 3 · 收尾与导出", size=14, family=SANS, weight="700", fill="#047857")
    svg.line(200, y - 5, R, y - 5, stroke="#e2e8f0", sw=1.2)
    y += 16
    gap2 = 22.0
    cw2 = (R - L - gap2) / 2.0
    heights2 = []
    for c in FINISH_CARDS:
        d = wrap(c["desc"][0], 11.5, cw2 - 30, False)
        heights2.append(max(27 + len(d) * 16 + 10 + len(c["params"]) * 16 + 16, 118.0))
    ch2 = max(heights2)
    for i, c in enumerate(FINISH_CARDS):
        x = L + i * (cw2 + gap2)
        svg.rect(x, y, cw2, ch2, fill="#f0fdf4", stroke=c["accent"], rx=9, sw=1.4)
        svg.check(c["name"], 13.5, cw2 - 28, False, "finish title")
        svg.text(x + 15, y + 26, c["name"], size=13.5, family=SANS, weight="700", fill=c["accent"])
        dy = y + 45
        for ln in wrap(c["desc"][0], 11.5, cw2 - 30, False):
            svg.check(ln, 11.5, cw2 - 30, False, "finish desc")
            svg.text(x + 15, dy, ln, size=11.5, family=SANS, fill=MUTED)
            dy += 16
        dy += 8
        for ln in c["params"]:
            svg.check(ln, 11.5, cw2 - 30, True, "finish param")
            svg.text(x + 15, dy, ln, size=11.5, family=MONO, fill=SLATE)
            dy += 16
    y += ch2 + 34

    svg.text(26, y, "由 scripts/gen_pipeline_diagrams.py 生成；门限取自 incremental_sfm_pipeline.h 与 "
                    "isat_incremental_sfm.cpp", size=11, family=SANS, fill=FAINT)
    svg.h = y + 12
    return svg

def _find_chrome() -> str | None:
    for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
        found = shutil.which(name)
        if found:
            return found
    return None


def rasterize(svg_path: Path, width: float, height: float, scale: int = PNG_SCALE) -> bool:
    """Render ``svg_path`` to a sibling PNG via headless Chrome."""
    chrome = _find_chrome()
    if not chrome:
        print(f"    ! no Chromium-based browser on PATH; skipping {svg_path.with_suffix('.png').name}")
        return False
    png_path = svg_path.with_suffix(".png")
    with tempfile.TemporaryDirectory(prefix="isat-diagram-") as profile:
        cmd = [chrome, "--headless=new", "--disable-gpu", "--no-sandbox",
               "--disable-dev-shm-usage", "--no-first-run", "--hide-scrollbars",
               f"--user-data-dir={profile}",
               f"--force-device-scale-factor={scale}",
               f"--window-size={width:.0f},{height:.0f}",
               f"--screenshot={png_path}", svg_path.as_uri()]
        proc = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    if proc.returncode != 0 or not png_path.exists():
        print(f"    ! chrome rasterization failed (rc={proc.returncode})")
        return False
    print(f"wrote {png_path.relative_to(ROOT)} ({scale}x)")
    return True


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, svg in (("isat_sfm_pipeline.svg", diagram_overview()),
                      ("isat_sfm_match_detail.svg", diagram_match_detail()),
                      ("isat_sfm_process.svg", diagram_sfm_process())):
        path = OUT_DIR / name
        path.write_text(svg.render(), encoding="utf-8")
        worst = max(svg.checks, key=lambda c: c[1] / c[2])
        print(f"wrote {path.relative_to(ROOT)}")
        print(f"    {len(svg.checks)} text bounds checked; tightest = {worst[0]!r} "
              f"{worst[1]:.0f}/{worst[2]:.0f}px ({100*worst[1]/worst[2]:.0f}%)")
        rasterize(path, svg.w, svg.h)


if __name__ == "__main__":
    main()
