#!/usr/bin/env python3
"""
Visualize SIFT features and matches for InsightAT image pairs.

──────────────────────────────────────────────────────────────────────────────
Single-pair PNG mode (original behavior):
    python3 viz_matches.py --images images_all.json \\
        --feat feat/ --match match/ --geo geo/ \\
        --pair 3,49 [--out viz_3_49.png]

Batch PNG mode (one PNG per pair):
    python3 viz_matches.py --images images_all.json \\
        --feat feat/ --match match/ --geo geo/ \\
        --batch-geo geo/ --out-dir viz_out/
        [--pairs 3,49 5,12]          # subset; omit = all pairs in geo/

Batch HTML mode (single interactive HTML, no pixel copying):
    python3 viz_matches.py --images images_all.json \\
        --feat feat/ --match match/ --geo geo/ \\
        --batch-geo geo/ --html-out viz_all.html
        [--pairs 3,49 5,12]

HTML viewer features:
  - Lazy canvas rendering on image load
  - Toggle: raw matches / F-inliers / keypoints
  - Statistics table per pair (n_matches, F_inliers, E_inliers, ratio)
  - Filter by pair ID in the nav bar
  - Thumbnail grid → click to expand full detail
  - Works in Firefox out of the box (file:// access to local images).
    Chrome: either serve with "python3 -m http.server" or launch with
            --allow-file-access-from-files flag.
──────────────────────────────────────────────────────────────────────────────
"""

import argparse
import json
import struct
import sys
import re
from pathlib import Path

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# IDC reader
# ─────────────────────────────────────────────────────────────────────────────

_DTYPE_MAP = {
    "float32": np.float32,
    "float64": np.float64,
    "uint8":   np.uint8,
    "uint16":  np.uint16,
    "int32":   np.int32,
    "int64":   np.int64,
}


def read_idc(path: str):
    """Return (header_dict, {blob_name: ndarray})."""
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"ISAT":
            raise ValueError(f"{path}: bad magic {magic!r}")
        _ver = struct.unpack("<I", f.read(4))[0]
        jsz  = struct.unpack("<Q", f.read(8))[0]
        hdr  = json.loads(f.read(jsz).decode("utf-8"))
        pos  = 16 + jsz
        pad  = (8 - pos % 8) % 8
        f.read(pad)
        payload = f.read()

    blobs = {}
    for b in hdr.get("blobs", []):
        name   = b["name"]
        offset = b["offset"]
        size   = b["size"]
        if size == 0:
            blobs[name] = np.array([], dtype=_DTYPE_MAP.get(b["dtype"], np.float32))
            continue
        dtype = _DTYPE_MAP.get(b["dtype"], np.float32)
        arr   = np.frombuffer(payload[offset: offset + size], dtype=dtype)
        blobs[name] = arr.reshape(b["shape"])
    return hdr, blobs


# ─────────────────────────────────────────────────────────────────────────────
# Data loading helpers
# ─────────────────────────────────────────────────────────────────────────────

def load_image_list(images_json: str) -> dict:
    """Return {image_index: path_str}."""
    with open(images_json) as f:
        data = json.load(f)
    return {e["image_index"]: e["path"] for e in data["images"]}


def load_pair_data(idx1: int, idx2: int,
                   feat_dir: str, match_dir: str, geo_dir,
                   image_paths: dict):
    """
    Load all data for a pair and return a dict ready for rendering.
    Returns None if any required file is missing.
    """
    lo, hi = min(idx1, idx2), max(idx1, idx2)

    feat1_path = Path(feat_dir)  / f"{idx1}.isat_feat"
    feat2_path = Path(feat_dir)  / f"{idx2}.isat_feat"
    match_path = Path(match_dir) / f"{lo}_{hi}.isat_match"

    for p in (feat1_path, feat2_path, match_path):
        if not p.exists():
            print(f"  [skip] missing {p}", file=sys.stderr)
            return None

    try:
        _, fb1 = read_idc(str(feat1_path))
        _, fb2 = read_idc(str(feat2_path))
        mhdr, mb = read_idc(str(match_path))
    except Exception as e:
        print(f"  [skip] error reading pair ({idx1},{idx2}): {e}", file=sys.stderr)
        return None

    kp1 = fb1["keypoints"].tolist()
    kp2 = fb2["keypoints"].tolist()

    coords = mb["coords_pixel"]
    # Normalise orientation: stored as (lo,hi), ensure idx1 is image1
    stored_i1 = mhdr["image_pair"]["image1_index"]
    if stored_i1 == idx2:
        coords = coords[:, [2, 3, 0, 1]]
    coords_list = coords.tolist()

    # Geo (optional)
    F_inliers = []
    E_inliers = []
    geo_meta  = {}
    if geo_dir:
        geo_path = Path(geo_dir) / f"{lo}_{hi}.isat_geo"
        if geo_path.exists():
            try:
                ghdr, gb = read_idc(str(geo_path))
                fi  = gb.get("F_inliers")
                ei  = gb.get("E_inliers")
                F_inliers = fi.tolist() if fi is not None and len(fi) else []
                E_inliers = ei.tolist() if ei is not None and len(ei) else []
                g = ghdr.get("geometry", {})
                geo_meta = {
                    "F_estimated": g.get("F", {}).get("estimated", False),
                    "F_inliers":   g.get("F", {}).get("num_inliers", 0),
                    "F_ratio":     round(g.get("F", {}).get("inlier_ratio", 0.0), 3),
                    "E_estimated": g.get("E", {}).get("estimated", False),
                    "E_inliers":   g.get("E", {}).get("num_inliers", 0),
                    "H_estimated": g.get("H", {}).get("estimated", False),
                    "H_inliers":   g.get("H", {}).get("num_inliers", 0),
                    "degenerate":  g.get("degeneracy", {}).get("is_degenerate", False),
                    "h_over_f":    round(g.get("degeneracy", {}).get("h_over_f_ratio", 0.0), 3),
                }
            except Exception as e:
                print(f"  [warn] geo read error for ({idx1},{idx2}): {e}", file=sys.stderr)

    return {
        "idx1":      idx1,
        "idx2":      idx2,
        "path1":     image_paths.get(idx1, ""),
        "path2":     image_paths.get(idx2, ""),
        "kp1":       kp1,
        "kp2":       kp2,
        "matches":   coords_list,
        "F_inliers": F_inliers,
        "E_inliers": E_inliers,
        "geo":       geo_meta,
    }


def parse_geo_dir_pairs(geo_dir: str):
    """Scan geo/ for *.isat_geo files and return list of (lo, hi) pairs."""
    pattern = re.compile(r'^(\d+)_(\d+)\.isat_geo$')
    pairs = []
    for f in sorted(Path(geo_dir).iterdir()):
        m = pattern.match(f.name)
        if m:
            pairs.append((int(m.group(1)), int(m.group(2))))
    return pairs


# ─────────────────────────────────────────────────────────────────────────────
# PNG rendering (single pair)
# ─────────────────────────────────────────────────────────────────────────────

def render_png(pair: dict, out_path: str, no_feat: bool = False, max_kp: int = 3000):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.lines import Line2D
    from PIL import Image as PILImage

    def load_img(p):
        return np.array(PILImage.open(p).convert("RGB"))

    def side_by_side(a, b):
        h = max(a.shape[0], b.shape[0])
        cv = np.zeros((h, a.shape[1] + b.shape[1], 3), dtype=np.uint8)
        cv[:a.shape[0], :a.shape[1]] = a
        cv[:b.shape[0], a.shape[1]:] = b
        return cv, a.shape[1]

    img1 = load_img(pair["path1"])
    img2 = load_img(pair["path2"])
    canvas, w1 = side_by_side(img1, img2)

    kp1    = np.array(pair["kp1"])    if pair["kp1"]    else np.zeros((0, 4))
    kp2    = np.array(pair["kp2"])    if pair["kp2"]    else np.zeros((0, 4))
    coords = np.array(pair["matches"]) if pair["matches"] else np.zeros((0, 4))
    F_mask = np.array(pair["F_inliers"], dtype=bool) if pair["F_inliers"] else None
    E_mask = np.array(pair["E_inliers"], dtype=bool) if pair["E_inliers"] else None
    n_m    = len(coords)
    n_F    = int(F_mask.sum()) if F_mask is not None else 0
    n_E    = int(E_mask.sum()) if E_mask is not None else 0
    geo    = pair["geo"]

    panels = []
    if not no_feat:
        panels.append("feat")
    panels.append("all_matches")
    if F_mask is not None:
        panels.append("geo")

    fig, axes = plt.subplots(len(panels), 1, figsize=(20, 8 * len(panels)), dpi=120)
    if len(panels) == 1:
        axes = [axes]

    fig.suptitle(
        f"img{pair['idx1']} ↔ img{pair['idx2']}  raw={n_m}  F_inliers={n_F}  E_inliers={n_E}",
        fontsize=11)

    for panel, ax in zip(panels, axes):
        ax.imshow(canvas); ax.axis("off")

        if panel == "feat":
            ax.set_title(f"Keypoints  img{pair['idx1']}:{len(kp1)}  img{pair['idx2']}:{len(kp2)}", fontsize=9)
            def draw_kp(kpts, color, ox=0):
                if len(kpts) > max_kp:
                    kpts = kpts[np.random.choice(len(kpts), max_kp, replace=False)]
                for x, y, s, _ in kpts:
                    ax.add_patch(mpatches.Circle((x+ox, y), radius=max(s, 1.5),
                                                  lw=0.4, edgecolor=color, facecolor="none", alpha=0.5))
            draw_kp(kp1, "cyan");  draw_kp(kp2, "yellow", w1)

        elif panel == "all_matches":
            ax.set_title(f"Raw matches  n={n_m}", fontsize=9)
            if n_m > 0:
                for x1, y1, x2, y2 in coords:
                    ax.plot([x1, x2+w1], [y1, y2], color="deepskyblue", lw=0.5, alpha=0.6)
                ax.scatter(coords[:,0], coords[:,1], s=6, c="lime", zorder=3, linewidths=0)
                ax.scatter(coords[:,2]+w1, coords[:,3], s=6, c="lime", zorder=3, linewidths=0)

        elif panel == "geo":
            ax.set_title(
                f"Geometry  F={n_F}  E={n_E}  degen={geo.get('degenerate','?')}  h/f={geo.get('h_over_f','?')}",
                fontsize=9)
            if n_m > 0:
                for i, (x1, y1, x2, y2) in enumerate(coords):
                    isf = F_mask is not None and F_mask[i]
                    ax.plot([x1, x2+w1], [y1, y2],
                            color=("lime" if isf else "red"),
                            lw=(0.8 if isf else 0.35), alpha=(0.75 if isf else 0.3))
                if E_mask is not None:
                    for i in np.where(E_mask)[0]:
                        ax.plot([coords[i,0], coords[i,2]+w1], [coords[i,1], coords[i,3]],
                                color="dodgerblue", lw=1.4, alpha=0.9)
            ax.legend(handles=[
                Line2D([0],[0], color="lime",       lw=1.5, label=f"F-inlier ({n_F})"),
                Line2D([0],[0], color="dodgerblue", lw=1.5, label=f"E-inlier ({n_E})"),
                Line2D([0],[0], color="red",        lw=1.5, label=f"Outlier ({n_m-n_F})"),
            ], loc="upper right", fontsize=8)

    plt.tight_layout()
    plt.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved → {out_path}")


# ─────────────────────────────────────────────────────────────────────────────
# HTML rendering (batch, interactive canvas)
# ─────────────────────────────────────────────────────────────────────────────

_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>InsightAT Match Viewer</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: monospace; background: #1a1a1a; color: #ddd; }
#toolbar {
  position: sticky; top: 0; z-index: 100;
  background: #111; border-bottom: 1px solid #444;
  padding: 8px 12px; display: flex; gap: 12px; align-items: center; flex-wrap: wrap;
}
#toolbar input { background:#2a2a2a; color:#eee; border:1px solid #555; padding:4px 8px; border-radius:4px; width:200px; }
#toolbar button { background:#2a2a2a; color:#ccc; border:1px solid #555; padding:4px 10px; border-radius:4px; cursor:pointer; font-size:12px; }
#toolbar button.active { background:#2a7a4a; color:#fff; border-color:#2a7a4a; }
#toolbar label { font-size:12px; color:#aaa; cursor:pointer; }
#count { color:#888; font-size:12px; }
#grid { display:flex; flex-wrap:wrap; gap:10px; padding:10px; }
.card {
  background:#252525; border:1px solid #3a3a3a; border-radius:6px;
  overflow:hidden; cursor:pointer; transition:border-color .15s; width:340px;
}
.card:hover { border-color:#5af; }
.card.hidden { display:none; }
.card-header { padding:5px 8px; display:flex; justify-content:space-between; background:#2d2d2d; font-size:11px; }
.pair-id { color:#7cf; font-weight:bold; }
.thumb-wrap { position:relative; width:100%; height:110px; overflow:hidden; background:#111; display:flex; }
.thumb-wrap img { flex:1; height:100%; object-fit:cover; }
.thumb-badge { position:absolute; bottom:3px; right:5px; background:rgba(0,0,0,.7); color:#7fc; font-size:10px; padding:1px 5px; border-radius:3px; }
.card-row { display:flex; gap:4px; padding:4px 8px; font-size:10px; color:#888; flex-wrap:wrap; }
.card-row span { background:#2d2d2d; padding:1px 6px; border-radius:3px; }
.ok { color:#6d6; } .warn { color:#fa7; } .bad { color:#f55; }
#overlay { display:none; position:fixed; inset:0; z-index:200; background:rgba(0,0,0,.88); overflow-y:auto; padding:16px; }
#modal { background:#1e1e1e; border:1px solid #555; border-radius:8px; max-width:1700px; margin:0 auto; padding:12px; }
#modal-close { float:right; background:none; border:none; color:#888; font-size:22px; cursor:pointer; }
#modal-title { color:#7cf; font-size:13px; margin-bottom:8px; font-weight:bold; }
#modal-bar { display:flex; gap:8px; margin-bottom:8px; align-items:center; flex-wrap:wrap; }
#modal-bar button { background:#2a2a2a; color:#ccc; border:1px solid #555; padding:4px 10px; border-radius:4px; cursor:pointer; font-size:12px; }
#modal-bar button.active { background:#2a7a4a; color:#fff; border-color:#2a7a4a; }
#modal-stats { font-size:11px; color:#aaa; margin-bottom:8px; }
#modal-stats b { color:#ddd; }
#modal-stats .bad { color:#f55; }
#cwrap { position:relative; display:inline-flex; max-width:100%; }
#cwrap img { flex:1; max-width:50%; display:block; }
#ov-canvas { position:absolute; top:0; left:0; pointer-events:none; }
::-webkit-scrollbar { width:5px; height:5px; }
::-webkit-scrollbar-track { background:#111; }
::-webkit-scrollbar-thumb { background:#444; border-radius:3px; }
</style>
</head>
<body>
<div id="toolbar">
  <span style="color:#7cf;font-weight:bold;font-size:13px">InsightAT Match Viewer</span>
  <input id="q" type="text" placeholder="Filter by image id, e.g. 3 or 3_49">
  <label><input type="checkbox" id="chk-f"> F est. only</label>
  <label><input type="checkbox" id="chk-d"> Degen only</label>
  <button id="s-id"    class="active">Sort: ID</button>
  <button id="s-f">Sort: F↓</button>
  <button id="s-r">Sort: ratio↓</button>
  <span id="count"></span>
</div>
<div id="grid"></div>

<div id="overlay" onclick="tryClose(event)">
 <div id="modal" onclick="event.stopPropagation()">
  <button id="modal-close" onclick="closeModal()">✕</button>
  <div id="modal-title"></div>
  <div id="modal-bar">
   <button id="m-all" class="active" onclick="setMode('all')">All matches</button>
   <button id="m-geo"               onclick="setMode('geo')">F-inliers</button>
   <button id="m-kp"                onclick="setMode('kp')">Keypoints</button>
   <span style="color:#666;font-size:12px">← → navigate  Esc close</span>
  </div>
  <div id="modal-stats"></div>
  <div id="cwrap">
   <img id="mi1" onload="imgLoaded()">
   <img id="mi2" onload="imgLoaded()">
   <canvas id="ov-canvas"></canvas>
  </div>
 </div>
</div>

<script>
const P = __PAIRS_JSON__;
let cur = -1, mode = 'all', sort = 'id', loaded = 0;

function imgUrl(p){ return p.startsWith('/') ? 'file://'+p : p; }

// ── grid ──────────────────────────────────────────────────────────────────────
function buildGrid(){
  const g = document.getElementById('grid');
  g.innerHTML='';
  const idxs = [...Array(P.length).keys()];
  if(sort==='f')   idxs.sort((a,b)=>(P[b].geo?.F_inliers||0)-(P[a].geo?.F_inliers||0));
  if(sort==='r')   idxs.sort((a,b)=>{
    const ra=P[a].matches.length?(P[a].geo?.F_inliers||0)/P[a].matches.length:0;
    const rb=P[b].matches.length?(P[b].geo?.F_inliers||0)/P[b].matches.length:0;
    return rb-ra;
  });
  idxs.forEach(i=>{
    const p=P[i], geo=p.geo||{};
    const nF=geo.F_inliers||0, nM=p.matches.length;
    const pct=nM?Math.round(nF/nM*100):0;
    const fok=geo.F_estimated?'ok':'bad';
    const eok=geo.E_estimated?'ok':'';
    const dg=geo.degenerate?'bad':'';
    const c=document.createElement('div');
    c.className='card'; c.dataset.i=i; c.onclick=()=>openModal(i);
    c.innerHTML=`
    <div class="card-header">
      <span class="pair-id">${p.idx1} ↔ ${p.idx2}</span>
      <span>F: <b>${nF}</b>/${nM} (${pct}%)</span>
    </div>
    <div class="thumb-wrap">
      <img src="${imgUrl(p.path1)}" loading="lazy">
      <img src="${imgUrl(p.path2)}" loading="lazy">
      <span class="thumb-badge">${nM} matches</span>
    </div>
    <div class="card-row">
      <span class="${fok}">F=${geo.F_estimated?'✓':'✗'}&nbsp;${nF}</span>
      <span class="${eok}">E=${geo.E_estimated?'✓':'✗'}&nbsp;${geo.E_inliers||0}</span>
      <span>H&nbsp;${geo.H_inliers||0}</span>
      <span>h/f&nbsp;${(geo.h_over_f||0).toFixed(2)}</span>
      ${geo.degenerate?'<span class="bad">⚠ degen</span>':''}
    </div>`;
    g.appendChild(c);
  });
  applyFilter();
}

function applyFilter(){
  const q=document.getElementById('q').value.trim().replace(',','_');
  const fOnly=document.getElementById('chk-f').checked;
  const dOnly=document.getElementById('chk-d').checked;
  let v=0;
  document.querySelectorAll('.card').forEach(c=>{
    const i=+c.dataset.i, p=P[i], geo=p.geo||{};
    const id=`${p.idx1}_${p.idx2}`;
    let ok=true;
    if(q && !id.includes(q) && !String(p.idx1).includes(q) && !String(p.idx2).includes(q)) ok=false;
    if(fOnly && !geo.F_estimated) ok=false;
    if(dOnly && !geo.degenerate)  ok=false;
    c.classList.toggle('hidden',!ok);
    if(ok) v++;
  });
  document.getElementById('count').textContent=`${v} / ${P.length} pairs`;
}

// ── modal ─────────────────────────────────────────────────────────────────────
function openModal(i){
  cur=i; mode='all'; updateModeBtns();
  const p=P[i], geo=p.geo||{};
  document.getElementById('modal-title').textContent=
    `img${p.idx1} ↔ img${p.idx2}   ${p.path1.split('/').pop()} / ${p.path2.split('/').pop()}`;
  document.getElementById('modal-stats').innerHTML=
    `<span>raw: <b>${p.matches.length}</b></span> `+
    `<span>F: <b class="${geo.F_estimated?'ok':'bad'}">${geo.F_inliers||0}</b> (${((geo.F_ratio||0)*100).toFixed(0)}%)</span> `+
    `<span>E: <b>${geo.E_inliers||0}</b></span> `+
    `<span>H: <b>${geo.H_inliers||0}</b></span> `+
    `<span>h/f: ${(geo.h_over_f||0).toFixed(3)}</span> `+
    (geo.degenerate?'<span class="bad">⚠ DEGENERATE</span>':'');
  loaded=0;
  const i1=document.getElementById('mi1'), i2=document.getElementById('mi2');
  i1.src=imgUrl(p.path1); i2.src=imgUrl(p.path2);
  if(i1.complete) imgLoaded();
  if(i2.complete) imgLoaded();
  document.getElementById('overlay').style.display='block';
}

function closeModal(){ document.getElementById('overlay').style.display='none'; cur=-1; }
function tryClose(e){ if(e.target===document.getElementById('overlay')) closeModal(); }

function imgLoaded(){ if(++loaded>=2) draw(); }

function setMode(m){ mode=m; updateModeBtns(); draw(); }
function updateModeBtns(){
  ['all','geo','kp'].forEach(m=>document.getElementById('m-'+m).classList.toggle('active',mode===m));
}

function draw(){
  if(cur<0) return;
  const p=P[cur];
  const i1=document.getElementById('mi1'), i2=document.getElementById('mi2');
  const cv=document.getElementById('ov-canvas');
  if(!i1.naturalWidth||!i2.naturalWidth) return;

  const dw1=i1.offsetWidth, dh1=i1.offsetHeight;
  const dw2=i2.offsetWidth, dh2=i2.offsetHeight;
  const sx1=dw1/i1.naturalWidth, sy1=dh1/i1.naturalHeight;
  const sx2=dw2/i2.naturalWidth, sy2=dh2/i2.naturalHeight;
  const W=dw1+dw2, H=Math.max(dh1,dh2);

  cv.width=W; cv.height=H;
  cv.style.width=W+'px'; cv.style.height=H+'px';

  const ctx=cv.getContext('2d');
  ctx.clearRect(0,0,W,H);

  const Fi=p.F_inliers, Ei=p.E_inliers, ms=p.matches;

  if(mode==='kp'){
    const maxK=2000;
    const drawKp=(kps,sx,sy,ox,col)=>{
      ctx.strokeStyle=col; ctx.lineWidth=0.8;
      const step=Math.max(1,Math.ceil(kps.length/maxK));
      for(let j=0;j<kps.length;j+=step){
        const [x,y,s]=kps[j];
        ctx.beginPath(); ctx.arc(x*sx+ox,y*sy,Math.max(s*sx,1.5),0,Math.PI*2); ctx.stroke();
      }
    };
    drawKp(p.kp1,sx1,sy1,0,'rgba(0,230,200,.55)');
    drawKp(p.kp2,sx2,sy2,dw1,'rgba(240,210,0,.55)');
    return;
  }

  for(let j=0;j<ms.length;j++){
    const [x1,y1,x2,y2]=ms[j];
    const isF=Fi.length>j&&Fi[j], isE=Ei.length>j&&Ei[j];
    if(mode==='geo'&&!isF&&!isE) continue;
    ctx.strokeStyle=isE?'rgba(60,140,255,.9)':isF?'rgba(80,220,80,.8)':'rgba(220,50,50,.3)';
    ctx.lineWidth=isE?1.6:isF?1.0:0.5;
    ctx.beginPath(); ctx.moveTo(x1*sx1,y1*sy1); ctx.lineTo(x2*sx2+dw1,y2*sy2); ctx.stroke();
  }
  for(let j=0;j<ms.length;j++){
    const [x1,y1,x2,y2]=ms[j];
    const isF=Fi.length>j&&Fi[j], isE=Ei.length>j&&Ei[j];
    if(mode==='geo'&&!isF&&!isE) continue;
    const col=isE?'#4af':isF?'#6e6':'#f44'; const r=isE?4:3;
    ctx.fillStyle=col;
    ctx.beginPath(); ctx.arc(x1*sx1,y1*sy1,r,0,Math.PI*2); ctx.fill();
    ctx.beginPath(); ctx.arc(x2*sx2+dw1,y2*sy2,r,0,Math.PI*2); ctx.fill();
  }
}

// ── wiring ────────────────────────────────────────────────────────────────────
document.getElementById('q').addEventListener('input',applyFilter);
document.getElementById('chk-f').addEventListener('change',applyFilter);
document.getElementById('chk-d').addEventListener('change',applyFilter);
['id','f','r'].forEach(s=>{
  document.getElementById('s-'+s).addEventListener('click',()=>{
    sort=s;
    document.querySelectorAll('#toolbar button[id^=s-]').forEach(b=>b.classList.remove('active'));
    document.getElementById('s-'+s).classList.add('active');
    buildGrid();
  });
});
document.addEventListener('keydown',e=>{
  if(document.getElementById('overlay').style.display==='block'){
    if(e.key==='Escape') closeModal();
    if(e.key==='ArrowRight'&&cur<P.length-1) openModal(cur+1);
    if(e.key==='ArrowLeft' &&cur>0)          openModal(cur-1);
  }
});
window.addEventListener('resize',()=>{ if(cur>=0) draw(); });

buildGrid();
</script>
</body>
</html>
"""


def render_html(pairs_data: list, out_path: str):
    """Write a single self-contained HTML viewer for all pairs."""
    pairs_json = json.dumps(pairs_data, separators=(',', ':'))
    html = _HTML_TEMPLATE.replace('__PAIRS_JSON__', pairs_json)
    Path(out_path).write_text(html, encoding="utf-8")
    print(f"Saved HTML → {out_path}  ({len(pairs_data)} pairs)")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Visualize InsightAT feature matches (single pair PNG or batch HTML/PNG)",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    ap.add_argument("--images",    required=True, help="images_all.json path")
    ap.add_argument("--feat",      required=True, help="feat/ directory")
    ap.add_argument("--match",     required=True, help="match/ directory")
    ap.add_argument("--geo",       default=None,  help="geo/ directory (optional)")

    # Single-pair mode
    ap.add_argument("--pair",      default=None,  help="Single pair: '3,49'")
    ap.add_argument("--out",       default=None,  help="Output PNG (single-pair mode)")
    ap.add_argument("--no-all-feat", action="store_true")
    ap.add_argument("--max-kp",    type=int, default=3000)

    # Batch mode
    ap.add_argument("--batch-geo", default=None, metavar="DIR",
                    help="Batch mode: process all *.isat_geo files in DIR")
    ap.add_argument("--pairs",     nargs="*", default=None, metavar="i,j",
                    help="Subset of pairs in batch, e.g. --pairs 3,49 5,12")
    ap.add_argument("--html-out",  default=None, metavar="FILE",
                    help="Batch HTML output (single interactive page)")
    ap.add_argument("--out-dir",   default=None, metavar="DIR",
                    help="Batch PNG output directory (one PNG per pair)")

    args = ap.parse_args()
    image_paths = load_image_list(args.images)

    # ── single-pair mode ──────────────────────────────────────────────────────
    if args.pair:
        idx1, idx2 = [int(x) for x in args.pair.split(",")]
        data = load_pair_data(idx1, idx2, args.feat, args.match, args.geo, image_paths)
        if data is None:
            sys.exit(f"Failed to load pair ({idx1},{idx2})")
        out = args.out or f"viz_{idx1}_{idx2}.png"
        try:
            render_png(data, out, no_feat=args.no_all_feat, max_kp=args.max_kp)
        except ImportError:
            sys.exit("PNG mode requires: pip install matplotlib Pillow")
        return

    # ── batch mode ────────────────────────────────────────────────────────────
    if not args.batch_geo:
        ap.error("Provide --pair for single-pair mode, or --batch-geo for batch mode")

    all_pairs = parse_geo_dir_pairs(args.batch_geo)
    if not all_pairs:
        sys.exit(f"No *.isat_geo files found in {args.batch_geo}")

    if args.pairs:
        keep = set()
        for s in args.pairs:
            a, b = [int(x) for x in s.split(",")]
            keep.add((min(a, b), max(a, b)))
        all_pairs = [(lo, hi) for lo, hi in all_pairs if (lo, hi) in keep]

    print(f"Processing {len(all_pairs)} pairs ...", file=sys.stderr)
    geo_src = args.geo or args.batch_geo

    if args.html_out:
        pairs_data = []
        for idx, (lo, hi) in enumerate(all_pairs):
            print(f"  [{idx+1}/{len(all_pairs)}] pair {lo}_{hi}", file=sys.stderr)
            data = load_pair_data(lo, hi, args.feat, args.match, geo_src, image_paths)
            if data is not None:
                pairs_data.append(data)
        render_html(pairs_data, args.html_out)

    elif args.out_dir:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        for idx, (lo, hi) in enumerate(all_pairs):
            print(f"  [{idx+1}/{len(all_pairs)}] pair {lo}_{hi}", file=sys.stderr)
            data = load_pair_data(lo, hi, args.feat, args.match, geo_src, image_paths)
            if data is not None:
                try:
                    render_png(data, str(out_dir / f"viz_{lo}_{hi}.png"),
                               no_feat=args.no_all_feat, max_kp=args.max_kp)
                except ImportError:
                    sys.exit("PNG mode requires: pip install matplotlib Pillow")
    else:
        ap.error("In batch mode provide --html-out <file> or --out-dir <dir>")


if __name__ == "__main__":
    main()
