'use strict';

const metaEl = document.getElementById('meta');
const gridEl = document.getElementById('grid');
const viewModeEl = document.getElementById('viewMode');
const pageSizeEl = document.getElementById('pageSize');
const patchRadiusEl = document.getElementById('patchRadius');
const patchRadiusVal = document.getElementById('patchRadiusVal');
const pageLabel = document.getElementById('pageLabel');
const prevBtn = document.getElementById('prevBtn');
const nextBtn = document.getElementById('nextBtn');

let track = null;
let page = 0;
const imageCache = new Map();

function pageSize() {
  return Number(pageSizeEl.value) || 9;
}

function patchRadius() {
  return Number(patchRadiusEl.value) || 96;
}

function loadImage(url) {
  if (!url) return Promise.reject(new Error('no url'));
  if (imageCache.has(url)) return imageCache.get(url);
  const p = new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => resolve(img);
    img.onerror = () => reject(new Error('load failed'));
    img.src = url;
  });
  imageCache.set(url, p);
  return p;
}

function drawPatch(canvas, img, obs, mode) {
  const ctx = canvas.getContext('2d');
  const css = canvas.clientWidth || 240;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.floor(css * dpr);
  canvas.height = Math.floor(css * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#0d0f13';
  ctx.fillRect(0, 0, css, css);

  const u = obs.u;
  const v = obs.v;
  const iw = img.naturalWidth || obs.width || 1;
  const ih = img.naturalHeight || obs.height || 1;

  let sx;
  let sy;
  let sw;
  let sh;
  if (mode === 'full') {
    const scale = Math.min(css / iw, css / ih);
    const dw = iw * scale;
    const dh = ih * scale;
    const ox = (css - dw) / 2;
    const oy = (css - dh) / 2;
    ctx.drawImage(img, 0, 0, iw, ih, ox, oy, dw, dh);
    sx = ox + (u / iw) * dw;
    sy = oy + (v / ih) * dh;
  } else {
    const r = patchRadius();
    const x0 = Math.max(0, Math.min(iw - 1, u - r));
    const y0 = Math.max(0, Math.min(ih - 1, v - r));
    const x1 = Math.max(x0 + 1, Math.min(iw, u + r));
    const y1 = Math.max(y0 + 1, Math.min(ih, v + r));
    sw = x1 - x0;
    sh = y1 - y0;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(img, x0, y0, sw, sh, 0, 0, css, css);
    sx = ((u - x0) / sw) * css;
    sy = ((v - y0) / sh) * css;
  }

  // Crosshair
  ctx.strokeStyle = '#ffe066';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(sx - 14, sy);
  ctx.lineTo(sx + 14, sy);
  ctx.moveTo(sx, sy - 14);
  ctx.lineTo(sx, sy + 14);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(sx, sy, 7, 0, Math.PI * 2);
  ctx.stroke();

  // Coordinate badge
  const label = `(${u.toFixed(1)}, ${v.toFixed(1)})`;
  ctx.font = '11px sans-serif';
  const tw = ctx.measureText(label).width + 8;
  ctx.fillStyle = 'rgba(0,0,0,0.65)';
  ctx.fillRect(6, css - 22, tw, 16);
  ctx.fillStyle = '#ffe066';
  ctx.fillText(label, 10, css - 10);
}

async function renderPage() {
  const size = pageSize();
  gridEl.className = `grid cols-${size === 12 ? 4 : 3}`;
  gridEl.innerHTML = '';

  if (!track || !track.observations || !track.observations.length) {
    pageLabel.textContent = '0 / 0';
    metaEl.textContent = 'No track selected — pick a point in the 3D view';
    return;
  }

  const obs = track.observations;
  const totalPages = Math.max(1, Math.ceil(obs.length / size));
  page = Math.min(page, totalPages - 1);
  pageLabel.textContent = `${page + 1} / ${totalPages}`;
  metaEl.textContent = `Track #${track.trackId} · (${track.xyz.map((v) => v.toFixed(3)).join(', ')}) · ${obs.length} observations`;

  const start = page * size;
  const slice = obs.slice(start, start + size);
  const mode = viewModeEl.value;

  for (let i = 0; i < slice.length; i++) {
    const o = slice[i];
    const tile = document.createElement('div');
    tile.className = 'tile';
    const head = document.createElement('div');
    head.className = 'tile-head';
    head.title = o.name || '';
    head.textContent = `${start + i + 1}. ${o.name || `image ${o.imageId}`} · ${o.width || '?'}×${o.height || '?'}`;
    const body = document.createElement('div');
    body.className = 'tile-body';
    const canvas = document.createElement('canvas');
    body.appendChild(canvas);
    tile.appendChild(head);
    tile.appendChild(body);
    gridEl.appendChild(tile);

    if (!o.url) {
      tile.classList.add('empty');
      body.textContent = 'Image not found';
      continue;
    }
    try {
      const img = await loadImage(o.url);
      drawPatch(canvas, img, o, mode);
    } catch (_) {
      tile.classList.add('empty');
      body.textContent = 'Load failed';
    }
  }
}

function onTrack(data) {
  track = data;
  page = 0;
  renderPage();
}

viewModeEl.addEventListener('change', renderPage);
pageSizeEl.addEventListener('change', () => {
  page = 0;
  renderPage();
});
patchRadiusEl.addEventListener('input', () => {
  patchRadiusVal.textContent = patchRadiusEl.value;
  if (viewModeEl.value === 'patch') renderPage();
});
prevBtn.addEventListener('click', () => {
  if (page > 0) {
    page -= 1;
    renderPage();
  }
});
nextBtn.addEventListener('click', () => {
  if (!track) return;
  const totalPages = Math.max(1, Math.ceil((track.observations || []).length / pageSize()));
  if (page < totalPages - 1) {
    page += 1;
    renderPage();
  }
});

window.addEventListener('resize', () => {
  if (track) renderPage();
});

window.sfmViewer.onGalleryTrack(onTrack);
window.sfmViewer.getGalleryTrack().then((data) => {
  if (data) onTrack(data);
});
