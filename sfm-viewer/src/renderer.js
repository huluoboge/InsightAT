import * as THREE from './vendor/three.module.js';
import { MeshLabTrackballControls } from './vendor/meshlab_trackball.js';

const canvas = document.getElementById('c');
const pathLabel = document.getElementById('pathLabel');
const statPoints = document.getElementById('statPoints');
const statCameras = document.getElementById('statCameras');
const statFormat = document.getElementById('statFormat');
const statFiltered = document.getElementById('statFiltered');
const errorEl = document.getElementById('error');
const minObsInput = document.getElementById('minObs');
const minObsVal = document.getElementById('minObsVal');
const pointSizeInput = document.getElementById('pointSize');
const frustumScaleInput = document.getElementById('frustumScale');
const showPointsInput = document.getElementById('showPoints');
const showCamerasInput = document.getElementById('showCameras');
const showAxesInput = document.getElementById('showAxes');
const showTrackballInput = document.getElementById('showTrackball');
const trackGalleryInput = document.getElementById('trackGallery');
const imagesLabel = document.getElementById('imagesLabel');
const pickPanel = document.getElementById('pickPanel');
const pickTitle = document.getElementById('pickTitle');
const pickInfo = document.getElementById('pickInfo');
const obsList = document.getElementById('obsList');
const imagePanel = document.getElementById('imagePanel');
const obsImage = document.getElementById('obsImage');
const crosshair = document.getElementById('crosshair');
const hintEl = document.getElementById('hint');

/** @type {'none'|'point'|'camera'} */
let pickMode = 'none';
let sceneData = null;
let cameraById = new Map();
let pointsMesh = null;
let frustumBody = null;
let frustumAxes = null;
let trackballGizmo = null;
let highlightGroup = null;
let visibleIndices = [];
let pickIndex = -1;
let sceneRadius = 1;
let filteredOutCount = 0;

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 1e6);
camera.position.set(0, 0, 5);
camera.up.set(0, 1, 0);

// MeshLab-style virtual-sphere trackball (same math as RenderRotationTool).
const controls = new MeshLabTrackballControls(camera, canvas);
controls.rotateSpeed = 1.0;
controls.panSpeed = 1.0;

scene.add(new THREE.AmbientLight(0xffffff, 0.9));

function shortPath(value) {
  if (!value) return '(auto)';
  if (value.length <= 42) return value;
  return `...${value.slice(-39)}`;
}

function setImagesLabel(root) {
  imagesLabel.textContent = `Images: ${shortPath(root)}`;
  imagesLabel.title = root || '';
}

/** Pivot gizmo: wire sphere + RGB world axes (MeshLab-style trackball cue). */
function ensureTrackballGizmo() {
  if (trackballGizmo) return;
  trackballGizmo = new THREE.Group();
  trackballGizmo.name = 'trackballGizmo';

  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(1, 32, 24),
    new THREE.MeshBasicMaterial({
      color: 0x88aacc,
      transparent: true,
      opacity: 0.12,
      depthWrite: false
    })
  );
  trackballGizmo.add(sphere);

  const wire = new THREE.LineSegments(
    new THREE.WireframeGeometry(new THREE.SphereGeometry(1, 16, 12)),
    new THREE.LineBasicMaterial({
      color: 0xa8c4e0,
      transparent: true,
      opacity: 0.45
    })
  );
  trackballGizmo.add(wire);

  // Meridian rings for rolling-ball feel
  const ringMat = new THREE.LineBasicMaterial({ color: 0x6a8aaa, transparent: true, opacity: 0.55 });
  const ringGeo = new THREE.BufferGeometry().setFromPoints(
    Array.from({ length: 65 }, (_, i) => {
      const t = (i / 64) * Math.PI * 2;
      return new THREE.Vector3(Math.cos(t), Math.sin(t), 0);
    })
  );
  const ringXY = new THREE.LineLoop(ringGeo, ringMat);
  const ringXZ = new THREE.LineLoop(ringGeo.clone(), ringMat.clone());
  ringXZ.rotation.x = Math.PI / 2;
  const ringYZ = new THREE.LineLoop(ringGeo.clone(), ringMat.clone());
  ringYZ.rotation.y = Math.PI / 2;
  trackballGizmo.add(ringXY);
  trackballGizmo.add(ringXZ);
  trackballGizmo.add(ringYZ);

  // World RGB axes
  const axisLen = 1.25;
  const mkAxis = (to, color) => {
    const g = new THREE.BufferGeometry().setAttribute(
      'position',
      new THREE.Float32BufferAttribute([0, 0, 0, to[0], to[1], to[2]], 3)
    );
    return new THREE.Line(g, new THREE.LineBasicMaterial({ color, linewidth: 2 }));
  };
  trackballGizmo.add(mkAxis([axisLen, 0, 0], 0xff4444));
  trackballGizmo.add(mkAxis([0, axisLen, 0], 0x44dd66));
  trackballGizmo.add(mkAxis([0, 0, axisLen], 0x4488ff));

  trackballGizmo.visible = showTrackballInput.checked;
  scene.add(trackballGizmo);
}

function syncTrackballGizmo() {
  if (!trackballGizmo) return;
  trackballGizmo.position.copy(controls.target);
  const dist = camera.position.distanceTo(controls.target);
  const s = Math.max(dist * 0.12, sceneRadius * 0.08, 0.05);
  trackballGizmo.scale.setScalar(s);
  trackballGizmo.visible = showTrackballInput.checked && Boolean(sceneData);
}

function percentile(sorted, q) {
  if (!sorted.length) return 0;
  const i = Math.min(sorted.length - 1, Math.max(0, Math.floor(q * (sorted.length - 1))));
  return sorted[i];
}

function medianOfCoords(positions, axis) {
  const vals = [];
  for (let i = 0; i < positions.length; i += 3) vals.push(positions[i + axis]);
  vals.sort((a, b) => a - b);
  return percentile(vals, 0.5);
}

/**
 * Drop far outliers that wreck framing / frustum scale.
 * Keep points within max(p95 of camera+inlier cloud, 3 * MAD) from robust center.
 */
function filterOutlierPoints(raw) {
  const pos = raw.points.positions;
  const col = raw.points.colors;
  const lens = raw.points.trackLengths;
  const obs = raw.points.observations;
  const n = raw.points.count;
  if (n === 0) {
    return { ...raw, filteredOut: 0, sceneRadius: 1 };
  }

  const cx = medianOfCoords(pos, 0);
  const cy = medianOfCoords(pos, 1);
  const cz = medianOfCoords(pos, 2);

  const dists = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    dists[i] = Math.hypot(pos[i * 3] - cx, pos[i * 3 + 1] - cy, pos[i * 3 + 2] - cz);
  }
  const sorted = Array.from(dists).sort((a, b) => a - b);
  const med = percentile(sorted, 0.5);
  const absDev = sorted.map((d) => Math.abs(d - med)).sort((a, b) => a - b);
  const mad = percentile(absDev, 0.5) || med * 0.1 || 1e-3;
  const p95 = percentile(sorted, 0.95);
  // Soft cap: keep most of the cloud, cut extreme flyaways.
  const limit = Math.max(p95 * 1.25, med + 4.5 * 1.4826 * mad, 1e-3);

  const keepPos = [];
  const keepCol = [];
  const keepLen = [];
  const keepObs = [];
  let kept = 0;
  for (let i = 0; i < n; i++) {
    if (dists[i] > limit) continue;
    keepPos.push(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
    keepCol.push(col[i * 3], col[i * 3 + 1], col[i * 3 + 2]);
    keepLen.push(lens[i]);
    keepObs.push(obs[i]);
    kept += 1;
  }

  // Robust radius from kept points + cameras
  const keptDists = [];
  for (let i = 0; i < kept; i++) {
    keptDists.push(
      Math.hypot(keepPos[i * 3] - cx, keepPos[i * 3 + 1] - cy, keepPos[i * 3 + 2] - cz)
    );
  }
  for (const cam of raw.cameras) {
    keptDists.push(Math.hypot(cam.center[0] - cx, cam.center[1] - cy, cam.center[2] - cz));
  }
  keptDists.sort((a, b) => a - b);
  const sceneRadius = Math.max(percentile(keptDists, 0.9), 1e-3);

  return {
    ...raw,
    points: {
      count: kept,
      positions: keepPos,
      colors: keepCol,
      trackLengths: keepLen,
      observations: keepObs
    },
    summary: {
      ...raw.summary,
      pointCount: kept,
      originalPointCount: n
    },
    filteredOut: n - kept,
    sceneRadius,
    robustCenter: [cx, cy, cz]
  };
}

function autoFrustumScale(radius) {
  // Absolute scale in world units ≈ fraction of scene; slider is a multiplier later.
  return Math.max(0.05, Math.min(2.0, radius * 0.08));
}

function camToWorld(R, C, p) {
  return [
    C[0] + R[0] * p[0] + R[3] * p[1] + R[6] * p[2],
    C[1] + R[1] * p[0] + R[4] * p[1] + R[7] * p[2],
    C[2] + R[2] * p[0] + R[5] * p[1] + R[8] * p[2]
  ];
}

/**
 * Body lines (pyramid + image frame) in viewer frame (S=diag(1,-1,-1)).
 * Axes: image/CV frame — X right, Y down, Z forward (uses Rcv).
 */
function buildFrustumParts(cam, scale) {
  const R = cam.R;
  const Rcv = cam.Rcv || cam.R;
  const C = cam.center;
  const w = cam.width || 1;
  const h = cam.height || 1;
  const f = cam.focal > 0 ? cam.focal : Math.max(w, h);
  const halfW = (0.5 * w * scale) / f;
  const halfH = (0.5 * h * scale) / f;
  const depth = scale;

  const cornersCam = [
    [-halfW, -halfH, -depth],
    [halfW, -halfH, -depth],
    [halfW, halfH, -depth],
    [-halfW, halfH, -depth]
  ];
  const corners = cornersCam.map((p) => camToWorld(R, C, p));
  const body = [];
  function push(arr, a, b) {
    arr.push(a[0], a[1], a[2], b[0], b[1], b[2]);
  }
  for (let i = 0; i < 4; i++) {
    push(body, C, corners[i]);
    push(body, corners[i], corners[(i + 1) % 4]);
  }

  const axisLen = scale * 0.55;
  return {
    body,
    axes: {
      x: [...C, ...camToWorld(Rcv, C, [axisLen, 0, 0])],
      y: [...C, ...camToWorld(Rcv, C, [0, axisLen, 0])],
      z: [...C, ...camToWorld(Rcv, C, [0, 0, axisLen])]
    }
  };
}

function resize() {
  const parent = canvas.parentElement;
  const w = parent.clientWidth;
  const h = parent.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
  controls.handleResize();
}

function disposeObject(obj) {
  if (!obj) return;
  scene.remove(obj);
  obj.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
      else child.material.dispose();
    }
  });
}

function clearSceneMeshes() {
  disposeObject(pointsMesh);
  pointsMesh = null;
  disposeObject(frustumBody);
  frustumBody = null;
  disposeObject(frustumAxes);
  frustumAxes = null;
  clearHighlight();
}

function clearHighlight() {
  disposeObject(highlightGroup);
  highlightGroup = null;
  pickIndex = -1;
  pickPanel.hidden = true;
  imagePanel.hidden = true;
  obsList.innerHTML = '';
}

function rebuildPoints() {
  if (!sceneData) return;
  const minObs = Number(minObsInput.value);
  const pos = sceneData.points.positions;
  const col = sceneData.points.colors;
  const lens = sceneData.points.trackLengths;
  const n = sceneData.points.count;
  const outPos = [];
  const outCol = [];
  visibleIndices = [];
  for (let i = 0; i < n; i++) {
    if ((lens[i] || 0) < minObs) continue;
    visibleIndices.push(i);
    outPos.push(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
    outCol.push(col[i * 3], col[i * 3 + 1], col[i * 3 + 2]);
  }
  disposeObject(pointsMesh);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(outPos, 3));
  geo.setAttribute('color', new THREE.Float32BufferAttribute(outCol, 3));
  pointsMesh = new THREE.Points(
    geo,
    new THREE.PointsMaterial({
      size: Number(pointSizeInput.value),
      vertexColors: true,
      sizeAttenuation: false
    })
  );
  pointsMesh.visible = showPointsInput.checked;
  scene.add(pointsMesh);
}

function rebuildFrustums() {
  if (!sceneData) return;
  disposeObject(frustumBody);
  disposeObject(frustumAxes);

  const userMul = Number(frustumScaleInput.value);
  const scale = autoFrustumScale(sceneRadius) * (userMul / 0.2);

  const body = [];
  const ax = [];
  const ay = [];
  const az = [];
  for (const cam of sceneData.cameras) {
    const parts = buildFrustumParts(cam, scale);
    body.push(...parts.body);
    ax.push(...parts.axes.x);
    ay.push(...parts.axes.y);
    az.push(...parts.axes.z);
  }

  frustumBody = new THREE.LineSegments(
    new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(body, 3)),
    new THREE.LineBasicMaterial({ color: 0x7eb6ff, transparent: true, opacity: 0.9 })
  );

  frustumAxes = new THREE.Group();
  const mk = (arr, color) =>
    new THREE.LineSegments(
      new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(arr, 3)),
      new THREE.LineBasicMaterial({ color, linewidth: 2 })
    );
  frustumAxes.add(mk(ax, 0xff5555));
  frustumAxes.add(mk(ay, 0x55dd77));
  frustumAxes.add(mk(az, 0x55aaff));

  const show = showCamerasInput.checked;
  frustumBody.visible = show;
  frustumAxes.visible = show && showAxesInput.checked;
  scene.add(frustumBody);
  scene.add(frustumAxes);
}

function resetView() {
  if (!sceneData) return;
  const c = sceneData.robustCenter
    ? new THREE.Vector3(...sceneData.robustCenter)
    : new THREE.Vector3(
      medianOfCoords(sceneData.points.positions, 0),
      medianOfCoords(sceneData.points.positions, 1),
      medianOfCoords(sceneData.points.positions, 2)
    );
  const r = sceneRadius;
  controls.target.copy(c);
  camera.position.set(c.x + r * 1.5, c.y + r * 0.6, c.z + r * 1.5);
  camera.up.set(0, 1, 0);
  camera.near = Math.max(r / 5000, 0.001);
  camera.far = Math.max(r * 200, 100);
  camera.updateProjectionMatrix();
  controls.maxDistance = r * 50;
  controls.minDistance = r * 0.01;
  controls.update();
}

function showError(msg) {
  if (!msg) {
    errorEl.hidden = true;
    errorEl.textContent = '';
    return;
  }
  errorEl.hidden = false;
  errorEl.textContent = msg;
}

function setPickMode(mode) {
  pickMode = mode;
  for (const btn of document.querySelectorAll('.mode-btn')) {
    btn.classList.toggle('active', btn.dataset.mode === mode);
  }
  if (mode === 'none') {
    hintEl.textContent =
      'MeshLab trackball: LMB rotate · Scroll ×0.9/×1.1 · RMB pan · Double-click point to center';
  } else if (mode === 'point') {
    hintEl.textContent = 'Click a 3D point (track) to inspect observations';
  } else {
    hintEl.textContent = 'Click near a camera frustum / center to select it';
  }
  if (mode === 'none') clearHighlight();
}

async function loadDirectory(dir) {
  showError(null);
  const result = await window.sfmViewer.load(dir);
  if (!result.ok) {
    showError(result.error);
    return;
  }
  sceneData = filterOutlierPoints(result.scene);
  filteredOutCount = sceneData.filteredOut || 0;
  sceneRadius = sceneData.sceneRadius || 1;
  cameraById = new Map(sceneData.cameras.map((c) => [c.id, c]));

  pathLabel.textContent = sceneData.root;
  statPoints.textContent = String(sceneData.summary.pointCount);
  statCameras.textContent = String(sceneData.summary.cameraCount);
  statFormat.textContent = sceneData.format;
  statFiltered.textContent =
    filteredOutCount > 0 ? `−${filteredOutCount} outliers` : 'none';

  // Default frustum slider around a sane absolute scale.
  frustumScaleInput.value = '0.2';

  clearSceneMeshes();
  rebuildPoints();
  rebuildFrustums();
  ensureTrackballGizmo();
  resetView();
}

function screenPickThreshold() {
  return Math.max(0.01 * sceneRadius, 0.05);
}

/** @returns {{ index: number, point: THREE.Vector3, pixelDist: number } | null} */
function findNearestVisiblePoint(clientX, clientY, maxPixel = 16) {
  if (!pointsMesh || !visibleIndices.length || !sceneData) return null;
  const rect = canvas.getBoundingClientRect();
  const mouse = new THREE.Vector2(
    ((clientX - rect.left) / rect.width) * 2 - 1,
    -((clientY - rect.top) / rect.height) * 2 + 1
  );
  const raycaster = new THREE.Raycaster();
  raycaster.params.Points.threshold = screenPickThreshold();
  raycaster.setFromCamera(mouse, camera);
  const hits = raycaster.intersectObject(pointsMesh);
  if (!hits.length) return null;

  let best = hits[0];
  let bestPix = Infinity;
  const proj = new THREE.Vector3();
  for (const hit of hits.slice(0, 12)) {
    proj.copy(hit.point).project(camera);
    const sx = (proj.x * 0.5 + 0.5) * rect.width;
    const sy = (-proj.y * 0.5 + 0.5) * rect.height;
    const d = Math.hypot(sx - (clientX - rect.left), sy - (clientY - rect.top));
    if (d < bestPix) {
      bestPix = d;
      best = hit;
    }
  }
  if (bestPix > maxPixel) return null;
  const index = visibleIndices[best.index];
  const pos = sceneData.points.positions;
  return {
    index,
    point: new THREE.Vector3(pos[index * 3], pos[index * 3 + 1], pos[index * 3 + 2]),
    pixelDist: bestPix
  };
}

function setOrbitCenterToPoint(point) {
  const offset = new THREE.Vector3().subVectors(camera.position, controls.target);
  controls.target.copy(point);
  camera.position.copy(point).add(offset);
  controls.update();
}

function pickTrack(clientX, clientY) {
  const hit = findNearestVisiblePoint(clientX, clientY, 16);
  if (!hit) {
    clearHighlight();
    return;
  }
  selectTrack(hit.index);
}

function pickCamera(clientX, clientY) {
  if (!sceneData || !sceneData.cameras.length) return;
  const rect = canvas.getBoundingClientRect();
  let best = null;
  let bestPix = Infinity;
  const proj = new THREE.Vector3();
  for (const cam of sceneData.cameras) {
    proj.set(cam.center[0], cam.center[1], cam.center[2]).project(camera);
    if (proj.z < -1 || proj.z > 1) continue;
    const sx = (proj.x * 0.5 + 0.5) * rect.width;
    const sy = (-proj.y * 0.5 + 0.5) * rect.height;
    const d = Math.hypot(sx - (clientX - rect.left), sy - (clientY - rect.top));
    if (d < bestPix) {
      bestPix = d;
      best = cam;
    }
  }
  if (!best || bestPix > 28) {
    clearHighlight();
    return;
  }
  selectCamera(best);
}

function selectTrack(pointIndex) {
  clearHighlight();
  pickIndex = pointIndex;
  const pos = sceneData.points.positions;
  const px = pos[pointIndex * 3];
  const py = pos[pointIndex * 3 + 1];
  const pz = pos[pointIndex * 3 + 2];
  const obs = sceneData.points.observations[pointIndex] || [];

  highlightGroup = new THREE.Group();
  const ptGeo = new THREE.BufferGeometry();
  ptGeo.setAttribute('position', new THREE.Float32BufferAttribute([px, py, pz], 3));
  highlightGroup.add(
    new THREE.Points(
      ptGeo,
      new THREE.PointsMaterial({
        color: 0xffe066,
        size: Number(pointSizeInput.value) * 2.5,
        sizeAttenuation: false
      })
    )
  );

  const linePos = [];
  for (const o of obs) {
    const cam = cameraById.get(o.imageId);
    if (!cam) continue;
    linePos.push(px, py, pz, cam.center[0], cam.center[1], cam.center[2]);
  }
  if (linePos.length) {
    highlightGroup.add(
      new THREE.LineSegments(
        new THREE.BufferGeometry().setAttribute(
          'position',
          new THREE.Float32BufferAttribute(linePos, 3)
        ),
        new THREE.LineBasicMaterial({ color: 0xff9f43 })
      )
    );
  }
  scene.add(highlightGroup);

  pickPanel.hidden = false;
  pickTitle.textContent = 'Selected track';
  pickInfo.textContent = `Track #${pointIndex} · (${px.toFixed(3)}, ${py.toFixed(3)}, ${pz.toFixed(3)}) · ${obs.length} obs`;
  obsList.innerHTML = '';
  obs.forEach((o, i) => {
    const cam = cameraById.get(o.imageId);
    const li = document.createElement('li');
    li.textContent = `${cam ? cam.name : `image ${o.imageId}`}  (${o.u.toFixed(1)}, ${o.v.toFixed(1)})`;
    li.addEventListener('click', () => {
      [...obsList.children].forEach((el) => el.classList.remove('active'));
      li.classList.add('active');
      showObservation(cam, o);
    });
    obsList.appendChild(li);
    if (i === 0 && cam) {
      li.classList.add('active');
      showObservation(cam, o);
    }
  });

  publishTrackToGallery(pointIndex, [px, py, pz], obs);
}

function buildGalleryPayload(trackId, xyz, obs) {
  return {
    trackId,
    xyz,
    observations: obs.map((o) => {
      const cam = cameraById.get(o.imageId);
      return {
        imageId: o.imageId,
        name: cam ? cam.name : `image_${o.imageId}`,
        u: o.u,
        v: o.v,
        width: cam ? cam.width : 0,
        height: cam ? cam.height : 0
      };
    })
  };
}

async function publishTrackToGallery(trackId, xyz, obs) {
  if (!trackGalleryInput.checked) return;
  const payload = buildGalleryPayload(trackId, xyz, obs);
  await window.sfmViewer.publishTrackGallery(payload);
}

function selectCamera(cam) {
  clearHighlight();
  highlightGroup = new THREE.Group();

  const userMul = Number(frustumScaleInput.value);
  const scale = autoFrustumScale(sceneRadius) * (userMul / 0.2);
  const parts = buildFrustumParts(cam, scale * 1.15);

  highlightGroup.add(
    new THREE.LineSegments(
      new THREE.BufferGeometry().setAttribute(
        'position',
        new THREE.Float32BufferAttribute(parts.body, 3)
      ),
      new THREE.LineBasicMaterial({ color: 0xff8c42 })
    )
  );

  // Lines to all tracks seen by this camera
  const linePos = [];
  const pos = sceneData.points.positions;
  let trackHits = 0;
  for (let i = 0; i < sceneData.points.count; i++) {
    const obs = sceneData.points.observations[i] || [];
    if (!obs.some((o) => o.imageId === cam.id)) continue;
    trackHits += 1;
    linePos.push(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2], cam.center[0], cam.center[1], cam.center[2]);
  }
  if (linePos.length) {
    highlightGroup.add(
      new THREE.LineSegments(
        new THREE.BufferGeometry().setAttribute(
          'position',
          new THREE.Float32BufferAttribute(linePos, 3)
        ),
        new THREE.LineBasicMaterial({ color: 0xffb347, transparent: true, opacity: 0.35 })
      )
    );
  }
  scene.add(highlightGroup);

  pickPanel.hidden = false;
  imagePanel.hidden = true;
  pickTitle.textContent = 'Selected camera';
  pickInfo.textContent = `${cam.name} · id ${cam.id} · ${trackHits} tracks · ${cam.width}×${cam.height}`;
  obsList.innerHTML = '';
  const li = document.createElement('li');
  li.textContent = `Center (${cam.center.map((v) => v.toFixed(3)).join(', ')})`;
  obsList.appendChild(li);
}

async function showObservation(cam, obs) {
  if (!cam) {
    imagePanel.hidden = true;
    return;
  }
  const url = await window.sfmViewer.resolveImage(cam.name);
  if (!url) {
    imagePanel.hidden = false;
    obsImage.removeAttribute('src');
    showError(`Image not found: ${cam.name}`);
    return;
  }
  showError(null);
  imagePanel.hidden = false;
  obsImage.onload = () => drawCrosshair(obs.u, obs.v, cam.width, cam.height);
  obsImage.src = url;
}

function drawCrosshair(u, v, imgW, imgH) {
  const wrap = document.getElementById('imageWrap');
  const w = wrap.clientWidth;
  const h = obsImage.clientHeight || wrap.clientHeight;
  crosshair.width = w;
  crosshair.height = h;
  const ctx = crosshair.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  if (!imgW || !imgH) return;
  const sx = (u / imgW) * w;
  const sy = (v / imgH) * h;
  ctx.strokeStyle = '#ffe066';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(sx - 12, sy);
  ctx.lineTo(sx + 12, sy);
  ctx.moveTo(sx, sy - 12);
  ctx.lineTo(sx, sy + 12);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(sx, sy, 6, 0, Math.PI * 2);
  ctx.stroke();
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  syncTrackballGizmo();
  renderer.render(scene, camera);
}

window.addEventListener('resize', resize);
minObsInput.addEventListener('input', () => {
  minObsVal.textContent = minObsInput.value;
  rebuildPoints();
});
pointSizeInput.addEventListener('input', () => {
  if (pointsMesh) pointsMesh.material.size = Number(pointSizeInput.value);
});
frustumScaleInput.addEventListener('input', rebuildFrustums);
showPointsInput.addEventListener('change', () => {
  if (pointsMesh) pointsMesh.visible = showPointsInput.checked;
});
showCamerasInput.addEventListener('change', () => {
  const show = showCamerasInput.checked;
  if (frustumBody) frustumBody.visible = show;
  if (frustumAxes) frustumAxes.visible = show && showAxesInput.checked;
});
showAxesInput.addEventListener('change', () => {
  if (frustumAxes) frustumAxes.visible = showCamerasInput.checked && showAxesInput.checked;
});
showTrackballInput.addEventListener('change', () => {
  if (trackballGizmo) trackballGizmo.visible = showTrackballInput.checked && Boolean(sceneData);
});
trackGalleryInput.addEventListener('change', async () => {
  if (trackGalleryInput.checked) {
    await window.sfmViewer.openTrackGallery();
    if (pickIndex >= 0 && sceneData) {
      const pos = sceneData.points.positions;
      const obs = sceneData.points.observations[pickIndex] || [];
      await publishTrackToGallery(
        pickIndex,
        [pos[pickIndex * 3], pos[pickIndex * 3 + 1], pos[pickIndex * 3 + 2]],
        obs
      );
    }
  } else {
    await window.sfmViewer.closeTrackGallery();
  }
});
document.getElementById('resetBtn').addEventListener('click', resetView);
document.getElementById('openBtn').addEventListener('click', async () => {
  const dir = await window.sfmViewer.openDirectory();
  if (dir) await loadDirectory(dir);
});
document.getElementById('imagesBtn').addEventListener('click', async () => {
  const root = await window.sfmViewer.chooseImagesRoot();
  setImagesLabel(root);
});
document.getElementById('imagesClearBtn').addEventListener('click', async () => {
  await window.sfmViewer.clearImagesRoot();
  setImagesLabel(null);
});
for (const btn of document.querySelectorAll('.mode-btn')) {
  btn.addEventListener('click', () => setPickMode(btn.dataset.mode));
}

let down = null;
canvas.addEventListener('pointerdown', (e) => {
  down = { x: e.clientX, y: e.clientY, button: e.button };
});
canvas.addEventListener('pointerup', (e) => {
  if (!down || down.button !== 0) {
    down = null;
    return;
  }
  const dx = e.clientX - down.x;
  const dy = e.clientY - down.y;
  down = null;
  if (Math.hypot(dx, dy) >= 6) return;
  if (pickMode === 'point') pickTrack(e.clientX, e.clientY);
  else if (pickMode === 'camera') pickCamera(e.clientX, e.clientY);
});

canvas.addEventListener('dblclick', (e) => {
  if (pickMode !== 'none') return;
  e.preventDefault();
  const hit = findNearestVisiblePoint(e.clientX, e.clientY, 20);
  if (!hit) return;
  setOrbitCenterToPoint(hit.point);
});

resize();
setPickMode('none');
animate();

(async () => {
  const root = await window.sfmViewer.getImagesRoot();
  setImagesLabel(root);
  const initial = await window.sfmViewer.getInitialPath();
  if (initial) await loadDirectory(initial);
})();
