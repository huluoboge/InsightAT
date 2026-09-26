'use strict';

/**
 * Scene helpers for SfM viewer (centroid, frustum edges, fit radius).
 */

function computeCentroid(positions) {
  const n = Math.floor(positions.length / 3);
  if (n === 0) return [0, 0, 0];
  let sx = 0;
  let sy = 0;
  let sz = 0;
  for (let i = 0; i < n; i++) {
    sx += positions[i * 3];
    sy += positions[i * 3 + 1];
    sz += positions[i * 3 + 2];
  }
  return [sx / n, sy / n, sz / n];
}

function fitRadius(positions, cameras, quantile = 0.88) {
  const centroid = computeCentroid(positions);
  const dists = [];
  const n = Math.floor(positions.length / 3);
  for (let i = 0; i < n; i++) {
    const dx = positions[i * 3] - centroid[0];
    const dy = positions[i * 3 + 1] - centroid[1];
    const dz = positions[i * 3 + 2] - centroid[2];
    dists.push(Math.hypot(dx, dy, dz));
  }
  for (const cam of cameras || []) {
    const c = cam.center;
    dists.push(Math.hypot(c[0] - centroid[0], c[1] - centroid[1], c[2] - centroid[2]));
  }
  if (!dists.length) return 1;
  dists.sort((a, b) => a - b);
  const idx = Math.min(dists.length - 1, Math.floor(quantile * dists.length));
  return Math.max(dists[idx], 1e-3);
}

/**
 * Build camera frustum as line segment positions (flat xyz array).
 * Uses viewer-frame R (row-major 3x3) and camera center.
 */
function buildFrustumLines(cam, scale) {
  const R = cam.R;
  const C = cam.center;
  const w = cam.width || 1;
  const h = cam.height || 1;
  const f = cam.focal > 0 ? cam.focal : Math.max(w, h);
  const halfW = (0.5 * w * scale) / f;
  const halfH = (0.5 * h * scale) / f;
  const depth = scale;

  // Camera axes in world (viewer): rows of R are camera axes expressed in world? 
  // COLMAP: Xc = R * Xw + t; camera looking +Z in CV after S flip looks -Z in OpenGL-ish.
  // Frustum corners in camera frame (after S): image plane at z = -depth (looking -Z).
  const cornersCam = [
    [-halfW, -halfH, -depth],
    [halfW, -halfH, -depth],
    [halfW, halfH, -depth],
    [-halfW, halfH, -depth]
  ];

  function camToWorld(p) {
    // Xw = R^T * (Xc - t') with center C = -R^T t'; equivalently Xw = C + R^T * Xc
    // R is row-major; R^T * v:
    const x = R[0] * p[0] + R[3] * p[1] + R[6] * p[2];
    const y = R[1] * p[0] + R[4] * p[1] + R[7] * p[2];
    const z = R[2] * p[0] + R[5] * p[1] + R[8] * p[2];
    return [C[0] + x, C[1] + y, C[2] + z];
  }

  const corners = cornersCam.map(camToWorld);
  const lines = [];
  function pushSeg(a, b) {
    lines.push(a[0], a[1], a[2], b[0], b[1], b[2]);
  }
  for (let i = 0; i < 4; i++) {
    pushSeg(C, corners[i]);
    pushSeg(corners[i], corners[(i + 1) % 4]);
  }
  // Bottom edge emphasis (duplicate)
  pushSeg(corners[0], corners[1]);
  return lines;
}

function buildAllFrustumLines(cameras, scale) {
  const out = [];
  for (const cam of cameras) {
    out.push(...buildFrustumLines(cam, scale));
  }
  return out;
}

module.exports = {
  computeCentroid,
  fitRadius,
  buildFrustumLines,
  buildAllFrustumLines
};
