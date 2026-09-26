'use strict';

/**
 * COLMAP sparse model loader (text + binary).
 * Layout matches COLMAP and InsightAT src/render/colmap_loader.cpp.
 */

const fs = require('fs');
const path = require('path');

const CAMERA_MODEL_PARAM_COUNT = {
  SIMPLE_PINHOLE: 3,
  PINHOLE: 4,
  SIMPLE_RADIAL: 4,
  RADIAL: 5,
  OPENCV: 8,
  OPENCV_FISHEYE: 8,
  FULL_OPENCV: 12,
  FOV: 5,
  SIMPLE_RADIAL_FISHEYE: 4,
  RADIAL_FISHEYE: 5,
  THIN_PRISM_FISHEYE: 12
};

const MODEL_NAMES = [
  'SIMPLE_PINHOLE',
  'PINHOLE',
  'SIMPLE_RADIAL',
  'RADIAL',
  'OPENCV',
  'OPENCV_FISHEYE',
  'FULL_OPENCV',
  'FOV',
  'SIMPLE_RADIAL_FISHEYE',
  'RADIAL_FISHEYE',
  'THIN_PRISM_FISHEYE'
];

function detectFormat(dir) {
  const root = path.resolve(dir);
  const hasTxt =
    fs.existsSync(path.join(root, 'cameras.txt')) &&
    fs.existsSync(path.join(root, 'images.txt')) &&
    fs.existsSync(path.join(root, 'points3D.txt'));
  if (hasTxt) return { root, format: 'text' };

  const hasBin =
    fs.existsSync(path.join(root, 'cameras.bin')) &&
    fs.existsSync(path.join(root, 'images.bin')) &&
    fs.existsSync(path.join(root, 'points3D.bin'));
  if (hasBin) return { root, format: 'binary' };

  return null;
}

function isCommentOrEmpty(line) {
  const t = line.trim();
  return t.length === 0 || t[0] === '#';
}

function parseFocal(model, params) {
  if (!params.length) return 0;
  if (model === 'PINHOLE' || model === 'OPENCV' || model === 'FULL_OPENCV' || model === 'OPENCV_FISHEYE') {
    return 0.5 * (params[0] + params[1]);
  }
  return params[0];
}

function parsePrincipal(model, params, width, height) {
  const defCx = 0.5 * width;
  const defCy = 0.5 * height;
  if (model === 'SIMPLE_PINHOLE' && params.length >= 3) return { cx: params[1], cy: params[2] };
  if (model === 'PINHOLE' && params.length >= 4) return { cx: params[2], cy: params[3] };
  if ((model === 'SIMPLE_RADIAL' || model === 'SIMPLE_RADIAL_FISHEYE') && params.length >= 3) {
    return { cx: params[1], cy: params[2] };
  }
  if ((model === 'RADIAL' || model === 'RADIAL_FISHEYE' || model === 'FOV') && params.length >= 3) {
    return { cx: params[1], cy: params[2] };
  }
  if ((model === 'OPENCV' || model === 'FULL_OPENCV' || model === 'OPENCV_FISHEYE') && params.length >= 4) {
    return { cx: params[2], cy: params[3] };
  }
  return { cx: defCx, cy: defCy };
}

function loadCamerasText(filePath) {
  const cameras = new Map();
  const lines = fs.readFileSync(filePath, 'utf8').split(/\r?\n/);
  for (const line of lines) {
    if (isCommentOrEmpty(line)) continue;
    const parts = line.trim().split(/\s+/);
    if (parts.length < 5) continue;
    const id = Number(parts[0]);
    const model = parts[1];
    const width = Number(parts[2]);
    const height = Number(parts[3]);
    const params = parts.slice(4).map(Number);
    cameras.set(id, {
      id,
      model,
      width,
      height,
      params,
      focal: parseFocal(model, params),
      ...parsePrincipal(model, params, width, height)
    });
  }
  return cameras;
}

function quatToRotationMatrix(qw, qx, qy, qz) {
  const n = Math.hypot(qw, qx, qy, qz) || 1;
  qw /= n;
  qx /= n;
  qy /= n;
  qz /= n;
  const xx = qx * qx;
  const yy = qy * qy;
  const zz = qz * qz;
  const xy = qx * qy;
  const xz = qx * qz;
  const yz = qy * qz;
  const wx = qw * qx;
  const wy = qw * qy;
  const wz = qw * qz;
  return [
    1 - 2 * (yy + zz),
    2 * (xy - wz),
    2 * (xz + wy),
    2 * (xy + wz),
    1 - 2 * (xx + zz),
    2 * (yz - wx),
    2 * (xz - wy),
    2 * (yz + wx),
    1 - 2 * (xx + yy)
  ];
}

function cameraCenterFromRt(R, t) {
  return [
    -(R[0] * t[0] + R[3] * t[1] + R[6] * t[2]),
    -(R[1] * t[0] + R[4] * t[1] + R[7] * t[2]),
    -(R[2] * t[0] + R[5] * t[1] + R[8] * t[2])
  ];
}

function applyCvToViewerRotation(R) {
  return [R[0], R[1], R[2], -R[3], -R[4], -R[5], -R[6], -R[7], -R[8]];
}

function makeImageRecord(imageId, name, cameraId, cam, qw, qx, qy, qz, tx, ty, tz) {
  const Rcv = quatToRotationMatrix(qw, qx, qy, qz);
  const t = [tx, ty, tz];
  const center = cameraCenterFromRt(Rcv, t);
  return {
    id: imageId,
    name,
    cameraId,
    width: cam.width,
    height: cam.height,
    focal: cam.focal,
    cx: cam.cx,
    cy: cam.cy,
    R: applyCvToViewerRotation(Rcv),
    t: [t[0], -t[1], -t[2]],
    center,
    Rcv,
    tcv: t
  };
}

/**
 * @returns {{ images: object[], uvByImage: Map<number, Array<{u:number,v:number}>> }}
 */
function loadImagesText(filePath, cameras) {
  const images = [];
  const uvByImage = new Map();
  const lines = fs.readFileSync(filePath, 'utf8').split(/\r?\n/);
  let i = 0;
  while (i < lines.length) {
    const line = lines[i++];
    if (isCommentOrEmpty(line)) continue;
    const parts = line.trim().split(/\s+/);
    if (parts.length < 10) continue;
    const imageId = Number(parts[0]);
    const qw = Number(parts[1]);
    const qx = Number(parts[2]);
    const qy = Number(parts[3]);
    const qz = Number(parts[4]);
    const tx = Number(parts[5]);
    const ty = Number(parts[6]);
    const tz = Number(parts[7]);
    const cameraId = Number(parts[8]);
    const name = parts.slice(9).join(' ');

    const points2D = [];
    if (i < lines.length) {
      const p2 = lines[i++];
      if (!isCommentOrEmpty(p2)) {
        const toks = p2.trim().split(/\s+/);
        for (let k = 0; k + 2 < toks.length; k += 3) {
          points2D.push({ u: Number(toks[k]), v: Number(toks[k + 1]) });
        }
      }
    }
    uvByImage.set(imageId, points2D);

    const cam = cameras.get(cameraId);
    if (!cam) continue;
    images.push(makeImageRecord(imageId, name, cameraId, cam, qw, qx, qy, qz, tx, ty, tz));
  }
  return { images, uvByImage };
}

function loadPointsText(filePath, uvByImage) {
  const lines = fs.readFileSync(filePath, 'utf8').split(/\r?\n/);
  const positions = [];
  const colors = [];
  const trackLengths = [];
  const observations = [];
  let count = 0;
  for (const line of lines) {
    if (isCommentOrEmpty(line)) continue;
    const parts = line.trim().split(/\s+/);
    if (parts.length < 8) continue;
    const x = Number(parts[1]);
    const y = Number(parts[2]);
    const z = Number(parts[3]);
    const r = Number(parts[4]) / 255;
    const g = Number(parts[5]) / 255;
    const b = Number(parts[6]) / 255;
    const track = [];
    for (let k = 8; k + 1 < parts.length; k += 2) {
      const imageId = Number(parts[k]);
      const point2DIdx = Number(parts[k + 1]);
      const uvs = uvByImage.get(imageId);
      let u = 0;
      let v = 0;
      if (uvs && point2DIdx >= 0 && point2DIdx < uvs.length) {
        u = uvs[point2DIdx].u;
        v = uvs[point2DIdx].v;
      }
      track.push({ imageId, u, v });
    }
    positions.push(x, y, z);
    colors.push(r, g, b);
    trackLengths.push(track.length);
    observations.push(track);
    count += 1;
  }
  return {
    count,
    positions: new Float32Array(positions),
    colors: new Float32Array(colors),
    trackLengths: new Uint32Array(trackLengths),
    observations
  };
}

class BinaryReader {
  constructor(buf) {
    this.buf = Buffer.isBuffer(buf) ? buf : Buffer.from(buf);
    this.offset = 0;
  }

  readUint64() {
    const lo = this.buf.readUInt32LE(this.offset);
    const hi = this.buf.readUInt32LE(this.offset + 4);
    this.offset += 8;
    return hi * 0x100000000 + lo;
  }

  readInt32() {
    const v = this.buf.readInt32LE(this.offset);
    this.offset += 4;
    return v;
  }

  readUint32() {
    const v = this.buf.readUInt32LE(this.offset);
    this.offset += 4;
    return v;
  }

  readUint8() {
    const v = this.buf.readUInt8(this.offset);
    this.offset += 1;
    return v;
  }

  readDouble() {
    const v = this.buf.readDoubleLE(this.offset);
    this.offset += 8;
    return v;
  }

  readString() {
    let end = this.offset;
    while (end < this.buf.length && this.buf[end] !== 0) end += 1;
    const s = this.buf.toString('utf8', this.offset, end);
    this.offset = end + 1;
    return s;
  }
}

function loadCamerasBinary(filePath) {
  const reader = new BinaryReader(fs.readFileSync(filePath));
  const num = reader.readUint64();
  const cameras = new Map();
  for (let i = 0; i < num; i++) {
    const id = reader.readInt32();
    const modelId = reader.readInt32();
    const width = Number(reader.readUint64());
    const height = Number(reader.readUint64());
    const model = MODEL_NAMES[modelId] || 'PINHOLE';
    const nParams = CAMERA_MODEL_PARAM_COUNT[model] || 4;
    const params = [];
    for (let p = 0; p < nParams; p++) params.push(reader.readDouble());
    cameras.set(id, {
      id,
      model,
      width,
      height,
      params,
      focal: parseFocal(model, params),
      ...parsePrincipal(model, params, width, height)
    });
  }
  return cameras;
}

function loadImagesBinary(filePath, cameras) {
  const reader = new BinaryReader(fs.readFileSync(filePath));
  const num = reader.readUint64();
  const images = [];
  const uvByImage = new Map();
  for (let i = 0; i < num; i++) {
    const imageId = reader.readInt32();
    const qw = reader.readDouble();
    const qx = reader.readDouble();
    const qy = reader.readDouble();
    const qz = reader.readDouble();
    const tx = reader.readDouble();
    const ty = reader.readDouble();
    const tz = reader.readDouble();
    const cameraId = reader.readInt32();
    const name = reader.readString();
    const numPoints2D = reader.readUint64();
    const points2D = [];
    for (let k = 0; k < numPoints2D; k++) {
      const u = reader.readDouble();
      const v = reader.readDouble();
      reader.readUint64(); // point3D_id
      points2D.push({ u, v });
    }
    uvByImage.set(imageId, points2D);
    const cam = cameras.get(cameraId);
    if (!cam) continue;
    images.push(makeImageRecord(imageId, name, cameraId, cam, qw, qx, qy, qz, tx, ty, tz));
  }
  return { images, uvByImage };
}

function loadPointsBinary(filePath, uvByImage) {
  const reader = new BinaryReader(fs.readFileSync(filePath));
  const num = reader.readUint64();
  const positions = new Float32Array(num * 3);
  const colors = new Float32Array(num * 3);
  const trackLengths = new Uint32Array(num);
  const observations = new Array(num);
  for (let i = 0; i < num; i++) {
    reader.readUint64(); // point3D_id
    positions[i * 3] = reader.readDouble();
    positions[i * 3 + 1] = reader.readDouble();
    positions[i * 3 + 2] = reader.readDouble();
    colors[i * 3] = reader.readUint8() / 255;
    colors[i * 3 + 1] = reader.readUint8() / 255;
    colors[i * 3 + 2] = reader.readUint8() / 255;
    reader.readDouble(); // error
    const trackLen = reader.readUint64();
    const track = [];
    for (let t = 0; t < trackLen; t++) {
      const imageId = reader.readInt32();
      const point2DIdx = reader.readUint32();
      const uvs = uvByImage.get(imageId);
      let u = 0;
      let v = 0;
      if (uvs && point2DIdx < uvs.length) {
        u = uvs[point2DIdx].u;
        v = uvs[point2DIdx].v;
      }
      track.push({ imageId, u, v });
    }
    trackLengths[i] = track.length;
    observations[i] = track;
  }
  return { count: num, positions, colors, trackLengths, observations };
}

function summarize(points, cameras) {
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;
  const pos = points.positions;
  for (let i = 0; i < pos.length; i += 3) {
    const x = pos[i];
    const y = pos[i + 1];
    const z = pos[i + 2];
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;
  }
  for (const cam of cameras) {
    const [x, y, z] = cam.center;
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;
  }
  if (!Number.isFinite(minX)) {
    minX = minY = minZ = -1;
    maxX = maxY = maxZ = 1;
  }
  return {
    pointCount: points.count,
    cameraCount: cameras.length,
    bbox: { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] }
  };
}

function loadColmapDirectory(dir) {
  const detected = detectFormat(dir);
  if (!detected) {
    throw new Error(
      `Not a COLMAP sparse model (need cameras/images/points3D .txt or .bin): ${dir}`
    );
  }
  const { root, format } = detected;
  let images;
  let points;
  let uvByImage;
  if (format === 'text') {
    const camerasMap = loadCamerasText(path.join(root, 'cameras.txt'));
    const loaded = loadImagesText(path.join(root, 'images.txt'), camerasMap);
    images = loaded.images;
    uvByImage = loaded.uvByImage;
    points = loadPointsText(path.join(root, 'points3D.txt'), uvByImage);
  } else {
    const camerasMap = loadCamerasBinary(path.join(root, 'cameras.bin'));
    const loaded = loadImagesBinary(path.join(root, 'images.bin'), camerasMap);
    images = loaded.images;
    uvByImage = loaded.uvByImage;
    points = loadPointsBinary(path.join(root, 'points3D.bin'), uvByImage);
  }
  const summary = summarize(points, images);
  return {
    root,
    format,
    points: {
      count: points.count,
      positions: Array.from(points.positions),
      colors: Array.from(points.colors),
      trackLengths: Array.from(points.trackLengths),
      observations: points.observations
    },
    cameras: images,
    summary
  };
}

module.exports = {
  detectFormat,
  loadColmapDirectory,
  quatToRotationMatrix,
  cameraCenterFromRt,
  applyCvToViewerRotation
};
