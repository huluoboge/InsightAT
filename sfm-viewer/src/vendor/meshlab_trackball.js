/**
 * MeshLab-style trackball — same idea as InsightAT
 * RenderRotationTool / RenderZoomTool / RenderPanTool.
 *
 * Rotate (LMB): virtual unit sphere around projected pivot; quaternion from
 *   last sphere hit → current (Shoemake / MeshLab).
 * Zoom (wheel): distance ×0.9 / ×1.1.
 * Pan (RMB/MMB): move camera + target in the view plane (depth-aware).
 */
import { Vector2, Vector3, Quaternion } from './three.module.js';

export class MeshLabTrackballControls {
  constructor(camera, domElement) {
    this.object = camera;
    this.domElement = domElement;
    this.target = new Vector3();

    this.enabled = true;
    this.rotateSpeed = 1.0;
    this.panSpeed = 1.0;
    this.minDistance = 0.001;
    this.maxDistance = Infinity;

    this._state = 'none';
    this._lastSphere = new Vector3();
    this._currSphere = new Vector3();
    this._panPrev = new Vector2();

    this._onPointerDown = this._onPointerDown.bind(this);
    this._onPointerMove = this._onPointerMove.bind(this);
    this._onPointerUp = this._onPointerUp.bind(this);
    this._onWheel = this._onWheel.bind(this);
    this._onContextMenu = (e) => e.preventDefault();

    domElement.style.touchAction = 'none';
    domElement.addEventListener('pointerdown', this._onPointerDown);
    domElement.addEventListener('pointermove', this._onPointerMove);
    domElement.addEventListener('pointerup', this._onPointerUp);
    domElement.addEventListener('pointercancel', this._onPointerUp);
    domElement.addEventListener('lostpointercapture', this._onPointerUp);
    domElement.addEventListener('wheel', this._onWheel, { passive: false });
    domElement.addEventListener('contextmenu', this._onContextMenu);
  }

  dispose() {
    const el = this.domElement;
    el.removeEventListener('pointerdown', this._onPointerDown);
    el.removeEventListener('pointermove', this._onPointerMove);
    el.removeEventListener('pointerup', this._onPointerUp);
    el.removeEventListener('pointercancel', this._onPointerUp);
    el.removeEventListener('lostpointercapture', this._onPointerUp);
    el.removeEventListener('wheel', this._onWheel);
    el.removeEventListener('contextmenu', this._onContextMenu);
  }

  handleResize() {}

  update() {
    this.object.lookAt(this.target);
  }

  /**
   * RenderRotationTool::convert_mouse_position_to_orientation
   */
  _mouseToSphere(clientX, clientY) {
    const rect = this.domElement.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;
    const x = clientX - rect.left;
    // Match C++: invert y to GL bottom-left
    const yGl = h - 1 - (clientY - rect.top);

    const pivot = this.target.clone().project(this.object);
    const xp = (pivot.x * 0.5 + 0.5) * w;
    // Three NDC y-up → pixel from bottom (GL)
    const ypGl = (pivot.y * 0.5 + 0.5) * h;

    const r = Math.max(w, h) / 2.0;
    let vx = x - xp;
    let vy = yGl - ypGl;
    const d1 = Math.hypot(vx, vy);

    vx /= r;
    vy /= r;
    const d2 = vx * vx + vy * vy;

    let vz;
    if (d2 > 1) {
      const d = Math.sqrt(d2);
      vx /= d;
      vy /= d;
      vz = 0;
    } else {
      vz = Math.sqrt(1.0 - d2);
    }
    if (d1 > r) vz *= -1;

    return new Vector3(vx, vy, vz);
  }

  /**
   * generate_gl_rotation_matrix_from_vectors, applied to the *camera*.
   * C++ rotates the model by R(from→to); camera orbit must use R⁻¹ for the same rolling-ball feel.
   */
  _rotateFromSphere(from, to) {
    const a = from.clone().normalize();
    const b = to.clone().normalize();
    let ps = a.dot(b);
    ps = Math.min(1, Math.max(-1, ps));
    // Negate angle: model-space R ↔ camera-orbit R^{-1}
    const angle = -Math.acos(ps) * this.rotateSpeed;
    if (Math.abs(angle) < 1e-8) return;

    const axisSphere = new Vector3().crossVectors(a, b);
    if (axisSphere.lengthSq() < 1e-12) return;
    axisSphere.normalize();

    const eye = new Vector3().subVectors(this.object.position, this.target);
    const towardCam = eye.clone().normalize();
    const up0 = this.object.up.clone().normalize();
    const right = new Vector3().crossVectors(up0, towardCam).normalize();
    const up = new Vector3().crossVectors(towardCam, right).normalize();

    const axisWorld = new Vector3()
      .addScaledVector(right, axisSphere.x)
      .addScaledVector(up, axisSphere.y)
      .addScaledVector(towardCam, axisSphere.z)
      .normalize();

    const q = new Quaternion().setFromAxisAngle(axisWorld, angle);
    eye.applyQuaternion(q);
    this.object.up.applyQuaternion(q);
    this.object.position.copy(this.target).add(eye);
    this.object.lookAt(this.target);
  }

  _pan(dxPix, dyPix) {
    const rect = this.domElement.getBoundingClientRect();
    const w = Math.max(rect.width, 1);
    const h = Math.max(rect.height, 1);

    const eye = new Vector3().subVectors(this.object.position, this.target);
    const dist = eye.length();
    const fov = (this.object.fov * Math.PI) / 180;
    const worldH = 2 * dist * Math.tan(fov / 2);
    const worldW = worldH * this.object.aspect;

    // Grab-pan: content follows the mouse (same as MeshLab moving the model).
    // Mouse right → scene right → move camera+target left.
    const dx = (-dxPix / w) * worldW * this.panSpeed;
    const dy = (dyPix / h) * worldH * this.panSpeed;

    const towardCam = eye.clone().normalize();
    const right = new Vector3().crossVectors(this.object.up, towardCam).normalize();
    const up = new Vector3().crossVectors(towardCam, right).normalize();

    const pan = new Vector3().addScaledVector(right, dx).addScaledVector(up, dy);
    this.object.position.add(pan);
    this.target.add(pan);
  }

  _onPointerDown(event) {
    if (!this.enabled) return;
    if (event.button === 0) {
      this._state = 'rotate';
      this._lastSphere.copy(this._mouseToSphere(event.clientX, event.clientY));
      this.domElement.setPointerCapture(event.pointerId);
    } else if (event.button === 2 || event.button === 1) {
      this._state = 'pan';
      this._panPrev.set(event.clientX, event.clientY);
      this.domElement.setPointerCapture(event.pointerId);
    }
  }

  _onPointerMove(event) {
    if (!this.enabled || this._state === 'none') return;
    if (this._state === 'rotate') {
      this._currSphere.copy(this._mouseToSphere(event.clientX, event.clientY));
      if (this._currSphere.distanceToSquared(this._lastSphere) > 1e-18) {
        this._rotateFromSphere(this._lastSphere, this._currSphere);
        this._lastSphere.copy(this._currSphere);
      }
    } else if (this._state === 'pan') {
      const dx = event.clientX - this._panPrev.x;
      const dy = event.clientY - this._panPrev.y;
      this._panPrev.set(event.clientX, event.clientY);
      if (dx || dy) this._pan(dx, dy);
    }
  }

  _onPointerUp(event) {
    if (this._state !== 'none' && event.pointerId != null) {
      try {
        this.domElement.releasePointerCapture(event.pointerId);
      } catch (_) {
        /* ignore */
      }
    }
    this._state = 'none';
  }

  _onWheel(event) {
    if (!this.enabled) return;
    event.preventDefault();
    // RenderZoomTool: delta>0 → scale 1.1 (object); camera zoom-out ≡ ×1.1 on distance
    const factor = event.deltaY > 0 ? 1.1 : 0.9;
    const eye = new Vector3().subVectors(this.object.position, this.target);
    const dist = eye.length();
    if (dist < 1e-12) return;
    const next = Math.min(this.maxDistance, Math.max(this.minDistance, dist * factor));
    eye.setLength(next);
    this.object.position.copy(this.target).add(eye);
  }
}
