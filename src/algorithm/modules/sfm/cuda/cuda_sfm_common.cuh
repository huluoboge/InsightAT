/**
 * @file  cuda_sfm_common.cuh
 * @brief Shared CUDA __device__ helper functions for SfM CUDA kernels.
 *
 * Header-only (no .cu required). Every function is __device__ __forceinline__.
 *
 * Contents
 * ────────────────────────────────────────────────────────────────────────────
 *  GpuIntrinsics          FP32 intrinsics struct {fx,fy,cx,cy,k1,k2,k3,p1,p2}
 *  dev_distort_f          Brown-Conrady forward distortion (Bentley convention)
 *  dev_undistort_nrm_f    Undistortion via fixed-point iteration, normalised space
 *  dev_project_f          World → distorted pixel with depth guard
 *  dev_reproj_sq_f        Squared reprojection error (1e20 if behind camera)
 *  dev_mat3_mul           3×3 row-major matrix multiply
 *  dev_so3_update         In-place Rodrigues rotation update R ← exp(ω)·R
 *  dev_triangulate_2view  Closest-point-on-two-rays (= 2-view DLT) triangulation
 *  dev_depth              Camera-space z-depth of a world point
 *  dev_ray_angle_deg      Angle in degrees between two rays to a 3D point
 *
 * Conventions
 * ────────────────────────────────────────────────────────────────────────────
 *  • R[9]   row-major 3×3 rotation matrix
 *  • C[3]   camera centre in world coords   (t_cam = -R·C)
 *  • K flat: {fx, fy, cx, cy, k1, k2, k3, p1, p2} — 9 floats, used by loaders
 *  • Brown-Conrady (Bentley ContextCapture):
 *        u_d = (1+k1r²+k2r⁴+k3r⁶)·u  +  2·p2·u·v  +  p1·(r²+2u²)
 *        v_d = (1+k1r²+k2r⁴+k3r⁶)·v  +  2·p1·u·v  +  p2·(r²+2v²)
 *    Note: p1/p2 roles are swapped relative to the OpenCV convention.
 */

#pragma once
#ifndef INSIGHT_CUDA_SFM_COMMON_CUH
#define INSIGHT_CUDA_SFM_COMMON_CUH

#include <cuda_runtime.h>
#include <math.h>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// GpuIntrinsics
// ─────────────────────────────────────────────────────────────────────────────

/// FP32 camera intrinsics for use inside CUDA kernels.
/// Load from a flat float[9] array: {fx,fy,cx,cy,k1,k2,k3,p1,p2}.
struct GpuIntrinsics {
    float fx, fy, cx, cy;
    float k1, k2, k3, p1, p2;

    __device__ __forceinline__
    static GpuIntrinsics from_array(const float* arr) {
        GpuIntrinsics K;
        K.fx = arr[0]; K.fy = arr[1]; K.cx = arr[2]; K.cy = arr[3];
        K.k1 = arr[4]; K.k2 = arr[5]; K.k3 = arr[6];
        K.p1 = arr[7]; K.p2 = arr[8];
        return K;
    }

    __device__ __forceinline__ bool has_distortion() const {
        return k1 != 0.0f || k2 != 0.0f || k3 != 0.0f || p1 != 0.0f || p2 != 0.0f;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Brown-Conrady distortion (Bentley/ContextCapture convention)
// ─────────────────────────────────────────────────────────────────────────────

/// Forward distortion: normalised → distorted normalised coordinates.
/// Matches camera_utils.cpp::distort_normalised exactly.
__device__ __forceinline__
void dev_distort_f(float xn, float yn,
                   float k1, float k2, float k3, float p1, float p2,
                   float* xd, float* yd)
{
    const float r2  = xn*xn + yn*yn;
    const float r4  = r2 * r2;
    const float r6  = r4 * r2;
    const float rad = 1.0f + k1*r2 + k2*r4 + k3*r6;
    *xd = xn*rad + 2.0f*p2*xn*yn + p1*(r2 + 2.0f*xn*xn);
    *yd = yn*rad + 2.0f*p1*xn*yn + p2*(r2 + 2.0f*yn*yn);
}

// ─────────────────────────────────────────────────────────────────────────────
// Undistortion (fixed-point iteration in normalised space)
// ─────────────────────────────────────────────────────────────────────────────

/// Undistort a distorted pixel observation to undistorted normalised coords.
/// Algorithm: fixed-point iteration  xn ← xn - (distort(xn) - xn_obs)
/// in normalised space (identical to camera_utils.cpp::undistort_point).
///
/// @param n_iters  8 iterations is sufficient for typical aerial lenses
///                 (CPU uses 20; converges in ~5).
__device__ __forceinline__
void dev_undistort_nrm_f(float u_px, float v_px,
                          float fx, float fy, float cx, float cy,
                          float k1, float k2, float k3, float p1, float p2,
                          float* xn_out, float* yn_out,
                          int n_iters = 8)
{
    // Observed normalised coords (distorted pixel → normalised space)
    const float xn_obs = (u_px - cx) / fx;
    const float yn_obs = (v_px - cy) / fy;

    float xn = xn_obs, yn = yn_obs;
    for (int i = 0; i < n_iters; ++i) {
        float xd, yd;
        dev_distort_f(xn, yn, k1, k2, k3, p1, p2, &xd, &yd);
        const float dx = xd - xn_obs;
        const float dy = yd - yn_obs;
        xn -= dx;
        yn -= dy;
        if (fabsf(dx) < 1e-5f && fabsf(dy) < 1e-5f) break;
    }
    *xn_out = xn;
    *yn_out = yn;
}

// ─────────────────────────────────────────────────────────────────────────────
// Projection
// ─────────────────────────────────────────────────────────────────────────────

/// Project world point (Xw,Yw,Zw) into distorted pixel (u,v).
/// R[9] row-major, C[3] camera centre.
/// Returns false if the point is behind the camera (z ≤ 1e-6).
__device__ __forceinline__
bool dev_project_f(const float* __restrict__ R,
                   const float* __restrict__ C,
                   const GpuIntrinsics& K,
                   float Xw, float Yw, float Zw,
                   float* u_out, float* v_out)
{
    // Xc = R * (X - C)
    const float dx = Xw - C[0], dy = Yw - C[1], dz = Zw - C[2];
    const float xc = R[0]*dx + R[1]*dy + R[2]*dz;
    const float yc = R[3]*dx + R[4]*dy + R[5]*dz;
    const float zc = R[6]*dx + R[7]*dy + R[8]*dz;
    if (zc <= 1e-6f) return false;

    const float inv_z = 1.0f / zc;
    const float xn = xc * inv_z;
    const float yn = yc * inv_z;

    float xd, yd;
    dev_distort_f(xn, yn, K.k1, K.k2, K.k3, K.p1, K.p2, &xd, &yd);
    *u_out = K.fx * xd + K.cx;
    *v_out = K.fy * yd + K.cy;
    return true;
}

/// Squared reprojection error (pixels²).
/// Returns 1e20f if behind camera.
__device__ __forceinline__
float dev_reproj_sq_f(const float* __restrict__ R,
                      const float* __restrict__ C,
                      const GpuIntrinsics& K,
                      float Xw, float Yw, float Zw,
                      float u_obs, float v_obs)
{
    float u_proj, v_proj;
    if (!dev_project_f(R, C, K, Xw, Yw, Zw, &u_proj, &v_proj))
        return 1e20f;
    const float du = u_obs - u_proj;
    const float dv = v_obs - v_proj;
    return du*du + dv*dv;
}

// ─────────────────────────────────────────────────────────────────────────────
// 3×3 matrix utilities (row-major)
// ─────────────────────────────────────────────────────────────────────────────

/// C = A * B  (3×3 row-major)
__device__ __forceinline__
void dev_mat3_mul(const float* __restrict__ A,
                  const float* __restrict__ B,
                  float* __restrict__ C)
{
#pragma unroll
    for (int i = 0; i < 3; ++i)
#pragma unroll
        for (int j = 0; j < 3; ++j)
            C[i*3+j] = A[i*3+0]*B[0*3+j]
                     + A[i*3+1]*B[1*3+j]
                     + A[i*3+2]*B[2*3+j];
}

/// In-place Rodrigues update: R ← exp(ω) · R
/// ω is an infinitesimal rotation vector (axis-angle).
__device__ __forceinline__
void dev_so3_update(float* R, const float* omega)
{
    const float angle = sqrtf(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
    if (angle < 1e-10f) return;

    const float inv_a = 1.0f / angle;
    const float ox = omega[0]*inv_a, oy = omega[1]*inv_a, oz = omega[2]*inv_a;
    const float s  = sinf(angle), c = cosf(angle), c1 = 1.0f - c;

    float Rw[9];
    Rw[0] = c + ox*ox*c1;        Rw[1] = ox*oy*c1 - oz*s; Rw[2] = ox*oz*c1 + oy*s;
    Rw[3] = oy*ox*c1 + oz*s;     Rw[4] = c + oy*oy*c1;    Rw[5] = oy*oz*c1 - ox*s;
    Rw[6] = oz*ox*c1 - oy*s;     Rw[7] = oz*oy*c1 + ox*s; Rw[8] = c + oz*oz*c1;

    float Rnew[9];
    dev_mat3_mul(Rw, R, Rnew);
#pragma unroll
    for (int i = 0; i < 9; ++i) R[i] = Rnew[i];
}

// ─────────────────────────────────────────────────────────────────────────────
// 2-view triangulation (closest-point-on-two-rays / midpoint method)
// ─────────────────────────────────────────────────────────────────────────────

/// Triangulate a 3D point from two views given undistorted normalised ray endpoints.
///
/// Method:  construct world-space ray directions d1, d2 = Rᵀ * [xn, yn, 1],
///          find the closest-approach midpoint (2×2 linear system).
///          Numerically equivalent to the DLT null-vector for exactly 2 views.
///
/// @param R1, C1   Rotation (row-major) and centre for view 1
/// @param xn1,yn1  Undistorted normalised coords for view 1
/// @param R2, C2   Rotation and centre for view 2
/// @param xn2,yn2  Undistorted normalised coords for view 2
/// @param X_out    Output 3D point [3]
/// @return false if rays are nearly parallel (degenerate triangulation)
__device__ __forceinline__
bool dev_triangulate_2view(const float* __restrict__ R1, const float* __restrict__ C1,
                            float xn1, float yn1,
                            const float* __restrict__ R2, const float* __restrict__ C2,
                            float xn2, float yn2,
                            float* X_out)
{
    // Ray directions: d = Rᵀ * [xn, yn, 1]ᵀ
    // Rᵀ * v = sum over j of R[j*3+i] * v[j]  (column i of R for row i of Rᵀ)
    float d1[3], d2[3];
    d1[0] = R1[0]*xn1 + R1[3]*yn1 + R1[6];
    d1[1] = R1[1]*xn1 + R1[4]*yn1 + R1[7];
    d1[2] = R1[2]*xn1 + R1[5]*yn1 + R1[8];

    d2[0] = R2[0]*xn2 + R2[3]*yn2 + R2[6];
    d2[1] = R2[1]*xn2 + R2[4]*yn2 + R2[7];
    d2[2] = R2[2]*xn2 + R2[5]*yn2 + R2[8];

    // Minimise ||C1 + t1·d1 − C2 − t2·d2||²
    // Normal equations (2×2 symmetric system):
    //   [ d1·d1  −d1·d2 ] [t1]   [(C2−C1)·d1]
    //   [ d1·d2  −d2·d2 ] [t2] = [(C2−C1)·d2]
    const float wx = C2[0]-C1[0], wy = C2[1]-C1[1], wz = C2[2]-C1[2];
    const float a11 = d1[0]*d1[0] + d1[1]*d1[1] + d1[2]*d1[2]; // d1·d1
    const float a12 = d1[0]*d2[0] + d1[1]*d2[1] + d1[2]*d2[2]; // d1·d2
    const float a22 = d2[0]*d2[0] + d2[1]*d2[1] + d2[2]*d2[2]; // d2·d2
    const float b1  = wx*d1[0] + wy*d1[1] + wz*d1[2];           // (C2-C1)·d1
    const float b2  = wx*d2[0] + wy*d2[1] + wz*d2[2];           // (C2-C1)·d2

    // System: [a11, -a12; a12, -a22] * [t1; t2] = [b1; b2]
    const float det = a12*a12 - a11*a22;  // determinant
    if (fabsf(det) < 1e-12f) return false;

    const float inv_det = 1.0f / det;
    const float t1 = (a12*b2 - a22*b1) * inv_det;
    const float t2 = (a11*b2 - a12*b1) * inv_det;

    // Midpoint of the two closest points
    X_out[0] = 0.5f * ((C1[0] + t1*d1[0]) + (C2[0] + t2*d2[0]));
    X_out[1] = 0.5f * ((C1[1] + t1*d1[1]) + (C2[1] + t2*d2[1]));
    X_out[2] = 0.5f * ((C1[2] + t1*d1[2]) + (C2[2] + t2*d2[2]));
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Geometric checks
// ─────────────────────────────────────────────────────────────────────────────

/// Camera-space z-depth of world point (Xw,Yw,Zw) for camera (R, C).
/// R[9] row-major, C[3] centre. Returns the z component of R·(X−C).
__device__ __forceinline__
float dev_depth(const float* __restrict__ R, const float* __restrict__ C,
                float Xw, float Yw, float Zw)
{
    return R[6]*(Xw-C[0]) + R[7]*(Yw-C[1]) + R[8]*(Zw-C[2]);
}

/// Half-angle (degrees) between rays from C1→X and C2→X.
/// Used for triangulation angle check.
__device__ __forceinline__
float dev_ray_angle_deg(const float* __restrict__ C1, const float* __restrict__ C2,
                        float Xw, float Yw, float Zw)
{
    float r1[3] = {Xw-C1[0], Yw-C1[1], Zw-C1[2]};
    float r2[3] = {Xw-C2[0], Yw-C2[1], Zw-C2[2]};
    const float n1 = sqrtf(r1[0]*r1[0] + r1[1]*r1[1] + r1[2]*r1[2]);
    const float n2 = sqrtf(r2[0]*r2[0] + r2[1]*r2[1] + r2[2]*r2[2]);
    if (n1 < 1e-8f || n2 < 1e-8f) return 0.0f;
    float dot = (r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2]) / (n1 * n2);
    dot = fmaxf(-1.0f, fminf(1.0f, dot));
    return acosf(dot) * (180.0f / 3.14159265358979323846f);
}

}  // namespace cuda
}  // namespace insight

#endif  // INSIGHT_CUDA_SFM_COMMON_CUH
