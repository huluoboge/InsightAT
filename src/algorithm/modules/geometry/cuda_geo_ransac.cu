/**
 * cuda_geo_ransac.cu
 * CUDA port of the OpenGL 4.3 compute path in gpu_geo_ransac.cpp (F/E/H RANSAC).
 * One thread per RANSAC iteration; Hartley normalization and denormalisation on host.
 */

#include "cuda_geo_ransac.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Globals (mirror gpu_geo_ransac.cpp)
// ─────────────────────────────────────────────────────────────────────────────

static int s_num_iter = 2000;
static int s_verbose = 0;
static int s_solver = 1;
static int s_cuda_device = 0;
static bool s_inited = false;

// ── Persistent device buffers (allocated once in cuda_geo_init, freed in shutdown) ──
// Avoids ~1 ms cudaMalloc overhead per call; same pattern as gpu_geo_ransac SSBOs.
static float4* s_d_matches  = nullptr;  // normalised match pairs (n × float4)
static int*    s_d_idx      = nullptr;  // RANSAC sample indices  (iter × maxK; maxK=8)
static float*  s_d_res      = nullptr;  // per-iteration results  (iter × 10)
static int     s_cap_n      = 0;        // allocated capacity for matches
static int     s_cap_iter   = 0;        // allocated capacity for iterations

// Async pipeline: dedicated stream + pinned H2D buffers + tiny device argmax scratch.
static cudaStream_t s_stream = nullptr;
static int*         s_d_best_idx = nullptr;
static float4*      s_h_matches_pin = nullptr;
static int*         s_h_idx_pin = nullptr;
static int          s_h_cap_n = 0, s_h_cap_iter = 0;

static constexpr int kIdxMaxK = 8;  // F/E use 8; H uses 4 — allocate idx row stride 8 always

// ── Batch path (separate buffers from single-pair) ───────────────────────────
static float4* s_db_matches = nullptr;
static float* s_db_res = nullptr;
static int* s_db_match_off = nullptr;
static int* s_db_res_off = nullptr;
static int* s_db_pair_n = nullptr;
static float* s_db_thresh = nullptr;
static int* s_db_best = nullptr;
static size_t s_db_cap_nfloat4 = 0;
static size_t s_db_cap_res = 0;
static bool s_db_aux_ok = false;

// ─────────────────────────────────────────────────────────────────────────────
// Host: Hartley normalization (same as gpu_geo_ransac.cpp)
// ─────────────────────────────────────────────────────────────────────────────

static inline void mat3_mul_h(const double A[9], const double B[9], double C[9]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      C[i * 3 + j] = 0.0;
      for (int k = 0; k < 3; k++)
        C[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
    }
}

static inline void mat3_transpose_h(const double A[9], double At[9]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      At[i * 3 + j] = A[j * 3 + i];
}

static inline void hartley_inverse_h(const double T[9], double Ti[9]) {
  double s = T[0];
  double cx = -T[2] / s;
  double cy = -T[5] / s;
  Ti[0] = 1.0 / s;
  Ti[1] = 0.0;
  Ti[2] = cx;
  Ti[3] = 0.0;
  Ti[4] = 1.0 / s;
  Ti[5] = cy;
  Ti[6] = 0.0;
  Ti[7] = 0.0;
  Ti[8] = 1.0;
}

static void hartley_norm_h(const Match2D* pts, int n, bool use_p2, double T[9]) {
  double cx = 0.0, cy = 0.0;
  for (int i = 0; i < n; i++) {
    cx += use_p2 ? pts[i].x2 : pts[i].x1;
    cy += use_p2 ? pts[i].y2 : pts[i].y1;
  }
  cx /= n;
  cy /= n;
  double rms = 0.0;
  for (int i = 0; i < n; i++) {
    double dx = (use_p2 ? pts[i].x2 : pts[i].x1) - cx;
    double dy = (use_p2 ? pts[i].y2 : pts[i].y1) - cy;
    rms += dx * dx + dy * dy;
  }
  rms = std::sqrt(rms / n);
  if (rms < 1e-10)
    rms = 1.0;
  double sc = std::sqrt(2.0) / rms;
  T[0] = sc;
  T[1] = 0.0;
  T[2] = -sc * cx;
  T[3] = 0.0;
  T[4] = sc;
  T[5] = -sc * cy;
  T[6] = 0.0;
  T[7] = 0.0;
  T[8] = 1.0;
}

static std::vector<Match2D> normalize_matches_h(const Match2D* src, int n, double T1[9],
                                                 double T2[9]) {
  hartley_norm_h(src, n, false, T1);
  hartley_norm_h(src, n, true, T2);
  std::vector<Match2D> out(static_cast<size_t>(n));
  for (int i = 0; i < n; i++) {
    double x1 = T1[0] * src[i].x1 + T1[2];
    double y1 = T1[4] * src[i].y1 + T1[5];
    double x2 = T2[0] * src[i].x2 + T2[2];
    double y2 = T2[4] * src[i].y2 + T2[5];
    out[static_cast<size_t>(i)] = {(float)x1, (float)y1, (float)x2, (float)y2};
  }
  return out;
}

static void denorm_H_h(const double T1[9], const double T2[9], const float Hn[9], float H[9]) {
  double T2i[9];
  hartley_inverse_h(T2, T2i);
  double tmp[9], res[9];
  double Hn_d[9];
  for (int i = 0; i < 9; i++)
    Hn_d[i] = Hn[i];
  mat3_mul_h(T2i, Hn_d, tmp);
  mat3_mul_h(tmp, T1, res);
  for (int i = 0; i < 9; i++)
    H[i] = (float)(res[i] / res[8]);
}

static void denorm_FE_h(const double T1[9], const double T2[9], const float Fn[9], float F[9]) {
  double T2t[9];
  mat3_transpose_h(T2, T2t);
  double tmp[9], res[9];
  double Fn_d[9];
  for (int i = 0; i < 9; i++)
    Fn_d[i] = Fn[i];
  mat3_mul_h(T2t, Fn_d, tmp);
  mat3_mul_h(tmp, T1, res);
  for (int i = 0; i < 9; i++)
    F[i] = (float)res[i];
}

// ─────────────────────────────────────────────────────────────────────────────
// Device helpers (ported from GLSL in gpu_geo_ransac.cpp)
// ─────────────────────────────────────────────────────────────────────────────

__device__ __forceinline__ float d_sign(float x) { return (x > 0.f) ? 1.f : ((x < 0.f) ? -1.f : 0.f); }

__device__ void sym_rot9(float* A, float* V, int p, int q) {
  float apq = A[p * 9 + q];
  if (fabsf(apq) < 1e-12f)
    return;
  float tau = 0.5f * (A[q * 9 + q] - A[p * 9 + p]) / apq;
  float t = d_sign(tau) / (fabsf(tau) + sqrtf(1.f + tau * tau));
  float c = 1.f / sqrtf(1.f + t * t);
  float s = t * c;
  float app = A[p * 9 + p], aqq = A[q * 9 + q];
  A[p * 9 + p] = app - t * apq;
  A[q * 9 + q] = aqq + t * apq;
  A[p * 9 + q] = 0.f;
  A[q * 9 + p] = 0.f;
  for (int r = 0; r < 9; r++) {
    if (r == p || r == q)
      continue;
    float arp = A[r * 9 + p], arq = A[r * 9 + q];
    A[r * 9 + p] = c * arp - s * arq;
    A[p * 9 + r] = A[r * 9 + p];
    A[r * 9 + q] = s * arp + c * arq;
    A[q * 9 + r] = A[r * 9 + q];
  }
  for (int r = 0; r < 9; r++) {
    float vrp = V[r * 9 + p], vrq = V[r * 9 + q];
    V[r * 9 + p] = c * vrp - s * vrq;
    V[r * 9 + q] = s * vrp + c * vrq;
  }
}

__device__ void null_vector_jacobi(const float* __restrict__ A72, float* __restrict__ x) {
  float B[81];
  for (int i = 0; i < 81; i++)
    B[i] = 0.f;
  for (int k = 0; k < 8; k++)
    for (int i = 0; i < 9; i++)
      for (int j = 0; j < 9; j++)
        B[i * 9 + j] += A72[k * 9 + i] * A72[k * 9 + j];
  float V[81];
  for (int i = 0; i < 81; i++)
    V[i] = 0.f;
  for (int i = 0; i < 9; i++)
    V[i * 9 + i] = 1.f;
  for (int sw = 0; sw < 20; sw++)
    for (int p = 0; p < 8; p++)
      for (int q = p + 1; q < 9; q++)
        sym_rot9(B, V, p, q);
  int imin = 0;
  for (int i = 1; i < 9; i++)
    if (B[i * 9 + i] < B[imin * 9 + imin])
      imin = i;
  for (int i = 0; i < 9; i++)
    x[i] = V[i * 9 + imin];
}

__device__ void null_vector_ipi(const float* __restrict__ A72, float* __restrict__ x) {
  float B[81];
  for (int i = 0; i < 81; i++)
    B[i] = 0.f;
  for (int k = 0; k < 8; k++)
    for (int i = 0; i < 9; i++)
      for (int j = 0; j < 9; j++)
        B[i * 9 + j] += A72[k * 9 + i] * A72[k * 9 + j];
  float tr = 0.f;
  for (int i = 0; i < 9; i++)
    tr += B[i * 9 + i];
  float mu = tr * 1e-3f + 1e-30f;
  for (int i = 0; i < 9; i++)
    B[i * 9 + i] += mu;
  for (int j = 0; j < 9; j++) {
    float s = B[j * 9 + j];
    for (int k = 0; k < j; k++)
      s -= B[j * 9 + k] * B[j * 9 + k];
    B[j * 9 + j] = sqrtf(fmaxf(s, 0.f)) + 1e-30f;
    for (int i = j + 1; i < 9; i++) {
      s = B[i * 9 + j];
      for (int k = 0; k < j; k++)
        s -= B[i * 9 + k] * B[j * 9 + k];
      B[i * 9 + j] = s / B[j * 9 + j];
    }
  }
  float inv3 = 1.f / 3.f;
  for (int i = 0; i < 9; i++)
    x[i] = inv3;
  float y[9];
  for (int it = 0; it < 6; it++) {
    for (int i = 0; i < 9; i++) {
      y[i] = x[i];
      for (int j = 0; j < i; j++)
        y[i] -= B[i * 9 + j] * y[j];
      y[i] /= B[i * 9 + i];
    }
    for (int i = 8; i >= 0; i--) {
      x[i] = y[i];
      for (int j = i + 1; j < 9; j++)
        x[i] -= B[j * 9 + i] * x[j];
      x[i] /= B[i * 9 + i];
    }
    float nrm = 0.f;
    for (int i = 0; i < 9; i++)
      nrm += x[i] * x[i];
    nrm = sqrtf(nrm) + 1e-30f;
    for (int i = 0; i < 9; i++)
      x[i] /= nrm;
  }
}

__device__ void sym_rot3(float* A, float* V, int p, int q) {
  float apq = A[p * 3 + q];
  if (fabsf(apq) < 1e-12f)
    return;
  float tau = 0.5f * (A[q * 3 + q] - A[p * 3 + p]) / apq;
  float t = d_sign(tau) / (fabsf(tau) + sqrtf(1.f + tau * tau));
  float c = 1.f / sqrtf(1.f + t * t);
  float s = t * c;
  float app = A[p * 3 + p], aqq = A[q * 3 + q];
  A[p * 3 + p] = app - t * apq;
  A[q * 3 + q] = aqq + t * apq;
  A[p * 3 + q] = 0.f;
  A[q * 3 + p] = 0.f;
  for (int r = 0; r < 3; r++) {
    if (r == p || r == q)
      continue;
    float arp = A[r * 3 + p], arq = A[r * 3 + q];
    A[r * 3 + p] = c * arp - s * arq;
    A[p * 3 + r] = A[r * 3 + p];
    A[r * 3 + q] = s * arp + c * arq;
    A[q * 3 + r] = A[r * 3 + q];
  }
  for (int r = 0; r < 3; r++) {
    float vrp = V[r * 3 + p], vrq = V[r * 3 + q];
    V[r * 3 + p] = c * vrp - s * vrq;
    V[r * 3 + q] = s * vrp + c * vrq;
  }
}

__device__ void svd3x3(const float* M, float* U, float* S, float* Vt) {
  float AtA[9];
  for (int i = 0; i < 9; i++)
    AtA[i] = 0.f;
  for (int k = 0; k < 3; k++)
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        AtA[i * 3 + j] += M[k * 3 + i] * M[k * 3 + j];
  float V[9];
  for (int i = 0; i < 9; i++)
    V[i] = 0.f;
  V[0] = 1.f;
  V[4] = 1.f;
  V[8] = 1.f;
  for (int sw = 0; sw < 12; sw++) {
    sym_rot3(AtA, V, 0, 1);
    sym_rot3(AtA, V, 0, 2);
    sym_rot3(AtA, V, 1, 2);
  }
  float sv[3];
  for (int i = 0; i < 3; i++)
    sv[i] = sqrtf(fmaxf(0.f, AtA[i * 3 + i]));
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++)
      if (sv[j] > sv[i]) {
        float tmp = sv[i];
        sv[i] = sv[j];
        sv[j] = tmp;
        for (int r = 0; r < 3; r++) {
          float t2 = V[r * 3 + i];
          V[r * 3 + i] = V[r * 3 + j];
          V[r * 3 + j] = t2;
        }
      }
  S[0] = sv[0];
  S[1] = sv[1];
  S[2] = sv[2];
  for (int i = 0; i < 9; i++)
    U[i] = 0.f;
  for (int j = 0; j < 3; j++) {
    if (S[j] < 1e-10f)
      continue;
    for (int i = 0; i < 3; i++) {
      float sm = 0.f;
      for (int k = 0; k < 3; k++)
        sm += M[i * 3 + k] * V[k * 3 + j];
      U[i * 3 + j] = sm / S[j];
    }
  }
  if (S[2] < 1e-10f) {
    float u0x = U[0], u0y = U[3], u0z = U[6];
    float u1x = U[1], u1y = U[4], u1z = U[7];
    float cx = u0y * u1z - u0z * u1y;
    float cy = u0z * u1x - u0x * u1z;
    float cz = u0x * u1y - u0y * u1x;
    float ln = sqrtf(cx * cx + cy * cy + cz * cz) + 1e-30f;
    cx /= ln;
    cy /= ln;
    cz /= ln;
    U[2] = cx;
    U[5] = cy;
    U[8] = cz;
  }
  float v0x = V[0], v0y = V[3], v0z = V[6];
  float v1x = V[1], v1y = V[4], v1z = V[7];
  float v2x = V[2], v2y = V[5], v2z = V[8];
  float cp_x = v0y * v1z - v0z * v1y;
  float cp_y = v0z * v1x - v0x * v1z;
  float cp_z = v0x * v1y - v0y * v1x;
  if (cp_x * v2x + cp_y * v2y + cp_z * v2z < 0.f) {
    V[2] = -V[2];
    V[5] = -V[5];
    V[8] = -V[8];
    U[2] = -U[2];
    U[5] = -U[5];
    U[8] = -U[8];
  }
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      Vt[i * 3 + j] = V[j * 3 + i];
}

__device__ void compose_svd(const float* U, const float* S, const float* Vt, float* M) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      float s = 0.f;
      for (int k = 0; k < 3; k++)
        s += U[i * 3 + k] * S[k] * Vt[k * 3 + j];
      M[i * 3 + j] = s;
    }
}

__device__ void build_dlt(const float2* p, const float2* q, int K, float* A72) {
  for (int i = 0; i < 72; i++)
    A72[i] = 0.f;
  if (K == 4) {
    for (int i = 0; i < 4; i++) {
      float x0 = p[i].x, y0 = p[i].y, u = q[i].x, v = q[i].y;
      int r0 = 2 * i, r1 = r0 + 1;
      A72[r0 * 9 + 0] = x0;
      A72[r0 * 9 + 1] = y0;
      A72[r0 * 9 + 2] = 1.f;
      A72[r0 * 9 + 6] = -u * x0;
      A72[r0 * 9 + 7] = -u * y0;
      A72[r0 * 9 + 8] = -u;
      A72[r1 * 9 + 3] = x0;
      A72[r1 * 9 + 4] = y0;
      A72[r1 * 9 + 5] = 1.f;
      A72[r1 * 9 + 6] = -v * x0;
      A72[r1 * 9 + 7] = -v * y0;
      A72[r1 * 9 + 8] = -v;
    }
  } else {
    for (int i = 0; i < 8; i++) {
      float x1 = p[i].x, y1 = p[i].y, x2 = q[i].x, y2 = q[i].y;
      A72[i * 9 + 0] = x2 * x1;
      A72[i * 9 + 1] = x2 * y1;
      A72[i * 9 + 2] = x2;
      A72[i * 9 + 3] = y2 * x1;
      A72[i * 9 + 4] = y2 * y1;
      A72[i * 9 + 5] = y2;
      A72[i * 9 + 6] = x1;
      A72[i * 9 + 7] = y1;
      A72[i * 9 + 8] = 1.f;
    }
  }
}

__device__ void enforce_rank2(float* F) {
  float U[9], S[3], Vt[9];
  svd3x3(F, U, S, Vt);
  S[2] = 0.f;
  compose_svd(U, S, Vt, F);
}

__device__ void enforce_essential(float* E) {
  float U[9], S[3], Vt[9];
  svd3x3(E, U, S, Vt);
  float sv = 0.5f * (S[0] + S[1]);
  S[0] = sv;
  S[1] = sv;
  S[2] = 0.f;
  compose_svd(U, S, Vt, E);
}

__device__ float error_H(const float* H, float px, float py, float qx, float qy) {
  const float EPS = 1e-9f;
  float hz = H[6] * px + H[7] * py + H[8];
  if (fabsf(hz) < EPS)
    return 1e9f;
  float hx = (H[0] * px + H[1] * py + H[2]) / hz;
  float hy = (H[3] * px + H[4] * py + H[5]) / hz;
  float dx = hx - qx, dy = hy - qy;
  return dx * dx + dy * dy;
}

__device__ float error_F(const float* F, float x1, float y1, float x2, float y2) {
  const float EPS = 1e-9f;
  float Fx1x = F[0] * x1 + F[1] * y1 + F[2];
  float Fx1y = F[3] * x1 + F[4] * y1 + F[5];
  float Fx1z = F[6] * x1 + F[7] * y1 + F[8];
  float Ftx2x = F[0] * x2 + F[3] * y2 + F[6];
  float Ftx2y = F[1] * x2 + F[4] * y2 + F[7];
  float num = x2 * Fx1x + y2 * Fx1y + Fx1z;
  float den = Fx1x * Fx1x + Fx1y * Fx1y + Ftx2x * Ftx2x + Ftx2y * Ftx2y;
  return (num * num) / (den + EPS);
}

// 128 threads/block gives good warp scheduling depth.
// min_blocks_per_sm=4 tells the compiler to limit register use so 4 blocks
// can reside on one SM simultaneously → better latency hiding.
__launch_bounds__(128, 4)
__global__ void geo_ransac_kernel(const float4* __restrict__ matches, const int* __restrict__ indices,
                                  float* __restrict__ results, int u_model, float u_thresh, int u_n,
                                  int u_solver, int num_iter, int K) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= num_iter)
    return;

  float2 p[8], q[8];
  for (int i = 0; i < K; i++) {
    int idx = indices[tid * K + i];
    float4 m = matches[idx];
    p[i] = make_float2(m.x, m.y);
    q[i] = make_float2(m.z, m.w);
  }

  float A72[72];
  build_dlt(p, q, K, A72);

  // Both solvers only READ A72 (they build their own internal matrices).
  // The old Acpy[72] copy was completely unnecessary – removing it saves
  // 288 bytes of local memory and reduces register spill pressure.
  float nv[9];
  if (u_solver == 1)
    null_vector_ipi(A72, nv);
  else
    null_vector_jacobi(A72, nv);

  float M[9];
  for (int i = 0; i < 9; i++)
    M[i] = nv[i];

  if (u_model == 1)
    enforce_rank2(M);
  if (u_model == 2)
    enforce_essential(M);

  if (u_model == 0 && fabsf(M[8]) > 1e-9f) {
    float sc = 1.f / M[8];
    for (int i = 0; i < 9; i++)
      M[i] *= sc;
  }

  int cnt = 0;
  for (int i = 0; i < u_n; i++) {
    float4 mm = matches[i];
    float e;
    if (u_model == 0)
      e = error_H(M, mm.x, mm.y, mm.z, mm.w);
    else
      e = error_F(M, mm.x, mm.y, mm.z, mm.w);
    if (e < u_thresh)
      cnt++;
  }

  int base = tid * 10;
  for (int i = 0; i < 9; i++)
    results[base + i] = M[i];
  results[base + 9] = (float)cnt;
}

// One block: each thread scans a strided subset of iterations, then parallel max-reduction
// on inlier counts (results[i*10+9]) to avoid copying the full (iter×10) buffer to the host.
__launch_bounds__(256, 1)
__global__ void find_best_ransac_iter_kernel(const float* __restrict__ results, int num_iter,
                                             int* __restrict__ out_best_idx) {
  __shared__ float s_cnt[256];
  __shared__ int s_idx[256];
  const int tid = threadIdx.x;
  const int nt = blockDim.x;
  float best_c = -1.f;
  int best_i = 0;
  for (int i = tid; i < num_iter; i += nt) {
    float c = results[i * 10 + 9];
    if (c > best_c) {
      best_c = c;
      best_i = i;
    }
  }
  s_cnt[tid] = best_c;
  s_idx[tid] = best_i;
  __syncthreads();
  for (unsigned int s = nt / 2; s > 0; s >>= 1) {
    if (tid < s) {
      if (s_cnt[tid + s] > s_cnt[tid]) {
        s_cnt[tid] = s_cnt[tid + s];
        s_idx[tid] = s_idx[tid + s];
      }
    }
    __syncthreads();
  }
  if (tid == 0)
    *out_best_idx = s_idx[0];
}

// ════════════════════════════════════════════════════════════════════════════
// Batch RANSAC: one block per image pair (blockIdx.x == pair id)
//
// In-kernel LCG random sampling: eliminates CPU index pre-generation and the
// enormous H2D transfer (P × iter × K ints).  Each thread runs its own LCG
// seeded by (pair, tid, global_seed) — statistically independent streams.
// Unique-sample guarantee: rejection loop; for K≤8, n≥K the expected extra
// draws per sample are K/n ≈ 4% overhead at n=200.  Inner loop has zero
// divergence after the first accepted draw because samp[] fits registers.
// ════════════════════════════════════════════════════════════════════════════

__device__ __forceinline__ uint32_t lcg_next(uint32_t& state) {
  // Numerical Recipes LCG (Knuth vol.2): full 32-bit cycle, good randomness
  state = state * 1664525u + 1013904223u;
  return state;
}

// Sample K unique indices from [0, n) using per-thread LCG (rejection sampling).
__device__ __forceinline__ void sample_unique_k(uint32_t& rng, int n, int K, int samp[8]) {
  for (int i = 0; i < K; ++i) {
    int idx;
    do {
      idx = (int)(lcg_next(rng) % (uint32_t)n);
      bool dup = false;
      for (int j = 0; j < i; ++j) dup |= (samp[j] == idx);
      if (!dup) break;
    } while (true);
    samp[i] = idx;
  }
}

__launch_bounds__(128, 4)
__global__ void geo_ransac_batch_kernel(const float4* __restrict__ all_matches,
                                        const int* __restrict__ match_off,
                                        const int* __restrict__ pair_n,
                                        float* __restrict__ all_results,
                                        const int* __restrict__ res_off,
                                        const float* __restrict__ pair_thresh_norm, int u_model,
                                        int u_solver, int num_iter, int K, int P,
                                        uint32_t seed) {
  const int pair = blockIdx.x;
  if (pair >= P)
    return;
  const int n = pair_n[pair];
  if (n < K)
    return;
  const float4* matches = all_matches + match_off[pair];
  float* results = all_results + res_off[pair];
  const float u_thresh = pair_thresh_norm[pair];

  for (int tid = threadIdx.x; tid < num_iter; tid += blockDim.x) {
    // Per-iteration unique LCG state: mix pair, tid, and global seed
    uint32_t rng = (uint32_t)pair * 2654435761u ^ (uint32_t)tid * 40503u ^ seed;
    lcg_next(rng);  // warm up one cycle

    float2 p[8], q[8];
    int samp[8];
    sample_unique_k(rng, n, K, samp);
    for (int i = 0; i < K; i++) {
      float4 m = matches[samp[i]];
      p[i] = make_float2(m.x, m.y);
      q[i] = make_float2(m.z, m.w);
    }

    float A72[72];
    build_dlt(p, q, K, A72);

    float nv[9];
    if (u_solver == 1)
      null_vector_ipi(A72, nv);
    else
      null_vector_jacobi(A72, nv);

    float M[9];
    for (int i = 0; i < 9; i++)
      M[i] = nv[i];

    if (u_model == 1)
      enforce_rank2(M);
    if (u_model == 2)
      enforce_essential(M);

    if (u_model == 0 && fabsf(M[8]) > 1e-9f) {
      float sc = 1.f / M[8];
      for (int i = 0; i < 9; i++)
        M[i] *= sc;
    }

    int cnt = 0;
    for (int i = 0; i < n; i++) {
      float4 mm = matches[i];
      float e;
      if (u_model == 0)
        e = error_H(M, mm.x, mm.y, mm.z, mm.w);
      else
        e = error_F(M, mm.x, mm.y, mm.z, mm.w);
      if (e < u_thresh)
        cnt++;
    }

    const int base = tid * 10;
    for (int i = 0; i < 9; i++)
      results[base + i] = M[i];
    results[base + 9] = (float)cnt;
  }
}

__launch_bounds__(256, 1)
__global__ void find_best_ransac_batch_kernel(const float* __restrict__ all_res,
                                              const int* __restrict__ res_off, int num_iter,
                                              int* __restrict__ out_best_idx, int P) {
  const int pair = blockIdx.x;
  if (pair >= P)
    return;
  const float* results = all_res + res_off[pair];
  __shared__ float s_cnt[256];
  __shared__ int s_idx[256];
  const int tid = threadIdx.x;
  const int nt = blockDim.x;
  float best_c = -1.f;
  int best_i = 0;
  for (int i = tid; i < num_iter; i += nt) {
    float c = results[i * 10 + 9];
    if (c > best_c) {
      best_c = c;
      best_i = i;
    }
  }
  s_cnt[tid] = best_c;
  s_idx[tid] = best_i;
  __syncthreads();
  for (unsigned int s = nt / 2; s > 0; s >>= 1) {
    if (tid < s) {
      if (s_cnt[tid + s] > s_cnt[tid]) {
        s_cnt[tid] = s_cnt[tid + s];
        s_idx[tid] = s_idx[tid + s];
      }
    }
    __syncthreads();
  }
  if (tid == 0)
    out_best_idx[pair] = s_idx[0];
}

// ─────────────────────────────────────────────────────────────────────────────
// Host dispatch
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// Persistent-buffer management helpers
// ─────────────────────────────────────────────────────────────────────────────

static bool ensure_buffers(int n, int iter, int K) {
  bool need_m = (n    > s_cap_n);
  bool need_r = (iter > s_cap_iter);

  if (need_m) {
    cudaFree(s_d_matches);
    s_d_matches = nullptr;
    s_cap_n = 0;
    size_t sz = static_cast<size_t>(n) * sizeof(float4);
    if (cudaMalloc(reinterpret_cast<void**>(&s_d_matches), sz) != cudaSuccess)
      return false;
    s_cap_n = n;
  }

  if (need_r || need_m) {
    // Indices and results: always allocate idx for max K=8 so F can follow H without OOB.
    cudaFree(s_d_idx);
    cudaFree(s_d_res);
    s_d_idx = nullptr;
    s_d_res = nullptr;
    s_cap_iter = 0;
    size_t sz_i = static_cast<size_t>(iter) * static_cast<size_t>(kIdxMaxK) * sizeof(int);
    size_t sz_r = static_cast<size_t>(iter) * 10u * sizeof(float);
    if (cudaMalloc(reinterpret_cast<void**>(&s_d_idx), sz_i) != cudaSuccess)
      return false;
    if (cudaMalloc(reinterpret_cast<void**>(&s_d_res), sz_r) != cudaSuccess) {
      cudaFree(s_d_idx);
      s_d_idx = nullptr;
      return false;
    }
    s_cap_iter = iter;
  }

  // Pinned host mirrors (grow-only) for fast async H2D.
  if (n > s_h_cap_n || iter > s_h_cap_iter) {
    cudaFreeHost(s_h_matches_pin);
    cudaFreeHost(s_h_idx_pin);
    s_h_matches_pin = nullptr;
    s_h_idx_pin = nullptr;
    s_h_cap_n = n;
    s_h_cap_iter = iter;
    size_t pm = static_cast<size_t>(n) * sizeof(float4);
    size_t pi = static_cast<size_t>(iter) * static_cast<size_t>(kIdxMaxK) * sizeof(int);
    if (cudaHostAlloc(reinterpret_cast<void**>(&s_h_matches_pin), pm, cudaHostAllocDefault) !=
            cudaSuccess ||
        cudaHostAlloc(reinterpret_cast<void**>(&s_h_idx_pin), pi, cudaHostAllocDefault) !=
            cudaSuccess) {
      cudaFreeHost(s_h_matches_pin);
      s_h_matches_pin = nullptr;
      cudaFreeHost(s_h_idx_pin);
      s_h_idx_pin = nullptr;
      s_h_cap_n = s_h_cap_iter = 0;
      return false;
    }
  }
  return true;
}

static void free_batch_db(void) {
  cudaFree(s_db_matches);
  s_db_matches = nullptr;
  cudaFree(s_db_res);
  s_db_res = nullptr;
  cudaFree(s_db_match_off);
  s_db_match_off = nullptr;
  cudaFree(s_db_res_off);
  s_db_res_off = nullptr;
  cudaFree(s_db_pair_n);
  s_db_pair_n = nullptr;
  cudaFree(s_db_thresh);
  s_db_thresh = nullptr;
  cudaFree(s_db_best);
  s_db_best = nullptr;
  s_db_cap_nfloat4 = s_db_cap_res = 0;
  s_db_aux_ok = false;
}

static bool ensure_db_aux_fixed(void) {
  if (s_db_aux_ok)
    return true;
  const int Pm = INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS;
  if (cudaMalloc(reinterpret_cast<void**>(&s_db_match_off), static_cast<size_t>(Pm + 1) * sizeof(int)) !=
          cudaSuccess ||
      cudaMalloc(reinterpret_cast<void**>(&s_db_res_off), static_cast<size_t>(Pm + 1) * sizeof(int)) !=
          cudaSuccess ||
      cudaMalloc(reinterpret_cast<void**>(&s_db_pair_n), static_cast<size_t>(Pm) * sizeof(int)) !=
          cudaSuccess ||
      cudaMalloc(reinterpret_cast<void**>(&s_db_thresh), static_cast<size_t>(Pm) * sizeof(float)) !=
          cudaSuccess ||
      cudaMalloc(reinterpret_cast<void**>(&s_db_best), static_cast<size_t>(Pm) * sizeof(int)) !=
          cudaSuccess) {
    free_batch_db();
    return false;
  }
  s_db_aux_ok = true;
  return true;
}

static bool ensure_db_grow(size_t sum_n_float4, size_t total_res_floats) {
  if (sum_n_float4 > s_db_cap_nfloat4) {
    cudaFree(s_db_matches);
    s_db_matches = nullptr;
    if (sum_n_float4 == 0 ||
        cudaMalloc(reinterpret_cast<void**>(&s_db_matches), sum_n_float4 * sizeof(float4)) !=
            cudaSuccess)
      return false;
    s_db_cap_nfloat4 = sum_n_float4;
  }
  if (total_res_floats > s_db_cap_res) {
    cudaFree(s_db_res);
    s_db_res = nullptr;
    if (total_res_floats == 0 ||
        cudaMalloc(reinterpret_cast<void**>(&s_db_res), total_res_floats * sizeof(float)) !=
            cudaSuccess)
      return false;
    s_db_cap_res = total_res_floats;
  }
  return true;
}

struct PairNormT {
  double T1[9], T2[9];
};

static int run_ransac_cuda_batch_int(int model, int K, const CudaGeoBatchItem* items, int P,
                                     float* mat_out, int* inliers_out) {
  if (!s_inited || P <= 0 || P > INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS)
    return -1;
  for (int b = 0; b < P; ++b) {
    if (items[b].n < K || !items[b].matches)
      return -1;
  }

  if (!ensure_db_aux_fixed())
    return -1;

  std::vector<PairNormT> norms(static_cast<size_t>(P));
  std::vector<int> h_match_off(static_cast<size_t>(P + 1));
  std::vector<int> h_res_off(static_cast<size_t>(P + 1));
  std::vector<int> h_pair_n(static_cast<size_t>(P));
  std::vector<float> h_thresh(static_cast<size_t>(P));

  h_match_off[0] = 0;
  h_res_off[0] = 0;
  int sum_n = 0;
  for (int b = 0; b < P; ++b) {
    h_pair_n[static_cast<size_t>(b)] = items[b].n;
    sum_n += items[b].n;
    h_match_off[static_cast<size_t>(b + 1)] = sum_n;
    h_res_off[static_cast<size_t>(b + 1)] =
        h_res_off[static_cast<size_t>(b)] + s_num_iter * 10;
  }

  const size_t total_res = static_cast<size_t>(h_res_off[static_cast<size_t>(P)]);

  if (!ensure_db_grow(static_cast<size_t>(sum_n), total_res))
    return -1;

  std::vector<float4> h_pack_m(static_cast<size_t>(sum_n));

  for (int b = 0; b < P; ++b) {
    PairNormT& pn = norms[static_cast<size_t>(b)];
    std::vector<Match2D> norm_pts =
        normalize_matches_h(items[b].matches, items[b].n, pn.T1, pn.T2);
    const double s1 = pn.T1[0], s2 = pn.T2[0];
    h_thresh[static_cast<size_t>(b)] =
        (model == 0)
            ? (float)((double)items[b].thresh_sq * s2 * s2)
            : (float)((double)items[b].thresh_sq * s1 * s2);

    const int mo = h_match_off[static_cast<size_t>(b)];
    for (int i = 0; i < items[b].n; ++i) {
      const Match2D& m = norm_pts[static_cast<size_t>(i)];
      h_pack_m[static_cast<size_t>(mo + i)] = make_float4(m.x1, m.y1, m.x2, m.y2);
    }

  }

  cudaMemcpyAsync(s_db_matches, h_pack_m.data(), h_pack_m.size() * sizeof(float4),
                  cudaMemcpyHostToDevice, s_stream);
  cudaMemcpyAsync(s_db_match_off, h_match_off.data(), static_cast<size_t>(P + 1) * sizeof(int),
                  cudaMemcpyHostToDevice, s_stream);
  cudaMemcpyAsync(s_db_res_off, h_res_off.data(), static_cast<size_t>(P + 1) * sizeof(int),
                  cudaMemcpyHostToDevice, s_stream);
  cudaMemcpyAsync(s_db_pair_n, h_pair_n.data(), static_cast<size_t>(P) * sizeof(int),
                  cudaMemcpyHostToDevice, s_stream);
  cudaMemcpyAsync(s_db_thresh, h_thresh.data(), static_cast<size_t>(P) * sizeof(float),
                  cudaMemcpyHostToDevice, s_stream);

  geo_ransac_batch_kernel<<<P, 128, 0, s_stream>>>(
      s_db_matches, s_db_match_off, s_db_pair_n, s_db_res, s_db_res_off,
      s_db_thresh, model, s_solver, s_num_iter, K, P, (uint32_t)rand());
  find_best_ransac_batch_kernel<<<P, 256, 0, s_stream>>>(s_db_res, s_db_res_off, s_num_iter,
                                                           s_db_best, P);

  if (cudaStreamSynchronize(s_stream) != cudaSuccess)
    return -1;

  std::vector<int> h_best(static_cast<size_t>(P));
  if (cudaMemcpy(h_best.data(), s_db_best, static_cast<size_t>(P) * sizeof(int),
                 cudaMemcpyDeviceToHost) != cudaSuccess)
    return -1;

  for (int b = 0; b < P; ++b) {
    float row[10];
    const int bi = h_best[static_cast<size_t>(b)];
    const size_t off = static_cast<size_t>(h_res_off[static_cast<size_t>(b)]) +
                       static_cast<size_t>(bi) * 10u;
    if (cudaMemcpy(row, s_db_res + off, sizeof(row), cudaMemcpyDeviceToHost) != cudaSuccess)
      return -1;
    float norm_mat[9];
    for (int i = 0; i < 9; ++i)
      norm_mat[i] = row[static_cast<size_t>(i)];
    inliers_out[b] = (int)row[9];
    PairNormT& pn = norms[static_cast<size_t>(b)];
    if (model == 0)
      denorm_H_h(pn.T1, pn.T2, norm_mat, mat_out + b * 9);
    else
      denorm_FE_h(pn.T1, pn.T2, norm_mat, mat_out + b * 9);
  }
  return 0;
}

static int run_ransac_cuda(int model, int K, const Match2D* matches_raw, int n, float mat[9],
                           float thresh) {
  if (n < K)
    return -1;

  double T1[9], T2[9];
  std::vector<Match2D> norm_pts = normalize_matches_h(matches_raw, n, T1, T2);

  double s1 = T1[0], s2 = T2[0];
  float thresh_norm =
      (model == 0) ? (float)((double)thresh * s2 * s2) : (float)((double)thresh * s1 * s2);

  // ── Reuse persistent device + pinned host buffers ─────────────────────────
  if (!ensure_buffers(n, s_num_iter, K)) {
    fprintf(stderr, "[cuda_geo] cudaMalloc failed (n=%d iter=%d K=%d)\n", n, s_num_iter, K);
    return -1;
  }

  for (int i = 0; i < n; i++) {
    const Match2D& m = norm_pts[static_cast<size_t>(i)];
    s_h_matches_pin[static_cast<size_t>(i)] = make_float4(m.x1, m.y1, m.x2, m.y2);
  }

  // Build RANSAC sample index table with deduplication within each K-sample.
  {
    std::vector<int> pool(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) pool[static_cast<size_t>(i)] = i;
    for (int it = 0; it < s_num_iter; it++) {
      int base = it * K;
      for (int k = 0; k < K; k++) {
        int r = k + rand() % (n - k);
        std::swap(pool[static_cast<size_t>(k)], pool[static_cast<size_t>(r)]);
        s_h_idx_pin[static_cast<size_t>(base + k)] = pool[static_cast<size_t>(k)];
      }
    }
  }

  size_t sz_m = static_cast<size_t>(n) * sizeof(float4);
  size_t sz_i = static_cast<size_t>(s_num_iter) * static_cast<size_t>(K) * sizeof(int);

  cudaMemcpyAsync(s_d_matches, s_h_matches_pin, sz_m, cudaMemcpyHostToDevice, s_stream);
  cudaMemcpyAsync(s_d_idx, s_h_idx_pin, sz_i, cudaMemcpyHostToDevice, s_stream);

  static constexpr int kCudaBlock = 128;
  int blocks = (s_num_iter + kCudaBlock - 1) / kCudaBlock;
  if (s_verbose) {
    fprintf(stderr, "[cuda_geo] dispatch model=%d n=%d iter=%d blocks=%d×%d (async stream)\n",
            model, n, s_num_iter, blocks, kCudaBlock);
  }
  geo_ransac_kernel<<<blocks, kCudaBlock, 0, s_stream>>>(s_d_matches, s_d_idx, s_d_res, model,
                                                          thresh_norm, n, s_solver, s_num_iter, K);
  find_best_ransac_iter_kernel<<<1, 256, 0, s_stream>>>(s_d_res, s_num_iter, s_d_best_idx);

  cudaError_t st = cudaStreamSynchronize(s_stream);
  if (st != cudaSuccess) {
    fprintf(stderr, "[cuda_geo] stream sync error: %s\n", cudaGetErrorString(st));
    return -1;
  }

  int h_best = 0;
  if (cudaMemcpy(&h_best, s_d_best_idx, sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess)
    return -1;

  float h_best_row[10];
  if (cudaMemcpy(h_best_row, s_d_res + static_cast<size_t>(h_best) * 10u, sizeof(h_best_row),
                 cudaMemcpyDeviceToHost) != cudaSuccess)
    return -1;

  float norm_mat[9];
  for (int i = 0; i < 9; i++)
    norm_mat[i] = h_best_row[static_cast<size_t>(i)];
  int inliers = (int)h_best_row[9];

  if (model == 0)
    denorm_H_h(T1, T2, norm_mat, mat);
  else
    denorm_FE_h(T1, T2, norm_mat, mat);

  return inliers;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

void cuda_geo_set_device(int device_id) { s_cuda_device = device_id; }

int cuda_geo_init(const GeoRansacConfig* cfg) {
  if (cudaSetDevice(s_cuda_device) != cudaSuccess)
    return -1;
  if (cfg) {
    if (cfg->num_iterations > 0)
      s_num_iter = cfg->num_iterations;
    if (cfg->local_size_x > 0)
      (void)cfg->local_size_x;
  }

  if (cudaStreamCreate(&s_stream) != cudaSuccess) {
    fprintf(stderr, "[cuda_geo] cudaStreamCreate failed\n");
    return -1;
  }
  if (cudaMalloc(reinterpret_cast<void**>(&s_d_best_idx), sizeof(int)) != cudaSuccess) {
    cudaStreamDestroy(s_stream);
    s_stream = nullptr;
    fprintf(stderr, "[cuda_geo] cudaMalloc(s_d_best_idx) failed\n");
    return -1;
  }

  // Pre-allocate persistent device buffers with sensible defaults.
  // K=8 (8-point F/E) is the largest sample size; matches capped at 4096
  // (covers almost all real pair sizes; grow automatically on demand).
  static constexpr int kInitN    = 4096;
  static constexpr int kInitK    = 8;
  if (!ensure_buffers(kInitN, s_num_iter, kInitK)) {
    cudaFree(s_d_best_idx);
    s_d_best_idx = nullptr;
    cudaStreamDestroy(s_stream);
    s_stream = nullptr;
    fprintf(stderr, "[cuda_geo] Failed to pre-allocate device buffers\n");
    return -1;
  }

  // Warm up the CUDA context so the first real kernel launch is not slow.
  geo_ransac_kernel<<<1, 1, 0, s_stream>>>(s_d_matches, s_d_idx, s_d_res, 0, 1.f, 0, 1, 1, 4);
  find_best_ransac_iter_kernel<<<1, 256, 0, s_stream>>>(s_d_res, 1, s_d_best_idx);
  cudaStreamSynchronize(s_stream);

  s_inited = true;
  fprintf(stderr, "[cuda_geo] Initialised OK – %d iterations / async stream / pinned H2D\n",
          s_num_iter);
  return 0;
}

void cuda_geo_shutdown(void) {
  free_batch_db();
  cudaFree(s_d_matches);
  s_d_matches = nullptr;
  cudaFree(s_d_idx);
  s_d_idx = nullptr;
  cudaFree(s_d_res);
  s_d_res = nullptr;
  cudaFree(s_d_best_idx);
  s_d_best_idx = nullptr;
  cudaFreeHost(s_h_matches_pin);
  s_h_matches_pin = nullptr;
  cudaFreeHost(s_h_idx_pin);
  s_h_idx_pin = nullptr;
  s_h_cap_n = s_h_cap_iter = 0;
  s_cap_n = 0;
  s_cap_iter = 0;
  if (s_stream) {
    cudaStreamDestroy(s_stream);
    s_stream = nullptr;
  }
  s_inited = false;
}

void cuda_geo_set_verbose(int v) { s_verbose = v; }

void cuda_geo_set_solver(int s) { s_solver = s; }

int cuda_ransac_H(const Match2D* m, int n, float mat[9], float thresh) {
  if (!s_inited)
    return -1;
  return run_ransac_cuda(0, 4, m, n, mat, thresh);
}

int cuda_ransac_F(const Match2D* m, int n, float mat[9], float thresh) {
  if (!s_inited)
    return -1;
  return run_ransac_cuda(1, 8, m, n, mat, thresh);
}

int cuda_ransac_E(const Match2D* m, int n, float mat[9], float thresh) {
  if (!s_inited)
    return -1;
  return run_ransac_cuda(2, 8, m, n, mat, thresh);
}

int cuda_ransac_F_batch(const CudaGeoBatchItem* items, int P, float* mat_out, int* inliers_out) {
  if (!s_inited)
    return -1;
  return run_ransac_cuda_batch_int(1, 8, items, P, mat_out, inliers_out);
}

int cuda_ransac_E_batch(const CudaGeoBatchItem* items, int P, float* mat_out, int* inliers_out) {
  if (!s_inited)
    return -1;
  return run_ransac_cuda_batch_int(2, 8, items, P, mat_out, inliers_out);
}

int cuda_ransac_H_batch(const CudaGeoBatchItem* items, int P, float* mat_out, int* inliers_out) {
  if (!s_inited)
    return -1;
  return run_ransac_cuda_batch_int(0, 4, items, P, mat_out, inliers_out);
}

// ─────────────────────────────────────────────────────────────────────────────
// E decomposition + triangulation: full GPU pipeline
// ─────────────────────────────────────────────────────────────────────────────

// Decompose E matrix → best (R, t) via cheirality test. One block per pair, 32 threads/block.
// Thread 0 computes SVD, builds 4 (R,t) candidates; all 32 threads vote on positive-depth count.
//
// 4 candidates:
//   c=0: R = U*W*Vt,   t = +U[:,2]
//   c=1: R = U*W*Vt,   t = -U[:,2]
//   c=2: R = U*Wt*Vt,  t = +U[:,2]
//   c=3: R = U*Wt*Vt,  t = -U[:,2]
// where W = [[0,-1,0],[1,0,0],[0,0,1]].
__launch_bounds__(32, 8)
__global__ void decompose_E_batch_kernel(const float* __restrict__ E_mats,
                                          const float4* __restrict__ all_pts,
                                          const int* __restrict__ pair_off,
                                          const int* __restrict__ pair_n,
                                          const uint8_t* __restrict__ masks,
                                          float* __restrict__ R_out, float* __restrict__ t_out,
                                          int* __restrict__ cheir_out, int B) {
  const int b = blockIdx.x;
  if (b >= B)
    return;
  const int tid = threadIdx.x; // 0..31

  __shared__ float sm_R[2][9]; // sm_R[0]=R1=U*W*Vt, sm_R[1]=R3=U*Wt*Vt
  __shared__ float sm_t[3];   // +U[:,2]; negated per candidate
  __shared__ int sm_cnt[4];   // positive-depth counter for each candidate

  if (tid < 4)
    sm_cnt[tid] = 0;
  __syncthreads();

  // Thread 0: SVD of E → build all 4 rotation candidates
  if (tid == 0) {
    float E[9], U[9], S[3], Vt[9];
    for (int i = 0; i < 9; i++)
      E[i] = E_mats[b * 9 + i];
    svd3x3(E, U, S, Vt);

    // t = U[:,2]  (col 2 of U: U[2], U[5], U[8])
    sm_t[0] = U[2];
    sm_t[1] = U[5];
    sm_t[2] = U[8];

    // W  = [[0,-1,0],[1,0,0],[0,0,1]]   → (UW)[i,k]: k=0→U[i,1], k=1→-U[i,0], k=2→U[i,2]
    // Wt = [[0,1,0],[-1,0,0],[0,0,1]]  → (UWt)[i,k]: k=0→-U[i,1], k=1→U[i,0], k=2→U[i,2]
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        // R1 = U*W*Vt
        sm_R[0][i * 3 + j] =
            U[i * 3 + 1] * Vt[0 * 3 + j] - U[i * 3 + 0] * Vt[1 * 3 + j] + U[i * 3 + 2] * Vt[2 * 3 + j];
        // R3 = U*Wt*Vt
        sm_R[1][i * 3 + j] =
            -U[i * 3 + 1] * Vt[0 * 3 + j] + U[i * 3 + 0] * Vt[1 * 3 + j] + U[i * 3 + 2] * Vt[2 * 3 + j];
      }
    }
  }
  __syncthreads();

  // Threads 0-7 → c=0, 8-15 → c=1, 16-23 → c=2, 24-31 → c=3
  const int c = tid / 8;
  const int local_t = tid % 8;
  const float tsign = (c == 1 || c == 3) ? -1.0f : 1.0f;
  const float tx = tsign * sm_t[0];
  const float ty = tsign * sm_t[1];
  const float tz = tsign * sm_t[2];
  const float* R = sm_R[c / 2]; // R1 for c=0,1; R3 for c=2,3

  const int off = pair_off[b];
  const int n = pair_n[b];
  const int max_sample = min(n, 64); // sample up to 64 points for cheirality

  int local_cnt = 0;
  for (int k = local_t; k < max_sample; k += 8) {
    if (!masks[off + k])
      continue; // skip non-inliers
    float4 pt = all_pts[off + k];
    float x1 = pt.x, y1 = pt.y, x2 = pt.z, y2 = pt.w;

    // Closed-form triangulation for P1=[I|0], P2=[R|t]:
    //   d2  = R[2,0]*x1 + R[2,1]*y1 + R[2,2]
    //   a   = x2*d2 - (R[0,0]*x1 + R[0,1]*y1 + R[0,2])
    //   b   = y2*d2 - (R[1,0]*x1 + R[1,1]*y1 + R[1,2])
    //   Z   = (a*(tx - x2*tz) + b*(ty - y2*tz)) / (a*a + b*b)
    //   Z2  = Z*d2 + tz      (depth in cam2)
    //   valid: Z > 0 && Z2 > 0
    float d2 = R[6] * x1 + R[7] * y1 + R[8];
    float a = x2 * d2 - (R[0] * x1 + R[1] * y1 + R[2]);
    float bv = y2 * d2 - (R[3] * x1 + R[4] * y1 + R[5]);
    float denom = a * a + bv * bv;
    if (denom < 1e-10f)
      continue;
    float Z = (a * (tx - x2 * tz) + bv * (ty - y2 * tz)) / denom;
    float Z2 = Z * d2 + tz;
    if (Z > 0.0f && Z2 > 0.0f)
      local_cnt++;
  }
  atomicAdd(&sm_cnt[c], local_cnt);
  __syncthreads();

  if (tid == 0) {
    // Pick candidate with most positive-depth inliers
    int best_c = 0, best_n = sm_cnt[0];
    for (int ci = 1; ci < 4; ci++) {
      if (sm_cnt[ci] > best_n) {
        best_n = sm_cnt[ci];
        best_c = ci;
      }
    }
    const float best_sign = (best_c == 1 || best_c == 3) ? -1.0f : 1.0f;
    float* Rb = R_out + b * 9;
    float* tb = t_out + b * 3;
    const float* Rm = sm_R[best_c / 2];
    for (int i = 0; i < 9; i++)
      Rb[i] = Rm[i];
    tb[0] = best_sign * sm_t[0];
    tb[1] = best_sign * sm_t[1];
    tb[2] = best_sign * sm_t[2];
    cheir_out[b] = best_n;
  }
}

// Triangulate all inlier points for B pairs.
// Launch: <<<B, 256>>>. Each block strides over its pair's matches.
// Non-inlier or degenerate points are written as (NaN, NaN, NaN).
__launch_bounds__(256, 2)
__global__ void triangulate_batch_kernel(const float* __restrict__ R_all,
                                          const float* __restrict__ t_all,
                                          const float4* __restrict__ all_pts,
                                          const int* __restrict__ pair_off,
                                          const uint8_t* __restrict__ masks,
                                          float* __restrict__ X_out,
                                          int* __restrict__ valid_out, int B) {
  const int b = blockIdx.x;
  if (b >= B)
    return;
  const int tid = threadIdx.x;

  const float* R = R_all + b * 9;
  const float* t = t_all + b * 3;
  const int off = pair_off[b];
  const int n = pair_off[b + 1] - pair_off[b];

  __shared__ int sm_valid;
  if (tid == 0)
    sm_valid = 0;
  __syncthreads();

  static constexpr float kNaN = __builtin_nanf("0");
  int local_valid = 0;

  for (int i = tid; i < n; i += blockDim.x) {
    const int gi = off + i;
    float* Xp = X_out + gi * 3;
    if (!masks[gi]) {
      Xp[0] = Xp[1] = Xp[2] = kNaN;
      continue;
    }
    float4 pt = all_pts[gi];
    float x1 = pt.x, y1 = pt.y, x2 = pt.z, y2 = pt.w;

    float d2 = R[6] * x1 + R[7] * y1 + R[8];
    float a = x2 * d2 - (R[0] * x1 + R[1] * y1 + R[2]);
    float bv = y2 * d2 - (R[3] * x1 + R[4] * y1 + R[5]);
    float denom = a * a + bv * bv;
    if (denom < 1e-10f || !isfinite(denom)) {
      Xp[0] = Xp[1] = Xp[2] = kNaN;
      continue;
    }
    float Z = (a * (t[0] - x2 * t[2]) + bv * (t[1] - y2 * t[2])) / denom;
    float Z2 = Z * d2 + t[2];
    if (Z > 0.0f && Z2 > 0.0f) {
      Xp[0] = x1 * Z;
      Xp[1] = y1 * Z;
      Xp[2] = Z;
      local_valid++;
    } else {
      Xp[0] = Xp[1] = Xp[2] = kNaN;
    }
  }

  atomicAdd(&sm_valid, local_valid);
  __syncthreads();
  if (tid == 0)
    valid_out[b] = sm_valid;
}

// ─────────────────────────────────────────────────────────────────────────────
// Host APIs: E decomposition + triangulation batch
// ─────────────────────────────────────────────────────────────────────────────

// Decompose B Essential matrices and select the best (R, t) via cheirality test.
//
// pts_norm:      host float array, sum_n × 4 floats (x1,y1,x2,y2) K-normalised
// pair_off:      host int[B+1], prefix sums of pair sizes (pair_off[B] == sum_n)
// pair_n:        host int[B], number of matches per pair
// inlier_masks:  host uint8[sum_n], 1=E-inlier for the corresponding pair
// R_out:         host float[B*9], row-major rotation matrix per pair
// t_out:         host float[B*3], translation vector per pair
// cheir_out:     host int[B], positive-depth inlier count for the winning candidate
//
// Returns 0 on success, -1 on error.
int cuda_decompose_E_batch(const float* E_mats, int B, const float* pts_norm,
                           const int* pair_off, const int* pair_n, const uint8_t* inlier_masks,
                           float* R_out, float* t_out, int* cheir_out) {
  if (!s_inited || B <= 0)
    return -1;
  const int sum_n = pair_off[B];

  float* d_E = nullptr;
  float* d_R = nullptr;
  float* d_t = nullptr;
  float4* d_pts = nullptr;
  int* d_off = nullptr;
  int* d_pn = nullptr;
  int* d_cheir = nullptr;
  uint8_t* d_masks = nullptr;

  bool ok = (cudaMalloc(&d_E, (size_t)B * 9 * sizeof(float)) == cudaSuccess) &&
            (cudaMalloc(&d_R, (size_t)B * 9 * sizeof(float)) == cudaSuccess) &&
            (cudaMalloc(&d_t, (size_t)B * 3 * sizeof(float)) == cudaSuccess) &&
            (cudaMalloc((void**)&d_pts, (size_t)sum_n * sizeof(float4)) == cudaSuccess) &&
            (cudaMalloc(&d_off, (size_t)(B + 1) * sizeof(int)) == cudaSuccess) &&
            (cudaMalloc(&d_pn, (size_t)B * sizeof(int)) == cudaSuccess) &&
            (cudaMalloc(&d_cheir, (size_t)B * sizeof(int)) == cudaSuccess) &&
            (cudaMalloc(&d_masks, (size_t)sum_n * sizeof(uint8_t)) == cudaSuccess);

  if (ok) {
    cudaMemcpyAsync(d_E, E_mats, (size_t)B * 9 * sizeof(float), cudaMemcpyHostToDevice, s_stream);
    cudaMemcpyAsync(d_pts, pts_norm, (size_t)sum_n * sizeof(float4), cudaMemcpyHostToDevice,
                   s_stream);
    cudaMemcpyAsync(d_off, pair_off, (size_t)(B + 1) * sizeof(int), cudaMemcpyHostToDevice,
                   s_stream);
    cudaMemcpyAsync(d_pn, pair_n, (size_t)B * sizeof(int), cudaMemcpyHostToDevice, s_stream);
    cudaMemcpyAsync(d_masks, inlier_masks, (size_t)sum_n * sizeof(uint8_t), cudaMemcpyHostToDevice,
                   s_stream);

    decompose_E_batch_kernel<<<B, 32, 0, s_stream>>>(d_E, d_pts, d_off, d_pn, d_masks, d_R, d_t,
                                                      d_cheir, B);

    cudaMemcpyAsync(R_out, d_R, (size_t)B * 9 * sizeof(float), cudaMemcpyDeviceToHost, s_stream);
    cudaMemcpyAsync(t_out, d_t, (size_t)B * 3 * sizeof(float), cudaMemcpyDeviceToHost, s_stream);
    cudaMemcpyAsync(cheir_out, d_cheir, (size_t)B * sizeof(int), cudaMemcpyDeviceToHost, s_stream);

    ok = (cudaStreamSynchronize(s_stream) == cudaSuccess);
  }

  cudaFree(d_E);
  cudaFree(d_R);
  cudaFree(d_t);
  cudaFree(d_pts);
  cudaFree(d_off);
  cudaFree(d_pn);
  cudaFree(d_cheir);
  cudaFree(d_masks);
  return ok ? 0 : -1;
}

// Triangulate all E-inlier correspondences for B pairs.
//
// R, t:          host float[B*9], float[B*3] — relative pose per pair (from cuda_decompose_E_batch)
// pts_norm:      host float[sum_n * 4] — same layout as cuda_decompose_E_batch
// pair_off:      host int[B+1], prefix sums
// inlier_masks:  host uint8[sum_n], 1=E-inlier to triangulate
// X_out:         host float[sum_n * 3], (NaN,NaN,NaN) for non-inliers / negative-depth
// valid_out:     host int[B], number of valid 3D points per pair
//
// Returns 0 on success, -1 on error.
int cuda_triangulate_batch(const float* R, const float* t, int B, const float* pts_norm,
                           const int* pair_off, const int* pair_n, const uint8_t* inlier_masks,
                           float* X_out, int* valid_out) {
  if (!s_inited || B <= 0)
    return -1;
  const int sum_n = pair_off[B];
  (void)pair_n; // unused; sizes derived from pair_off

  float* d_R = nullptr;
  float* d_t = nullptr;
  float4* d_pts = nullptr;
  int* d_off = nullptr;
  uint8_t* d_masks = nullptr;
  float* d_X = nullptr;
  int* d_valid = nullptr;

  bool ok = (cudaMalloc(&d_R, (size_t)B * 9 * sizeof(float)) == cudaSuccess) &&
            (cudaMalloc(&d_t, (size_t)B * 3 * sizeof(float)) == cudaSuccess) &&
            (cudaMalloc((void**)&d_pts, (size_t)sum_n * sizeof(float4)) == cudaSuccess) &&
            (cudaMalloc(&d_off, (size_t)(B + 1) * sizeof(int)) == cudaSuccess) &&
            (cudaMalloc(&d_masks, (size_t)sum_n * sizeof(uint8_t)) == cudaSuccess) &&
            (cudaMalloc(&d_X, (size_t)sum_n * 3 * sizeof(float)) == cudaSuccess) &&
            (cudaMalloc(&d_valid, (size_t)B * sizeof(int)) == cudaSuccess);

  if (ok) {
    cudaMemcpyAsync(d_R, R, (size_t)B * 9 * sizeof(float), cudaMemcpyHostToDevice, s_stream);
    cudaMemcpyAsync(d_t, t, (size_t)B * 3 * sizeof(float), cudaMemcpyHostToDevice, s_stream);
    cudaMemcpyAsync(d_pts, pts_norm, (size_t)sum_n * sizeof(float4), cudaMemcpyHostToDevice,
                   s_stream);
    cudaMemcpyAsync(d_off, pair_off, (size_t)(B + 1) * sizeof(int), cudaMemcpyHostToDevice,
                   s_stream);
    cudaMemcpyAsync(d_masks, inlier_masks, (size_t)sum_n * sizeof(uint8_t), cudaMemcpyHostToDevice,
                   s_stream);

    triangulate_batch_kernel<<<B, 256, 0, s_stream>>>(d_R, d_t, d_pts, d_off, d_masks, d_X,
                                                       d_valid, B);

    cudaMemcpyAsync(X_out, d_X, (size_t)sum_n * 3 * sizeof(float), cudaMemcpyDeviceToHost,
                   s_stream);
    cudaMemcpyAsync(valid_out, d_valid, (size_t)B * sizeof(int), cudaMemcpyDeviceToHost, s_stream);

    ok = (cudaStreamSynchronize(s_stream) == cudaSuccess);
  }

  cudaFree(d_R);
  cudaFree(d_t);
  cudaFree(d_pts);
  cudaFree(d_off);
  cudaFree(d_masks);
  cudaFree(d_X);
  cudaFree(d_valid);
  return ok ? 0 : -1;
}
