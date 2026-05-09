/**
 * @file cuda_geo_ransac.h
 * @brief CUDA implementation of two-view RANSAC (H / F / E), same numerics and
 *        API contract as gpu_geo_ransac.h (Hartley norm, Sampson / transfer errors).
 *
 * Use when InsightAT is built with CUDAToolkit (INSIGHTAT_HAS_CUDA_GEO).
 * isat_geo --backend gpu prefers this path over OpenGL compute shaders.
 */
#pragma once

#include <cstdint>

#include "gpu_geo_ransac.h"  // Match2D, GeoRansacConfig

#ifdef __cplusplus
extern "C" {
#endif

/** Optional: call before cuda_geo_init (default device 0). */
void cuda_geo_set_device(int device_id);

int cuda_geo_init(const GeoRansacConfig* cfg);
void cuda_geo_shutdown(void);
void cuda_geo_set_verbose(int v);
void cuda_geo_set_solver(int s);  // 0 = Jacobi null-vector, 1 = Cholesky IPI (default)

int cuda_ransac_H(const Match2D* matches, int n, float mat[9], float thresh);
int cuda_ransac_F(const Match2D* matches, int n, float mat[9], float thresh);
int cuda_ransac_E(const Match2D* matches, int n, float mat[9], float thresh);

/** Max image-pairs per one launch (F / E / H batch). Tuned for occupancy vs VRAM.
 *  Increased from 48 to 10000 for maximum GPU occupancy and VRAM utilization.
 *  Memory footprint: ~50-500MB per batch (scales with avg correspondences per pair).
 *  Typical: 10000 pairs × 500 corrs/pair = 5M points = ~40-50MB (with overhead ~100-200MB).
 */
#ifndef INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS
#define INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS 50000
#endif

/** One element of a batch: matches in the same space as the single-pair API (pixels or
 *  K-normalised coords for E). `thresh_sq` is the squared threshold passed to RANSAC for that
 *  item (same convention as cuda_ransac_F / E / H). */
typedef struct {
  const Match2D* matches;
  int n;
  float thresh_sq;
} CudaGeoBatchItem;

/** Batch RANSAC: `P` pairs in one GPU launch. `mat_out` is `P×9` row-major floats; `inliers_out`
 *  has length `P`. Items with `n < K` should not be passed (undefined). Returns 0 on success. */
int cuda_ransac_F_batch(const CudaGeoBatchItem* items, int P, float* mat_out, int* inliers_out);
int cuda_ransac_E_batch(const CudaGeoBatchItem* items, int P, float* mat_out, int* inliers_out);
int cuda_ransac_H_batch(const CudaGeoBatchItem* items, int P, float* mat_out, int* inliers_out);

/** Decompose B Essential matrices and select the best (R, t) via a GPU cheirality test.
 *
 *  pts_norm:      flat float[sum_n * 4] (x1,y1,x2,y2) K-normalised; same layout used
 *                 for cuda_ransac_E_batch inputs.
 *  pair_off:      int[B+1] prefix sums of pair sizes (pair_off[B] == sum_n).
 *  pair_n:        int[B]   number of matches per pair.
 *  inlier_masks:  uint8[sum_n]  1 = E-inlier (used as sample for cheirality voting).
 *  R_out:         float[B*9]   row-major rotation matrix per pair.
 *  t_out:         float[B*3]   translation vector per pair.
 *  cheir_out:     int[B]       positive-depth count for the winning candidate.
 *
 *  Returns 0 on success, -1 on error. Requires cuda_geo_init() to have been called. */
int cuda_decompose_E_batch(const float* E_mats, int B, const float* pts_norm,
                           const int* pair_off, const int* pair_n, const uint8_t* inlier_masks,
                           float* R_out, float* t_out, int* cheir_out);

/** Triangulate all E-inlier correspondences for B pairs on the GPU (linear DLT).
 *
 *  R, t:          float[B*9], float[B*3] — relative pose per pair.
 *  pts_norm:      float[sum_n * 4] — same layout as cuda_decompose_E_batch.
 *  pair_off:      int[B+1] prefix sums.
 *  pair_n:        int[B] (unused internally; pass for documentation clarity).
 *  inlier_masks:  uint8[sum_n]  1 = triangulate this point.
 *  X_out:         float[sum_n * 3] output in cam1 frame; (NaN,NaN,NaN) for skipped/negative-depth.
 *  valid_out:     int[B] number of valid 3D points per pair.
 *
 *  Returns 0 on success, -1 on error. */
int cuda_triangulate_batch(const float* R, const float* t, int B, const float* pts_norm,
                           const int* pair_off, const int* pair_n, const uint8_t* inlier_masks,
                           float* X_out, int* valid_out);

#ifdef __cplusplus
}
#endif
