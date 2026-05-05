/**
 * @file cuda_geo_ransac.h
 * @brief CUDA implementation of two-view RANSAC (H / F / E), same numerics and
 *        API contract as gpu_geo_ransac.h (Hartley norm, Sampson / transfer errors).
 *
 * Use when InsightAT is built with CUDAToolkit (INSIGHTAT_HAS_CUDA_GEO).
 * isat_geo --backend gpu prefers this path over OpenGL compute shaders.
 */
#pragma once

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

/** Max image-pairs per one launch (F / E / H batch). Tuned for occupancy vs VRAM. */
#ifndef INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS
#define INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS 48
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

#ifdef __cplusplus
}
#endif
