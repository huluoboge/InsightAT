/**
 * @file  gpu_twoview_sfm_cuda.h
 * @brief GPU-accelerated two-view SfM primitives via CUDA (DLT triangulation +
 *        reprojection residuals). Replaces OpenGL-based gpu_twoview_sfm.
 *
 * Batch Architecture
 * ──────────────────
 *
 * gpu_triangulate_batch():
 *   One CUDA kernel processes 1000+ (R, t, correspondences) sets in parallel.
 *   - Thread i processes point i of correspondence set k
 *   - Warp computes DLT matrix for one correspondence (4×4 matrix)
 *   - Shared memory accelerates per-warp Cholesky + inverse power iteration
 *   - Output: [X, Y, Z, valid_flag] per point, NaN for invalid
 *
 * Data Layout:
 *   - Correspondences: sorted by pair_id, then by point index
 *   - pair_offsets[i]: start index in points/results for pair i
 *   - pair_Rt[i]: [R[9], t[3]] for pair i
 *   - Per-pair: variable number of correspondences (up to max_per_pair)
 *
 * Threading
 * ─────────
 *   CPU-GPU asynchronous via CUDA streams.
 *   Multiple streams allow overlapping computation + CPU preparation of next batch.
 */

#pragma once
#ifndef GPU_TWOVIEW_SFM_CUDA_H
#define GPU_TWOVIEW_SFM_CUDA_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize CUDA device (choose best available GPU).
 * @return 0 on success, negative on failure.
 */
int gpu_twoview_cuda_init(void);

/** Release all GPU resources. */
void gpu_twoview_cuda_shutdown(void);

/* ── Batch Triangulation ────────────────────────────────────────────────── */

/**
 * Triangulate multiple (R, t) pairs with variable number of correspondences per pair.
 * One GPU kernel launch processes the entire batch.
 *
 * @param pts_n_flat   Flattened normalised correspondences [x1n, y1n, x2n, y2n, ...],
 *                     row-major, sorted by pair_id.
 * @param pair_offsets Array[num_pairs+1]: offsets into pts_n_flat
 *                     pair_offsets[i*2] = start index for pair i
 *                     pair_offsets[i*2+1] = end index (exclusive)
 * @param num_pairs    Number of (R,t) pairs.
 * @param pair_Rt      Array[num_pairs*12]: [R[9], t[3], R[9], t[3], ...]
 * @param X_out        Output 3D points [X, Y, Z, valid_flag(float)],
 *                     same flattened layout as pts_n_flat.
 *                     Size: 4 * total_correspondences * sizeof(float).
 * @return             Total number of valid (finite, positive-depth) points.
 */
int gpu_triangulate_batch(const float* pts_n_flat, const int* pair_offsets, int num_pairs,
                          const float* pair_Rt, float* X_out);

/* ── Batch BA Residuals ─────────────────────────────────────────────────── */

/**
 * Compute Huber-weighted reprojection residuals for multiple pairs + 3D points.
 *
 * @param pts_px_flat    Pixel observations [u1,v1,u2,v2, ...], flattened.
 * @param X_flat         3D points [X, Y, Z, ...], flattened, same structure as X_out
 *                       from gpu_triangulate_batch().
 * @param pair_offsets   Same as gpu_triangulate_batch().
 * @param num_pairs      Number of (R,t) pairs.
 * @param pair_Rt        Array[num_pairs*12]: [R[9], t[3], ...].
 * @param focal_length   Focal length (pixels), assumed same for both cameras.
 * @param cx, cy         Principal point (pixels), assumed same for both cameras.
 * @param huber_k        Huber threshold (pixels).
 * @param residuals_out  Output [r0, r1, r2, r3] per point.
 *                       May be NULL (only stats computed).
 *                       Size: 4 * total_correspondences * sizeof(float).
 * @param wrss_out       Output weighted residual sum-of-squares (divided by valid_count).
 * @param valid_out      Output count of valid (positive-depth, finite) points.
 */
void gpu_ba_residuals_batch(const float* pts_px_flat, const float* X_flat,
                            const int* pair_offsets, int num_pairs, const float* pair_Rt,
                            float focal_length, float cx, float cy, float huber_k,
                            float* residuals_out, float* wrss_out, int* valid_out);

/* ── Async Stream API (optional, for advanced pipelining) ──────────────── */

/**
 * @brief Opaque stream handle for async batch processing.
 */
typedef void* GpuTwoviewStream;

/**
 * Create a new CUDA stream for async batch operations.
 * @return Stream handle, or NULL on failure.
 */
GpuTwoviewStream gpu_twoview_create_stream(void);

/**
 * Destroy a stream (calls cudaStreamDestroy).
 */
void gpu_twoview_destroy_stream(GpuTwoviewStream stream);

/**
 * Async version of gpu_triangulate_batch (enqueued on stream, returns immediately).
 * Call gpu_twoview_stream_synchronize() to wait for completion.
 */
int gpu_triangulate_batch_async(GpuTwoviewStream stream, const float* pts_n_flat,
                                 const int* pair_offsets, int num_pairs,
                                 const float* pair_Rt, float* X_out);

/**
 * Block until all tasks on stream complete.
 * @return 0 on success, negative on CUDA error.
 */
int gpu_twoview_stream_synchronize(GpuTwoviewStream stream);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* GPU_TWOVIEW_SFM_CUDA_H */
