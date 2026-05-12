/**
 * @file  cuda_ba_subset.cuh
 * @brief GPU 3-pass BA subset selection: filters tracks for Ceres BA using grid-NMS.
 *
 * Mirrors the CPU `select_ba_subset` algorithm in incremental_sfm_pipeline.cpp:
 *   Pass 1 – per-observation atomic: count registered degree and record first/last
 *            registered image per track; compute sin²(parallax) for degree-2 tracks.
 *   Pass 2 – per-image block: grid-NMS selects one best-scoring track per image cell.
 *   Pass 3 – per-track: write d_track_flags_ba = 1 for tracks NOT selected (skip BA).
 *
 * Result is stored in `CudaSfMState::d_track_flags_ba` (1 = skip from Ceres BA).
 * `GpuSfMScene::pack_for_ba` honours this flag via the optional `d_skip_ba` argument.
 */

#pragma once
#ifndef INSIGHT_CUDA_BA_SUBSET_CUH
#define INSIGHT_CUDA_BA_SUBSET_CUH

#include "cuda_sfm_state.h"

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Options controlling the GPU grid-NMS BA subset selection.
 * Mirrors `GlobalBAOptions::ba_grid_*` fields.
 */
struct BaSubsetOptions {
    /// Target number of selected tracks per image.
    /// Adaptive cell size = ceil(sqrt(W * H / target)).
    int   target_per_image = 3000;

    /// Score weight for track degree: score = w_degree * clamp(deg/degree_cap, 0, 1)
    ///                                       + w_score * sin2(parallax).
    float w_degree         = 0.6f;

    /// Score weight for geometric stability (sin²θ).
    /// sin²θ = 1.0 for degree ≥ 3; computed from first/last registered image for degree-2.
    float w_score          = 0.4f;

    /// Degree cap: clamp(deg / degree_cap, 0, 1) → score saturates at this degree.
    int   degree_cap       = 8;
};

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Run GPU 3-pass BA subset selection, writing results into
 * `state->d_track_flags_ba` (1 = skip, 0 = include in BA).
 *
 * Requires `state->d_img_obs_ptr/idx` (image→obs CSR) to be current.
 * Safe to call before `pack_for_ba` and after each outlier/triangulation pass.
 */
void gpu_select_ba_subset(CudaSfMState* state, const BaSubsetOptions& opts);

} // namespace cuda
} // namespace insight

#endif // INSIGHT_CUDA_BA_SUBSET_CUH
