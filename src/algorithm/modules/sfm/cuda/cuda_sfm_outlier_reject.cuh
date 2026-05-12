/**
 * @file  cuda_sfm_outlier_reject.cuh
 * @brief GPU multi-view reprojection / parallax-angle outlier rejection.
 *
 * Scratch buffers live on `CudaSfMState` (`d_reproj_reject_mark`, `d_angle_*`) sized at
 * `gpu_sfm_state_create(n_images, n_tracks, n_obs, …)` — no separate max_obs context.
 */

#pragma once
#ifndef INSIGHT_CUDA_SFM_OUTLIER_REJECT_CUH
#define INSIGHT_CUDA_SFM_OUTLIER_REJECT_CUH

#include <stdint.h>
#include <vector>
#include "cuda_sfm_state.h"

namespace insight {
namespace cuda {

/**
 * For every alive observation of a triangulated track in a registered image,
 * compute squared reprojection error; flags are written to `state->d_reproj_reject_mark`.
 */
int gpu_reject_reproj_multiview(CudaSfMState*     state,
                                float             threshold_px,
                                std::vector<int>* deleted_restorable_out,
                                std::vector<int>* deleted_permanent_out);

/**
 * Rebuild per-track CSR from current `d_obs_valid` / `d_obs_track_idx` into
 * `state->d_angle_csr_ptr` and `state->d_angle_csr_obs`. Call after obs deletions.
 */
void gpu_outlier_angle_build_csr(CudaSfMState* state);

int gpu_reject_angle_multiview(CudaSfMState*     state,
                               float             min_angle_deg,
                               float             max_angle_deg,
                               std::vector<int>* deleted_obs_out);

} // namespace cuda
} // namespace insight

#endif // INSIGHT_CUDA_SFM_OUTLIER_REJECT_CUH
