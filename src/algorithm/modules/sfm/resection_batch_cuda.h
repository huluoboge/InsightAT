#pragma once

#include "../camera/camera_types.h"
#include "track_store.h"
#include <Eigen/Core>
#include <vector>

namespace insight {
namespace cuda {
struct CudaSfMState;
struct GpuPnpBatchContext;
struct GpuResectionContext;
} // namespace cuda

namespace sfm {

/**
 * Batch resection on GPU: CUDA P3P RANSAC (undistorted normalised coords) + GPU LM refine
 * (pinhole via zero-distortion intrinsics in pixel space), then CPU prune + pose write-back.
 *
 * @param host_obs_xn/yn  length store.num_observations(); must match current GPU undistortion
 *                        (same as CudaSfMState after gpu_sfm_state_undistort_all).
 * @param pnp_iterations   RANSAC hypotheses per image (e.g. 2000).
 * @return number of newly registered images on success (≥0). Returns -1 on fatal error
 *         (CUDA-only pipeline: no CPU batch resection); caller must abort and not fall back.
 */
int run_batch_resection_cuda(TrackStore& store, const std::vector<int>& image_indices,
                             const std::vector<camera::Intrinsics>& cameras,
                             const std::vector<int>& image_to_camera_index,
                             const float* host_obs_xn, const float* host_obs_yn,
                             ::insight::cuda::CudaSfMState* gs, ::insight::cuda::GpuPnpBatchContext* pnp_ctx,
                             ::insight::cuda::GpuResectionContext* res_ctx,
                             std::vector<Eigen::Matrix3d>* poses_R, std::vector<Eigen::Vector3d>* poses_C,
                             std::vector<bool>* registered, int min_inliers, int pnp_iterations,
                             std::vector<int>* registered_images_out = nullptr,
                             double post_resection_reproj_thresh_px = 0.0);

} // namespace sfm
} // namespace insight
