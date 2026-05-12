/**
 * @file  incremental_sfm_cuda_pipeline.h
 * @brief GPU-accelerated incremental SfM pipeline (drop-in for run_incremental_sfm_pipeline).
 *
 * Same interface as run_incremental_sfm_pipeline() in incremental_sfm_pipeline.h.
 * Key differences from the CPU pipeline:
 *   - Batch triangulation uses the GPU DLT kernel (cuda_triangulation).
 *   - Multi-view outlier rejection uses the GPU reprojection kernel (cuda_sfm_outlier_reject).
 *   - GpuSfMScene (RAII over CudaSfMState) keeps poses, track XYZ, and observation
 *     normalised coords (d_obs_xn/yn) on the device; those coords are recomputed on
 *     the GPU whenever intrinsics change (not a one-shot preprocess).  Host↔device
 *     syncs are concentrated at BA entry/exit boundaries.  TrackStore remains the
 *     host source of truth for graph / BA; the GPU mirror is explicit.
 *   - Ceres bundle adjustment is unchanged (CPU only).
 *   - Batch resection uses CUDA P3P RANSAC + GPU LM refine (resection_batch_cuda).
 *     No CPU resection or CPU triangulation fallbacks: GPU failure returns false from the entry point.
 */

#pragma once

#include "../camera/camera_types.h"
#include "incremental_sfm_pipeline.h"
#include "track_store.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

/**
 * GPU-accelerated incremental SfM pipeline.
 *
 * Same contract as run_incremental_sfm_pipeline():
 *   - Loads tracks + view graph from disk.
 *   - Runs initial-pair loop (CPU).
 *   - Main registration loop: candidate selection → resection (GPU) → triangulation (GPU) →
 *     outlier rejection (GPU) → local BA (Ceres, CPU) → global BA (Ceres, CPU).
 *   - Writes back poses, registered flags, updated intrinsics.
 *
 * @return true on success, false on fatal error.
 */
bool run_incremental_sfm_pipeline_cuda(const std::string& tracks_idc_path,
                                       const std::string& pairs_json_path,
                                       const std::string& geo_dir,
                                       std::vector<camera::Intrinsics>* cameras,
                                       const std::vector<int>& image_to_camera_index,
                                       const IncrementalSfMOptions& opts,
                                       TrackStore* store_out,
                                       std::vector<Eigen::Matrix3d>* poses_R_out,
                                       std::vector<Eigen::Vector3d>* poses_C_out,
                                       std::vector<bool>* registered_out);

} // namespace sfm
} // namespace insight
