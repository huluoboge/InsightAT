/**
 * @file resection_batch_cuda.cu
 * @brief GPU batch resection (CUDA P3P RANSAC + GPU LM refine) for incremental SfM.
 */

#include "resection_batch_cuda.h"

#include "cuda/cuda_pnp_ransac.cuh"
#include "cuda/cuda_resection.cuh"
#include "cuda/cuda_sfm_math.cuh"
#include "cuda/cuda_sfm_state.h"
#include "resection.h"
#include "track_store.h"

#include <cuda_runtime.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <vector>

namespace insight {
namespace sfm {

namespace {

using insight::cuda::intrinsics_to_float9;

} // namespace

int run_batch_resection_cuda(TrackStore& store, const std::vector<int>& image_indices,
                             const std::vector<camera::Intrinsics>& cameras,
                             const std::vector<int>& image_to_camera_index,
                             const float* host_obs_xn, const float* host_obs_yn,
                             insight::cuda::CudaSfMState* gs, insight::cuda::GpuPnpBatchContext* pnp_ctx,
                             insight::cuda::GpuResectionContext* res_ctx,
                             std::vector<Eigen::Matrix3d>* poses_R, std::vector<Eigen::Vector3d>* poses_C,
                             std::vector<bool>* registered, int min_inliers, int pnp_iterations,
                             std::vector<int>* registered_images_out, double post_resection_reproj_thresh_px)
{
    if (!poses_R || !poses_C || !registered || !gs || !pnp_ctx || !res_ctx)
        return 0;
    if (!host_obs_xn || !host_obs_yn) {
        LOG(ERROR) << "run_batch_resection_cuda: null host undistorted obs arrays";
        return 0;
    }
    if (registered_images_out)
        registered_images_out->clear();

    const int K = static_cast<int>(image_indices.size());
    if (K <= 0)
        return 0;

    // Caller is responsible for keeping d_obs_xn/yn current (call undistort_all after
    // intrinsics change or initial upload). Do NOT re-run it here; the resection kernel
    // uses the pre-supplied host_obs_xn/yn cache, and an extra GPU undistort would be
    // a no-op for unchanged intrinsics but needlessly slow for large scenes.

    const int n_obs_store = static_cast<int>(store.num_observations());
    std::vector<float> h_pack_xn, h_pack_yn, h_pack_xyz;
    std::vector<int>    pack_offsets(static_cast<size_t>(K + 1), 0);
    std::vector<std::vector<float>> per_im_xn(K), per_im_yn(K), per_im_xyz(K);

    for (int k = 0; k < K; ++k) {
        const int im = image_indices[static_cast<size_t>(k)];
        if (im < 0 || im >= store.num_images())
            continue;
        std::vector<int> obs_ids;
        store.get_image_all_obs_ids(im, &obs_ids);
        for (int oi : obs_ids) {
            if (oi < 0 || oi >= n_obs_store)
                continue;
            if (!store.is_obs_valid(oi))
                continue;
            const int tid = store.obs_track_id(oi);
            if (tid < 0 || !store.track_has_triangulated_xyz(tid))
                continue;
            float X, Y, Z;
            store.get_track_xyz(tid, &X, &Y, &Z);
            per_im_xn[static_cast<size_t>(k)].push_back(host_obs_xn[oi]);
            per_im_yn[static_cast<size_t>(k)].push_back(host_obs_yn[oi]);
            per_im_xyz[static_cast<size_t>(k)].push_back(X);
            per_im_xyz[static_cast<size_t>(k)].push_back(Y);
            per_im_xyz[static_cast<size_t>(k)].push_back(Z);
        }
    }

    int total_pack = 0;
    for (int k = 0; k < K; ++k) {
        pack_offsets[static_cast<size_t>(k)] = total_pack;
        const int nk = static_cast<int>(per_im_xn[static_cast<size_t>(k)].size());
        total_pack += nk;
    }
    pack_offsets[static_cast<size_t>(K)] = total_pack;

    if (total_pack == 0) {
        LOG(ERROR) << "run_batch_resection_cuda: no valid 3D–2D correspondences (CUDA-only pipeline)";
        return -1;
    }
    if (total_pack > pnp_ctx->max_obs_total) {
        LOG(ERROR) << "run_batch_resection_cuda: pack size " << total_pack << " exceeds GpuPnpBatchContext capacity "
                   << pnp_ctx->max_obs_total << " (CUDA-only: enlarge pnp_ctx max_obs_total or reduce batch)";
        return -1;
    }

    h_pack_xn.resize(static_cast<size_t>(total_pack));
    h_pack_yn.resize(static_cast<size_t>(total_pack));
    h_pack_xyz.resize(static_cast<size_t>(total_pack) * 3u);
    for (int k = 0; k < K; ++k) {
        const int base = pack_offsets[static_cast<size_t>(k)];
        const int nk   = static_cast<int>(per_im_xn[static_cast<size_t>(k)].size());
        for (int i = 0; i < nk; ++i) {
            h_pack_xn[static_cast<size_t>(base + i)] = per_im_xn[static_cast<size_t>(k)][static_cast<size_t>(i)];
            h_pack_yn[static_cast<size_t>(base + i)] = per_im_yn[static_cast<size_t>(k)][static_cast<size_t>(i)];
            const size_t pi                        = static_cast<size_t>(i) * 3u;
            h_pack_xyz[(static_cast<size_t>(base + i)) * 3u + 0] =
                    per_im_xyz[static_cast<size_t>(k)][pi + 0];
            h_pack_xyz[(static_cast<size_t>(base + i)) * 3u + 1] =
                    per_im_xyz[static_cast<size_t>(k)][pi + 1];
            h_pack_xyz[(static_cast<size_t>(base + i)) * 3u + 2] =
                    per_im_xyz[static_cast<size_t>(k)][pi + 2];
        }
    }

    cudaMemcpy(pnp_ctx->d_pack_xn, h_pack_xn.data(), static_cast<size_t>(total_pack) * sizeof(float),
               cudaMemcpyHostToDevice);
    cudaMemcpy(pnp_ctx->d_pack_yn, h_pack_yn.data(), static_cast<size_t>(total_pack) * sizeof(float),
               cudaMemcpyHostToDevice);
    cudaMemcpy(pnp_ctx->d_pack_xyz, h_pack_xyz.data(), static_cast<size_t>(total_pack) * 3u * sizeof(float),
               cudaMemcpyHostToDevice);

    std::vector<::insight::cuda::GpuPnpImageDesc> descs(static_cast<size_t>(K));
    for (int k = 0; k < K; ++k) {
        const int base = pack_offsets[static_cast<size_t>(k)];
        const int nk   = static_cast<int>(per_im_xn[static_cast<size_t>(k)].size());
        descs[static_cast<size_t>(k)].d_obs_xn = pnp_ctx->d_pack_xn + base;
        descs[static_cast<size_t>(k)].d_obs_yn = pnp_ctx->d_pack_yn + base;
        descs[static_cast<size_t>(k)].d_pts3d  = pnp_ctx->d_pack_xyz + base * 3;
        descs[static_cast<size_t>(k)].n_obs    = nk;
        descs[static_cast<size_t>(k)].seed     = static_cast<uint32_t>(image_indices[static_cast<size_t>(k)]) * 1315423911u;
    }

    const int im0 = image_indices[0];
    int cam_idx0 = 0;
    if (im0 >= 0 && im0 < static_cast<int>(image_to_camera_index.size()))
        cam_idx0 = image_to_camera_index[static_cast<size_t>(im0)];
    if (cam_idx0 < 0 || cam_idx0 >= static_cast<int>(cameras.size()))
        cam_idx0 = 0;
    const camera::Intrinsics& Kref = cameras[static_cast<size_t>(cam_idx0)];
    const float fx = static_cast<float>(Kref.fx);
    const float ransac_thr_px = 4.0f;
    const float thr_n_sq      = (fx > 1e-6f) ? (ransac_thr_px / fx) * (ransac_thr_px / fx) : 1e-4f;

    std::vector<float> outR(static_cast<size_t>(K) * 9u);
    std::vector<float> outT(static_cast<size_t>(K) * 3u);
    std::vector<int>   out_inl(static_cast<size_t>(K));
    insight::cuda::gpu_pnp_ransac_batch(pnp_ctx, descs.data(), K, pnp_iterations, thr_n_sq, outR.data(), outT.data(),
                                        out_inl.data());

    int added = 0;
    for (int k = 0; k < K; ++k) {
        const int im = image_indices[static_cast<size_t>(k)];
        if (im < 0 || static_cast<size_t>(im) >= image_to_camera_index.size())
            continue;
        const int nk = static_cast<int>(per_im_xn[static_cast<size_t>(k)].size());
        if (nk < min_inliers) {
            LOG(INFO) << "  resection_cuda image " << im << ": skip (3D-2D=" << nk << ")";
            continue;
        }

        const camera::Intrinsics& Kcam = cameras[static_cast<size_t>(image_to_camera_index[im])];
        float K9[9];
        intrinsics_to_float9(Kcam, K9);

        std::vector<float> hu(static_cast<size_t>(nk)), hv(static_cast<size_t>(nk));
        for (int i = 0; i < nk; ++i) {
            const float xn = per_im_xn[static_cast<size_t>(k)][static_cast<size_t>(i)];
            const float yn = per_im_yn[static_cast<size_t>(k)][static_cast<size_t>(i)];
            hu[static_cast<size_t>(i)] = static_cast<float>(Kcam.fx * xn + Kcam.cx);
            hv[static_cast<size_t>(i)] = static_cast<float>(Kcam.fy * yn + Kcam.cy);
        }

        const float* Rinit = outR.data() + k * 9;
        const float* tinit = outT.data() + k * 3;
        if (out_inl[static_cast<size_t>(k)] < std::max(4, min_inliers / 3)) {
            LOG(ERROR) << "run_batch_resection_cuda: image " << im
                       << " insufficient CUDA PnP inliers (" << out_inl[static_cast<size_t>(k)] << " < "
                       << std::max(4, min_inliers / 3) << "); CUDA-only pipeline aborts this batch";
            return -1;
        }

        const int base = pack_offsets[static_cast<size_t>(k)];
        const float* hxyz = h_pack_xyz.data() + static_cast<size_t>(base) * 3u;

        insight::cuda::gpu_resection_upload(res_ctx, nk, hxyz, hu.data(), hv.data(), Rinit, tinit);
        float Rf[9], tf[3], rmse_px = 0.f;
        if (!insight::cuda::gpu_resection_refine(res_ctx, K9, nk, 12, Rf, tf, &rmse_px)) {
            LOG(ERROR) << "run_batch_resection_cuda: gpu_resection_refine failed for image " << im
                         << " (CUDA-only pipeline)";
            return -1;
        }

        Eigen::Matrix3d Re;
        Eigen::Vector3d te;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Re(r, c) = static_cast<double>(Rf[r * 3 + c]);
        te(0) = static_cast<double>(tf[0]);
        te(1) = static_cast<double>(tf[1]);
        te(2) = static_cast<double>(tf[2]);
        Eigen::Vector3d C = -Re.transpose() * te;

        if (post_resection_reproj_thresh_px > 0.0) {
            const int n_pr = prune_resection_observations_reprojection(&store, im, Re, C, Kcam,
                                                                       post_resection_reproj_thresh_px);
            if (n_pr > 0)
                VLOG(1) << "  post_resection_reproj: image " << im << " pruned " << n_pr << " obs";
        }

        if (static_cast<size_t>(im) >= poses_R->size())
            poses_R->resize(static_cast<size_t>(im) + 1);
        if (static_cast<size_t>(im) >= poses_C->size())
            poses_C->resize(static_cast<size_t>(im) + 1);
        if (static_cast<size_t>(im) >= registered->size())
            registered->resize(static_cast<size_t>(im) + 1);
        (*poses_R)[static_cast<size_t>(im)] = Re;
        (*poses_C)[static_cast<size_t>(im)] = C;
        (*registered)[static_cast<size_t>(im)] = true;
        if (registered_images_out)
            registered_images_out->push_back(im);
        ++added;
        LOG(INFO) << "  resection_cuda image " << im << ": OK (GPU RANSAC inliers=" << out_inl[static_cast<size_t>(k)]
                  << ", refine rmse_px=" << rmse_px << ")";
    }

    LOG(INFO) << "run_batch_resection_cuda: " << added << "/" << K << " images registered";
    return added;
}

} // namespace sfm
} // namespace insight
