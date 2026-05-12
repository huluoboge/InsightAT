/**
 * @file  incremental_sfm_cuda_pipeline.cu
 * @brief GPU-accelerated incremental SfM pipeline.
 *
 * CUDA-first incremental SfM: triangulation, resection, and outlier passes run on GPU.
 * There is no CPU fallback for batch resection or batch triangulation (failures return false).
 * Policy/scheduling (BA schedule, resection candidate ranking, retriangulation scoping)
 * largely reuses the CPU pipeline helpers.
 *
 * GPU acceleration points (relative to CPU pipeline):
 *   1. Batch triangulation        — GPU DLT kernel (cuda_triangulation)
 *   2. Multi-view outlier reject  — GPU reprojection kernel (cuda_sfm_outlier_reject)
 *   3. Resection                  — CUDA batch P3P RANSAC + GPU LM refine (resection_batch_cuda);
 *                                    no EGL/gpu_geo warmup (legacy GLSL PnP not used on hot path)
 *   4. Undistort state            — GPU d_obs_xn/yn recomputed whenever intrinsics change; host cache for triangulation pack
 *
 * Device working set: GpuSfMScene (cuda/gpu_sfm_scene.h) wraps CudaSfMState; use its methods for
 * host↔device sync and scene-scoped kernels. Pass `gpu_scene->cuda_state()` only into legacy C APIs
 * (e.g. batch resection / tri context) that still take `CudaSfMState*`.
 *
 * Sync points (host↔device):
 *   - After initial pair: upload obs, intrinsics, poses → undistort_all
 *   - After each resection: upload_one_pose + set_registered
 *   - After each triangulation batch: update tracks on device
 *   - After each BA: upload all poses + intrinsics; undistort_all if intrinsics changed
 */

#include "incremental_sfm_cuda_pipeline.h"

#include "../../io/idc_reader.h"
#include "../../io/track_store_idc.h"
#include "bundle_adjustment_analytic.h"
#include "cuda/cuda_pnp_ransac.cuh"
#include "cuda/cuda_resection.cuh"
#include "cuda/cuda_sfm_math.cuh"
#include "cuda/cuda_sfm_state.h"
#include "cuda/gpu_sfm_scene.h"
#include "cuda/cuda_triangulation.cuh"
#include "incremental_sfm_pipeline.h"
#include "incremental_triangulation.h"
#include "resection.h"
#include "resection_batch_cuda.h"
#include "scene_normalization.h"
#include "track_store.h"
#include "two_view_reconstruction.h"
#include "view_graph.h"
#include "view_graph_loader.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <glog/logging.h>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <vector>

namespace insight {
namespace sfm {

namespace {

using namespace insight::cuda;

using insight::cuda::eigen_R_to_float9;
using insight::cuda::eigen_C_to_float3;
using insight::cuda::intrinsics_to_float9;

// ─────────────────────────────────────────────────────────────────────────────
// GPU batch triangulation
// ─────────────────────────────────────────────────────────────────────────────

struct GpuTriBatch {
    GpuTriContext*    ctx        = nullptr;
    int               max_obs   = 0;
    int               max_tri   = 0;
    int               max_pairs = 0;
    // Persistent device scratch: pre-allocated for max_tri tracks, avoids
    // cudaMalloc/cudaFree per-iteration inside gpu_sfm_update_tracks_batch.
    int*   d_tid_scratch = nullptr; ///< [max_tri]
    float* d_xyz_scratch = nullptr; ///< [max_tri * 3]
};

static GpuTriBatch* create_tri_batch(int max_obs, int max_tri, int max_pairs)
{
    auto* b = new GpuTriBatch;
    b->max_obs   = max_obs;
    b->max_tri   = max_tri;
    b->max_pairs = max_pairs;
    b->ctx = gpu_tri_ctx_create(max_obs, max_tri, max_pairs);
    cudaMalloc(&b->d_tid_scratch, static_cast<size_t>(max_tri) * sizeof(int));
    cudaMalloc(&b->d_xyz_scratch, static_cast<size_t>(max_tri) * 3 * sizeof(float));
    return b;
}

static void free_tri_batch(GpuTriBatch* b)
{
    if (!b) return;
    gpu_tri_ctx_free(b->ctx);
    cudaFree(b->d_tid_scratch);
    cudaFree(b->d_xyz_scratch);
    delete b;
}

// ── shared_ptr + custom deleters for main CUDA pipeline GPU resources ────────

using GpuSfMScenePtr      = std::shared_ptr<cuda::GpuSfMScene>;
using GpuPnpBatchPtr      = std::shared_ptr<GpuPnpBatchContext>;
using GpuResectionCtxPtr  = std::shared_ptr<GpuResectionContext>;
using GpuTriBatchPtr      = std::shared_ptr<GpuTriBatch>;

static GpuPnpBatchPtr make_pnp_batch_shared(GpuPnpBatchContext* raw)
{
    return GpuPnpBatchPtr(raw, [](GpuPnpBatchContext* p) { gpu_pnp_batch_ctx_free(p); });
}

static GpuResectionCtxPtr make_resection_ctx_shared(GpuResectionContext* raw)
{
    return GpuResectionCtxPtr(raw, [](GpuResectionContext* p) { gpu_resection_ctx_free(p); });
}

static GpuTriBatchPtr make_tri_batch_shared(GpuTriBatch* raw)
{
    return GpuTriBatchPtr(raw, [](GpuTriBatch* p) { free_tri_batch(p); });
}

static void reset_cuda_pipeline_stack(GpuTriBatchPtr&      tri_batch,
                                      GpuPnpBatchPtr&      pnp_ctx,
                                      GpuResectionCtxPtr& res_ctx,
                                      GpuSfMScenePtr&      gpu_scene)
{
    tri_batch.reset();
    pnp_ctx.reset();
    res_ctx.reset();
    gpu_scene.reset();
}

/// GPU batch triangulation.
///
/// For each track in `track_ids_to_tri`, gather its valid observations from
/// the TrackStore, build the per-obs arrays for the GPU DLT kernel,
/// run gpu_tri_upload + gpu_tri_run, and write the results back to both the
/// TrackStore and the CudaSfMState.
///
/// Returns the count of successfully triangulated tracks, or -1 on fatal error (no CPU fallback).
static int run_batch_triangulation_gpu(
        TrackStore*                           store,
        CudaSfMState*                         gs,
        GpuTriBatch*                          tri_batch,
        const std::vector<int>&               track_ids_to_tri,
        const std::vector<Eigen::Matrix3d>&   poses_R,
        const std::vector<Eigen::Vector3d>&   poses_C,
        const std::vector<bool>&              registered,
        const std::vector<camera::Intrinsics>& cameras,
        const std::vector<int>&               image_to_camera_index,
        const std::vector<float>&             obs_xn_cache, // from GPU, length = n_obs
        const std::vector<float>&             obs_yn_cache,
        double                                min_tri_angle_deg,
        std::vector<int>*                     new_track_ids_out,
        double                                commit_reproj_px)
{
    if (track_ids_to_tri.empty()) return 0;
    if (new_track_ids_out) new_track_ids_out->clear();

    // ── Per-track obs gather ───────────────────────────────────────────────
    // Build: csr_ptr, obs_xn, obs_yn, obs_u, obs_v, obs_R, obs_C, obs_K
    // For the GPU DLT kernel, all per-obs arrays are flat (track0_obs0, track0_obs1, ..., track1_obs0, ...).

    const int n_tracks_batch = static_cast<int>(track_ids_to_tri.size());

    std::vector<int>   csr_ptr;
    csr_ptr.reserve(static_cast<size_t>(n_tracks_batch + 1));
    csr_ptr.push_back(0);

    std::vector<float> obs_xn_batch, obs_yn_batch;
    std::vector<float> obs_u_batch,  obs_v_batch;
    std::vector<float> obs_R_batch,  obs_C_batch, obs_K_batch;

    const int total_obs_store = static_cast<int>(store->num_observations());

    // We iterate obs of the store to build per-track obs.
    // TrackStore allows get_track_observations — use that.
    std::vector<int> obs_ids_tmp;
    for (const int tid : track_ids_to_tri) {
        if (!store->is_track_valid(tid)) {
            csr_ptr.push_back(csr_ptr.back());
            continue;
        }
        // We need obs with valid images that are registered.
        obs_ids_tmp.clear();
        store->get_track_obs_ids(tid, &obs_ids_tmp);

        int obs_added = 0;
        for (const int oi : obs_ids_tmp) {
            if (!store->is_obs_valid(oi)) continue;
            Observation o;
            store->get_obs(oi, &o);
            const int im = static_cast<int>(o.image_index);
            if (im < 0 || im >= static_cast<int>(registered.size())) continue;
            if (!registered[static_cast<size_t>(im)]) continue;

            const int cam = image_to_camera_index[static_cast<size_t>(im)];
            const camera::Intrinsics& K = cameras[static_cast<size_t>(cam)];

            // Undistorted normalised coords: use cached host copy (from GPU undistort).
            float xn, yn;
            if (oi < 0 || oi >= total_obs_store || obs_xn_cache.size() < static_cast<size_t>(total_obs_store) ||
                obs_yn_cache.size() < static_cast<size_t>(total_obs_store)) {
                LOG(ERROR) << "[GPU tri] obs normalised cache missing or short (oi=" << oi
                             << "); CUDA-only pipeline requires gpu_sfm_download_obs_norm cache";
                return -1;
            }
            xn = obs_xn_cache[static_cast<size_t>(oi)];
            yn = obs_yn_cache[static_cast<size_t>(oi)];

            obs_xn_batch.push_back(xn);
            obs_yn_batch.push_back(yn);
            obs_u_batch.push_back(o.u);
            obs_v_batch.push_back(o.v);

            float R9[9], C3[3], K9[9];
            eigen_R_to_float9(poses_R[static_cast<size_t>(im)], R9);
            eigen_C_to_float3(poses_C[static_cast<size_t>(im)], C3);
            intrinsics_to_float9(K, K9);
            for (int k = 0; k < 9; ++k) obs_R_batch.push_back(R9[k]);
            for (int k = 0; k < 3; ++k) obs_C_batch.push_back(C3[k]);
            for (int k = 0; k < 9; ++k) obs_K_batch.push_back(K9[k]);

            ++obs_added;
        }
        csr_ptr.push_back(csr_ptr.back() + obs_added);
    }

    // csr_ptr has n_tracks_batch + 1 entries (one zero-range entry per invalid track).
    const int n_obs_batch = csr_ptr.back();
    if (n_obs_batch < 2 || n_tracks_batch == 0) return 0;

    // ── Ensure capacity (no CPU triangulation fallback) ───────────────────
    const int max_pairs_approx = n_tracks_batch * 500;
    if (n_obs_batch > tri_batch->max_obs || n_tracks_batch > tri_batch->max_tri ||
        max_pairs_approx > tri_batch->max_pairs) {
        LOG(ERROR) << "[GPU tri] batch exceeds GpuTriContext (obs=" << n_obs_batch << " max_obs=" << tri_batch->max_obs
                   << ", tracks=" << n_tracks_batch << " max_tri=" << tri_batch->max_tri
                   << ", pairs≈" << max_pairs_approx << " max_pairs=" << tri_batch->max_pairs
                   << "); CUDA-only: enlarge create_tri_batch caps";
        return -1;
    }

    // ── Upload + run ──────────────────────────────────────────────────────
    GpuTriOptions opt;
    opt.ransac_inlier_px    = static_cast<float>(commit_reproj_px);
    opt.min_tri_angle_deg   = static_cast<float>(min_tri_angle_deg);
    opt.max_tri_angle_deg   = 170.0f;
    opt.max_pairs_per_track = 500;
    opt.min_inlier_views    = 2;

    gpu_tri_upload(tri_batch->ctx, n_tracks_batch, n_obs_batch,
                   csr_ptr.data(),
                   obs_xn_batch.data(), obs_yn_batch.data(),
                   obs_u_batch.data(),  obs_v_batch.data(),
                   obs_R_batch.data(),  obs_C_batch.data(),
                   obs_K_batch.data());

    std::vector<float>   out_xyz(static_cast<size_t>(n_tracks_batch) * 3);
    std::vector<uint8_t> out_valid(static_cast<size_t>(n_tracks_batch), 0);

    const int n_succeeded = gpu_tri_run(tri_batch->ctx, n_tracks_batch, opt,
                                         out_xyz.data(), out_valid.data());

    // ── Write results back to TrackStore + GPU state ───────────────────────
    std::vector<int>   tid_list_ok;
    std::vector<float> xyz_ok;
    tid_list_ok.reserve(static_cast<size_t>(n_succeeded));
    xyz_ok.reserve(static_cast<size_t>(n_succeeded) * 3);

    for (int i = 0; i < n_tracks_batch; ++i) {
        if (!out_valid[static_cast<size_t>(i)]) continue;
        const int tid = track_ids_to_tri[static_cast<size_t>(i)];
        const float x = out_xyz[static_cast<size_t>(i) * 3 + 0];
        const float y = out_xyz[static_cast<size_t>(i) * 3 + 1];
        const float z = out_xyz[static_cast<size_t>(i) * 3 + 2];

        store->set_track_xyz(tid, x, y, z);
        if (new_track_ids_out) new_track_ids_out->push_back(tid);

        tid_list_ok.push_back(tid);
        xyz_ok.push_back(x);
        xyz_ok.push_back(y);
        xyz_ok.push_back(z);
    }

    // GPU state scatter update — use pre-allocated device scratch to avoid per-call cudaMalloc.
    if (!tid_list_ok.empty())
        gpu_sfm_update_tracks_batch_prealloc(gs,
                                              tid_list_ok.data(), xyz_ok.data(),
                                              static_cast<int>(tid_list_ok.size()),
                                              tri_batch->d_tid_scratch,
                                              tri_batch->d_xyz_scratch);

    return n_succeeded;
}

// ─────────────────────────────────────────────────────────────────────────────
// GPU outlier rejection shim
// ─────────────────────────────────────────────────────────────────────────────

/// Run GPU reprojection outlier rejection; update TrackStore and GPU valid flags.
/// Returns number of deleted observations.
///
/// TODO(M4): Once GPU-side obs-valid flags are the sole source of truth, remove the
/// store->mark_observation_deleted* calls below and let the GPU flags drive everything.
static int reject_outliers_gpu(TrackStore*         store,
                               cuda::GpuSfMScene* scene,
                               double             threshold_px)
{
    if (!store || !scene) return 0;
    std::vector<int> deleted_restorable, deleted_permanent;
    const int n_deleted = scene->reject_reproj_multiview(static_cast<float>(threshold_px),
                                                         &deleted_restorable,
                                                         &deleted_permanent);
    if (n_deleted <= 0) return 0;

    // Restorable: reproj error outliers — retriangulation can recover them.
    // TODO(M4): These writes to store are transitional; GPU obs-valid flags are already updated.
    for (const int obs_id : deleted_restorable)
        store->mark_observation_deleted_restorable(obs_id);

    // Permanent: cheirality failures (point behind camera) — never recoverable.
    for (const int obs_id : deleted_permanent)
        store->mark_observation_deleted(obs_id);

    // Clear XYZ for tracks that have become insufficiently supported (< 2 alive obs).
    // For tracks with restorable deletions but still ≥ 2 alive obs, mark as needing retri.
    std::unordered_map<int, int> alive_cnt;
    alive_cnt.reserve(static_cast<size_t>(n_deleted));
    auto check_track = [&](int oi) {
        const int tid = store->obs_track_id(oi);
        if (tid < 0) return;
        if (alive_cnt.find(tid) == alive_cnt.end()) {
            std::vector<int> alive_ids;
            alive_cnt[tid] = store->get_track_obs_ids(tid, &alive_ids);
        }
        if (alive_cnt[tid] < 2) {
            store->clear_track_xyz(tid);
            scene->set_track_valid_on_device(tid, 0u);
        }
    };
    for (const int oi : deleted_restorable) check_track(oi);
    for (const int oi : deleted_permanent)  check_track(oi);

    // Mark tracks with restorable-deleted obs (and still ≥ 2 alive obs) as needing retri.
    std::vector<int> retri_candidates;
    retri_candidates.reserve(deleted_restorable.size());
    for (const int oi : deleted_restorable) {
        const int tid = store->obs_track_id(oi);
        if (tid < 0) continue;
        const auto it = alive_cnt.find(tid);
        if (it != alive_cnt.end() && it->second >= 2)
            retri_candidates.push_back(tid);
    }
    if (!retri_candidates.empty()) {
        // Deduplicate and mark on device.
        std::sort(retri_candidates.begin(), retri_candidates.end());
        retri_candidates.erase(std::unique(retri_candidates.begin(), retri_candidates.end()),
                               retri_candidates.end());
        scene->mark_tracks_need_retri(retri_candidates);
    }

    return n_deleted;
}

/// GPU per-track parallax angle outlier pass.
/// Builds the track CSR, runs the angle check, marks bad obs as restorable-deleted,
/// clears under-supported tracks, and syncs d_obs_valid back to device.
///
/// TODO(M4): Store writes below are transitional; remove once GPU obs flags are authoritative.
static int reject_angle_outliers_gpu(TrackStore*         store,
                                     cuda::GpuSfMScene* scene,
                                     double             min_angle_deg)
{
    if (!store || !scene) return 0;
    scene->outlier_angle_build_csr();
    std::vector<int> deleted_angle;
    const int n_del = scene->reject_angle_multiview(static_cast<float>(min_angle_deg),
                                                    170.0f, &deleted_angle);
    if (n_del <= 0) return 0;

    for (const int oi : deleted_angle)
        store->mark_observation_deleted_restorable(oi);

    // Clear tracks that became under-supported (< 2 alive obs).
    std::unordered_map<int, int> alive_cnt;
    alive_cnt.reserve(static_cast<size_t>(n_del));
    auto check_track = [&](int oi) {
        const int tid = store->obs_track_id(oi);
        if (tid < 0) return;
        if (alive_cnt.find(tid) == alive_cnt.end()) {
            std::vector<int> alive_ids;
            alive_cnt[tid] = store->get_track_obs_ids(tid, &alive_ids);
        }
        if (alive_cnt[tid] < 2) {
            store->clear_track_xyz(tid);
            scene->set_track_valid_on_device(tid, 0u);
        }
    };
    for (const int oi : deleted_angle) check_track(oi);

    // Mark tracks with restorable-deleted obs and ≥ 2 alive obs as needing retriangulation
    // (same pattern as reject_outliers_gpu; angle outliers are always restorable).
    std::vector<int> retri_candidates;
    retri_candidates.reserve(deleted_angle.size());
    for (const int oi : deleted_angle) {
        const int tid = store->obs_track_id(oi);
        if (tid < 0) continue;
        const auto it = alive_cnt.find(tid);
        if (it != alive_cnt.end() && it->second >= 2)
            retri_candidates.push_back(tid);
    }
    if (!retri_candidates.empty()) {
        std::sort(retri_candidates.begin(), retri_candidates.end());
        retri_candidates.erase(std::unique(retri_candidates.begin(), retri_candidates.end()),
                               retri_candidates.end());
        scene->mark_tracks_need_retri(retri_candidates);
    }

    // Push updated obs-valid mask to GPU.
    scene->upload_obs_valid_from_host(*store);
    return n_del;
}

// ─────────────────────────────────────────────────────────────────────────────
// GPU-first BA helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Scatter BA workset results back to CPU pose vectors and TrackStore.
/// Called after a successful run_ba_from_workset to propagate Ceres-optimised
/// poses and point XYZ to the CPU representations consumed by retriangulation.
static void scatter_workset_to_cpu(
        const cuda::LocalBAWorkset&   workset,
        std::vector<Eigen::Matrix3d>* poses_R_out,
        std::vector<Eigen::Vector3d>* poses_C_out,
        sfm::TrackStore*              store_out)
{
    for (int i = 0; i < workset.n_local_images; ++i) {
        const int gim = workset.image_ids[static_cast<size_t>(i)];
        const double* R = workset.poses_R.data() + i * 9;
        const double* C = workset.poses_C.data() + i * 3;
        (*poses_R_out)[static_cast<size_t>(gim)]
            << R[0],R[1],R[2], R[3],R[4],R[5], R[6],R[7],R[8];
        (*poses_C_out)[static_cast<size_t>(gim)] = Eigen::Vector3d(C[0], C[1], C[2]);
    }
    for (int t = 0; t < workset.n_local_tracks; ++t) {
        const int gtid = workset.track_ids[static_cast<size_t>(t)];
        const double* xyz = workset.track_xyz.data() + t * 3;
        store_out->set_track_xyz(gtid, xyz[0], xyz[1], xyz[2]);
    }
}

/// Convert LocalBAWorkset → BAInput → run global_bundle_analytic → update workset in-place.
///
/// @param workset               In/out: poses_R/C and track_xyz updated on success.
/// @param cameras               Full global intrinsics list; updated in-place when optimised.
/// @param opts                  BA and intrinsics options.
/// @param num_registered_global Total number of globally registered images; used to gate
///                              intrinsics optimisation via optimize_intrinsics_min_images
///                              so early-phase behaviour matches the full-scene context.
/// @param rmse_px_out           Optional RMSE output (pixels).
/// @return true on Ceres success.
static bool run_ba_from_workset(
        cuda::LocalBAWorkset&            workset,
        std::vector<camera::Intrinsics>* cameras,
        const IncrementalSfMOptions&     opts,
        int                              num_registered_global,
        double*                          rmse_px_out)
{
    const int n_li = workset.n_local_images;
    const int n_lt = workset.n_local_tracks;
    const int n_lo = workset.n_local_obs;
    const int n_gc = static_cast<int>(cameras->size());
    if (n_li < 2 || n_lt < 5 || n_lo < 10 || n_gc == 0) return false;

    // ── Per-image global camera index (inferred from first matching obs) ──
    std::vector<int> local_image_global_cam(static_cast<size_t>(n_li), -1);
    for (int j = 0; j < n_lo; ++j) {
        const int li = workset.obs_image_local[j];
        if (li >= 0 && li < n_li && local_image_global_cam[li] < 0)
            local_image_global_cam[li] = workset.obs_cam_idx[j];
    }
    for (int i = 0; i < n_li; ++i)
        if (local_image_global_cam[i] < 0) local_image_global_cam[i] = 0;

    // ── Build compact local camera list ───────────────────────────────────
    std::vector<int> global_to_local_cam(static_cast<size_t>(n_gc), -1);
    std::vector<int> local_cam_to_global;
    for (const int gc : local_image_global_cam) {
        if (gc >= 0 && gc < n_gc && global_to_local_cam[gc] < 0) {
            global_to_local_cam[gc] = static_cast<int>(local_cam_to_global.size());
            local_cam_to_global.push_back(gc);
        }
    }
    const int n_lc = static_cast<int>(local_cam_to_global.size());
    if (n_lc == 0) return false;

    // ── Assemble BAInput ──────────────────────────────────────────────────
    BAInput ba_in;
    // Gate intrinsics on the *global* registered count, not the local subset size,
    // so early-phase behaviour is consistent with the full-scene context.
    // Gate intrinsics: use IntrinsicsSchedule (phase1_min_images) to match CPU pipeline behaviour.
    ba_in.optimize_intrinsics = opts.global_ba.optimize_intrinsics &&
                                (num_registered_global >= opts.intrinsics.phase1_min_images);

    ba_in.poses_R.resize(static_cast<size_t>(n_li));
    ba_in.poses_C.resize(static_cast<size_t>(n_li));
    ba_in.fix_pose.resize(static_cast<size_t>(n_li), false);
    ba_in.image_camera_index.resize(static_cast<size_t>(n_li));

    for (int i = 0; i < n_li; ++i) {
        const double* R = workset.poses_R.data() + i * 9;
        const double* C = workset.poses_C.data() + i * 3;
        ba_in.poses_R[i] << R[0],R[1],R[2], R[3],R[4],R[5], R[6],R[7],R[8];
        ba_in.poses_C[i]  = Eigen::Vector3d(C[0], C[1], C[2]);
        ba_in.fix_pose[i] = (workset.fixed_pose[i] != 0u);
        const int gc = local_image_global_cam[i];
        ba_in.image_camera_index[i] = (gc >= 0 ? global_to_local_cam[gc] : 0);
    }

    ba_in.points3d.resize(static_cast<size_t>(n_lt));
    ba_in.fix_point.resize(static_cast<size_t>(n_lt), false);
    for (int t = 0; t < n_lt; ++t) {
        const double* xyz = workset.track_xyz.data() + t * 3;
        ba_in.points3d[t] = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    }

    ba_in.observations.resize(static_cast<size_t>(n_lo));
    for (int j = 0; j < n_lo; ++j) {
        auto& ob       = ba_in.observations[j];
        ob.image_index = workset.obs_image_local[j];
        ob.point_index = workset.obs_track_local[j];
        ob.u           = workset.obs_u[j];
        ob.v           = workset.obs_v[j];
    }

    ba_in.cameras.resize(static_cast<size_t>(n_lc));
    for (int lc = 0; lc < n_lc; ++lc)
        ba_in.cameras[lc] = (*cameras)[local_cam_to_global[lc]];

    // ── Per-camera intrinsics fix masks ───────────────────────────────────
    {
        const std::vector<bool> all_reg(static_cast<size_t>(n_li), true);
        ba_in.fix_intrinsics_flags = opts.intrinsics.fix_masks_per_camera(
                all_reg, ba_in.image_camera_index, n_lc);
    }

    // ── Ceres solver settings ─────────────────────────────────────────────
    const BASolverOverrides& ov    = opts.global_ba.solver_overrides;
    ba_in.focal_prior_weight       = opts.intrinsics.focal_prior_weight;
    ba_in.huber_loss_delta         = ov.huber_loss_delta > 0.0 ? ov.huber_loss_delta : 4.0;
    ba_in.tikhonov_lambda          = ov.tikhonov_lambda;
    ba_in.solver_gradient_tolerance  = ov.gradient_tolerance;
    ba_in.solver_function_tolerance  = ov.function_tolerance;
    ba_in.solver_parameter_tolerance = ov.parameter_tolerance;
    ba_in.num_threads                = ov.num_threads;
    ba_in.solver_dense_schur_max_variable_cams = ov.dense_schur_max_variable_cams;
    // camera_total_obs is not filled (workset is a subset); set threshold to 0 so
    // global_bundle_analytic always uses tight distortion bounds rather than
    // computing obs counts from the subset — which would fluctuate between calls
    // and cause the relax strategy to oscillate.
    ba_in.relax_intrinsics_obs_threshold = 0;

    // ── Run Ceres ─────────────────────────────────────────────────────────
    BAResult ba_result;
    const bool ok = global_bundle_analytic(ba_in, &ba_result, opts.global_ba.max_iterations);
    if (!ok) return false;
    if (rmse_px_out) *rmse_px_out = ba_result.rmse_px;

    // ── Scatter optimised values back into workset ────────────────────────
    for (int i = 0; i < n_li; ++i) {
        double* R = workset.poses_R.data() + i * 9;
        double* C = workset.poses_C.data() + i * 3;
        const Eigen::Matrix3d& Rm = ba_result.poses_R[i];
        const Eigen::Vector3d& Cm = ba_result.poses_C[i];
        for (int k = 0; k < 3; ++k)
            for (int l = 0; l < 3; ++l)
                R[k*3+l] = Rm(k, l);
        C[0] = Cm[0]; C[1] = Cm[1]; C[2] = Cm[2];
    }
    for (int t = 0; t < n_lt; ++t) {
        double* xyz = workset.track_xyz.data() + t * 3;
        const Eigen::Vector3d& pt = ba_result.points3d[t];
        xyz[0] = pt[0]; xyz[1] = pt[1]; xyz[2] = pt[2];
    }

    // ── Propagate optimised intrinsics to global camera list ──────────────
    if (ba_in.optimize_intrinsics) {
        for (int lc = 0; lc < n_lc; ++lc) {
            const int gc = local_cam_to_global[lc];
            if (gc >= 0 && gc < n_gc)
                (*cameras)[gc] = ba_result.cameras[lc];
        }
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Tracks to triangulate for a given set of newly-registered images
// ─────────────────────────────────────────────────────────────────────────────

/// Collect tracks that should be triangulated after registering `new_image_indices`.
/// Only untriangulated (no XYZ) tracks with ≥2 observations from registered images are returned.
static std::vector<int> collect_untri_tracks(
        const TrackStore&       store,
        const std::vector<int>& new_image_indices,
        const std::vector<bool>& registered)
{
    // Collect all tracks visible from the newly-registered images.
    std::vector<int>        track_ids_to_tri;
    std::unordered_map<int, int> tri_cnt; // track_id → registered-obs count

    for (const int im : new_image_indices) {
        if (im < 0 || im >= static_cast<int>(registered.size())) continue;
        std::vector<int> img_obs;
        store.get_image_observation_indices(im, &img_obs);
        for (const int oi : img_obs) {
            if (!store.is_obs_valid(oi)) continue;
            const int tid = store.obs_track_id(oi);
            if (tid < 0) continue;
            if (!store.is_track_valid(tid)) continue;
            if (store.track_has_triangulated_xyz(tid)) continue; // already done
            tri_cnt[tid]++;
        }
    }

    // Also add triangulated tracks that share observations with new images to potentially
    // extend them (for angle improvement), but only if explicitly not-yet triangulated.
    track_ids_to_tri.reserve(tri_cnt.size());
    for (const auto& kv : tri_cnt) {
        const int tid = kv.first;
        // Check total registered obs count across all images.
        if (kv.second >= 2)
            track_ids_to_tri.push_back(tid);
        else {
            int cnt = 0;
            // Count registered obs of this track across all images:
            std::vector<int> all_obs;
            store.get_track_obs_ids(tid, &all_obs);
            for (const int oi2 : all_obs) {
                Observation o2;
                store.get_obs(oi2, &o2);
                const int im2 = static_cast<int>(o2.image_index);
                if (im2 >= 0 && im2 < static_cast<int>(registered.size()) &&
                    registered[static_cast<size_t>(im2)])
                    ++cnt;
            }
            if (cnt >= 2)
                track_ids_to_tri.push_back(tid);
        }
    }

    return track_ids_to_tri;
}

} // namespace (anonymous)

// ─────────────────────────────────────────────────────────────────────────────
// Main CUDA pipeline
// ─────────────────────────────────────────────────────────────────────────────

bool run_incremental_sfm_pipeline_cuda(const std::string& tracks_idc_path,
                                       const std::string& pairs_json_path,
                                       const std::string& geo_dir,
                                       std::vector<camera::Intrinsics>* cameras,
                                       const std::vector<int>& image_to_camera_index,
                                       const IncrementalSfMOptions& opts,
                                       TrackStore* store_out,
                                       std::vector<Eigen::Matrix3d>* poses_R_out,
                                       std::vector<Eigen::Vector3d>* poses_C_out,
                                       std::vector<bool>* registered_out)
{
    if (!store_out || !poses_R_out || !poses_C_out || !registered_out || !cameras)
        return false;

    // ── Load data ─────────────────────────────────────────────────────────
    ViewGraph view_graph;
    if (!load_track_store_from_idc(tracks_idc_path, store_out, nullptr, &view_graph)) {
        LOG(ERROR) << "run_incremental_sfm_pipeline_cuda: failed to load tracks from "
                   << tracks_idc_path;
        return false;
    }
    const int n_images = store_out->num_images();
    if (n_images == 0 || static_cast<int>(image_to_camera_index.size()) != n_images) {
        LOG(ERROR) << "run_incremental_sfm_pipeline_cuda: num_images mismatch";
        return false;
    }
    poses_R_out->resize(static_cast<size_t>(n_images));
    poses_C_out->resize(static_cast<size_t>(n_images));
    registered_out->resize(static_cast<size_t>(n_images), false);

    if (view_graph.num_pairs() == 0) {
        if (!build_view_graph_from_geo(pairs_json_path, geo_dir, &view_graph)) {
            LOG(ERROR) << "run_incremental_sfm_pipeline_cuda: failed to build view graph";
            return false;
        }
    }

    // ── Initial pair (CPU, unchanged) ─────────────────────────────────────
    uint32_t local_im0 = 0, local_im1 = 0;
    if (!run_initial_pair_loop(view_graph, store_out, *cameras, image_to_camera_index,
                               opts.init.min_tracks_for_intital_pair,
                               opts.init.min_median_angle_deg,
                               &local_im0, &local_im1,
                               poses_R_out, poses_C_out, registered_out)) {
        LOG(ERROR) << "run_incremental_sfm_pipeline_cuda: no initial pair succeeded";
        return false;
    }
    int num_registered = 2;
    const int anchor_image = static_cast<int>(local_im0);
    const int initial_pair_im1 = static_cast<int>(local_im1);

    LOG(INFO) << "[CUDA SfM] Initial pair done. im0=" << anchor_image
              << " im1=" << initial_pair_im1
              << "  registered=2/" << n_images
              << "  tri_tracks=" << store_out->num_triangulated_tracks();

    // Resection / triangulation / outlier passes are CUDA-only (no CPU resection or CPU tri fallback).

    // ── Build GPU SfM state (shared_ptr → stable release on all exit paths) ─
    GpuSfMScenePtr gpu_scene = cuda::GpuSfMScene::try_create_from_track_store(
            *store_out, *cameras, image_to_camera_index,
            *poses_R_out, *poses_C_out, *registered_out);
    if (!gpu_scene) {
        LOG(ERROR) << "run_incremental_sfm_pipeline_cuda: failed to create GpuSfMScene";
        return false;
    }

    // ── TrackStore lifecycle policy ───────────────────────────────────────
    // After this point `gpu_scene` (GpuSfMScene / CudaSfMState) is the authoritative
    // GPU working set.  `store_out` transitions to a secondary role:
    //
    //   READS of store_out ARE PERMITTED throughout the main loop for:
    //     • choose_resection_candidates   (obs counts / pixel coords)
    //     • run_batch_triangulation_gpu   (obs gather — TODO: migrate to GPU-side CSR)
    //     • run_retriangulation           (CPU path — TODO: replace with GPU retri)
    //
    //   BA is now GPU-first (M3 complete): select_ba_subset → pack_for_ba →
    //   run_ba_from_workset → scatter_workset_to_cpu → upload_ba_result →
    //   sync_after_bundle_adjust.  TrackStore XYZ writes from BA are limited to
    //   the workset subset via scatter_workset_to_cpu.
    //
    //   WRITES to store_out (marks, xyz, valid flags) occur as side-effects of
    //   the CPU helpers above.  Once the remaining TODO paths migrate to GPU,
    //   all writes to store_out will be removed from the hot loop.
    //
    //   At the TAIL of the function, store_out track XYZ / obs-valid are NOT explicitly
    //   re-downloaded here (see export note below).  The caller receives the current GPU
    //   state through poses_R_out / poses_C_out / registered_out (kept in sync
    //   throughout) and store_out (kept in sync via CPU helpers above until M4/M5).

    // Download undistorted obs coords to host cache (used by GPU triangulation data prep).
    std::vector<float> obs_xn_cache, obs_yn_cache;
    gpu_scene->download_obs_norm_to_host(&obs_xn_cache, &obs_yn_cache);

    // ── GPU triangulation context ─────────────────────────────────────────
    // Allocate for a large batch: up to all obs and all tracks at once.
    const int n_obs_total     = gpu_scene->num_observations();
    const int max_obs_batch   = std::min(n_obs_total, 500000);
    const int max_tri_batch   = std::min(gpu_scene->num_tracks(), 200000);
    // Use the same multiplier as GpuTriOptions::max_pairs_per_track (500) so that the
    // capacity check in run_batch_triangulation_gpu never trips on a well-formed batch.
    const int max_pairs_batch = max_tri_batch * 500;
    GpuTriBatchPtr tri_batch = make_tri_batch_shared(create_tri_batch(max_obs_batch, max_tri_batch, max_pairs_batch));

    constexpr int kPnpRansacIterations       = 2000;
    constexpr int kResectionCandidateListMax = 40;
    GpuPnpBatchPtr pnp_ctx = make_pnp_batch_shared(
            gpu_pnp_batch_ctx_create(std::min(kResectionCandidateListMax, n_images), kPnpRansacIterations,
                                     n_obs_total));
    // Per-image GPU pose refine only; nk ≤ that image's 3D–2D count (≤ per-image features, e.g. 10k).
    const int max_resection_pts = std::max(1024, opts.resection.max_gpu_resection_points_per_image);
    GpuResectionCtxPtr res_ctx = make_resection_ctx_shared(gpu_resection_ctx_create(max_resection_pts));

    auto count_tri_tracks = [&]() -> int { return store_out->num_triangulated_tracks(); };

    using Clock = std::chrono::steady_clock;
    auto add_ms = [](uint64_t* acc, const Clock::time_point& t0, const Clock::time_point& t1) {
        *acc += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());
    };
    uint64_t ms_resection = 0, ms_triangulation = 0, ms_outlier = 0;
    uint64_t ms_local_ba = 0, ms_global_ba = 0, ms_retriangulation = 0;

    // Re-triangulate all triangulated tracks NOT in the BA workset using new poses.
    // After GPU BA only updates workset track XYZ; non-workset tracks retain stale XYZ
    // from before BA.  With new poses but old XYZ, the next outlier pass would compute
    // large reprojection errors for those tracks and delete their observations.
    // Calling run_batch_triangulation_gpu here refreshes non-workset track XYZ so that
    // all tracks are consistent with the updated poses before the next outlier pass.
    //
    // NOTE: obs_xn_cache must already be up-to-date (i.e. call this after
    // sync_after_bundle_adjust + download_obs_norm_to_host when intrinsics changed).
    auto retri_non_workset_tracks = [&](const std::vector<int>& workset_track_ids,
                                        const char* tag) {
        if (workset_track_ids.empty()) return;
        std::unordered_set<int> wset(workset_track_ids.begin(), workset_track_ids.end());
        std::vector<int> nw_ids;
        const int n_total = store_out->num_tracks();
        for (int t = 0; t < n_total; ++t) {
            if (!store_out->is_track_valid(t)) continue;
            if (!store_out->track_has_triangulated_xyz(t)) continue;
            if (wset.count(t)) continue;
            nw_ids.push_back(t);
        }
        if (nw_ids.empty()) return;
        auto t0 = Clock::now();
        const int n_ok = run_batch_triangulation_gpu(
                store_out, gpu_scene->cuda_state(), tri_batch.get(),
                nw_ids, *poses_R_out, *poses_C_out, *registered_out,
                *cameras, image_to_camera_index,
                obs_xn_cache, obs_yn_cache,
                opts.triangulation.min_angle_deg, nullptr,
                opts.triangulation.commit_reproj_px);
        add_ms(&ms_retriangulation, t0, Clock::now());
        LOG(INFO) << "  [" << tag << "] non-workset retri: " << n_ok
                  << "/" << static_cast<int>(nw_ids.size())
                  << " tracks refreshed with new poses";
    };

    ResectionScoreCache resection_score_cache;
    std::vector<ResectionCandidate> resection_candidates;

    // Snapshot of cameras before each global BA for undistort-change detection.
    std::vector<camera::Intrinsics> cameras_ba_snapshot;

    int sfm_iter = 0;
    int no_candidate_consecutive = 0;
    constexpr int kMaxNoCandidateRetries = 2;

    auto pipeline_start = Clock::now();

    for (;;) {
        ++sfm_iter;
        LOG(INFO) << "════ [CUDA SfM] iter #" << sfm_iter
                  << ": registered=" << num_registered << "/" << n_images
                  << " tri=" << count_tri_tracks() << " ════";

        // ── GPU per-image 3D-2D count (M5) ───────────────────────────────
        // Pre-populate ResectionScoreCache with GPU-computed n_tri counts so that
        // choose_resection_candidates can use cached visibility scores for images
        // whose 3D-2D count hasn't changed (avoids per-image TrackStore traversal
        // on cache hits).
        {
            std::vector<int> gpu_n_tri;
            gpu_scene->count_tri_per_image(&gpu_n_tri);
            resection_score_cache.ensure_size(n_images);
            for (int i = 0; i < n_images; ++i) {
                // Invalidate cache for any image whose GPU n_tri differs from cached.
                // choose_resection_candidates will recompute visibility for those images.
                auto& entry = resection_score_cache.entries[static_cast<size_t>(i)];
                if (entry.n_tri != gpu_n_tri[static_cast<size_t>(i)])
                    entry.n_tri = -1; // force recompute
            }
        }

        // ── Resection candidates ──────────────────────────────────────────
        auto t_choose0 = Clock::now();
        resection_candidates = choose_resection_candidates(
                *store_out, *registered_out, *cameras, image_to_camera_index,
                opts.resection.min_3d2d_count, kResectionCandidateListMax,
                opts.resection.min_visibility_coverage,
                static_cast<size_t>(std::max(1, opts.resection.visibility_pyramid_levels)),
                &resection_score_cache);
        add_ms(&ms_resection, t_choose0, Clock::now());

        if (resection_candidates.empty()) {
            if (no_candidate_consecutive >= kMaxNoCandidateRetries) {
                LOG(INFO) << "  No resection candidates after "
                          << no_candidate_consecutive << " retries — stopping.";
                break;
            }
            ++no_candidate_consecutive;
            LOG(INFO) << "  No resection candidates (retry " << no_candidate_consecutive
                      << "/" << kMaxNoCandidateRetries
                      << "): running global BA + full-scan retriangulation";

            // Global BA (no-candidate rescue path).
            cameras_ba_snapshot = *cameras;
            auto t_gba0 = Clock::now();
            double ba_rmse_early = 0.0;
            std::vector<int> rescue_workset_track_ids;
            {
                cuda::BaSubsetOptions sub_opts;
                sub_opts.target_per_image = opts.global_ba.ba_grid_target_per_image;
                sub_opts.w_degree  = static_cast<float>(opts.global_ba.ba_grid_w_degree);
                sub_opts.w_score   = static_cast<float>(opts.global_ba.ba_grid_w_score);
                sub_opts.degree_cap = opts.global_ba.ba_grid_degree_cap;
                gpu_scene->select_ba_subset(sub_opts);

                cuda::LocalBAWorkset workset;
                if (gpu_scene->pack_for_ba(nullptr, anchor_image, &workset,
                                           gpu_scene->track_flags_ba())) {
                    const bool ba_ok = run_ba_from_workset(
                            workset, cameras, opts, num_registered, &ba_rmse_early);
                    if (ba_ok) {
                        scatter_workset_to_cpu(workset, poses_R_out, poses_C_out, store_out);
                        gpu_scene->upload_ba_result(workset);
                        rescue_workset_track_ids = workset.track_ids;
                        LOG(INFO) << "  [GPU rescue BA] rmse=" << ba_rmse_early << " px"
                                  << "  images=" << workset.n_local_images
                                  << "  workset_tracks=" << workset.n_local_tracks
                                  << "  total_tri=" << count_tri_tracks()
                                  << "  non-workset=" << (count_tri_tracks() - workset.n_local_tracks);
                    } else {
                        LOG(WARNING) << "  [GPU rescue BA] Ceres failed; poses unchanged";
                    }
                }
            }
            add_ms(&ms_global_ba, t_gba0, Clock::now());
            {
                const bool undistorted =
                        gpu_scene->sync_after_bundle_adjust(*poses_R_out, *poses_C_out, *registered_out,
                                                             *cameras, cameras_ba_snapshot);
                if (undistorted) {
                    LOG(INFO) << "  [GPU rescue BA] intrinsics changed → re-undistort, refresh obs_xn_cache";
                    gpu_scene->download_obs_norm_to_host(&obs_xn_cache, &obs_yn_cache);
                }
            }
            gpu_scene->upload_obs_valid_from_host(*store_out);

            retri_non_workset_tracks(rescue_workset_track_ids, "GPU rescue BA");

            // Full-scan retriangulation (CPU — TODO M4: replace with GpuSfMScene::retri_pending_tracks).
            auto t_retri0 = Clock::now();
            IncrementalRetriangulationOptions retri_opts;
            retri_opts.scope = RetriangulationScope::kFullScan;
            run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                *cameras, image_to_camera_index, retri_opts);
            add_ms(&ms_retriangulation, t_retri0, Clock::now());
            // TODO(M4): These H2D syncs will be removed once retriangulation is fully on GPU.
            gpu_scene->upload_tracks_from_host(*store_out);
            gpu_scene->upload_obs_valid_from_host(*store_out);
            continue;
        }
        no_candidate_consecutive = 0;

        // ── Resection (GPU via run_batch_resection – already GPU-accelerated) ──
        auto t_res0 = Clock::now();
        std::vector<int> cand_ids;
        cand_ids.reserve(resection_candidates.size());
        for (const auto& rc : resection_candidates)
            cand_ids.push_back(rc.image_index);

        std::vector<int> newly_registered_images;
        const int n_new = run_batch_resection_cuda(
                *store_out, cand_ids, *cameras, image_to_camera_index, obs_xn_cache.data(), obs_yn_cache.data(),
                gpu_scene->cuda_state(), pnp_ctx.get(), res_ctx.get(), poses_R_out, poses_C_out, registered_out,
                opts.resection.min_inliers, kPnpRansacIterations, &newly_registered_images,
                opts.resection.post_resection_reproj_thresh_px);
        add_ms(&ms_resection, t_res0, Clock::now());

        if (n_new < 0) {
            LOG(ERROR) << "[CUDA SfM] run_batch_resection_cuda failed (CUDA-only; no CPU fallback)";
            reset_cuda_pipeline_stack(tri_batch, pnp_ctx, res_ctx, gpu_scene);
            return false;
        }

        if (n_new == 0) {
            LOG(INFO) << "  All " << cand_ids.size() << " resection candidates failed.";
            // Treat as no-candidate iteration (triggers retry).
            ++no_candidate_consecutive;
            if (no_candidate_consecutive >= kMaxNoCandidateRetries)
                break;
            continue;
        }
        num_registered += n_new;
        LOG(INFO) << "  Registered " << n_new << " new images → " << num_registered
                  << "/" << n_images;

        // Sync newly-registered poses to GPU (O(n_new)).
        for (const int im : newly_registered_images)
            gpu_scene->upload_one_pose_register(im,
                                  (*poses_R_out)[static_cast<size_t>(im)],
                                  (*poses_C_out)[static_cast<size_t>(im)]);

        // ── Triangulation (GPU DLT kernel) ────────────────────────────────
        auto t_tri0 = Clock::now();
        const std::vector<int> tracks_to_tri = collect_untri_tracks(
                *store_out, newly_registered_images, *registered_out);

        std::vector<int> new_track_ids;
        int n_new_tri = run_batch_triangulation_gpu(
                store_out, gpu_scene->cuda_state(), tri_batch.get(),
                tracks_to_tri,
                *poses_R_out, *poses_C_out, *registered_out,
                *cameras, image_to_camera_index,
                obs_xn_cache, obs_yn_cache,
                opts.triangulation.min_angle_deg,
                &new_track_ids,
                opts.triangulation.commit_reproj_px);

        if (n_new_tri < 0) {
            LOG(ERROR) << "[CUDA SfM] GPU triangulation failed (CUDA-only; no CPU fallback)";
            reset_cuda_pipeline_stack(tri_batch, pnp_ctx, res_ctx, gpu_scene);
            return false;
        }
        add_ms(&ms_triangulation, t_tri0, Clock::now());
        LOG(INFO) << "  Triangulated " << n_new_tri << " new tracks (total="
                  << count_tri_tracks() << ")";

        // ── GPU outlier rejection (post-triangulation) ────────────────────
        auto t_rej0 = Clock::now();
        const int n_deleted = reject_outliers_gpu(store_out, gpu_scene.get(),
                                                  opts.outlier.threshold_px);
        add_ms(&ms_outlier, t_rej0, Clock::now());
        if (n_deleted > 0)
            LOG(INFO) << "  GPU outlier reject: deleted " << n_deleted << " obs";

        // GPU parallax-angle outlier pass (low-parallax tracks flagged as bad).
        {
            auto t_ang0 = Clock::now();
            const int n_ang = reject_angle_outliers_gpu(store_out, gpu_scene.get(),
                                                        opts.triangulation.min_angle_deg);
            add_ms(&ms_outlier, t_ang0, Clock::now());
            if (n_ang > 0)
                LOG(INFO) << "  GPU angle outlier reject: deleted " << n_ang << " obs";
        }

        // ── M4: GPU retriangulation of pending tracks ────────────────────
        // Drain tracks marked as needing retriangulation by the outlier passes.
        // These are typically a small fraction; re-triangulating immediately
        // improves track quality before the next resection step.
        {
            std::vector<int> retri_pending;
            gpu_scene->drain_retri_pending(&retri_pending);
            if (!retri_pending.empty()) {
                auto t_retri_p0 = Clock::now();
                std::vector<int> retri_new_tids;
                const int n_retri = run_batch_triangulation_gpu(
                        store_out, gpu_scene->cuda_state(), tri_batch.get(),
                        retri_pending,
                        *poses_R_out, *poses_C_out, *registered_out,
                        *cameras, image_to_camera_index,
                        obs_xn_cache, obs_yn_cache,
                        opts.triangulation.min_angle_deg,
                        &retri_new_tids,
                        opts.triangulation.commit_reproj_px);
                add_ms(&ms_retriangulation, t_retri_p0, Clock::now());
                if (n_retri > 0)
                    LOG(INFO) << "  GPU pending retri: re-triangulated " << n_retri
                              << "/" << retri_pending.size() << " tracks";
            }
        }

        // ── Determine BA strategy ─────────────────────────────────────────
        const bool do_global_ba =
                opts.global_ba.every_n_images > 0 &&
                (sfm_iter % opts.global_ba.every_n_images == 0 ||
                 num_registered < opts.global_ba.early_phase_global_only_images);

        const bool do_local_ba =
                opts.local_ba.enable &&
                num_registered > opts.local_ba.switch_after_n_images &&
                !do_global_ba;

        if (do_local_ba) {
            auto t_lba0 = Clock::now();
            bool lba_ok = false;
            const int gross_min_lba = effective_local_ba_gross_constant_cam_min_reg(opts.local_ba);
            BASolverOverrides cuda_lba_ov{};
            if (opts.local_ba.strategy == LocalBAStrategy::kBatchNeighbor) {
                double local_rmse = 0.0;
                lba_ok = run_local_ba_batch_neighbor(
                        store_out, poses_R_out, poses_C_out, *registered_out,
                        image_to_camera_index, *cameras,
                        newly_registered_images, new_track_ids,
                        opts.local_ba.neighbor_k,
                        opts.local_ba.max_iterations, &local_rmse,
                        opts.local_ba.max_observations_per_track,
                        cuda_lba_ov, num_registered,
                        opts.local_ba.constant_cam_gross_outlier_px, gross_min_lba);
            } else {
                double local_rmse = 0.0;
                lba_ok = run_local_ba_colmap(
                        store_out, poses_R_out, poses_C_out, *registered_out,
                        image_to_camera_index, *cameras,
                        newly_registered_images,
                        opts.local_ba.colmap_max_variable_images,
                        opts.local_ba.max_iterations, &local_rmse,
                        opts.local_ba.max_observations_per_track,
                        cuda_lba_ov, num_registered,
                        opts.local_ba.constant_cam_gross_outlier_px, gross_min_lba);
            }
            add_ms(&ms_local_ba, t_lba0, Clock::now());
            // Sync poses to GPU (intrinsics not changed by local BA).
            gpu_scene->upload_poses_from_host(*poses_R_out, *poses_C_out, *registered_out);

            if (lba_ok && opts.triangulation.enable_post_local_ba_retriangulation) {
                auto t_retri_l0 = Clock::now();
                IncrementalRetriangulationOptions retri_opts;
                retri_opts.scope = RetriangulationScope::kNewImages;
                retri_opts.restrict_image_indices = newly_registered_images;
                run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                    *cameras, image_to_camera_index, retri_opts);
                add_ms(&ms_retriangulation, t_retri_l0, Clock::now());
                gpu_scene->upload_tracks_from_host(*store_out);
                gpu_scene->upload_obs_valid_from_host(*store_out);
            }
        }

        if (do_global_ba) {
            cameras_ba_snapshot = *cameras;
            auto t_gba0 = Clock::now();
            double ba_rmse_glob = 0.0;
            std::vector<int> ba_workset_track_ids; // saved to drive non-workset retri after sync
            // GPU-first BA: GPU grid-NMS subset → GPU→host pack → Ceres → scatter back.
            {
                cuda::BaSubsetOptions sub_opts;
                sub_opts.target_per_image = opts.global_ba.ba_grid_target_per_image;
                sub_opts.w_degree  = static_cast<float>(opts.global_ba.ba_grid_w_degree);
                sub_opts.w_score   = static_cast<float>(opts.global_ba.ba_grid_w_score);
                sub_opts.degree_cap = opts.global_ba.ba_grid_degree_cap;
                gpu_scene->select_ba_subset(sub_opts);

                cuda::LocalBAWorkset workset;
                if (gpu_scene->pack_for_ba(nullptr, anchor_image, &workset,
                                           gpu_scene->track_flags_ba())) {
                    const bool ba_ok = run_ba_from_workset(workset, cameras, opts, num_registered, &ba_rmse_glob);
                    if (ba_ok) {
                        scatter_workset_to_cpu(workset, poses_R_out, poses_C_out, store_out);
                        gpu_scene->upload_ba_result(workset);
                        ba_workset_track_ids = workset.track_ids;
                        LOG(INFO) << "  [GPU BA] rmse=" << ba_rmse_glob << " px"
                                  << "  images=" << workset.n_local_images
                                  << "  workset_tracks=" << workset.n_local_tracks
                                  << "  total_tri=" << count_tri_tracks()
                                  << "  non-workset=" << (count_tri_tracks() - workset.n_local_tracks);
                    } else {
                        LOG(WARNING) << "  [GPU BA] Ceres failed; poses unchanged";
                    }
                } else {
                    LOG(WARNING) << "  [GPU BA] pack_for_ba returned empty workset";
                }
            }
            add_ms(&ms_global_ba, t_gba0, Clock::now());

            const bool undistorted =
                gpu_scene->sync_after_bundle_adjust(*poses_R_out, *poses_C_out, *registered_out, *cameras,
                                       cameras_ba_snapshot);
            if (undistorted) {
                LOG(INFO) << "  [GPU BA] intrinsics changed → re-undistort, refresh obs_xn_cache";
                gpu_scene->download_obs_norm_to_host(&obs_xn_cache, &obs_yn_cache);
            }
            gpu_scene->upload_obs_valid_from_host(*store_out);

            // Refresh XYZ for triangulated tracks NOT in BA workset.
            // After BA, poses changed but non-workset track XYZ is stale (upload_ba_result only
            // updates workset tracks).  Without this step, the next outlier pass computes large
            // reprojection errors for non-workset tracks and deletes their observations.
            retri_non_workset_tracks(ba_workset_track_ids, "GPU BA");

            // Post-global-BA retriangulation (CPU: adds newly-triangulable tracks).
            if (opts.triangulation.enable_full_scan_after_global_ba) {
                auto t_pfs0 = Clock::now();
                IncrementalRetriangulationOptions retri_opts;
                retri_opts.scope = RetriangulationScope::kFullScan;
                run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                    *cameras, image_to_camera_index, retri_opts);
                add_ms(&ms_retriangulation, t_pfs0, Clock::now());
                gpu_scene->upload_tracks_from_host(*store_out);
                gpu_scene->upload_obs_valid_from_host(*store_out);
            }
        }

        // ── Periodic global BA (CUDA): legacy fixed step only (n % periodic_every_n_images).
        // CPU incremental pipeline uses linear spacing; CUDA path is not production-maintained.
        if (opts.global_ba.periodic_every_n_images > 0 &&
            num_registered % opts.global_ba.periodic_every_n_images == 0) {
            cameras_ba_snapshot = *cameras;
            auto t_pgba0 = Clock::now();
            double ba_rmse_periodic = 0.0;
            std::vector<int> periodic_workset_track_ids;
            {
                cuda::BaSubsetOptions sub_opts;
                sub_opts.target_per_image = opts.global_ba.ba_grid_target_per_image;
                sub_opts.w_degree  = static_cast<float>(opts.global_ba.ba_grid_w_degree);
                sub_opts.w_score   = static_cast<float>(opts.global_ba.ba_grid_w_score);
                sub_opts.degree_cap = opts.global_ba.ba_grid_degree_cap;
                gpu_scene->select_ba_subset(sub_opts);

                cuda::LocalBAWorkset workset;
                if (gpu_scene->pack_for_ba(nullptr, anchor_image, &workset,
                                           gpu_scene->track_flags_ba())) {
                    const bool ba_ok = run_ba_from_workset(
                            workset, cameras, opts, num_registered, &ba_rmse_periodic);
                    if (ba_ok) {
                        scatter_workset_to_cpu(workset, poses_R_out, poses_C_out, store_out);
                        gpu_scene->upload_ba_result(workset);
                        periodic_workset_track_ids = workset.track_ids;
                        LOG(INFO) << "  [GPU periodic BA] rmse=" << ba_rmse_periodic << " px"
                                  << "  images=" << workset.n_local_images
                                  << "  workset_tracks=" << workset.n_local_tracks
                                  << "  total_tri=" << count_tri_tracks()
                                  << "  non-workset=" << (count_tri_tracks() - workset.n_local_tracks);
                    } else {
                        LOG(WARNING) << "  [GPU periodic BA] Ceres failed; poses unchanged";
                    }
                } else {
                    LOG(WARNING) << "  [GPU periodic BA] pack_for_ba returned empty workset";
                }
            }
            add_ms(&ms_global_ba, t_pgba0, Clock::now());

            const bool undistorted =
                gpu_scene->sync_after_bundle_adjust(*poses_R_out, *poses_C_out, *registered_out, *cameras,
                                       cameras_ba_snapshot);
            if (undistorted) {
                LOG(INFO) << "  [GPU periodic BA] intrinsics changed → re-undistort, refresh obs_xn_cache";
                gpu_scene->download_obs_norm_to_host(&obs_xn_cache, &obs_yn_cache);
            }
            gpu_scene->upload_obs_valid_from_host(*store_out);

            retri_non_workset_tracks(periodic_workset_track_ids, "GPU periodic BA");

            auto t_retri_p0 = Clock::now();
            IncrementalRetriangulationOptions retri_opts;
            retri_opts.scope = RetriangulationScope::kFullScan;
            run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                                *cameras, image_to_camera_index, retri_opts);
            add_ms(&ms_retriangulation, t_retri_p0, Clock::now());
            gpu_scene->upload_tracks_from_host(*store_out);
            gpu_scene->upload_obs_valid_from_host(*store_out);
        }

        // ── Debug snapshot hook ───────────────────────────────────────────
        if (opts.debug.on_snapshot && sfm_iter % opts.debug.snapshot_every_n_iters == 0) {
            opts.debug.on_snapshot(sfm_iter, num_registered,
                                   *poses_R_out, *poses_C_out, *registered_out, *store_out);
        }
    } // for(;;) main SfM loop

    // ── Pre-final BA retriangulation ──────────────────────────────────────
    {
        IncrementalRetriangulationOptions retri_opts;
        retri_opts.scope = RetriangulationScope::kFullScan;
        run_retriangulation(store_out, *poses_R_out, *poses_C_out, *registered_out,
                            *cameras, image_to_camera_index, retri_opts);
        LOG(INFO) << "[CUDA SfM] Pre-final-BA retriangulation: tri="
                  << count_tri_tracks();
    }

    // ── Final BA ──────────────────────────────────────────────────────────
    cameras_ba_snapshot = *cameras;
    double ba_rmse_final = 0.0;
    std::vector<int> final_workset_track_ids;
    {
        cuda::BaSubsetOptions sub_opts;
        sub_opts.target_per_image = opts.global_ba.ba_grid_target_per_image;
        sub_opts.w_degree  = static_cast<float>(opts.global_ba.ba_grid_w_degree);
        sub_opts.w_score   = static_cast<float>(opts.global_ba.ba_grid_w_score);
        sub_opts.degree_cap = opts.global_ba.ba_grid_degree_cap;
        gpu_scene->select_ba_subset(sub_opts);

        cuda::LocalBAWorkset workset;
        if (gpu_scene->pack_for_ba(nullptr, anchor_image, &workset,
                                   gpu_scene->track_flags_ba())) {
            const bool ba_ok = run_ba_from_workset(workset, cameras, opts, num_registered, &ba_rmse_final);
            if (ba_ok) {
                scatter_workset_to_cpu(workset, poses_R_out, poses_C_out, store_out);
                gpu_scene->upload_ba_result(workset);
                final_workset_track_ids = workset.track_ids;
                LOG(INFO) << "[CUDA SfM] Final GPU BA rmse=" << ba_rmse_final << " px"
                          << "  images=" << workset.n_local_images
                          << "  workset_tracks=" << workset.n_local_tracks
                          << "  total_tri=" << count_tri_tracks()
                          << "  non-workset=" << (count_tri_tracks() - workset.n_local_tracks);
            } else {
                LOG(WARNING) << "[CUDA SfM] Final GPU BA Ceres failed; poses unchanged";
            }
        } else {
            LOG(WARNING) << "[CUDA SfM] Final GPU BA pack_for_ba returned empty workset";
        }
    }
    {
        const bool undistorted = gpu_scene->sync_after_bundle_adjust(
                *poses_R_out, *poses_C_out, *registered_out, *cameras, cameras_ba_snapshot);
        if (undistorted) {
            LOG(INFO) << "[CUDA SfM] Final BA: intrinsics changed → re-undistort, refresh obs_xn_cache";
            gpu_scene->download_obs_norm_to_host(&obs_xn_cache, &obs_yn_cache);
        }
    }
    retri_non_workset_tracks(final_workset_track_ids, "final GPU BA");

    // ── Final GPU outlier rejection ───────────────────────────────────────
    {
        const double tight_px = opts.outlier.threshold_px * 0.5;
        const int n_final_del = reject_outliers_gpu(store_out, gpu_scene.get(), tight_px);
        if (n_final_del > 0)
            LOG(INFO) << "[CUDA SfM] Final GPU outlier pass: deleted " << n_final_del << " obs";
        // Final angle pass with the tight threshold.
        const int n_final_ang = reject_angle_outliers_gpu(store_out, gpu_scene.get(),
                                                        opts.triangulation.min_angle_deg);
        if (n_final_ang > 0)
            LOG(INFO) << "[CUDA SfM] Final GPU angle outlier pass: deleted "
                      << n_final_ang << " obs";
        // Drain any retri-pending marks left by the final outlier passes.
        // No retriangulation is done at this stage; just reset the flags so they
        // don't linger as stale state if the GPU scene object is reused.
        std::vector<int> _drain;
        gpu_scene->drain_retri_pending(&_drain);
    }

    // GPU resources released by shared_ptr destructors (gpu_scene, tri_batch, …).

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                Clock::now() - pipeline_start).count();
    LOG(INFO) << "[CUDA SfM] Done. registered=" << num_registered
              << "/" << n_images
              << "  tri=" << count_tri_tracks()
              << "  total=" << elapsed_ms << "ms"
              << "  resection=" << ms_resection
              << "  triangulation=" << ms_triangulation
              << "  outlier=" << ms_outlier
              << "  local_ba=" << ms_local_ba
              << "  global_ba=" << ms_global_ba
              << "  retriangulation=" << ms_retriangulation;

    return true;
}

} // namespace sfm
} // namespace insight
