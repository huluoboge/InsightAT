/**
 * @file  resection_batch.cpp
 * @brief Batch resection candidate selection and run_batch_resection (see resection_batch.h).
 */

#include "resection_batch.h"

#include "resection.h"
#include "visibility_pyramid.h"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>  // 添加chrono头文件以支持时间测量
#include <cmath>
#include <glog/logging.h>
#include <omp.h>   // 添加OpenMP头文件
#include <vector>

namespace insight {
namespace sfm {

namespace {

/// Absolute image size for pyramid (pixels). Prefer Intrinsics::width/height from project export.
static void image_size_for_pyramid(const camera::Intrinsics& K, size_t* out_w, size_t* out_h) {
  if (K.has_image_size()) {
    *out_w = static_cast<size_t>(K.width);
    *out_h = static_cast<size_t>(K.height);
    return;
  }
  // Fallback: assume principal point near image centre (common for estimated intrinsics).
  const double w = std::max(2.0 * K.cx, 1.0);
  const double h = std::max(2.0 * K.cy, 1.0);
  *out_w = static_cast<size_t>(std::ceil(w));
  *out_h = static_cast<size_t>(std::ceil(h));
  VLOG(2) << "image_size_for_pyramid: missing width/height; using ceil(2*cx) x ceil(2*cy) = "
          << *out_w << " x " << *out_h;
}

/// Build the VisibilityPyramid score from structural image observation ids.
/// Returns normalized score [0,1] and sets *n_tri_out to the number of triangulated points used.
static float build_visibility_score_from_obs_ids(const TrackStore& store,
                                                 const std::vector<int>& obs_ids,
                                                 const camera::Intrinsics& K,
                                                 size_t num_levels, int* n_tri_out) {
  *n_tri_out = 0;
  size_t W = 0, H = 0;
  image_size_for_pyramid(K, &W, &H);
  if (W == 0 || H == 0 || num_levels == 0)
    return 0.0f;

  VisibilityPyramid pyr(num_levels, W, H);
  for (int obs_id : obs_ids) {
    if (!store.is_obs_valid(obs_id))
      continue;
    const int track_id = store.obs_track_id(obs_id);
    if (!store.track_has_triangulated_xyz(track_id))
      continue;
    pyr.SetPoint(static_cast<double>(store.obs_u(obs_id)),
                 static_cast<double>(store.obs_v(obs_id)));
    ++(*n_tri_out);
  }
  if (*n_tri_out < 4)
    return 0.0f;
  const size_t max_s = pyr.MaxScore();
  if (max_s == 0)
    return 0.0f;
  return static_cast<float>(static_cast<double>(pyr.Score()) / static_cast<double>(max_s));
}

struct ScoredUnreg {
  int count = 0;
  float coverage = 0.f;
  float rank_score = 0.f;
  int image_index = -1;
};

static int count_triangulated_image_observations(const TrackStore& store,
                                                 const std::vector<int>& obs_ids) {
  int n_tri = 0;
  for (int obs_id : obs_ids) {
    if (!store.is_obs_valid(obs_id))
      continue;
    if (store.track_has_triangulated_xyz(store.obs_track_id(obs_id)))
      ++n_tri;
  }
  return n_tri;
}

static float compute_resection_rank_score(int count, float coverage) {
  return coverage + 0.1f * std::log1p(static_cast<float>(count)) / std::log1p(1000.f);
}

static bool resection_candidate_better(const ScoredUnreg& a, const ScoredUnreg& b,
                                       float min_coverage_good) {
  const bool ga = a.coverage >= min_coverage_good;
  const bool gb = b.coverage >= min_coverage_good;
  if (ga != gb)
    return ga > gb;
  if (a.rank_score != b.rank_score)
    return a.rank_score > b.rank_score;
  return a.image_index < b.image_index;
}

} // namespace

std::vector<ResectionCandidate> choose_resection_candidates(
  TrackStore& store, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    int min_3d2d_count, int max_candidates, float min_coverage_good,
    size_t visibility_pyramid_levels, ResectionScoreCache* score_cache) {
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || max_candidates <= 0)
    return {};
  if (static_cast<int>(image_to_camera_index.size()) != n_images || cameras.empty()) {
    LOG(WARNING) << "choose_resection_candidates: cameras / image_to_camera_index mismatch";
    return {};
  }
  const size_t n_levels = std::max<size_t>(1, visibility_pyramid_levels);

  if (score_cache)
    score_cache->ensure_size(n_images);

  int n_registed = 0;
  std::vector<ScoredUnreg> scored;
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)])
      ++n_registed;
  }
  // cap candidates to 30% of registered views, but at least 2
  int max_count = std::max<int>(2, static_cast<int>(n_registed * 0.3));
  max_candidates = std::min(max_candidates, max_count);

  if (score_cache && score_cache->last_obs_epoch == store.obs_epoch() &&
      score_cache->last_tri_status_epoch == store.tri_status_epoch() &&
      score_cache->last_registration_epoch == store.registration_epoch() &&
      score_cache->last_registered_count == n_registed &&
      score_cache->last_min_3d2d_count == min_3d2d_count &&
      score_cache->last_max_candidates == max_candidates &&
      score_cache->last_min_coverage_good == min_coverage_good &&
      score_cache->last_visibility_pyramid_levels == n_levels) {
    VLOG(1) << "choose_resection_candidates: snapshot cache hit → "
            << score_cache->cached_candidates.size() << " candidates";
    return score_cache->cached_candidates;
  }

  int cache_hits = 0, cache_misses = 0;
  bool full_refresh = (score_cache == nullptr);
  std::vector<int> dirty_images;
  using Clock = std::chrono::steady_clock;
  auto t_choose_start = Clock::now();
  if (score_cache) {
    const bool cache_uninitialized =
        score_cache->last_obs_epoch == std::numeric_limits<uint64_t>::max() ||
        score_cache->last_tri_status_epoch == std::numeric_limits<uint64_t>::max() ||
        score_cache->last_registration_epoch == std::numeric_limits<uint64_t>::max();
    const bool pyramid_layout_changed =
        score_cache->last_visibility_pyramid_levels != 0 &&
        score_cache->last_visibility_pyramid_levels != n_levels;
    full_refresh = cache_uninitialized || pyramid_layout_changed;
    if (!full_refresh &&
        (score_cache->last_obs_epoch != store.obs_epoch() ||
         score_cache->last_tri_status_epoch != store.tri_status_epoch() ||
         score_cache->last_registration_epoch != store.registration_epoch())) {
      store.consume_dirty_images(&dirty_images);
      if (dirty_images.empty()) {
        full_refresh = true;
      }
    }
  }
  VLOG(1) << "[choose_diag] full_refresh=" << full_refresh
          << "  n_dirty=" << dirty_images.size()
          << "  n_reg=" << n_registed << "/"  << n_images
          << "  tri_epoch=" << store.tri_status_epoch();

  // Thresholds for pyramid rebuild shortcuts.
  // kSaturationThreshold: n_tri >= this → every pyramid cell is statistically
  //   occupied → coverage = 1.0 guaranteed.
  //   For n_levels=6: 4 × (1<<12) = 16384.  For n_levels=4: 4 × 256 = 1024.
  const int kSaturationThreshold = 4 * static_cast<int>(1u << (2u * n_levels));
  // kStableBaseline: previous n_tri must have been at least this many for
  //   the "reuse old coverage" fast path to be safe (avoids reusing a stale
  //   near-zero score from the initial cold fill).
  const int kStableBaseline = std::max(min_3d2d_count * 4, 100);

  int n_pyramid_builds = 0, n_pyramid_sat = 0, n_pyramid_reuse = 0;
  auto refresh_entry = [&](int im) {
    // Registered images are never resection candidates; skip expensive pyramid build.
    if (registered[static_cast<size_t>(im)]) {
      score_cache->entries[static_cast<size_t>(im)] = {0, 0.0f};
      ++cache_misses;
      return;
    }
    // O(1) n_tri lookup from incremental counter; pyramid only when above threshold.
    const int n_tri = store.image_tri_count(im);
    float coverage = 0.0f;
    if (n_tri >= min_3d2d_count) {
      const int cam_idx = image_to_camera_index[static_cast<size_t>(im)];
      if (cam_idx >= 0 && cam_idx < static_cast<int>(cameras.size())) {
        const auto& prev = score_cache->entries[static_cast<size_t>(im)];
        // Fast-path A: guaranteed saturation → coverage = 1.0.
        if (n_tri >= kSaturationThreshold) {
          coverage = 1.0f;
          ++n_pyramid_sat;
        // Fast-path B: n_tri increased modestly from a stable baseline.
        // Monotone property: true_coverage >= prev_score (obs only added).
        // Skip rebuild when count grew by < 2× so we don't miss newly-popular images.
        } else if (prev.score > 0.0f &&
                   n_tri >= prev.n_tri &&
                   n_tri <= prev.n_tri * 2 + 100 &&
                   prev.n_tri >= kStableBaseline) {
          coverage = prev.score; // conservative: true coverage ≥ this estimate
          ++n_pyramid_reuse;
        } else {
          // Full pyramid rebuild.
          int n_tri_check = 0;
          coverage = build_visibility_score_from_obs_ids(
              store, store.image_all_obs_ids_view(im),
              cameras[static_cast<size_t>(cam_idx)], n_levels, &n_tri_check);
          ++n_pyramid_builds;
        }
      }
    }
    score_cache->entries[static_cast<size_t>(im)] = {n_tri, coverage};
    ++cache_misses;
  };

  auto t_refresh_start = Clock::now();
  if (score_cache) {
    if (full_refresh) {
      for (int im = 0; im < n_images; ++im)
        refresh_entry(im);
    } else {
      for (int im : dirty_images) {
        if (im < 0 || im >= n_images)
          continue;
        refresh_entry(im);
      }
    }
  }
  auto t_refresh_end = Clock::now();
  VLOG(1) << "[choose_diag] refresh="
          << std::chrono::duration_cast<std::chrono::milliseconds>(t_refresh_end - t_refresh_start).count()
          << "ms  n_pyramid_builds=" << n_pyramid_builds
          << "  n_pyramid_sat=" << n_pyramid_sat
          << "  n_pyramid_reuse=" << n_pyramid_reuse;

  auto t_score_start = Clock::now();
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)])
      continue;

    int n_tri = 0;
    float coverage = 0.0f;
    if (score_cache) {
      const auto& entry = score_cache->entries[static_cast<size_t>(im)];
      n_tri = entry.n_tri;
      coverage = entry.score;
      if (!full_refresh && std::find(dirty_images.begin(), dirty_images.end(), im) == dirty_images.end())
        ++cache_hits;
    } else {
      // No cache: read O(1) count, build pyramid only if above threshold.
      n_tri = store.image_tri_count(im);
      const int cam_idx = image_to_camera_index[static_cast<size_t>(im)];
      if (cam_idx >= 0 && cam_idx < static_cast<int>(cameras.size()) && n_tri >= min_3d2d_count) {
        if (n_tri >= kSaturationThreshold) {
          coverage = 1.0f;
        } else {
          int n_tri_check = 0;
          coverage = build_visibility_score_from_obs_ids(
              store, store.image_all_obs_ids_view(im),
              cameras[static_cast<size_t>(cam_idx)], n_levels, &n_tri_check);
        }      }
    }

    if (n_tri < min_3d2d_count)
      continue;

    ScoredUnreg row;
    row.count = n_tri;
    row.coverage = coverage;
    row.rank_score = compute_resection_rank_score(row.count, row.coverage);
    row.image_index = im;
    scored.push_back(row);
  }
  auto t_score_end = Clock::now();
  VLOG(1) << "[choose_diag] score_loop="
          << std::chrono::duration_cast<std::chrono::milliseconds>(t_score_end - t_score_start).count()
          << "ms  eligible=" << scored.size()
          << "  score_find_ms="
          << std::chrono::duration_cast<std::chrono::milliseconds>(t_score_end - t_refresh_end).count() << "ms";
  if (scored.empty()) {
    if (score_cache) {
      score_cache->cached_candidates.clear();
      score_cache->last_obs_epoch = store.obs_epoch();
      score_cache->last_tri_status_epoch = store.tri_status_epoch();
      score_cache->last_registration_epoch = store.registration_epoch();
      score_cache->last_registered_count = n_registed;
      score_cache->last_min_3d2d_count = min_3d2d_count;
      score_cache->last_max_candidates = max_candidates;
      score_cache->last_min_coverage_good = min_coverage_good;
      score_cache->last_visibility_pyramid_levels = n_levels;
    }
    return {};
  }

  const int n_list = std::min(static_cast<int>(scored.size()), max_candidates);
  auto better = [min_coverage_good](const ScoredUnreg& a, const ScoredUnreg& b) {
    return resection_candidate_better(a, b, min_coverage_good);
  };
  if (static_cast<int>(scored.size()) > n_list) {
    std::partial_sort(scored.begin(), scored.begin() + n_list, scored.end(), better);
  } else {
    std::sort(scored.begin(), scored.end(), better);
  }

  std::vector<ResectionCandidate> out;
  out.reserve(static_cast<size_t>(n_list));
  for (int i = 0; i < n_list; ++i) {
    const auto& p = scored[static_cast<size_t>(i)];
    ResectionCandidate c;
    c.image_index = p.image_index;
    c.num_3d2d = p.count;
    c.coverage = p.coverage;
    out.push_back(c);
  }
  // NOTE: the old hard-coded `num_3d2d < 100` list truncation has been removed.
  // It incorrectly dropped valid late-stage candidates that met min_3d2d_count but had
  // fewer than 100 triangulated correspondences — a situation that commonly arises when
  // the triangulation pipeline has lagged (observation deletions from local-BA outlier
  // rejection temporarily starve some images of 3D-2D pairs).  The minimum is already
  // enforced by the `if (n_tri < min_3d2d_count) continue;` guard above.
  if (score_cache) {
    score_cache->cached_candidates = out;
    score_cache->last_obs_epoch = store.obs_epoch();
    score_cache->last_tri_status_epoch = store.tri_status_epoch();
    score_cache->last_registration_epoch = store.registration_epoch();
    score_cache->last_registered_count = n_registed;
    score_cache->last_min_3d2d_count = min_3d2d_count;
    score_cache->last_max_candidates = max_candidates;
    score_cache->last_min_coverage_good = min_coverage_good;
    score_cache->last_visibility_pyramid_levels = n_levels;
    VLOG(1) << "choose_resection_candidates: " << scored.size() << " eligible → " << n_list
            << "  pyramid_cache: hits=" << cache_hits << " misses=" << cache_misses
            << "  (L=" << n_levels << ", cov_thresh=" << min_coverage_good << ")";
  } else {
    VLOG(1) << "choose_resection_candidates: " << scored.size() << " eligible → " << n_list
            << "  (VisibilityPyramid L=" << n_levels
            << ", sort: cov>=" << min_coverage_good << " first, then 3d2d desc, cov desc)";
  }
  return out;
}

int run_batch_resection(TrackStore& store, const std::vector<int>& image_indices,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        std::vector<Eigen::Matrix3d>* poses_R,
                        std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                        int min_inliers, std::vector<int>* registered_images_out,
                        double post_resection_reproj_thresh_px) {
  if (!poses_R || !poses_C || !registered)
    return 0;
  if (registered_images_out)
    registered_images_out->clear();
  int added = 0;
  for (int im : image_indices) {
    if (im < 0 || static_cast<size_t>(im) >= image_to_camera_index.size())
      continue;
    const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
    const int n_3d2d = store.image_tri_count(im);
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    int inliers = 0;
    double rmse_px = 0.0;
    const double ransac_thresh_px =
        4.0; // tighter threshold since we have good initialization from tracks
    if (!resection_single_image(K, store, im, &R, &t, min_inliers, ransac_thresh_px, &inliers,
                                &rmse_px)) {
      LOG(INFO) << "  resection image " << im << ": FAILED (3D-2D=" << n_3d2d
                << ", inliers=" << inliers << ", rmse=" << rmse_px << ", need " << min_inliers
                << ")";
      continue;
    }
    Eigen::Vector3d C = -R.transpose() * t;
    if (post_resection_reproj_thresh_px > 0.0) {
      const int n_pr = prune_resection_observations_reprojection(&store, im, R, C, K,
                                                                 post_resection_reproj_thresh_px);
      if (n_pr > 0)
        VLOG(1) << "  post_resection_reproj: image " << im << " pruned " << n_pr
                << " obs (thr=" << post_resection_reproj_thresh_px << " px)";
    }
    LOG(INFO) << "  resection image " << im << ": OK (3D-2D=" << n_3d2d << ", inliers=" << inliers
              << ", rmse=" << rmse_px << ", C=[" << C(0) << "," << C(1) << "," << C(2) << "])";
    if (static_cast<size_t>(im) >= poses_R->size())
      poses_R->resize(static_cast<size_t>(im) + 1);
    if (static_cast<size_t>(im) >= poses_C->size())
      poses_C->resize(static_cast<size_t>(im) + 1);
    if (static_cast<size_t>(im) >= registered->size())
      registered->resize(static_cast<size_t>(im) + 1);
    (*poses_R)[static_cast<size_t>(im)] = R;
    (*poses_C)[static_cast<size_t>(im)] = C;
    (*registered)[static_cast<size_t>(im)] = true;
    if (registered_images_out)
      registered_images_out->push_back(im);
    ++added;
  }
  LOG(INFO) << "run_batch_resection: " << added << "/" << image_indices.size()
            << " images registered";
  return added;
}

} // namespace sfm
} // namespace insight
