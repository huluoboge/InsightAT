/**
 * @file  resection_batch.cpp
 * @brief Batch resection candidate selection and run_batch_resection (see resection_batch.h).
 */

#include "resection_batch.h"

#include "resection.h"
#include "visibility_pyramid.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
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

/// Build the VisibilityPyramid score from pre-fetched track_ids + obs.
/// Returns normalized score [0,1] and sets *n_tri_out to the number of triangulated points used.
static float build_visibility_score(const TrackStore& store,
                                    const std::vector<int>& track_ids,
                                    const std::vector<Observation>& obs,
                                    const camera::Intrinsics& K, size_t num_levels,
                                    int* n_tri_out) {
  *n_tri_out = 0;
  size_t W = 0, H = 0;
  image_size_for_pyramid(K, &W, &H);
  if (W == 0 || H == 0 || num_levels == 0)
    return 0.0f;

  VisibilityPyramid pyr(num_levels, W, H);
  for (size_t i = 0; i < track_ids.size(); ++i) {
    if (!store.track_has_triangulated_xyz(track_ids[i]))
      continue;
    pyr.SetPoint(static_cast<double>(obs[i].u), static_cast<double>(obs[i].v));
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
  int image_index = -1;
};

} // namespace

std::vector<ResectionCandidate> choose_resection_candidates(
    const TrackStore& store, const std::vector<bool>& registered,
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

  int cache_hits = 0, cache_misses = 0;
  // One get_image_track_observations call per image (shared for both n_tri count and pyramid).
  std::vector<int> track_ids;
  std::vector<Observation> obs;
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)])
      continue;

    track_ids.clear();
    obs.clear();
    store.get_image_track_observations(im, &track_ids, &obs);

    // Count n_tri (cheap linear scan — no Eigen allocation).
    int n_tri = 0;
    for (int tid : track_ids)
      if (store.track_has_triangulated_xyz(tid))
        ++n_tri;
    if (n_tri < min_3d2d_count) 
      continue;

    ScoredUnreg row;
    row.count = n_tri;
    row.image_index = im;

    const int cam_idx = image_to_camera_index[static_cast<size_t>(im)];
    if (cam_idx < 0 || cam_idx >= static_cast<int>(cameras.size())) {
      row.coverage = 0.f;
      scored.push_back(row);
      continue;
    }

    // Cache lookup: reuse score when n_tri has not changed since last computation.
    if (score_cache) {
      auto& entry = score_cache->entries[static_cast<size_t>(im)];
      if (entry.n_tri == n_tri) {
        row.coverage = entry.score;
        ++cache_hits;
        scored.push_back(row);
        continue;
      }
    }

    // Cache miss (or no cache): build pyramid. obs already fetched above.
    int n_tri_check = 0;
    row.coverage = build_visibility_score(store, track_ids, obs,
                                          cameras[static_cast<size_t>(cam_idx)],
                                          n_levels, &n_tri_check);
    if (score_cache) {
      score_cache->entries[static_cast<size_t>(im)] = {n_tri, row.coverage};
      ++cache_misses;
    }
    scored.push_back(row);
  }
  if (scored.empty()) 
    return {};

  // Weighted score: coverage dominates (0~1), count provides a log-scale boost (0~0.1).
  // This keeps coverage as the primary signal while letting count break near-ties.
  auto resection_score = [](const ScoredUnreg& s) -> float {
    return s.coverage +
           0.1f * std::log1p(static_cast<float>(s.count)) / std::log1p(1000.f);
  };
  std::sort(scored.begin(), scored.end(),
            [&resection_score, min_coverage_good](const ScoredUnreg& a, const ScoredUnreg& b) {
              // Images meeting the coverage threshold always come first.
              const bool ga = a.coverage >= min_coverage_good;
              const bool gb = b.coverage >= min_coverage_good;
              if (ga != gb)
                return ga > gb;
              // Within the same group, rank by weighted score.
              const float sa = resection_score(a);
              const float sb = resection_score(b);
              if (sa != sb)
                return sa > sb;
              return a.image_index < b.image_index;
            });

  const int n_list = std::min(static_cast<int>(scored.size()), max_candidates);
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
   if (out.size() > 1) {
    for (int i = 1; i < out.size(); ++i) {
      if (out[i].num_3d2d < 100) {
        out.resize(i);
        break;
      }
    }
  }
  // NOTE: The old hard-coded `num_3d2d < 100` truncation was removed.
  // It incorrectly dropped valid late-stage candidates that met min_3d2d_count but had
  // fewer than 100 triangulated correspondences — a situation that commonly arises when
  // the triangulation pipeline has lagged (observation deletions from local-BA outlier
  // rejection temporarily starve some images of 3D-2D pairs).  The minimum is already
  // enforced by the `if (n_tri < min_3d2d_count) continue;` guard above.
  if (score_cache) {
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
    std::vector<int> track_ids_for_log;
    store.get_image_track_observations(im, &track_ids_for_log, nullptr);
    int n_3d2d = 0;
    for (int tid : track_ids_for_log) {
      if (store.track_has_triangulated_xyz(tid))
        ++n_3d2d;
    }
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
