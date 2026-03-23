/**
 * @file  resection_batch.cpp
 * @brief Batch resection candidate selection and run_batch_resection (see resection_batch.h).
 */

#include "resection_batch.h"

#include "resection.h"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <glog/logging.h>
#include <vector>

namespace insight {
namespace sfm {

namespace {

/// Compute the spatial coverage score [0,1] for the triangulated 3D-2D observations
/// of a given image.  The score is the fraction of a 3×3 bounding-box grid that
/// contains ≥1 observation.  Low values (< 0.33) indicate collinear/clustered
/// geometry that is likely to produce a degenerate PnP solution.
float compute_3d2d_coverage(const TrackStore& store, int image_index) {
  std::vector<int> track_ids;
  std::vector<Observation> obs;
  store.get_image_track_observations(image_index, &track_ids, &obs);

  float min_u = 1e9f, max_u = -1e9f, min_v = 1e9f, max_v = -1e9f;
  int n_tri = 0;
  for (size_t i = 0; i < track_ids.size(); ++i) {
    if (!store.track_has_triangulated_xyz(track_ids[i]))
      continue;
    min_u = std::min(min_u, obs[i].u);
    max_u = std::max(max_u, obs[i].u);
    min_v = std::min(min_v, obs[i].v);
    max_v = std::max(max_v, obs[i].v);
    ++n_tri;
  }
  if (n_tri < 4)
    return 0.0f;

  const float ru = max_u - min_u + 1.0f;
  const float rv = max_v - min_v + 1.0f;
  std::array<bool, 9> cells{};
  for (size_t i = 0; i < track_ids.size(); ++i) {
    if (!store.track_has_triangulated_xyz(track_ids[i]))
      continue;
    const int cx = std::min(2, static_cast<int>(3.0f * (obs[i].u - min_u) / ru));
    const int cy = std::min(2, static_cast<int>(3.0f * (obs[i].v - min_v) / rv));
    cells[static_cast<size_t>(cy * 3 + cx)] = true;
  }
  return static_cast<float>(std::count(cells.cbegin(), cells.cend(), true)) / 9.0f;
}

struct ScoredUnreg {
  int count = 0;
  float coverage = 0.f;
  int image_index = -1;
};

} // namespace

std::vector<ResectionCandidate> choose_resection_candidates(const TrackStore& store,
                                                            const std::vector<bool>& registered,
                                                            int min_3d2d_count, int max_candidates,
                                                            float min_coverage_good) {
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered.size()) != n_images || max_candidates <= 0)
    return {};

  int n_registed = 0;
  std::vector<ScoredUnreg> scored;
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)]) {
      ++n_registed;
    }
  }
  // cap candidates to 30% of registered views, but at least 2
  int max_count = std::max<int>(2, static_cast<int>(n_registed * 0.3));
  max_candidates = std::min(max_candidates, max_count);
  for (int im = 0; im < n_images; ++im) {
    if (registered[static_cast<size_t>(im)])
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int count = 0;
    for (int tid : track_ids)
      if (store.track_has_triangulated_xyz(tid))
        ++count;
    if (count < min_3d2d_count)
      continue;
    ScoredUnreg row;
    row.count = count;
    row.image_index = im;
    row.coverage = compute_3d2d_coverage(store, im);
    scored.push_back(row);
  }
  if (scored.empty())
    return {};

  std::sort(scored.begin(), scored.end(),
            [min_coverage_good](const ScoredUnreg& a, const ScoredUnreg& b) {
              const bool ga = a.coverage >= min_coverage_good;
              const bool gb = b.coverage >= min_coverage_good;
              if (ga != gb)
                return ga > gb; // sufficient spread first
              if (a.count != b.count)
                return a.count > b.count;
              if (a.coverage != b.coverage)
                return a.coverage > b.coverage;
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
  VLOG(1) << "choose_resection_candidates: " << scored.size() << " eligible → " << n_list
          << " listed  (sort: cov>=" << min_coverage_good << " first, then 3d2d desc, cov desc)";
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
