/**
 * @file  scene_normalization.cpp
 */

#include "scene_normalization.h"
#include "track_store.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <glog/logging.h>

namespace insight {
namespace sfm {

namespace {

constexpr double kMinScale = 1e-12;
constexpr int kMinPoints = 3;

// TODO(perf): replace with histogram / streaming quantile for large point clouds.
double median_inplace(std::vector<double>* v) {
  if (!v || v->empty())
    return 0.0;
  const size_t n = v->size();
  const size_t mid = n / 2;
  auto begin = v->begin();
  std::nth_element(begin, begin + static_cast<std::ptrdiff_t>(mid), v->end());
  if ((n & 1u) != 0u)
    return (*v)[mid];
  const double upper = (*v)[mid];
  std::nth_element(begin, begin + static_cast<std::ptrdiff_t>(mid - 1), begin + static_cast<std::ptrdiff_t>(mid));
  return 0.5 * ((*v)[mid - 1] + upper);
}

double median_component(const std::vector<Eigen::Vector3d>& pts, int axis) {
  std::vector<double> v;
  v.reserve(pts.size());
  for (const auto& p : pts)
    v.push_back(p[axis]);
  return median_inplace(&v);
}

} // namespace

bool normalize_scene_median_tracks_and_poses(TrackStore* store, std::vector<Eigen::Vector3d>* poses_C,
                                             const std::vector<bool>& registered) {
  if (!store || !poses_C)
    return false;

  std::vector<Eigen::Vector3d> pts;
  const int n_tracks = static_cast<int>(store->num_tracks());
  for (int tid = 0; tid < n_tracks; ++tid) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    float fx, fy, fz;
    store->get_track_xyz(tid, &fx, &fy, &fz);
    pts.emplace_back(static_cast<double>(fx), static_cast<double>(fy), static_cast<double>(fz));
  }

  if (static_cast<int>(pts.size()) < kMinPoints) {
    LOG(WARNING) << "normalize_scene_median_tracks_and_poses: too few triangulated points "
                 << pts.size();
    return false;
  }

  const Eigen::Vector3d mu(median_component(pts, 0), median_component(pts, 1), median_component(pts, 2));

  std::vector<double> radii;
  radii.reserve(pts.size());
  for (const auto& p : pts)
    radii.push_back((p - mu).norm());
  const double s = median_inplace(&radii);
  if (s < kMinScale) {
    LOG(WARNING) << "normalize_scene_median_tracks_and_poses: median radial scale too small s=" << s;
    return false;
  }

  for (int tid = 0; tid < n_tracks; ++tid) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    float fx, fy, fz;
    store->get_track_xyz(tid, &fx, &fy, &fz);
    Eigen::Vector3d X(static_cast<double>(fx), static_cast<double>(fy), static_cast<double>(fz));
    X = (X - mu) / s;
    store->set_track_xyz(tid, static_cast<float>(X.x()), static_cast<float>(X.y()), static_cast<float>(X.z()));
  }

  for (size_t i = 0; i < poses_C->size() && i < registered.size(); ++i) {
    if (registered[i])
      (*poses_C)[i] = ((*poses_C)[i] - mu) / s;
  }

  LOG(INFO) << "normalize_scene_median_tracks_and_poses: applied mu=(" << mu.transpose() << ") s=" << s
            << " n_pts=" << pts.size();
  return true;
}

} // namespace sfm
} // namespace insight
