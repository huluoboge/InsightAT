/**
 * @file  resection.cpp
 * @brief Resection via OpenCV PnP RANSAC + pose_only_bundle.
 */

#include "resection.h"
#include "bundle_adjustment.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <vector>

namespace insight {
namespace sfm {

namespace {

void rvec_tvec_to_rt(const cv::Mat& rvec, const cv::Mat& tvec,
                     Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out) {
    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            (*R_out)(i, j) = Rcv.at<double>(i, j);
    (*t_out)(0) = tvec.at<double>(0, 0);
    (*t_out)(1) = tvec.at<double>(1, 0);
    (*t_out)(2) = tvec.at<double>(2, 0);
}

}  // namespace

bool is_resection_stable(int inlier_count,
                         int total_correspondences,
                         double rmse_px,
                         double min_inlier_ratio,
                         double max_rmse_px) {
  if (total_correspondences <= 0) return false;
  double ratio = static_cast<double>(inlier_count) / static_cast<double>(total_correspondences);
  return ratio >= min_inlier_ratio && rmse_px <= max_rmse_px;
}

bool resection_single_image(const TrackStore& store,
                            int image_index,
                            double fx, double fy, double cx, double cy,
                            Eigen::Matrix3d* R_out,
                            Eigen::Vector3d* t_out,
                            int min_inliers,
                            double ransac_thresh_px,
                            int* inliers_out) {
    if (!R_out || !t_out) return false;

    std::vector<int> track_ids;
    std::vector<Observation> obs_list;
    const int n = store.get_image_track_observations(image_index, &track_ids, &obs_list);
    if (n < static_cast<int>(min_inliers)) return false;

    std::vector<Eigen::Vector3d> pts3d;
    std::vector<Eigen::Vector2d> pts2d;
    pts3d.reserve(static_cast<size_t>(n));
    pts2d.reserve(static_cast<size_t>(n));
    for (size_t i = 0; i < static_cast<size_t>(n); ++i) {
        const int tid = track_ids[i];
        if (!store.track_has_triangulated_xyz(tid)) continue;
        float x, y, z;
        store.get_track_xyz(tid, &x, &y, &z);
        pts3d.push_back(Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)));
        pts2d.push_back(Eigen::Vector2d(static_cast<double>(obs_list[i].u), static_cast<double>(obs_list[i].v)));
    }
    if (static_cast<int>(pts3d.size()) < min_inliers) return false;

    std::vector<cv::Point3d> obj_pts;
    std::vector<cv::Point2d> img_pts;
    obj_pts.reserve(pts3d.size());
    img_pts.reserve(pts2d.size());
    for (size_t i = 0; i < pts3d.size(); ++i) {
        obj_pts.emplace_back(pts3d[i].x(), pts3d[i].y(), pts3d[i].z());
        img_pts.emplace_back(pts2d[i].x(), pts2d[i].y());
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat rvec, tvec;
    std::vector<int> inlier_indices;
    const bool pnp_ok = cv::solvePnPRansac(
        obj_pts, img_pts,
        K, distCoeffs, rvec, tvec,
        false,  // useExtrinsicGuess
        1000,   // iterationsCount
        static_cast<float>(ransac_thresh_px),
        0.99,   // confidence
        inlier_indices,
        cv::SOLVEPNP_ITERATIVE
    );
    if (!pnp_ok || static_cast<int>(inlier_indices.size()) < min_inliers) {
        if (inliers_out) *inliers_out = pnp_ok ? static_cast<int>(inlier_indices.size()) : 0;
        return false;
    }

    Eigen::Matrix3d R_init;
    Eigen::Vector3d t_init;
    rvec_tvec_to_rt(rvec, tvec, &R_init, &t_init);

    std::vector<Eigen::Vector3d> inlier_pts3d;
    std::vector<Eigen::Vector2d> inlier_pts2d;
    inlier_pts3d.reserve(inlier_indices.size());
    inlier_pts2d.reserve(inlier_indices.size());
    for (int idx : inlier_indices) {
        inlier_pts3d.push_back(pts3d[static_cast<size_t>(idx)]);
        inlier_pts2d.push_back(pts2d[static_cast<size_t>(idx)]);
    }

#if defined(INSIGHTAT_USE_CERES) && INSIGHTAT_USE_CERES
    Eigen::Matrix3d R_refined = R_init;
    Eigen::Vector3d t_refined = t_init;
    double rmse = 0.0;
    if (!pose_only_bundle(inlier_pts3d, inlier_pts2d, fx, fy, cx, cy,
                          R_init, t_init, &R_refined, &t_refined, &rmse, 30)) {
        if (inliers_out) *inliers_out = static_cast<int>(inlier_indices.size());
        *R_out = R_init;
        *t_out = t_init;
        return true;
    }
    *R_out = R_refined;
    *t_out = t_refined;
#else
    *R_out = R_init;
    *t_out = t_init;
#endif
    if (inliers_out) *inliers_out = static_cast<int>(inlier_indices.size());
    return true;
}

int resection_image_grid_coverage(const TrackStore& store,
                                  int image_index,
                                  int grid_cols,
                                  int grid_rows) {
  std::vector<int> track_ids;
  std::vector<Observation> obs_list;
  const int n = store.get_image_track_observations(image_index, &track_ids, &obs_list);
  if (n == 0 || grid_cols <= 0 || grid_rows <= 0) return 0;
  std::vector<float> uu, vv;
  for (size_t i = 0; i < static_cast<size_t>(n); ++i) {
    if (!store.track_has_triangulated_xyz(track_ids[i])) continue;
    uu.push_back(obs_list[i].u);
    vv.push_back(obs_list[i].v);
  }
  if (uu.size() < 2u) return uu.empty() ? 0 : 1;
  float umin = uu[0], umax = uu[0], vmin = vv[0], vmax = vv[0];
  for (size_t i = 1; i < uu.size(); ++i) {
    if (uu[i] < umin) umin = uu[i];
    if (uu[i] > umax) umax = uu[i];
    if (vv[i] < vmin) vmin = vv[i];
    if (vv[i] > vmax) vmax = vv[i];
  }
  float ur = umax - umin, vr = vmax - vmin;
  if (ur < 1e-6f) ur = 1e-6f;
  if (vr < 1e-6f) vr = 1e-6f;
  std::vector<std::vector<bool>> cell_used(static_cast<size_t>(grid_rows),
                                           std::vector<bool>(static_cast<size_t>(grid_cols), false));
  for (size_t i = 0; i < uu.size(); ++i) {
    int c = static_cast<int>((uu[i] - umin) / ur * grid_cols);
    int r = static_cast<int>((vv[i] - vmin) / vr * grid_rows);
    if (c >= grid_cols) c = grid_cols - 1;
    if (r >= grid_rows) r = grid_rows - 1;
    if (c < 0) c = 0;
    if (r < 0) r = 0;
    cell_used[static_cast<size_t>(r)][static_cast<size_t>(c)] = true;
  }
  int cells = 0;
  for (int r = 0; r < grid_rows; ++r)
    for (int c = 0; c < grid_cols; ++c)
      if (cell_used[static_cast<size_t>(r)][static_cast<size_t>(c)]) ++cells;
  return cells;
}

int choose_next_resection_image(const TrackStore& store,
                                const std::vector<bool>& registered_indices,
                                const std::vector<bool>* skip_indices,
                                int min_grid_cells) {
  const int n_images = store.num_images();
  if (n_images == 0 || static_cast<int>(registered_indices.size()) != n_images)
    return -1;
  if (skip_indices && static_cast<int>(skip_indices->size()) != n_images)
    return -1;

  int best_image = -1;
  int best_count = 0;
  for (int im = 0; im < n_images; ++im) {
    if (registered_indices[static_cast<size_t>(im)]) continue;
    if (skip_indices && (*skip_indices)[static_cast<size_t>(im)]) continue;
    if (min_grid_cells > 0 && resection_image_grid_coverage(store, im, 4, 4) < min_grid_cells)
      continue;
    std::vector<int> track_ids;
    store.get_image_track_observations(im, &track_ids, nullptr);
    int count = 0;
    for (int tid : track_ids) {
      if (store.track_has_triangulated_xyz(tid)) ++count;
    }
    if (count > best_count) {
      best_count = count;
      best_image = im;
    }
  }
  return best_image;
}

}  // namespace sfm
}  // namespace insight
