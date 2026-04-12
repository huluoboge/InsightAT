/**
 * @file incremental_triangulation_test.cpp
 * @brief Unit tests for run_batch_triangulation (strategy B: point BA + reproj gate + outlier delete).
 */

#include "incremental_triangulation.h"
#include "track_store.h"

#include "../camera/camera_types.h"

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

void project_pinhole(const Eigen::Matrix3d& R, const Eigen::Vector3d& C,
                     const insight::camera::Intrinsics& K, const Eigen::Vector3d& Xw, double* u,
                     double* v) {
  Eigen::Vector3d p = R * (Xw - C);
  if (p(2) <= 1e-12) {
    *u = *v = 0.0;
    return;
  }
  *u = K.fx * (p(0) / p(2)) + K.cx;
  *v = K.fy * (p(1) / p(2)) + K.cy;
}

insight::camera::Intrinsics make_default_K() {
  insight::camera::Intrinsics K{};
  K.fx = K.fy = 500.0;
  K.width = 800;
  K.height = 600;
  K.cx = 320.0;
  K.cy = 240.0;
  return K;
}

bool test_two_view_clean() {
  const insight::camera::Intrinsics K = make_default_K();

  const Eigen::Vector3d Xtrue(0.0, 0.0, 10.0);
  const Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C0 = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C1(1.0, 0.0, 0.0);

  double u0, v0, u1, v1;
  project_pinhole(R0, C0, K, Xtrue, &u0, &v0);
  project_pinhole(R1, C1, K, Xtrue, &u1, &v1);

  insight::sfm::TrackStore store;
  store.set_num_images(2);
  store.reserve_tracks(8);
  store.reserve_observations(16);
  const int tid = store.add_track(0.f, 0.f, 0.f);
  store.add_observation(tid, 0, 0, static_cast<float>(u0), static_cast<float>(v0));
  store.add_observation(tid, 1, 0, static_cast<float>(u1), static_cast<float>(v1));

  std::vector<Eigen::Matrix3d> poses_R = {R0, R1};
  std::vector<Eigen::Vector3d> poses_C = {C0, C1};
  std::vector<bool> registered = {true, true};
  std::vector<insight::camera::Intrinsics> cameras = {K};
  std::vector<int> image_to_camera_index = {0, 0};

  const int n = insight::sfm::run_batch_triangulation(&store, {1}, poses_R, poses_C, registered,
                                                      cameras, image_to_camera_index, 2.0, nullptr,
                                                      4.0);
  if (n != 1) {
    std::fprintf(stderr, "[FAIL] two_view_clean: expected newly_tri=1, got %d\n", n);
    return false;
  }
  if (!store.track_has_triangulated_xyz(tid)) {
    std::fprintf(stderr, "[FAIL] two_view_clean: track should have XYZ\n");
    return false;
  }
  float x, y, z;
  store.get_track_xyz(tid, &x, &y, &z);
  const Eigen::Vector3d Xest(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
  const double err = (Xest - Xtrue).norm();
  if (err > 0.25) {
    std::fprintf(stderr, "[FAIL] two_view_clean: |X-Xtrue|=%g (tol 0.25)\n", err);
    return false;
  }
  std::printf("[PASS] two_view_clean  |X-Xtrue|=%.6g\n", err);
  return true;
}

/// Second view observation is garbage → two-view path must reject (no triangulated flag).
bool test_two_view_bad_second_obs() {
  const insight::camera::Intrinsics K = make_default_K();

  const Eigen::Vector3d Xtrue(0.0, 0.0, 10.0);
  const Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C0 = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C1(1.0, 0.0, 0.0);

  double u0, v0, u1, v1;
  project_pinhole(R0, C0, K, Xtrue, &u0, &v0);
  project_pinhole(R1, C1, K, Xtrue, &u1, &v1);
  u1 += 400.0; // break correspondence

  insight::sfm::TrackStore store;
  store.set_num_images(2);
  store.reserve_tracks(8);
  store.reserve_observations(16);
  const int tid = store.add_track(0.f, 0.f, 0.f);
  store.add_observation(tid, 0, 0, static_cast<float>(u0), static_cast<float>(v0));
  store.add_observation(tid, 1, 0, static_cast<float>(u1), static_cast<float>(v1));

  std::vector<Eigen::Matrix3d> poses_R = {R0, R1};
  std::vector<Eigen::Vector3d> poses_C = {C0, C1};
  std::vector<bool> registered = {true, true};
  std::vector<insight::camera::Intrinsics> cameras = {K};
  std::vector<int> image_to_camera_index = {0, 0};

  const int n = insight::sfm::run_batch_triangulation(&store, {1}, poses_R, poses_C, registered,
                                                      cameras, image_to_camera_index, 2.0, nullptr,
                                                      4.0);
  if (n != 0 || store.track_has_triangulated_xyz(tid)) {
    std::fprintf(stderr, "[FAIL] two_view_bad: expected fail, n=%d tri=%d\n", n,
                 store.track_has_triangulated_xyz(tid) ? 1 : 0);
    return false;
  }
  std::printf("[PASS] two_view_bad_second_obs (rejected as expected)\n");
  return true;
}

/// Three registered views, consistent geometry → robust path + commit succeeds.
bool test_three_view_robust_clean() {
  const insight::camera::Intrinsics K = make_default_K();
  const Eigen::Vector3d Xtrue(0.0, 0.0, 10.0);
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C0 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d C1(1.0, 0.0, 0.0);
  const Eigen::Vector3d C2(0.0, 0.5, 0.0);

  double u0, v0, u1, v1, u2, v2;
  project_pinhole(R, C0, K, Xtrue, &u0, &v0);
  project_pinhole(R, C1, K, Xtrue, &u1, &v1);
  project_pinhole(R, C2, K, Xtrue, &u2, &v2);

  insight::sfm::TrackStore store;
  store.set_num_images(3);
  store.reserve_tracks(8);
  store.reserve_observations(16);
  const int tid = store.add_track(0.f, 0.f, 0.f);
  store.add_observation(tid, 0, 0, static_cast<float>(u0), static_cast<float>(v0));
  store.add_observation(tid, 1, 0, static_cast<float>(u1), static_cast<float>(v1));
  store.add_observation(tid, 2, 0, static_cast<float>(u2), static_cast<float>(v2));

  std::vector<Eigen::Matrix3d> poses_R = {R, R, R};
  std::vector<Eigen::Vector3d> poses_C = {C0, C1, C2};
  std::vector<bool> registered = {true, true, true};
  std::vector<insight::camera::Intrinsics> cameras = {K};
  std::vector<int> image_to_camera_index = {0, 0, 0};

  const int n = insight::sfm::run_batch_triangulation(&store, {2}, poses_R, poses_C, registered,
                                                      cameras, image_to_camera_index, 2.0, nullptr,
                                                      4.0);
  if (n != 1) {
    std::fprintf(stderr, "[FAIL] three_view_robust_clean: expected newly_tri=1, got %d\n", n);
    return false;
  }
  if (!store.track_has_triangulated_xyz(tid)) {
    std::fprintf(stderr, "[FAIL] three_view_robust_clean: track should have XYZ\n");
    return false;
  }
  float x, y, z;
  store.get_track_xyz(tid, &x, &y, &z);
  const Eigen::Vector3d Xest(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
  const double err = (Xest - Xtrue).norm();
  if (err > 0.5) {
    std::fprintf(stderr, "[FAIL] three_view_robust_clean: |X-Xtrue|=%g (tol 0.5)\n", err);
    return false;
  }
  std::printf("[PASS] three_view_robust_clean  |X-Xtrue|=%.6g\n", err);
  return true;
}

/// `robust_triangulate_point_multiview`: 4 views with one gross mismatch → still finds ≥3 inliers.
bool test_robust_api_four_views_one_spurious() {
  const insight::camera::Intrinsics K = make_default_K();
  const Eigen::Vector3d Xtrue(0.0, 0.0, 10.0);
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C0 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d C1(1.0, 0.0, 0.0);
  const Eigen::Vector3d C2(0.0, 0.7, 0.0);
  const Eigen::Vector3d C3(-0.6, 0.0, 0.0);

  double u0, v0, u1, v1, u2, v2, u3, v3;
  project_pinhole(R, C0, K, Xtrue, &u0, &v0);
  project_pinhole(R, C1, K, Xtrue, &u1, &v1);
  project_pinhole(R, C2, K, Xtrue, &u2, &v2);
  project_pinhole(R, C3, K, Xtrue, &u3, &v3);
  u3 += 180.0;

  std::vector<Eigen::Matrix3d> Rl = {R, R, R, R};
  std::vector<Eigen::Vector3d> Cl = {C0, C1, C2, C3};
  std::vector<Eigen::Vector2d> rays = {
      Eigen::Vector2d((u0 - K.cx) / K.fx, (v0 - K.cy) / K.fy),
      Eigen::Vector2d((u1 - K.cx) / K.fx, (v1 - K.cy) / K.fy),
      Eigen::Vector2d((u2 - K.cx) / K.fx, (v2 - K.cy) / K.fy),
      Eigen::Vector2d((u3 - K.cx) / K.fx, (v3 - K.cy) / K.fy),
  };
  std::vector<double> up = {u0, u1, u2, u3};
  std::vector<double> vp = {v0, v1, v2, v3};
  std::vector<insight::camera::Intrinsics> Kpv(4, K);

  insight::sfm::RobustTriangulationOptions opt;
  opt.ransac_inlier_px = 6.0;
  opt.min_inlier_views = 2;
  opt.min_tri_angle_deg = 0.5;
  insight::sfm::RobustTriangulationResult rr;
  if (!insight::sfm::robust_triangulate_point_multiview(Rl, Cl, rays, Kpv, up, vp, opt, &rr) ||
      !rr.success) {
    std::fprintf(stderr, "[FAIL] robust_api_four_views: RANSAC multiview should succeed\n");
    return false;
  }
  int inliers = 0;
  for (bool b : rr.inlier_mask)
    if (b)
      ++inliers;
  if (inliers < 3) {
    std::fprintf(stderr, "[FAIL] robust_api_four_views: expected >=3 inliers, got %d\n", inliers);
    return false;
  }
  std::printf("[PASS] robust_api_four_views_one_spurious  inliers=%d\n", inliers);
  return true;
}

/// Track visible in new image but second hit is on an unregistered image → cannot triangulate yet.
bool test_skip_few_views_only_one_registered_obs() {
  const insight::camera::Intrinsics K = make_default_K();
  insight::sfm::TrackStore store;
  store.set_num_images(3);
  store.reserve_tracks(8);
  store.reserve_observations(16);
  const int tid = store.add_track(0.f, 0.f, 0.f);
  store.add_observation(tid, 1, 0, 100.f, 100.f);
  store.add_observation(tid, 2, 0, 110.f, 110.f);

  std::vector<Eigen::Matrix3d> poses_R(3, Eigen::Matrix3d::Identity());
  std::vector<Eigen::Vector3d> poses_C = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                          Eigen::Vector3d::Zero()};
  std::vector<bool> registered = {true, true, false};
  std::vector<insight::camera::Intrinsics> cameras = {K};
  std::vector<int> image_to_camera_index = {0, 0, 0};

  const int n = insight::sfm::run_batch_triangulation(&store, {1}, poses_R, poses_C, registered,
                                                      cameras, image_to_camera_index, 2.0, nullptr,
                                                      4.0);
  if (n != 0 || store.track_has_triangulated_xyz(tid)) {
    std::fprintf(stderr,
                 "[FAIL] skip_few_views: expected no tri (n=%d) tri_flag=%d\n", n,
                 store.track_has_triangulated_xyz(tid) ? 1 : 0);
    return false;
  }
  std::printf("[PASS] skip_few_views_only_one_registered_obs\n");
  return true;
}

/// `commit_reproj_px` is forwarded to the accept gate: clean pair succeeds even with a loose value.
bool test_loose_commit_reproj_clean_two_view() {
  const insight::camera::Intrinsics K = make_default_K();
  const Eigen::Vector3d Xtrue(0.0, 0.0, 10.0);
  const Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C0 = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C1(1.0, 0.0, 0.0);
  double u0, v0, u1, v1;
  project_pinhole(R0, C0, K, Xtrue, &u0, &v0);
  project_pinhole(R1, C1, K, Xtrue, &u1, &v1);
  insight::sfm::TrackStore store;
  store.set_num_images(2);
  store.reserve_tracks(4);
  store.reserve_observations(8);
  const int tid = store.add_track(0.f, 0.f, 0.f);
  store.add_observation(tid, 0, 0, static_cast<float>(u0), static_cast<float>(v0));
  store.add_observation(tid, 1, 0, static_cast<float>(u1), static_cast<float>(v1));
  std::vector<Eigen::Matrix3d> poses_R = {R0, R1};
  std::vector<Eigen::Vector3d> poses_C = {C0, C1};
  std::vector<bool> registered = {true, true};
  std::vector<insight::camera::Intrinsics> cameras = {K};
  std::vector<int> image_to_camera_index = {0, 0};
  const int n = insight::sfm::run_batch_triangulation(&store, {1}, poses_R, poses_C, registered,
                                                      cameras, image_to_camera_index, 2.0, nullptr,
                                                      12.0);
  if (n != 1 || !store.track_has_triangulated_xyz(tid)) {
    std::fprintf(stderr, "[FAIL] loose_commit: expected success with commit_reproj_px=12\n");
    return false;
  }
  std::printf("[PASS] loose_commit_reproj_clean_two_view\n");
  return true;
}

} // namespace

int main() {
  int fails = 0;
  if (!test_two_view_clean())
    ++fails;
  if (!test_two_view_bad_second_obs())
    ++fails;
  if (!test_three_view_robust_clean())
    ++fails;
  if (!test_robust_api_four_views_one_spurious())
    ++fails;
  if (!test_skip_few_views_only_one_registered_obs())
    ++fails;
  if (!test_loose_commit_reproj_clean_two_view())
    ++fails;
  if (fails > 0) {
    std::fprintf(stderr, "\n%d test(s) FAILED\n", fails);
    return EXIT_FAILURE;
  }
  std::printf("\nAll incremental_triangulation tests PASSED.\n");
  return EXIT_SUCCESS;
}
