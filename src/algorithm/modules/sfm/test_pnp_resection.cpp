/**
 * @file  test_pnp_resection.cpp
 * @brief Unit tests for gpu_ransac_pnp (DLT PnP) and resection_single_image.
 *
 * Tests
 * ──────
 *  1. gpu_ransac_pnp: basic PnP with synthetic 3D-2D (known R,t → verify recovery).
 *  2. gpu_ransac_pnp: SfM-like scenario (small baseline, points near origin, pixel coords ~4000).
 *  3. gpu_ransac_pnp: camera looking at distant scene (large depth range).
 *  4. gpu_ransac_pnp: with 20% outliers → inlier count accuracy.
 *  5. resection_single_image: full pipeline via TrackStore with synthetic data.
 *  6. resection_single_image: mimics the real failing scenario (image 12 regression test).
 *
 * Build:  test_pnp_resection  (see sfm/CMakeLists.txt)
 *
 * Requires GPU (EGL) – same as test_geo_ransac.
 */

#include "../geometry/gpu_geo_ransac.h"
#include "resection.h"
#include "track_store.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace insight::sfm;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static const double kPi = 3.14159265358979323846;

/// Build (R, t) for a camera at position C looking toward a target point.
static void look_at(const Eigen::Vector3d& C, const Eigen::Vector3d& target,
                    Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out) {
  Eigen::Vector3d z = (target - C).normalized();
  Eigen::Vector3d up(0, -1, 0); // Y-down convention for camera
  if (std::abs(z.dot(up)) > 0.95)
    up = Eigen::Vector3d(1, 0, 0);
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x);
  R_out->row(0) = x.transpose();
  R_out->row(1) = y.transpose();
  R_out->row(2) = z.transpose();
  *t_out = -(*R_out) * C;
}

/// Generate random 3D points in a box [lo, hi]^3.
static std::vector<Eigen::Vector3d> random_points_box(int n, double lo, double hi,
                                                      unsigned seed = 42) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(lo, hi);
  std::vector<Eigen::Vector3d> pts(static_cast<size_t>(n));
  for (auto& p : pts)
    p = Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
  return pts;
}

/// Project a 3D point through [R|t] and K → pixel (u,v). Returns false if behind camera.
static bool project_point(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double fx, double fy,
                          double cx, double cy, const Eigen::Vector3d& X, double& u, double& v) {
  Eigen::Vector3d Xc = R * X + t;
  if (Xc(2) <= 0.0)
    return false;
  u = fx * Xc(0) / Xc(2) + cx;
  v = fy * Xc(1) / Xc(2) + cy;
  return true;
}

/// Compute rotation angle between two rotation matrices (geodesic distance, degrees).
static double rotation_error_deg(const Eigen::Matrix3d& R_gt, const Eigen::Matrix3d& R_est) {
  Eigen::Matrix3d dR = R_gt.transpose() * R_est;
  double trace_val = std::max(-1.0, std::min(3.0, dR.trace()));
  double angle_rad = std::acos((trace_val - 1.0) / 2.0);
  return angle_rad * 180.0 / kPi;
}

/// Compute translation direction error in degrees.
static double translation_direction_error_deg(const Eigen::Vector3d& t_gt,
                                              const Eigen::Vector3d& t_est) {
  double n1 = t_gt.norm(), n2 = t_est.norm();
  if (n1 < 1e-12 || n2 < 1e-12)
    return 180.0;
  double cos_angle = std::max(-1.0, std::min(1.0, t_gt.dot(t_est) / (n1 * n2)));
  return std::acos(cos_angle) * 180.0 / kPi;
}

/// Build Point3D2D array from 3D points projected through a known camera.
static std::vector<Point3D2D> build_projections(const std::vector<Eigen::Vector3d>& pts3d,
                                                const Eigen::Matrix3d& R,
                                                const Eigen::Vector3d& t, double fx, double fy,
                                                double cx, double cy, double noise_px = 0.0,
                                                unsigned noise_seed = 123) {
  std::mt19937 rng(noise_seed);
  std::normal_distribution<double> ndist(0.0, noise_px);
  std::vector<Point3D2D> out;
  out.reserve(pts3d.size());
  for (const auto& X : pts3d) {
    double u, v;
    if (!project_point(R, t, fx, fy, cx, cy, X, u, v))
      continue;
    Point3D2D pt;
    pt.x = static_cast<float>(X(0));
    pt.y = static_cast<float>(X(1));
    pt.z = static_cast<float>(X(2));
    pt.u = static_cast<float>(u + (noise_px > 0 ? ndist(rng) : 0.0));
    pt.v = static_cast<float>(v + (noise_px > 0 ? ndist(rng) : 0.0));
    out.push_back(pt);
  }
  return out;
}

static int g_tests_run = 0;
static int g_tests_passed = 0;

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      printf("  FAIL: %s\n", msg);                                                                \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

#define RUN_TEST(fn)                                                                               \
  do {                                                                                             \
    g_tests_run++;                                                                                 \
    printf("\n── Test %d: %s ──\n", g_tests_run, #fn);                                             \
    if (fn()) {                                                                                    \
      g_tests_passed++;                                                                            \
      printf("  PASS\n");                                                                          \
    }                                                                                              \
  } while (0)

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: Basic PnP recovery with clean synthetic data
// ─────────────────────────────────────────────────────────────────────────────

static bool test_basic_pnp() {
  // Camera intrinsics (typical 4000×3000 sensor)
  const double fx = 3500, fy = 3500, cx = 2000, cy = 1500;
  float K[9] = {(float)fx, 0, (float)cx, 0, (float)fy, (float)cy, 0, 0, 1};

  // Generate 50 random 3D points in [-5, 5]³
  auto pts3d = random_points_box(50, -5.0, 5.0, 42);

  // Camera at (0, 0, -15) looking at origin
  Eigen::Matrix3d R_gt;
  Eigen::Vector3d t_gt;
  look_at(Eigen::Vector3d(0, 0, -15), Eigen::Vector3d::Zero(), &R_gt, &t_gt);

  // Build projections (no noise)
  auto pts = build_projections(pts3d, R_gt, t_gt, fx, fy, cx, cy, 0.0);
  printf("  points: %zu projected\n", pts.size());
  CHECK(pts.size() >= 10, "need at least 10 projections");

  // Run PnP RANSAC
  float R_out[9], t_out[3];
  std::vector<unsigned char> mask(pts.size(), 0);
  int inliers = gpu_ransac_pnp(pts.data(), static_cast<int>(pts.size()), K, R_out, t_out, 16.0f,
                                mask.data());

  printf("  inliers: %d / %zu\n", inliers, pts.size());
  CHECK(inliers >= static_cast<int>(pts.size()) * 0.8, "too few inliers");

  // Convert to Eigen
  Eigen::Matrix3d R_est;
  Eigen::Vector3d t_est;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R_est(i, j) = R_out[i * 3 + j];
  t_est << t_out[0], t_out[1], t_out[2];

  // Check rotation error
  double rot_err = rotation_error_deg(R_gt, R_est);
  printf("  rotation error: %.3f deg\n", rot_err);
  CHECK(rot_err < 5.0, "rotation error > 5 deg");

  // Check camera center
  Eigen::Vector3d C_gt = -R_gt.transpose() * t_gt;
  Eigen::Vector3d C_est = -R_est.transpose() * t_est;
  double center_err = (C_gt - C_est).norm();
  printf("  center error: %.4f (GT=[%.2f,%.2f,%.2f], Est=[%.2f,%.2f,%.2f])\n", center_err,
         C_gt(0), C_gt(1), C_gt(2), C_est(0), C_est(1), C_est(2));
  CHECK(center_err < 2.0, "camera center error > 2.0");

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: SfM-like scenario (two-view reconstruction frame)
//   - Camera 0 at identity → 3D points are near origin with small baseline (~1)
//   - Camera 2 (the one to resect) is at moderate offset
// ─────────────────────────────────────────────────────────────────────────────

static bool test_sfm_scenario() {
  const double fx = 3612.5, fy = 3612.5, cx = 2000, cy = 1500;
  float K[9] = {(float)fx, 0, (float)cx, 0, (float)fy, (float)cy, 0, 0, 1};

  // 3D points typical of a normalized two-view reconstruction:
  // Z in [3, 8] (in front of camera, depth ~ a few baseline units)
  // X, Y in [-2, 2]
  std::mt19937 rng(99);
  std::uniform_real_distribution<double> xdist(-2.0, 2.0);
  std::uniform_real_distribution<double> zdist(3.0, 8.0);
  std::vector<Eigen::Vector3d> pts3d(200);
  for (auto& p : pts3d)
    p = Eigen::Vector3d(xdist(rng), xdist(rng), zdist(rng));

  // Camera to resect: slightly rotated and translated (~2 units lateral offset)
  Eigen::Matrix3d R_gt;
  R_gt = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX());
  Eigen::Vector3d C_gt(2.0, 0.3, -0.5); // camera center in world frame
  Eigen::Vector3d t_gt = -R_gt * C_gt;

  // Build projections with 0.5px noise
  auto pts = build_projections(pts3d, R_gt, t_gt, fx, fy, cx, cy, 0.5, 77);
  printf("  points: %zu projected (fx=%.1f, world range Z=[3,8])\n", pts.size(), fx);
  CHECK(pts.size() >= 50, "need at least 50 projections");

  // Run PnP
  float R_out[9], t_out[3];
  std::vector<unsigned char> mask(pts.size(), 0);
  int inliers = gpu_ransac_pnp(pts.data(), static_cast<int>(pts.size()), K, R_out, t_out, 16.0f,
                                mask.data());

  printf("  inliers: %d / %zu\n", inliers, pts.size());
  CHECK(inliers >= static_cast<int>(pts.size()) * 0.7, "too few inliers");

  Eigen::Matrix3d R_est;
  Eigen::Vector3d t_est;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R_est(i, j) = R_out[i * 3 + j];
  t_est << t_out[0], t_out[1], t_out[2];

  double rot_err = rotation_error_deg(R_gt, R_est);
  Eigen::Vector3d C_est = -R_est.transpose() * t_est;
  double center_err = (C_gt - C_est).norm();
  printf("  rotation error: %.3f deg\n", rot_err);
  printf("  center GT=[%.3f,%.3f,%.3f], Est=[%.3f,%.3f,%.3f], err=%.4f\n", C_gt(0), C_gt(1),
         C_gt(2), C_est(0), C_est(1), C_est(2), center_err);
  CHECK(rot_err < 2.0, "rotation error > 2 deg (SfM scenario)");
  CHECK(center_err < 0.5, "camera center error > 0.5 (SfM scenario)");

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: Large depth / distant scene
//   - 3D points at Z ~ [50, 200], camera at large offset
// ─────────────────────────────────────────────────────────────────────────────

static bool test_large_depth() {
  const double fx = 2800, fy = 2800, cx = 2000, cy = 1500;
  float K[9] = {(float)fx, 0, (float)cx, 0, (float)fy, (float)cy, 0, 0, 1};

  auto pts3d = random_points_box(100, -20.0, 20.0, 55);
  // Move all points to Z ∈ [50, 200] range
  for (auto& p : pts3d)
    p(2) = std::abs(p(2)) + 80.0;

  // Camera at (30, -10, -20) looking toward centroid of points
  Eigen::Vector3d centroid(0, 0, 120);
  Eigen::Matrix3d R_gt;
  Eigen::Vector3d t_gt;
  look_at(Eigen::Vector3d(30, -10, -20), centroid, &R_gt, &t_gt);

  auto pts = build_projections(pts3d, R_gt, t_gt, fx, fy, cx, cy, 0.3, 200);
  printf("  points: %zu projected (depth ~100m)\n", pts.size());
  CHECK(pts.size() >= 30, "need at least 30 projections");

  float R_out[9], t_out[3];
  std::vector<unsigned char> mask(pts.size(), 0);
  int inliers = gpu_ransac_pnp(pts.data(), static_cast<int>(pts.size()), K, R_out, t_out, 16.0f,
                                mask.data());

  printf("  inliers: %d / %zu\n", inliers, pts.size());
  CHECK(inliers >= static_cast<int>(pts.size()) * 0.7, "too few inliers for large depth");

  Eigen::Matrix3d R_est;
  Eigen::Vector3d t_est;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R_est(i, j) = R_out[i * 3 + j];
  t_est << t_out[0], t_out[1], t_out[2];

  double rot_err = rotation_error_deg(R_gt, R_est);
  Eigen::Vector3d C_gt = -R_gt.transpose() * t_gt;
  Eigen::Vector3d C_est = -R_est.transpose() * t_est;
  double center_err = (C_gt - C_est).norm();
  printf("  rotation error: %.3f deg\n", rot_err);
  printf("  center error: %.4f\n", center_err);
  CHECK(rot_err < 3.0, "rotation error > 3 deg (large depth)");
  CHECK(center_err < 5.0, "camera center error > 5.0 (large depth)");

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: 20% outliers (random pixel perturbation)
// ─────────────────────────────────────────────────────────────────────────────

static bool test_with_outliers() {
  const double fx = 3500, fy = 3500, cx = 2000, cy = 1500;
  float K[9] = {(float)fx, 0, (float)cx, 0, (float)fy, (float)cy, 0, 0, 1};

  auto pts3d = random_points_box(200, -5.0, 5.0, 66);

  Eigen::Matrix3d R_gt;
  Eigen::Vector3d t_gt;
  look_at(Eigen::Vector3d(3, -2, -12), Eigen::Vector3d::Zero(), &R_gt, &t_gt);

  auto pts = build_projections(pts3d, R_gt, t_gt, fx, fy, cx, cy, 0.5, 88);
  printf("  inlier points: %zu\n", pts.size());
  CHECK(pts.size() >= 50, "need at least 50 inlier projections");

  // Add 20% outliers: random pixel coordinates
  const size_t n_outliers = pts.size() / 4;
  std::mt19937 rng(444);
  std::uniform_real_distribution<float> udist(0.f, 4000.f);
  std::uniform_real_distribution<float> vdist(0.f, 3000.f);
  for (size_t i = 0; i < n_outliers; i++) {
    Point3D2D pt;
    pt.x = pts3d[i](0);
    pt.y = pts3d[i](1);
    pt.z = pts3d[i](2);
    pt.u = udist(rng);
    pt.v = vdist(rng);
    pts.push_back(pt);
  }
  printf("  total points (with outliers): %zu (%.0f%% outliers)\n", pts.size(),
         100.0 * n_outliers / pts.size());

  float R_out[9], t_out[3];
  std::vector<unsigned char> mask(pts.size(), 0);
  int inliers =
      gpu_ransac_pnp(pts.data(), static_cast<int>(pts.size()), K, R_out, t_out, 64.0f, mask.data());

  printf("  RANSAC inliers: %d\n", inliers);
  CHECK(inliers >= static_cast<int>(pts.size() - n_outliers) * 0.7,
        "RANSAC didn't find enough inliers despite outliers");

  Eigen::Matrix3d R_est;
  Eigen::Vector3d t_est;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R_est(i, j) = R_out[i * 3 + j];
  t_est << t_out[0], t_out[1], t_out[2];

  double rot_err = rotation_error_deg(R_gt, R_est);
  Eigen::Vector3d C_gt = -R_gt.transpose() * t_gt;
  Eigen::Vector3d C_est = -R_est.transpose() * t_est;
  double center_err = (C_gt - C_est).norm();
  printf("  rotation error: %.3f deg\n", rot_err);
  printf("  center error: %.4f\n", center_err);
  CHECK(rot_err < 5.0, "rotation error > 5 deg with outliers");
  CHECK(center_err < 2.0, "camera center error > 2.0 with outliers");

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: resection_single_image via TrackStore
//   Builds a minimal TrackStore with known 3D + 2D observations, tests full pipeline.
// ─────────────────────────────────────────────────────────────────────────────

static bool test_resection_trackstore() {
  const double fx = 3500, fy = 3500, cx = 2000, cy = 1500;

  // 3 images: 0 and 1 are "registered" (initial pair), 2 is to be resected.
  TrackStore store;
  store.set_num_images(3);

  // Camera 2 pose (ground truth)
  Eigen::Matrix3d R_gt;
  R_gt = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitX());
  Eigen::Vector3d C_gt(1.5, -0.5, -0.2);
  Eigen::Vector3d t_gt = -R_gt * C_gt;

  // Generate 100 3D points, seen by all 3 images
  auto pts3d = random_points_box(100, -3.0, 3.0, 101);
  // Push Z to be in front of all cameras
  for (auto& p : pts3d)
    p(2) = std::abs(p(2)) + 5.0;

  int added_tracks = 0;
  for (size_t i = 0; i < pts3d.size(); i++) {
    const auto& X = pts3d[i];
    // Project into camera 2
    double u, v;
    if (!project_point(R_gt, t_gt, fx, fy, cx, cy, X, u, v))
      continue;
    // Also verify visible in image range
    if (u < 0 || u > 4000 || v < 0 || v > 3000)
      continue;

    // Add a track with triangulated XYZ
    int tid = store.add_track(static_cast<float>(X(0)), static_cast<float>(X(1)),
                              static_cast<float>(X(2)));
    // Mark as triangulated
    store.set_track_xyz(tid, static_cast<float>(X(0)), static_cast<float>(X(1)),
                        static_cast<float>(X(2)));

    // Observation for image 0 (fake, we don't need it accurate for image 0)
    store.add_observation(tid, 0, static_cast<uint32_t>(i), 2000.f, 1500.f, 1.f);
    // Observation for image 2 (the one being resected)
    store.add_observation(tid, 2, static_cast<uint32_t>(i), static_cast<float>(u),
                          static_cast<float>(v), 1.f);
    added_tracks++;
  }
  printf("  tracks with observations in image 2: %d\n", added_tracks);
  CHECK(added_tracks >= 30, "need at least 30 tracks visible in image 2");

  // Run resection
  Eigen::Matrix3d R_out;
  Eigen::Vector3d t_out;
  int inliers = 0;
  bool ok = resection_single_image(store, 2, fx, fy, cx, cy, &R_out, &t_out, 6, 8.0, &inliers);

  printf("  resection ok: %s, inliers: %d\n", ok ? "yes" : "no", inliers);
  CHECK(ok, "resection_single_image returned false");
  CHECK(inliers >= 20, "resection inliers < 20");

  double rot_err = rotation_error_deg(R_gt, R_out);
  Eigen::Vector3d C_est = -R_out.transpose() * t_out;
  double center_err = (C_gt - C_est).norm();
  printf("  rotation error: %.3f deg\n", rot_err);
  printf("  center GT=[%.3f,%.3f,%.3f], Est=[%.3f,%.3f,%.3f], err=%.4f\n", C_gt(0), C_gt(1),
         C_gt(2), C_est(0), C_est(1), C_est(2), center_err);
  CHECK(rot_err < 2.0, "rotation error > 2 deg (TrackStore resection)");
  CHECK(center_err < 0.5, "camera center error > 0.5 (TrackStore resection)");

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6: Regression test – mimics the failing scenario from real pipeline
//   - Initial pair: cam0=identity, cam1 at small baseline (~1)
//   - 3D points triangulated from these two views (Z ~ [3,10], XY ~ [-2,2])
//   - Image 12 is at a lateral offset – was getting C≈[0,0,0] before the fix
// ─────────────────────────────────────────────────────────────────────────────

static bool test_regression_image12() {
  const double fx = 3612.5, fy = 3612.5, cx = 2000, cy = 1500;
  float K[9] = {(float)fx, 0, (float)cx, 0, (float)fy, (float)cy, 0, 0, 1};

  // Cam0: identity
  Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t0 = Eigen::Vector3d::Zero();

  // Cam1: small lateral translation (baseline ~1)
  Eigen::Matrix3d R1;
  R1 = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY());
  Eigen::Vector3d C1(1.0, 0.0, 0.0);
  Eigen::Vector3d t1 = -R1 * C1;

  // 1000 3D points (triangulated from cam0 & cam1): Z in [3,10], XY in [-2,2]
  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> xdist(-2.0, 2.0);
  std::uniform_real_distribution<double> zdist(3.0, 10.0);
  std::vector<Eigen::Vector3d> pts3d(1000);
  for (auto& p : pts3d)
    p = Eigen::Vector3d(xdist(rng), xdist(rng), zdist(rng));

  // "Image 12" camera: moderate rotation + lateral translation
  Eigen::Matrix3d R_gt;
  R_gt = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d C_gt(3.0, 0.5, -1.0);
  Eigen::Vector3d t_gt = -R_gt * C_gt;

  // Build projections with 1px noise
  auto pts = build_projections(pts3d, R_gt, t_gt, fx, fy, cx, cy, 1.0, 999);
  printf("  points: %zu projected (fx=%.1f, 3D range Z=[3,10], cam offset=(3,0.5,-1))\n",
         pts.size(), fx);
  CHECK(pts.size() >= 100, "need at least 100 projections");

  // Run PnP
  float R_out[9], t_out[3];
  std::vector<unsigned char> mask(pts.size(), 0);
  int inliers = gpu_ransac_pnp(pts.data(), static_cast<int>(pts.size()), K, R_out, t_out, 64.0f,
                                mask.data());

  printf("  inliers: %d / %zu (expect >> 100)\n", inliers, pts.size());
  CHECK(inliers >= static_cast<int>(pts.size()) * 0.5,
        "REGRESSION: too few inliers (was 9/1366 before fix)");

  Eigen::Matrix3d R_est;
  Eigen::Vector3d t_est;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R_est(i, j) = R_out[i * 3 + j];
  t_est << t_out[0], t_out[1], t_out[2];

  double rot_err = rotation_error_deg(R_gt, R_est);
  Eigen::Vector3d C_est = -R_est.transpose() * t_est;
  double center_err = (C_gt - C_est).norm();
  printf("  rotation error: %.3f deg\n", rot_err);
  printf("  center GT=[%.3f,%.3f,%.3f], Est=[%.3f,%.3f,%.3f], err=%.4f\n", C_gt(0), C_gt(1),
         C_gt(2), C_est(0), C_est(1), C_est(2), center_err);
  CHECK(center_err < 3.0, "REGRESSION: camera center far from GT (was ~[0,0,0] before fix)");
  CHECK(rot_err < 3.0, "REGRESSION: rotation error > 3 deg");

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main() {
  printf("=== PnP Resection Unit Tests ===\n");

  // Initialize GPU
  printf("Initializing EGL/GPU context...\n");
  if (gpu_geo_init(nullptr) != 0) {
    fprintf(stderr, "FATAL: Failed to initialize GPU context (EGL). Cannot run PnP tests.\n");
    return 1;
  }

  RUN_TEST(test_basic_pnp);
  RUN_TEST(test_sfm_scenario);
  RUN_TEST(test_large_depth);
  RUN_TEST(test_with_outliers);
  RUN_TEST(test_resection_trackstore);
  RUN_TEST(test_regression_image12);

  gpu_geo_shutdown();

  printf("\n══════════════════════════════════════════\n");
  printf("  %d / %d tests passed\n", g_tests_passed, g_tests_run);
  printf("══════════════════════════════════════════\n");

  return (g_tests_passed == g_tests_run) ? 0 : 1;
}
