/**
 * test_focal_from_view_graph.cpp
 *
 * Synthetic check: known shared focal → F = K^{-T} [t]_× R K^{-1} → recover f.
 */
#include "focal_from_view_graph.h"
#include "two_view_reconstruction.h"

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using insight::sfm::FocalFromViewGraphOptions;
using insight::sfm::FocalPairConstraint;
using insight::sfm::estimate_focals_from_view_graph;
using insight::sfm::focal_from_fundamental;

static Eigen::Matrix3d skew(const Eigen::Vector3d& t) {
  Eigen::Matrix3d S;
  S << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0;
  return S;
}

static Eigen::Matrix3d make_F(double f, double cx, double cy, const Eigen::Matrix3d& R,
                              const Eigen::Vector3d& t) {
  Eigen::Matrix3d K;
  K << f, 0, cx, 0, f, cy, 0, 0, 1;
  Eigen::Matrix3d Kinv = K.inverse();
  Eigen::Matrix3d E = skew(t.normalized()) * R;
  return Kinv.transpose() * E * Kinv;
}

static int g_fail = 0;
#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                                     \
      ++g_fail;                                                                                    \
    } else {                                                                                       \
      std::printf("OK: %s\n", msg);                                                                \
    }                                                                                              \
  } while (0)

int main() {
  const double f_gt = 3410.0;
  const double cx = 3024.0, cy = 2016.0;
  const double w = 6048.0;

  // Single-pair Hartley recovery.
  {
    Eigen::Matrix3d R = Eigen::AngleAxisd(8.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Vector3d t(1.0, 0.05, 0.1);
    Eigen::Matrix3d F = make_F(f_gt, cx, cy, R, t);
    const double f_est = focal_from_fundamental(F, cx, cy, 200.0, 20000.0);
    CHECK(f_est > 0.0, "single-pair focal > 0");
    CHECK(std::abs(f_est - f_gt) / f_gt < 0.05, "single-pair relative error < 5%");
    std::printf("  single-pair f_est=%.2f (gt=%.2f)\n", f_est, f_gt);
  }

  // View-graph recovery from several noisy-ish baselines (exact F, different poses).
  {
    std::vector<FocalPairConstraint> pairs;
    const double angles_deg[] = {5.0, 8.0, 12.0, 15.0, 6.0, 10.0, 18.0, 7.0};
    for (double ang : angles_deg) {
      Eigen::Matrix3d R =
          Eigen::AngleAxisd(ang * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
      Eigen::Vector3d t(1.0, 0.02 * ang, 0.05);
      FocalPairConstraint c;
      c.F = make_F(f_gt, cx, cy, R, t);
      c.cx1 = c.cx2 = cx;
      c.cy1 = c.cy2 = cy;
      c.camera_id1 = c.camera_id2 = 0;
      c.weight = 200;
      c.is_degenerate = false;
      pairs.push_back(c);
    }

    FocalFromViewGraphOptions opts;
    opts.soft_prior_focal = 0.7 * w; // ~4233, far from gt but within 0.1–10×
    opts.min_pairs_per_camera = 3;
    opts.min_inliers = 30;
    opts.refine_with_ceres = true;

    const auto res = estimate_focals_from_view_graph(pairs, opts);
    CHECK(res.ok, "view-graph estimate ok");
    CHECK(res.cameras.size() == 1, "one camera");
    CHECK(std::abs(res.shared_focal - f_gt) / f_gt < 0.03, "view-graph relative error < 3%");
    std::printf("  view-graph f=%.2f median=%.2f method=%s pairs=%d\n", res.shared_focal,
                res.cameras[0].median_focal, res.method.c_str(), res.num_pairs_used);
  }

  // Wrong soft prior should not lock us to 5880 if many good pairs exist.
  {
    std::vector<FocalPairConstraint> pairs;
    for (int i = 0; i < 10; ++i) {
      const double ang = 5.0 + i;
      Eigen::Matrix3d R =
          Eigen::AngleAxisd(ang * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
      FocalPairConstraint c;
      c.F = make_F(f_gt, cx, cy, R, Eigen::Vector3d(1, 0, 0.05));
      c.cx1 = c.cx2 = cx;
      c.cy1 = c.cy2 = cy;
      c.camera_id1 = c.camera_id2 = 0;
      c.weight = 150;
      pairs.push_back(c);
    }
    FocalFromViewGraphOptions opts;
    opts.prior_focal = 5880.0; // bad EXIF fallback
    opts.prior_ratio_lo = 0.1;
    opts.prior_ratio_hi = 10.0;
    opts.refine_with_ceres = true;
    const auto res = estimate_focals_from_view_graph(pairs, opts);
    CHECK(res.ok, "recovers despite bad prior");
    CHECK(std::abs(res.shared_focal - f_gt) / f_gt < 0.05, "not stuck at 5880");
    std::printf("  with prior=5880 → f=%.2f\n", res.shared_focal);
  }

  if (g_fail) {
    std::fprintf(stderr, "%d check(s) failed\n", g_fail);
    return 1;
  }
  std::printf("All focal_from_view_graph tests passed.\n");
  return 0;
}
