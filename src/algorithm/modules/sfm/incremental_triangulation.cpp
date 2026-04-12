/**
 * @file  incremental_triangulation.cpp
 * @brief Multiview triangulation for incremental SfM (see incremental_triangulation.h).
 */

#include "incremental_triangulation.h"

#include "../camera/camera_utils.h"
#include "track_store.h"
#include "util/numeric.h"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <glog/logging.h>
#include <limits>
#include <random>
#include <sstream>
#include <unordered_set>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace insight {
namespace sfm {

namespace {

// #region tri_diag — verbose per-failure triangulation logs for diagnosis. Set true to enable.
static constexpr bool kTriangulationDiagLog = false;

static std::string tri_diag_format_depths(const Eigen::Vector3d& X,
                                          const std::vector<Eigen::Matrix3d>& R_list,
                                          const std::vector<Eigen::Vector3d>& C_list,
                                          const std::vector<int>& reg_inds) {
  std::ostringstream o;
  o << "X_world=(" << X.x() << "," << X.y() << "," << X.z() << ")";
  for (size_t i = 0; i < R_list.size(); ++i) {
    const Eigen::Vector3d Xc = R_list[i] * (X - C_list[i]);
    o << " im" << reg_inds[i] << ":Zc=" << Xc(2);
  }
  return o.str();
}

static const char* tri_diag_angle_reason(double ang_deg, double min_deg, double max_deg) {
  if (ang_deg < min_deg)
    return "too_small";
  if (ang_deg > max_deg)
    return "too_large";
  return "ok";
}

static std::string tri_diag_format_reg_inds(const std::vector<int>& reg_inds) {
  std::ostringstream o;
  for (size_t i = 0; i < reg_inds.size(); ++i) {
    if (i)
      o << ',';
    o << reg_inds[i];
  }
  return o.str();
}

static std::string tri_diag_format_reproj_per_view(const std::vector<double>& e_px,
                                                   const std::vector<int>& reg_inds) {
  std::ostringstream o;
  for (size_t i = 0; i < e_px.size(); ++i) {
    if (i)
      o << ',';
    o << "im" << reg_inds[i] << "=" << e_px[i];
  }
  return o.str();
}

/// When rebuild_registered_arrays fails: explain whether this is "only 1 registered view" (common,
/// not a triangulation formula bug) vs outlier deletion inside triangulate_track_common (pass>0).
static std::string tri_diag_track_registered_obs_summary(TrackStore* store, int track_id,
                                                         int n_images,
                                                         const std::vector<bool>& registered) {
  std::vector<int> ids;
  store->get_track_obs_ids(track_id, &ids);
  int n_in_reg = 0;
  std::ostringstream reg_imgs;
  std::ostringstream all_imgs;
  for (int oid : ids) {
    Observation o;
    store->get_obs(oid, &o);
    const int im = static_cast<int>(o.image_index);
    if (im < 0 || im >= n_images)
      continue;
    if (!all_imgs.str().empty())
      all_imgs << ',';
    all_imgs << im;
    if (registered[static_cast<size_t>(im)]) {
      ++n_in_reg;
      if (!reg_imgs.str().empty())
        reg_imgs << ',';
      reg_imgs << im;
    }
  }
  std::ostringstream out;
  out << "n_valid_obs=" << ids.size() << " n_in_registered_images=" << n_in_reg << " reg_imgs=["
      << reg_imgs.str() << "] all_imgs=[" << all_imgs.str() << "]";
  return out.str();
}
// #endregion

Eigen::Vector3d triangulate_point_multiview(const std::vector<Eigen::Matrix3d>& R_list,
                                            const std::vector<Eigen::Vector3d>& C_list,
                                            const std::vector<Eigen::Vector2d>& rays_n) {
  const int N = static_cast<int>(R_list.size());
  if (N < 2 || static_cast<int>(C_list.size()) != N || static_cast<int>(rays_n.size()) != N)
    return Eigen::Vector3d(0, 0, 0);
  Eigen::MatrixXd A(2 * N, 4);
  for (int i = 0; i < N; ++i) {
    const Eigen::Matrix3d& R = R_list[i];
    const Eigen::Vector3d t = -R * C_list[i];
    const double nx = rays_n[i](0), ny = rays_n[i](1);
    // DLT constraints from q_x/q_z = nx and q_y/q_z = ny (q = R*X + t):
    //   -(q_x - nx*q_z) = 0  →  (nx*R[2] - R[0])*X + (nx*t2 - t0) = 0
    //   -(q_y - ny*q_z) = 0  →  (ny*R[2] - R[1])*X + (ny*t2 - t1) = 0
    A.row(2 * i) << nx * R.row(2) - R.row(0), nx * t(2) - t(0);
    A.row(2 * i + 1) << ny * R.row(2) - R.row(1), ny * t(2) - t(1);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d v = svd.matrixV().col(3);
  if (std::fabs(v(3)) < 1e-12)
    return Eigen::Vector3d(0, 0, 0);
  return v.head<3>() / v(3);
}

// OpenMVG-style DLT (HZ 12.2 p.312): projection matrix P = K * [R | t] with undistorted pixel
// coordinates.  rays_n[i] = ((u_undist-cx)/fx, (v_undist-cy)/fy), so undistorted pixels are
// recovered as  u_undist = rays_n[i](0)*fx + cx.  Mathematically equivalent to

bool check_all_depths_positive(const Eigen::Vector3d& X, const std::vector<Eigen::Matrix3d>& R_list,
                               const std::vector<Eigen::Vector3d>& C_list) {
  for (size_t i = 0; i < R_list.size(); ++i) {
    const Eigen::Vector3d Xc = R_list[i] * (X - C_list[i]);
    if (Xc(2) <= 0.0)
      return false;
  }
  return true;
}

double compute_max_ray_angle_deg(const Eigen::Vector3d& X,
                                 const std::vector<Eigen::Vector3d>& C_list) {
  double max_angle = 0.0;
  const int N = static_cast<int>(C_list.size());
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d ri = (X - C_list[i]).normalized();
    for (int j = i + 1; j < N; ++j) {
      const Eigen::Vector3d rj = (X - C_list[j]).normalized();
      double d = ri.dot(rj);
      d = d < -1.0 ? -1.0 : (d > 1.0 ? 1.0 : d);
      const double angle_deg = std::acos(d) * (180.0 / M_PI);
      if (angle_deg > max_angle)
        max_angle = angle_deg;
    }
  }
  return max_angle;
}

void world_to_pixel(const Eigen::Matrix3d& R, const Eigen::Vector3d& C, const camera::Intrinsics& K,
                    const Eigen::Vector3d& X, double* u_out, double* v_out) {
  Eigen::Vector3d p = R * (X - C);
  if (p(2) <= 1e-12) {
    *u_out = *v_out = std::numeric_limits<double>::quiet_NaN();
    return;
  }
  const double xn = p(0) / p(2), yn = p(1) / p(2);
  double xd, yd;
  camera::apply_distortion(xn, yn, K, &xd, &yd);
  *u_out = K.fx * xd + K.cx;
  *v_out = K.fy * yd + K.cy;
}

double reproj_error_px(const Eigen::Vector3d& X, const Eigen::Matrix3d& R, const Eigen::Vector3d& C,
                       const camera::Intrinsics& K, double u_obs, double v_obs) {
  double up, vp;
  world_to_pixel(R, C, K, X, &up, &vp);
  if (!std::isfinite(up) || !std::isfinite(vp))
    return 1e20;
  const double du = u_obs - up, dv = v_obs - vp;
  return std::sqrt(du * du + dv * dv);
}

Eigen::Vector3d
refine_point_multiview_gn(Eigen::Vector3d X, const std::vector<Eigen::Matrix3d>& R_list,
                          const std::vector<Eigen::Vector3d>& C_list,
                          const std::vector<camera::Intrinsics>& K_list,
                          const std::vector<double>& u_px, const std::vector<double>& v_px,
                          const std::vector<size_t>& view_idx, int max_iter, double tol) {
  const size_t m = view_idx.size();
  if (m < 2)
    return X;
  const double h = 1e-6;
  for (int it = 0; it < max_iter; ++it) {
    Eigen::VectorXd r(2 * static_cast<Eigen::Index>(m));
    int row = 0;
    for (size_t ii = 0; ii < m; ++ii) {
      const size_t i = view_idx[ii];
      double up, vp;
      world_to_pixel(R_list[i], C_list[i], K_list[i], X, &up, &vp);
      r(row++) = u_px[i] - up;
      r(row++) = v_px[i] - vp;
    }
    if (r.norm() < tol * std::sqrt(static_cast<double>(m)))
      break;
    Eigen::MatrixXd J(r.size(), 3);
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d Xp = X;
      Xp(k) += h;
      Eigen::VectorXd rp(2 * static_cast<Eigen::Index>(m));
      row = 0;
      for (size_t ii = 0; ii < m; ++ii) {
        const size_t i = view_idx[ii];
        double up, vp;
        world_to_pixel(R_list[i], C_list[i], K_list[i], Xp, &up, &vp);
        rp(row++) = u_px[i] - up;
        rp(row++) = v_px[i] - vp;
      }
      J.col(k) = (rp - r) / h;
    }
    const Eigen::Matrix3d JtJ = J.transpose() * J;
    const Eigen::Vector3d Jtr = J.transpose() * r;
    const Eigen::Vector3d dx = JtJ.ldlt().solve(Jtr);
    if (!dx.allFinite())
      break;
    const Eigen::Vector3d X_new = X - dx; // correct GN descent: X_new = X - (J^T J)^{-1} J^T r
    // Only accept the step if it actually reduces the residual (no line search, so guard against
    // ill-conditioned J^T J producing an overshooting step).
    Eigen::VectorXd r_new(r.size());
    {
      int row2 = 0;
      for (size_t ii = 0; ii < m; ++ii) {
        const size_t i = view_idx[ii];
        double up, vp;
        world_to_pixel(R_list[i], C_list[i], K_list[i], X_new, &up, &vp);
        r_new(row2++) = u_px[i] - up;
        r_new(row2++) = v_px[i] - vp;
      }
    }
    if (r_new.norm() >= r.norm())
      break;
    X = X_new;
    if (dx.norm() < tol)
      break;
  }
  return X;
}

void count_inliers_and_mask(const Eigen::Vector3d& X, const std::vector<Eigen::Matrix3d>& R_list,
                            const std::vector<Eigen::Vector3d>& C_list,
                            const std::vector<camera::Intrinsics>& K_list,
                            const std::vector<double>& u_px, const std::vector<double>& v_px,
                            double inlier_px, std::vector<bool>* inlier_mask, int* inlier_count,
                            double* mean_err) {
  const int N = static_cast<int>(R_list.size());
  inlier_mask->assign(static_cast<size_t>(N), false);
  *inlier_count = 0;
  double sum = 0.0;
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d Xc = R_list[i] * (X - C_list[i]);
    if (Xc(2) <= 0.0)
      continue;
    const double e = reproj_error_px(X, R_list[i], C_list[i], K_list[i], u_px[i], v_px[i]);
    if (e <= inlier_px) {
      (*inlier_mask)[static_cast<size_t>(i)] = true;
      ++(*inlier_count);
      sum += e;
    }
  }
  *mean_err = (*inlier_count > 0) ? (sum / static_cast<double>(*inlier_count)) : 1e20;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostic helpers for robust_triangulate_point_multiview counters
// ─────────────────────────────────────────────────────────────────────────────
namespace {
struct RobustTriDiag {
  int n2_total = 0, n2_dlt_fail = 0, n2_depth = 0, n2_angle = 0, n2_reproj = 0, n2_ok = 0;
  int n3p_total = 0, n3p_no_inliers = 0, n3p_ok = 0;
  // N>=3 failure sub-reasons
  int n3p_best_cnt_hist[4] = {}; // [0]=best_cnt<0 (no valid pair), [1]=best_cnt==0,
                                 // [2]=best_cnt==1, [3]=best_cnt>=2 (shouldn't fail)
  // Per-pair pre-check failures (accumulated across all failing tracks)
  int64_t n3p_pair_total = 0;    // total pairs tried in failing tracks
  int64_t n3p_pair_dlt_fail = 0; // DLT not finite
  int64_t n3p_pair_depth = 0;    // negative depth
  int64_t n3p_pair_angle = 0;    // angle out of range
  int64_t n3p_pair_passed = 0;   // passed pre-checks, went to count_inliers
  // Seed-pair reproj error distribution for passed pairs in failing tracks.
  // Values are capped at 1000px before summing to avoid Brown-Conrady blowup
  // (k3*r^6 diverges when DLT yields a point far outside the image FOV).
  // Counters use the raw (uncapped) value so they correctly reflect within-range pairs.
  int64_t n3p_seed_reproj_cnt = 0;
  double n3p_seed_reproj_sum_capped = 0.0; // sum of min(max(ea,eb), 1000) — for meaningful avg
  int n3p_seed_reproj_le16 = 0;            // max(ea,eb) <= 16px  (current RANSAC threshold)
  int n3p_seed_reproj_le20 = 0;            // max(ea,eb) <= 20px
  int n3p_seed_reproj_le30 = 0;            // max(ea,eb) <= 30px
  int n3p_seed_reproj_le40 = 0;            // max(ea,eb) <= 40px
  int n3p_seed_reproj_le50 = 0;            // max(ea,eb) <= 50px
};
static thread_local RobustTriDiag s_rtd;
void reset_robust_tri_diag() { s_rtd = {}; }
void log_robust_tri_diag() {
  VLOG(1) << "[DIAG] robust_tri N=2: total=" << s_rtd.n2_total << " dlt_fail=" << s_rtd.n2_dlt_fail
          << " depth=" << s_rtd.n2_depth << " angle=" << s_rtd.n2_angle
          << " reproj=" << s_rtd.n2_reproj << " ok=" << s_rtd.n2_ok;
  VLOG(1) << "[DIAG] robust_tri N>=3: total=" << s_rtd.n3p_total
          << " no_inliers=" << s_rtd.n3p_no_inliers << " ok=" << s_rtd.n3p_ok;
  VLOG(1) << "[DIAG] N>=3 fail best_cnt hist: <0(no_valid_pair)=" << s_rtd.n3p_best_cnt_hist[0]
          << " 0=" << s_rtd.n3p_best_cnt_hist[1] << " 1=" << s_rtd.n3p_best_cnt_hist[2]
          << " >=2=" << s_rtd.n3p_best_cnt_hist[3];
  VLOG(1) << "[DIAG] N>=3 fail pair pre-checks: total_pairs=" << s_rtd.n3p_pair_total
          << " dlt_fail=" << s_rtd.n3p_pair_dlt_fail << " depth=" << s_rtd.n3p_pair_depth
          << " angle=" << s_rtd.n3p_pair_angle << " passed=" << s_rtd.n3p_pair_passed;
  const double avg_seed_capped =
      s_rtd.n3p_seed_reproj_cnt > 0
          ? s_rtd.n3p_seed_reproj_sum_capped / static_cast<double>(s_rtd.n3p_seed_reproj_cnt)
          : 0.0;
  VLOG(1) << "[DIAG] N>=3 fail seed-pair reproj (cap=1000px for avg): n="
          << s_rtd.n3p_seed_reproj_cnt << " avg_capped=" << avg_seed_capped
          << " <=16px=" << s_rtd.n3p_seed_reproj_le16 << " <=20px=" << s_rtd.n3p_seed_reproj_le20
          << " <=30px=" << s_rtd.n3p_seed_reproj_le30 << " <=40px=" << s_rtd.n3p_seed_reproj_le40
          << " <=50px=" << s_rtd.n3p_seed_reproj_le50;
}

// ─────────────────────────────────────────────────────────────────────────────
// Triangulation backend switch
// ─────────────────────────────────────────────────────────────────────────────
//
// kTriDLTUndistPx = true   → TriangulateDLT(P = K[R|t], x = undistorted pixel)
//                   false  → triangulate_point_dlt_openmvg (rays_n + K)
//
// Both formulas build exactly the same DLT normal equations and are
// mathematically equivalent.  The switch exists to make the choice explicit
// and allows a one-line revert.  Undistorted pixel coords are recovered from
// rays_n as  x = rays_n * (fx, fy) + (cx, cy)  — a lossless linear transform.
constexpr bool kTriDLTUndistPx = true;

/// Dispatch triangulation for a single view-pair.
/// @param ray1 / ray2  Undistorted normalised coordinates (= K⁻¹ × undist_pixel).
/// @param K1 / K2      Per-view intrinsics.
static Eigen::Vector3d triangulate_pair(const Eigen::Matrix3d& R1, const Eigen::Vector3d& C1,
                                        const Eigen::Vector2d& ray1,
                                        const camera::Intrinsics& /*K1*/, const Eigen::Matrix3d& R2,
                                        const Eigen::Vector3d& C2, const Eigen::Vector2d& ray2,
                                        const camera::Intrinsics& /*K2*/) {
  // DLT in normalised coords: P = [R | -R*C], input = undistorted normalised ray.
  return triangulate_point_multiview({R1, R2}, {C1, C2}, {ray1, ray2});
}

} // namespace

bool robust_triangulate_point_multiview(const std::vector<Eigen::Matrix3d>& R_list,
                                        const std::vector<Eigen::Vector3d>& C_list,
                                        const std::vector<Eigen::Vector2d>& rays_n,
                                        const std::vector<camera::Intrinsics>& cameras_per_view,
                                        const std::vector<double>& u_px,
                                        const std::vector<double>& v_px,
                                        const RobustTriangulationOptions& opt,
                                        RobustTriangulationResult* out) {
  if (!out)
    return false;
  out->success = false;
  out->inlier_mask.clear();
  const int N = static_cast<int>(R_list.size());
  if (N < 2 || static_cast<int>(C_list.size()) != N || static_cast<int>(rays_n.size()) != N ||
      static_cast<int>(cameras_per_view.size()) != N || static_cast<int>(u_px.size()) != N ||
      static_cast<int>(v_px.size()) != N) {
    LOG(ERROR) << "Logic error: Invalid input sizes: N=" << N << ", R_list.size()=" << R_list.size()
               << ", C_list.size()=" << C_list.size() << ", rays_n.size()=" << rays_n.size()
               << ", cameras_per_view.size()=" << cameras_per_view.size()
               << ", u_px.size()=" << u_px.size() << ", v_px.size()=" << v_px.size();

    return false;
  }

  // Diagnostic counters accumulated in s_rtd (see RobustTriDiag).
  // Caller must call reset_robust_tri_diag() before a batch and log_robust_tri_diag() after.

  if (N == 2) {
    ++s_rtd.n2_total;
    if (opt.min_inlier_views > 2) {
      return false;
    }

    std::vector<Eigen::Matrix3d> R2 = {R_list[0], R_list[1]};
    std::vector<Eigen::Vector3d> C2 = {C_list[0], C_list[1]};

    Eigen::Vector3d X = triangulate_pair(R_list[0], C_list[0], rays_n[0], cameras_per_view[0],
                                         R_list[1], C_list[1], rays_n[1], cameras_per_view[1]);
    if (!X.allFinite() || X.norm() < 1e-12) {
      ++s_rtd.n2_dlt_fail;
      return false;
    }
    if (!check_all_depths_positive(X, R_list, C_list)) {
      ++s_rtd.n2_depth;
      return false;
    }
    const double ang_pre = compute_max_ray_angle_deg(X, C2);
    if (ang_pre < opt.min_tri_angle_deg || ang_pre > opt.max_tri_angle_deg) {
      ++s_rtd.n2_angle;
      return false;
    }
    double e0 = reproj_error_px(X, R_list[0], C_list[0], cameras_per_view[0], u_px[0], v_px[0]);
    double e1 = reproj_error_px(X, R_list[1], C_list[1], cameras_per_view[1], u_px[1], v_px[1]);
    if (e0 > opt.ransac_inlier_px || e1 > opt.ransac_inlier_px) {
      ++s_rtd.n2_reproj;
      return false;
    }
    // std::vector<size_t> idx = {0, 1};
    // Eigen::Vector3d X_gn =
    //     refine_point_multiview_gn(X, R_list, C_list, cameras_per_view, u_px, v_px, idx,
    //                               opt.gn_max_iterations, opt.gn_tolerance);
    // // Re-check reproj after GN.
    // const double e0_gn =
    //     reproj_error_px(X_gn, R_list[0], C_list[0], cameras_per_view[0], u_px[0], v_px[0]);
    // const double e1_gn =
    //     reproj_error_px(X_gn, R_list[1], C_list[1], cameras_per_view[1], u_px[1], v_px[1]);
    // // LOG(INFO) << "[dlt_cmp] pre_gn=(" << e0 << "," << e1 << ") post_gn=(" << e0_gn << "," <<
    // // e1_gn << ")";
    // if (e0_gn > opt.ransac_inlier_px || e1_gn > opt.ransac_inlier_px) {
    //   // LOG(INFO) << "post-GN reproj " << e0_gn << " or " << e1_gn << " > " <<
    //   opt.ransac_inlier_px
    //   //           << ", reject";
    //   return false;
    // }
    ++s_rtd.n2_ok;
    out->inlier_mask.resize(2);
    out->inlier_mask[0] = true;
    out->inlier_mask[1] = true;
    out->success = true;
    out->X = X;
    return true;
  }

  ++s_rtd.n3p_total;
  std::vector<std::pair<int, int>> pairs;
  pairs.reserve(static_cast<size_t>(N * (N - 1) / 2));
  for (int a = 0; a < N; ++a)
    for (int b = a + 1; b < N; ++b)
      pairs.push_back({a, b});

  std::mt19937 rng(42u);
  if (static_cast<int>(pairs.size()) > opt.ransac_max_pair_samples) {
    std::shuffle(pairs.begin(), pairs.end(), rng);
    pairs.resize(static_cast<size_t>(opt.ransac_max_pair_samples));
  }

  // Allow at most one outlier view: for N==min_inlier_views the strict threshold
  // would demand 100% agreement, leaving no tolerance. Clamp so there is always room
  // for one bad view; the outer reproj-filtering loop will delete it.
  const int effective_min_inliers = std::max(2, std::min(opt.min_inlier_views, N - 1));
  int best_cnt = -1;
  double best_mean = 1e20;
  Eigen::Vector3d best_X = Eigen::Vector3d::Zero();
  std::vector<bool> best_mask;

  // Track per-pair pre-check failures for diagnostic (only for this call).
  int local_pair_total = 0, local_pair_dlt = 0, local_pair_depth = 0, local_pair_angle = 0;
  int local_pair_passed = 0;
  // Seed-pair reproj stats — accumulated locally, merged into s_rtd only on failure so
  // that the reported distribution reflects FAILING tracks exclusively.
  int64_t local_seed_cnt = 0;
  double local_seed_sum_capped = 0.0;
  int local_seed_le16 = 0, local_seed_le20 = 0, local_seed_le30 = 0;
  int local_seed_le40 = 0, local_seed_le50 = 0;

  for (const auto& ab : pairs) {
    const int a = ab.first, b = ab.second;
    ++local_pair_total;
    const std::vector<Eigen::Vector3d> C2 = {C_list[a], C_list[b]};
    Eigen::Vector3d X = triangulate_pair(R_list[a], C_list[a], rays_n[a], cameras_per_view[a],
                                         R_list[b], C_list[b], rays_n[b], cameras_per_view[b]);
    if (!X.allFinite() || X.norm() < 1e-12) {
      ++local_pair_dlt;
      continue;
    }
    if ((R_list[a] * (X - C_list[a]))(2) <= 0.0 || (R_list[b] * (X - C_list[b]))(2) <= 0.0) {
      ++local_pair_depth;
      continue;
    }
    // Reject pairs with degenerate triangulation angle — small angles produce points at infinity
    // that artificially inflate the inlier count across views.
    const double pair_angle = compute_max_ray_angle_deg(X, C2);
    if (pair_angle < opt.min_tri_angle_deg || pair_angle > opt.max_tri_angle_deg) {
      ++local_pair_angle;
      continue;
    }
    ++local_pair_passed;
    std::vector<bool> mask;
    int ic = 0;
    double mean_e = 0.0;
    count_inliers_and_mask(X, R_list, C_list, cameras_per_view, u_px, v_px, opt.ransac_inlier_px,
                           &mask, &ic, &mean_e);
    // Track seed-pair reproj stats locally — will be merged into s_rtd only if this track fails.
    {
      const double ea =
          reproj_error_px(X, R_list[a], C_list[a], cameras_per_view[a], u_px[a], v_px[a]);
      const double eb =
          reproj_error_px(X, R_list[b], C_list[b], cameras_per_view[b], u_px[b], v_px[b]);
      const double seed_max = std::max(ea, eb); // raw; may be 1e20/1e31 for garbage DLT points
      ++local_seed_cnt;
      local_seed_sum_capped += std::min(seed_max, 1000.0);
      if (seed_max <= 16.0)
        ++local_seed_le16;
      if (seed_max <= 20.0)
        ++local_seed_le20;
      if (seed_max <= 30.0)
        ++local_seed_le30;
      if (seed_max <= 40.0)
        ++local_seed_le40;
      if (seed_max <= 50.0)
        ++local_seed_le50;
    }
    if (ic > best_cnt || (ic == best_cnt && mean_e < best_mean)) {
      best_cnt = ic;
      best_mean = mean_e;
      best_X = X;
      best_mask = std::move(mask);
    }
  }

  if (best_cnt < effective_min_inliers) {
    ++s_rtd.n3p_no_inliers;
    // Accumulate per-pair pre-check stats for failing tracks.
    s_rtd.n3p_pair_total += local_pair_total;
    s_rtd.n3p_pair_dlt_fail += local_pair_dlt;
    s_rtd.n3p_pair_depth += local_pair_depth;
    s_rtd.n3p_pair_angle += local_pair_angle;
    s_rtd.n3p_pair_passed += local_pair_passed;
    // Merge seed-pair reproj stats (only for failing tracks).
    s_rtd.n3p_seed_reproj_cnt += local_seed_cnt;
    s_rtd.n3p_seed_reproj_sum_capped += local_seed_sum_capped;
    s_rtd.n3p_seed_reproj_le16 += local_seed_le16;
    s_rtd.n3p_seed_reproj_le20 += local_seed_le20;
    s_rtd.n3p_seed_reproj_le30 += local_seed_le30;
    s_rtd.n3p_seed_reproj_le40 += local_seed_le40;
    s_rtd.n3p_seed_reproj_le50 += local_seed_le50;
    // Histogram of best_cnt.
    if (best_cnt < 0)
      ++s_rtd.n3p_best_cnt_hist[0];
    else if (best_cnt == 0)
      ++s_rtd.n3p_best_cnt_hist[1];
    else if (best_cnt == 1)
      ++s_rtd.n3p_best_cnt_hist[2];
    else
      ++s_rtd.n3p_best_cnt_hist[3];
    return false;
  }

  ++s_rtd.n3p_ok;
  out->inlier_mask = std::move(best_mask);
  out->X = best_X;
  out->success = true;
  return true;
}

/// Set on failure (success → 0). Used only for debug stats in run_batch_triangulation.
enum class TriFailCode : int {
  kOk = 0,
  kRobustMultiviewFailed = 1,
  kTwoViewDltDepthOrFinite = 2,
  kTwoViewAngleBeforeGn = 3,
  kTwoViewDepthAfterGn = 4,
  kTwoViewAngleAfterGn = 5,
  kTwoViewReprojReject = 6,
  kTooFewObsAfterOutlierDelete = 7,
  kConsolidateTriangulationFailed = 8,
  kInsufficientRegisteredViews = 9, ///< <2 registered observations (initial pass only)
};

/// Rebuild registered observation arrays from store. reg_obs_ids[i] ∥ reg_inds[i].
/// @param include_deleted_restorable  If false: only alive observations (normal incremental tri).
///   If true: also include logically-deleted observations that carry obs_flags::kRestorable
///   (reproj outliers), so run_retriangulation can re-triangulate after BA clears XYZ when only
///   restorable deleted views would bring nv back to >= 2.
static bool rebuild_registered_arrays_from_track(
    TrackStore* store, int track_id, int n_images, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    std::vector<int>* reg_obs_ids, std::vector<int>* reg_inds, std::vector<Eigen::Vector2d>* rays_n,
    std::vector<double>* u_px, std::vector<double>* v_px,
    std::vector<camera::Intrinsics>* K_per_view, bool include_deleted_restorable = false) {
  reg_obs_ids->clear();
  reg_inds->clear();
  rays_n->clear();
  u_px->clear();
  v_px->clear();
  K_per_view->clear();
  std::vector<int> ids;
  if (!include_deleted_restorable) {
    store->get_track_obs_ids(track_id, &ids);
  } else {
    store->get_track_all_obs_ids(track_id, &ids);
  }
  for (int oid : ids) {
    if (!store->is_obs_valid(oid)) {
      if (!include_deleted_restorable || !store->is_obs_restorable(oid))
        continue;
    }
    Observation o;
    store->get_obs(oid, &o);
    const int im = static_cast<int>(o.image_index);
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    const double u_raw = static_cast<double>(o.u), v_raw = static_cast<double>(o.v);
    const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
    // Undistort for ray computation (DLT needs undistorted normalized rays).
    // u_px/v_px keep the ORIGINAL distorted pixel coordinates so that
    // reproj_error_px (which projects 3D→distorted pixel via apply_distortion)
    // compares apples-to-apples.  Previously u_px/v_px stored undistorted coords
    // while reproj_error_px returned distorted → systematic mismatch that inflated
    // apparent error and killed most RANSAC inliers once K had non-zero distortion.
    double u_undist = u_raw, v_undist = v_raw;
    if (K.has_distortion()) {
      camera::undistort_point(K, u_raw, v_raw, &u_undist, &v_undist);
    }
    reg_obs_ids->push_back(oid);
    reg_inds->push_back(im);
    rays_n->push_back(Eigen::Vector2d((u_undist - K.cx) / K.fx, (v_undist - K.cy) / K.fy));
    u_px->push_back(u_raw); // distorted pixel — matches reproj_error_px output
    v_px->push_back(v_raw); // distorted pixel — matches reproj_error_px output
    K_per_view->push_back(K);
  }
  return static_cast<int>(reg_inds->size()) >= 2;
}

/**
 * Single entry for incremental triangulation (2 or N views).
 *
 * Flow:
 *  1. Build registered-view arrays from the store.
 *  2. Call robust_triangulate_point_multiview -- it runs RANSAC internally, refines on the inlier
 *     subset, and returns rr.X + rr.inlier_mask. Angle/depth are already checked inside.
 *  3. On success: use rr.inlier_mask to permanently mark outlier observations deleted in the store,
 *     then commit rr.X. No extra GN or outer loop needed.
 *  4. On failure with nv==2: fall back to bare DLT + GN (robust skips nv==2 when
 *     min_inlier_views > 2). Bad observation is deleted so retriangulation can recover later.
 *  5. On failure with nv>=3: report and return.
 */
static bool triangulate_track_common(
    TrackStore* store, int track_id, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, int n_images, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    double min_tri_angle_deg, const RobustTriangulationOptions& robust_opt,
    TriFailCode* fail_out = nullptr, bool include_deleted_restorable_obs = false) {
  auto set_fail = [&](TriFailCode c) {
    if (fail_out)
      *fail_out = c;
    return false;
  };

  RobustTriangulationOptions opt = robust_opt;
  opt.min_tri_angle_deg = min_tri_angle_deg;
  const double accept_px = opt.ransac_inlier_px;

  // --- Step 1: build registered arrays ---
  std::vector<int> reg_obs_ids, reg_inds;
  std::vector<Eigen::Vector2d> rays_n;
  std::vector<double> u_px, v_px;
  std::vector<camera::Intrinsics> K_per_view;
  if (!rebuild_registered_arrays_from_track(
          store, track_id, n_images, registered, cameras, image_to_camera_index, &reg_obs_ids,
          &reg_inds, &rays_n, &u_px, &v_px, &K_per_view, include_deleted_restorable_obs))
    return set_fail(TriFailCode::kInsufficientRegisteredViews);
  const int nv = static_cast<int>(reg_inds.size());
  if (nv < 2)
    return set_fail(TriFailCode::kInsufficientRegisteredViews);

  std::vector<Eigen::Matrix3d> R_list;
  std::vector<Eigen::Vector3d> C_list;
  R_list.reserve(static_cast<size_t>(nv));
  C_list.reserve(static_cast<size_t>(nv));
  for (int im : reg_inds) {
    R_list.push_back(poses_R[static_cast<size_t>(im)]);
    C_list.push_back(poses_C[static_cast<size_t>(im)]);
  }

  // --- Step 2: robust triangulation ---
  RobustTriangulationResult rr;
  const bool robust_ok = robust_triangulate_point_multiview(R_list, C_list, rays_n, K_per_view,
                                                            u_px, v_px, opt, &rr) &&
                         rr.success;

  if (!robust_ok) {
    return set_fail(TriFailCode::kRobustMultiviewFailed);
  }

  // --- Step 3: robust succeeded ---
  // rr.X is GN-refined on the inlier subset; angle and depth already checked inside robust.
  // Mark robust-outlier observations deleted and verify final reproj for each inlier view.
  // (N==2 inlier_mask is set post-GN; N>=3 inlier_mask is from count_inliers_and_mask post-GN.)
  int n_committed = 0;
  for (int i = 0; i < nv; ++i) {
    const bool is_robust_inlier = rr.inlier_mask[static_cast<size_t>(i)];
    if (is_robust_inlier) {
      // Final safety check: verify the actual reproj against rr.X matches what robust promised.
      const double e =
          reproj_error_px(rr.X, R_list[static_cast<size_t>(i)], C_list[static_cast<size_t>(i)],
                          K_per_view[static_cast<size_t>(i)], u_px[static_cast<size_t>(i)],
                          v_px[static_cast<size_t>(i)]);
      CHECK_LE(e, accept_px) << "reproj error " << e << " is less than accept px " << accept_px;
      const int oid_in = reg_obs_ids[static_cast<size_t>(i)];
      if (!store->is_obs_valid(oid_in))
        store->mark_observation_restored(oid_in);
      ++n_committed;
    } else {
      // RANSAC outlier: always mark as kRestorable (soft-delete) regardless of which pass.
      //
      // Earlier logic used permanent deletion for kFullScan on the assumption that it represents
      // a "final verdict", but that reasoning is wrong: poses improve continuously throughout the
      // incremental SfM run.  Permanently deleting registered-image observations in early kFullScan
      // passes (when poses are still rough) causes those observations to be unrecoverable later,
      // leaving tracks with too few registered-camera views to re-triangulate.  This starves
      // late-stage unregistered images of 3D-2D correspondences (they rely on those same tracks
      // being triangulated).  Marking as kRestorable lets retri_restore_deleted_obs_branch_a and
      // subsequent kFullScan passes reconsider the observation once poses have been refined by BA.
      store->mark_observation_deleted_restorable(reg_obs_ids[static_cast<size_t>(i)]);
    }
  }

  CHECK_GE(n_committed, 2) << "n_committed " << n_committed << " is less than 2";

  store->set_track_xyz(track_id, static_cast<float>(rr.X(0)), static_cast<float>(rr.X(1)),
                       static_cast<float>(rr.X(2)));
  if (fail_out)
    *fail_out = TriFailCode::kOk;
  return true;
}

static void reproj_mean_max_for_views(const Eigen::Vector3d& X, const std::vector<int>& reg_inds,
                                      const std::vector<double>& u_px,
                                      const std::vector<double>& v_px,
                                      const std::vector<camera::Intrinsics>& K_per_view,
                                      const std::vector<Eigen::Matrix3d>& poses_R,
                                      const std::vector<Eigen::Vector3d>& poses_C, double* mean_px,
                                      double* max_px) {
  const int nv = static_cast<int>(reg_inds.size());
  double s = 0.0;
  double mx = 0.0;
  for (int i = 0; i < nv; ++i) {
    const int im = reg_inds[i];
    const double e =
        reproj_error_px(X, poses_R[static_cast<size_t>(im)], poses_C[static_cast<size_t>(im)],
                        K_per_view[static_cast<size_t>(i)], u_px[static_cast<size_t>(i)],
                        v_px[static_cast<size_t>(i)]);
    s += e;
    if (e > mx)
      mx = e;
  }
  *mean_px = (nv > 0) ? (s / static_cast<double>(nv)) : 0.0;
  *max_px = mx;
}

static double percentile_on_sorted(const std::vector<double>& sorted, double p01) {
  if (sorted.empty())
    return 0.0;
  const size_t n = sorted.size();
  if (n == 1)
    return sorted[0];
  const double x = p01 * static_cast<double>(n - 1);
  const size_t i = static_cast<size_t>(std::floor(x));
  const size_t j = std::min(i + 1, n - 1);
  const double t = x - static_cast<double>(i);
  return sorted[i] * (1.0 - t) + sorted[j] * t;
}

// #region agent log
static void agent_log_tri_batch_ndjson(const std::string& data_json) {
  std::ofstream f("/home/jones/Git/01jones/InsightAT/InsightAT/.cursor/debug-2ba246.log",
                  std::ios::app);
  if (!f)
    return;
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
  f << "{\"sessionId\":\"2ba246\",\"hypothesisId\":\"H_tri_batch\",\"location\":"
       "\"incremental_triangulation.cpp:run_batch_triangulation\",\"message\":\"tri_batch_stats\","
       "\"data\":"
    << data_json << ",\"timestamp\":" << ms << "}\n";
}
// #endregion

/// Restore soft-deleted (kRestorable) observations on an already-triangulated track.
/// If only_images is non-null, only observations belonging to those image indices are considered.
/// Each candidate observation is checked against the current 3-D point: if the reprojection error
/// is within restore_strict_reproj_px the observation is marked alive again.
static int retri_restore_deleted_obs_branch_a(
    TrackStore* store, int track_id, int n_images, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    double restore_strict_reproj_px, const std::unordered_set<int>* only_images) {
  float tx, ty, tz;
  store->get_track_xyz(track_id, &tx, &ty, &tz);
  const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                          static_cast<double>(tz));
  std::vector<int> obs_ids_out;
  store->get_track_all_obs_ids(track_id, &obs_ids_out);
  int n_restored_obs = 0;
  for (int oid : obs_ids_out) {
    if (store->is_obs_valid(oid))
      continue;
    Observation obs;
    store->get_obs(oid, &obs);
    const int im = static_cast<int>(obs.image_index);
    if (only_images && !only_images->count(im))
      continue;
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
    const double e =
        reproj_error_px(X, poses_R[static_cast<size_t>(im)], poses_C[static_cast<size_t>(im)], K,
                        static_cast<double>(obs.u), static_cast<double>(obs.v));
    if (e <= restore_strict_reproj_px) {
      store->mark_observation_restored(oid);
      ++n_restored_obs;
    }
  }
  return n_restored_obs;
}

int run_batch_triangulation(TrackStore* store, const std::vector<int>& new_registered_image_indices,
                            const std::vector<Eigen::Matrix3d>& poses_R,
                            const std::vector<Eigen::Vector3d>& poses_C,
                            const std::vector<bool>& registered,
                            const std::vector<camera::Intrinsics>& cameras,
                            const std::vector<int>& image_to_camera_index, double min_tri_angle_deg,
                            std::vector<int>* new_track_ids_out, double commit_reproj_px) {
  if (new_track_ids_out)
    new_track_ids_out->clear();
  using Clock = std::chrono::steady_clock;
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;
  RobustTriangulationOptions robust_defaults;
  robust_defaults.min_tri_angle_deg = min_tri_angle_deg;
  robust_defaults.ransac_inlier_px = commit_reproj_px;

  // Build the set of new-image indices for fast lookup inside restore branch.
  std::unordered_set<int> new_images_set(new_registered_image_indices.begin(),
                                         new_registered_image_indices.end());

  // Scan ALL observations (valid + soft-deleted) on each newly registered image so we don't
  // miss tracks whose observation was previously marked kRestorable.
  std::unordered_set<int> candidate_set;     // no-XYZ tracks: need triangulation
  std::unordered_set<int> xyz_candidate_set; // already-XYZ tracks: need obs restore
  {
    std::vector<int> obs_ids_buf;
    for (int new_im : new_registered_image_indices) {
      obs_ids_buf.clear();
      store->get_image_all_obs_ids(new_im, &obs_ids_buf);
      for (int oid : obs_ids_buf) {
        const int tid = store->obs_track_id(oid);
        if (!store->is_track_valid(tid))
          continue;
        if (!store->track_has_triangulated_xyz(tid))
          candidate_set.insert(tid);
        else
          xyz_candidate_set.insert(tid);
      }
    }
  }
  int updated = 0;
  int tracks_scanned = 0, tracks_skipped_few_views = 0;
  int tracks_skipped_fail = 0;
  std::array<int, 11> fail_by_code{};
  std::vector<double> success_max_px;
  std::vector<double> success_mean_px;
  success_max_px.reserve(static_cast<size_t>(candidate_set.size()));
  success_mean_px.reserve(static_cast<size_t>(candidate_set.size()));
  std::vector<std::pair<double, int>> worst_tracks;
  worst_tracks.reserve(16);

  auto t0 = Clock::now();
  for (int track_id : candidate_set) {
    ++tracks_scanned;
    TriFailCode fcode = TriFailCode::kOk;
    if (triangulate_track_common(store, track_id, poses_R, poses_C, n_images, registered, cameras,
                                 image_to_camera_index, min_tri_angle_deg, robust_defaults,
                                 &fcode)) {
      ++updated;
      if (new_track_ids_out)
        new_track_ids_out->push_back(track_id);
      float tx, ty, tz;
      store->get_track_xyz(track_id, &tx, &ty, &tz);
      const Eigen::Vector3d Xstore(static_cast<double>(tx), static_cast<double>(ty),
                                   static_cast<double>(tz));
      std::vector<int> roid, rind;
      std::vector<Eigen::Vector2d> rn2;
      std::vector<double> up2, vp2;
      std::vector<camera::Intrinsics> Kp2;
      rebuild_registered_arrays_from_track(store, track_id, n_images, registered, cameras,
                                           image_to_camera_index, &roid, &rind, &rn2, &up2, &vp2,
                                           &Kp2);
      double mpx = 0.0, mxpx = 0.0;
      reproj_mean_max_for_views(Xstore, rind, up2, vp2, Kp2, poses_R, poses_C, &mpx, &mxpx);
      success_mean_px.push_back(mpx);
      success_max_px.push_back(mxpx);
      worst_tracks.push_back({mxpx, track_id});
      std::sort(worst_tracks.begin(), worst_tracks.end(),
                [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                  return a.first > b.first;
                });
      if (worst_tracks.size() > 12)
        worst_tracks.resize(12);
    } else {
      if (fcode == TriFailCode::kInsufficientRegisteredViews)
        ++tracks_skipped_few_views;
      else {
        ++tracks_skipped_fail;
        const int ci = static_cast<int>(fcode);
        if (ci >= 1 && ci < static_cast<int>(fail_by_code.size()))
          ++fail_by_code[static_cast<size_t>(ci)];
      }
    }
  }
  auto t1 = std::chrono::steady_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  // ── Pass 2: restore soft-deleted observations on already-triangulated tracks ──────────────────
  // When a new image registers, its observations on existing 3-D tracks are already in the store
  // (flagged kRestorable or simply valid but previously excluded because registered[im]=false).
  // retri_restore_deleted_obs_branch_a checks whether the current 3-D point reprojects within
  // commit_reproj_px to the new image and marks the observation alive again.  Valid observations
  // need no action (BA will automatically include them now that registered[im]=true).
  int n_xyz_restored_obs = 0, n_xyz_restored_tracks = 0;
  for (int tid : xyz_candidate_set) {
    if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
      continue;
    const int n = retri_restore_deleted_obs_branch_a(store, tid, n_images, poses_R, poses_C,
                                                     registered, cameras, image_to_camera_index,
                                                     commit_reproj_px, &new_images_set);
    if (n > 0) {
      n_xyz_restored_obs += n;
      ++n_xyz_restored_tracks;
    }
  }
  // ─────────────────────────────────────────────────────────────────────────────

  VLOG(1) << "[PERF] run_batch_triangulation: " << ms << "ms"
          << "  total_tracks=" << store->num_tracks() << "  candidates=" << candidate_set.size()
          << "  scanned=" << tracks_scanned << "  skip_few_views=" << tracks_skipped_few_views
          << "  fail=" << tracks_skipped_fail << "  newly_tri=" << updated
          << "  xyz_touched=" << xyz_candidate_set.size()
          << "  xyz_restored_tracks=" << n_xyz_restored_tracks
          << "  xyz_restored_obs=" << n_xyz_restored_obs
          << "  commit_reproj_px=" << commit_reproj_px;
  if (!success_max_px.empty()) {
    std::vector<double> sorted_max = success_max_px;
    std::sort(sorted_max.begin(), sorted_max.end());
    double sum_max = 0.0, sum_mean = 0.0;
    for (double x : success_max_px)
      sum_max += x;
    for (double x : success_mean_px)
      sum_mean += x;
    const double avg_max = sum_max / static_cast<double>(success_max_px.size());
    const double avg_mean = sum_mean / static_cast<double>(success_mean_px.size());
    const double p50 = percentile_on_sorted(sorted_max, 0.5);
    const double p90 = percentile_on_sorted(sorted_max, 0.9);
    const double worst = sorted_max.back();
    int n_gt16 = 0, n_gt64 = 0;
    for (double x : success_max_px) {
      if (x > 16.0)
        ++n_gt16;
      if (x > 64.0)
        ++n_gt64;
    }
    LOG(INFO) << "[tri_debug] newly_tri reproj (per track, registered views only): n=" << updated
              << "  mean_of_mean_px=" << avg_mean << "  mean_of_max_px=" << avg_max
              << "  p50_max_px=" << p50 << "  p90_max_px=" << p90 << "  global_max_max_px=" << worst
              << "  n_max_gt_16px=" << n_gt16 << "  n_max_gt_64px=" << n_gt64
              << "  min_tri_angle_deg=" << min_tri_angle_deg;
    LOG(INFO) << "[tri_debug] fail_reasons: robust=" << fail_by_code[1]
              << "  two_dlt_depth=" << fail_by_code[2] << "  ang_before_gn=" << fail_by_code[3]
              << "  depth_after_gn=" << fail_by_code[4] << "  ang_after_gn=" << fail_by_code[5]
              << "  two_reproj=" << fail_by_code[6] << "  few_after_del=" << fail_by_code[7]
              << "  consolidate=" << fail_by_code[8] << "  insufficient_views=" << fail_by_code[9];
    std::ostringstream w;
    w << "  worst_max_reproj_tracks: ";
    for (size_t i = 0; i < worst_tracks.size() && i < 8; ++i)
      w << "tid=" << worst_tracks[i].second << " max=" << worst_tracks[i].first << "px  ";
    LOG(INFO) << "[tri_debug]" << w.str();
  } else if (tracks_skipped_fail > 0) {
    LOG(INFO) << "[tri_debug] fail_reasons: robust=" << fail_by_code[1]
              << "  two_dlt_depth=" << fail_by_code[2] << "  ang_before_gn=" << fail_by_code[3]
              << "  depth_after_gn=" << fail_by_code[4] << "  ang_after_gn=" << fail_by_code[5]
              << "  two_reproj=" << fail_by_code[6] << "  few_after_del=" << fail_by_code[7]
              << "  consolidate=" << fail_by_code[8] << "  insufficient_views=" << fail_by_code[9];
  }

  {
    std::ostringstream dj;
    dj << "{"
       << "\"candidates\":" << candidate_set.size() << ",\"scanned\":" << tracks_scanned
       << ",\"skip_few_views\":" << tracks_skipped_few_views << ",\"fail\":" << tracks_skipped_fail
       << ",\"newly_tri\":" << updated << ",\"min_tri_angle_deg\":" << min_tri_angle_deg
       << ",\"commit_reproj_px\":" << commit_reproj_px << ",\"fail_robust\":" << fail_by_code[1]
       << ",\"fail_two_dlt\":" << fail_by_code[2] << ",\"fail_ang0\":" << fail_by_code[3]
       << ",\"fail_depth_gn\":" << fail_by_code[4] << ",\"fail_ang1\":" << fail_by_code[5]
       << ",\"fail_two_reproj\":" << fail_by_code[6]
       << ",\"fail_few_after_del\":" << fail_by_code[7]
       << ",\"fail_consolidate\":" << fail_by_code[8]
       << ",\"fail_insufficient_views\":" << fail_by_code[9];
    if (!success_max_px.empty()) {
      std::vector<double> sorted_max = success_max_px;
      std::sort(sorted_max.begin(), sorted_max.end());
      double sum_max = 0.0, sum_mean = 0.0;
      for (double x : success_max_px)
        sum_max += x;
      for (double x : success_mean_px)
        sum_mean += x;
      int n_gt16 = 0, n_gt64 = 0;
      for (double x : success_max_px) {
        if (x > 16.0)
          ++n_gt16;
        if (x > 64.0)
          ++n_gt64;
      }
      dj << ",\"mean_of_max_px\":" << (sum_max / static_cast<double>(success_max_px.size()))
         << ",\"mean_of_mean_px\":" << (sum_mean / static_cast<double>(success_mean_px.size()))
         << ",\"p50_max_px\":" << percentile_on_sorted(sorted_max, 0.5)
         << ",\"p90_max_px\":" << percentile_on_sorted(sorted_max, 0.9)
         << ",\"worst_max_px\":" << sorted_max.back() << ",\"n_max_gt_16\":" << n_gt16
         << ",\"n_max_gt_64\":" << n_gt64;
    }
    dj << "}";
    agent_log_tri_batch_ndjson(dj.str());
  }

  return updated;
}

namespace {

const char* retri_scope_name(RetriangulationScope s) {
  switch (s) {
  case RetriangulationScope::kPendingOnly:
    return "pending";
  case RetriangulationScope::kNewImages:
    return "new_images";
  case RetriangulationScope::kFullScan:
    return "full_scan";
  default:
    return "?";
  }
}

// retri_restore_deleted_obs_branch_a — defined above run_batch_triangulation (needs it there too).

} // namespace

int run_retriangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                        const std::vector<Eigen::Vector3d>& poses_C,
                        const std::vector<bool>& registered,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index,
                        const IncrementalRetriangulationOptions& opts) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;
  using Clock = std::chrono::steady_clock;
  auto t0 = Clock::now();

  const RetriangulationScope scope = opts.scope;
  int n_restored_track = 0, n_restored_obs = 0;
  int n_retri = 0, n_retri_fail = 0;
  size_t input_list_size = 0;

  if (scope == RetriangulationScope::kFullScan) {
    // Empty pending queue so flags and queue stay consistent with a full pass over all tracks.
    std::vector<int> drained;
    store->drain_retriangulation_pending(&drained);
    (void)drained;

    RobustTriangulationOptions robust_no_xyz = opts.robust;
    if (opts.full_scan_commit_reproj_px > 0.0)
      robust_no_xyz.ransac_inlier_px = opts.full_scan_commit_reproj_px;

    // Failure-reason histogram for diagnostics (indexed by TriFailCode).
    int fail_hist[10] = {};
    int n_already_tri = 0, n_invalid = 0;
    int n_nv_hist[8] = {}; // nv=0..6,7+
    reset_robust_tri_diag();

    for (int track_id = 0; track_id < static_cast<int>(store->num_tracks()); ++track_id) {
      if (!store->is_track_valid(track_id)) {
        store->set_track_retriangulation_flag(track_id, false);
        ++n_invalid;
        continue;
      }
      if (!store->track_has_triangulated_xyz(track_id)) {
        TriFailCode fcode = TriFailCode::kOk;
        if (triangulate_track_common(store, track_id, poses_R, poses_C, n_images, registered,
                                     cameras, image_to_camera_index,
                                     robust_no_xyz.min_tri_angle_deg, robust_no_xyz, &fcode,
                                     /*include_deleted_restorable_obs=*/true)) {
          ++n_retri;
        } else {
          ++n_retri_fail;
          fail_hist[std::min(static_cast<int>(fcode), 9)]++;
          // Count nv for insufficient-views failures
          if (fcode == TriFailCode::kInsufficientRegisteredViews) {
            // Quick count of registered views for this track
            std::vector<int> ids;
            store->get_track_all_obs_ids(track_id, &ids);
            int nv = 0;
            for (int oid : ids) {
              if (!store->is_obs_valid(oid) && !store->is_obs_restorable(oid))
                continue;
              Observation o;
              store->get_obs(oid, &o);
              int im = static_cast<int>(o.image_index);
              if (im >= 0 && im < n_images && registered[static_cast<size_t>(im)])
                ++nv;
            }
            n_nv_hist[std::min(nv, 7)]++;
          }
        }
      } else {
        ++n_already_tri;
        const int n_obs_rest = retri_restore_deleted_obs_branch_a(
            store, track_id, n_images, poses_R, poses_C, registered, cameras, image_to_camera_index,
            opts.restore_strict_reproj_px, /*only_images=*/nullptr);
        if (n_obs_rest > 0) {
          n_restored_obs += n_obs_rest;
          ++n_restored_track;
        }
      }
      store->set_track_retriangulation_flag(track_id, false);
    }

    auto t1 = Clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    LOG(INFO) << "[PERF] run_retriangulation: " << ms << "ms"
              << "  scope=" << retri_scope_name(scope) << "  retri_ok=" << n_retri
              << "  retri_fail=" << n_retri_fail << "  already_tri=" << n_already_tri
              << "  invalid=" << n_invalid << "  restored_tracks=" << n_restored_track
              << "  restored_obs=" << n_restored_obs;
    LOG(INFO) << "[DIAG] fail reasons:"
              << " InsufficientViews="
              << fail_hist[static_cast<int>(TriFailCode::kInsufficientRegisteredViews)]
              << " RobustFailed="
              << fail_hist[static_cast<int>(TriFailCode::kRobustMultiviewFailed)]
              << " DltDepth=" << fail_hist[static_cast<int>(TriFailCode::kTwoViewDltDepthOrFinite)]
              << " PreGnAngle=" << fail_hist[static_cast<int>(TriFailCode::kTwoViewAngleBeforeGn)]
              << " PostGnDepth=" << fail_hist[static_cast<int>(TriFailCode::kTwoViewDepthAfterGn)]
              << " PostGnAngle=" << fail_hist[static_cast<int>(TriFailCode::kTwoViewAngleAfterGn)]
              << " ReprojReject=" << fail_hist[static_cast<int>(TriFailCode::kTwoViewReprojReject)];
    log_robust_tri_diag();
    LOG(INFO) << "[DIAG] InsufficientViews nv distribution: nv0=" << n_nv_hist[0]
              << " nv1=" << n_nv_hist[1] << " nv2+=" << n_nv_hist[2];
    return n_retri + n_restored_track;
  }

  // kFullScan is fully handled above and has already returned.
  // Only kPendingOnly and kNewImages reach here.
  if (scope != RetriangulationScope::kPendingOnly && scope != RetriangulationScope::kNewImages) {
    LOG(ERROR) << "run_retriangulation: unhandled scope " << static_cast<int>(scope);
    return 0;
  }

  std::vector<int> retri_track_ids;
  std::unordered_set<int> restrict_set;
  const std::unordered_set<int>* restrict_ptr = nullptr;

  if (scope == RetriangulationScope::kPendingOnly) {
    store->drain_retriangulation_pending(&retri_track_ids);
    std::unordered_set<int> seen;
    std::vector<int> uniq;
    uniq.reserve(retri_track_ids.size());
    for (int tid : retri_track_ids) {
      if (seen.insert(tid).second)
        uniq.push_back(tid);
    }
    retri_track_ids = std::move(uniq);
    input_list_size = retri_track_ids.size();
  } else {
    // kNewImages
    for (int im : opts.restrict_image_indices)
      restrict_set.insert(im);
    restrict_ptr = &restrict_set;
    if (restrict_set.empty()) {
      LOG(WARNING) << "run_retriangulation: scope=new_images but restrict_image_indices is empty";
      return 0;
    }
    std::unordered_set<int> seen_tracks;
    for (int tid = 0; tid < static_cast<int>(store->num_tracks()); ++tid) {
      if (!store->is_track_valid(tid))
        continue;
      std::vector<int> oids;
      store->get_track_all_obs_ids(tid, &oids);
      bool touches = false;
      for (int oid : oids) {
        Observation obs;
        store->get_obs(oid, &obs);
        const int im = static_cast<int>(obs.image_index);
        if (restrict_set.count(im)) {
          touches = true;
          break;
        }
      }
      if (touches && seen_tracks.insert(tid).second)
        retri_track_ids.push_back(tid);
    }
    input_list_size = retri_track_ids.size();
  }

  for (int track_id : retri_track_ids) {
    if (!store->is_track_valid(track_id)) {
      store->set_track_retriangulation_flag(track_id, false);
      continue;
    }
    if (store->track_has_triangulated_xyz(track_id)) {
      const int n_obs_rest = retri_restore_deleted_obs_branch_a(
          store, track_id, n_images, poses_R, poses_C, registered, cameras, image_to_camera_index,
          opts.restore_strict_reproj_px, restrict_ptr);
      if (n_obs_rest > 0) {
        n_restored_obs += n_obs_rest;
        ++n_restored_track;
      }
    } else {
      TriFailCode fcode = TriFailCode::kOk;
      if (triangulate_track_common(store, track_id, poses_R, poses_C, n_images, registered, cameras,
                                   image_to_camera_index, opts.robust.min_tri_angle_deg,
                                   opts.robust, &fcode,
                                   /*include_deleted_restorable_obs=*/true)) {
        ++n_retri;
      } else {
        ++n_retri_fail;
      }
    }
    store->set_track_retriangulation_flag(track_id, false);
  }

  auto t1 = Clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG(INFO) << "[PERF] run_retriangulation: " << ms << "ms"
            << "  scope=" << retri_scope_name(scope) << "  input_tracks=" << input_list_size
            << "  retri_ok=" << n_retri << "  retri_fail=" << n_retri_fail
            << "  restored_tracks=" << n_restored_track << "  restored_obs=" << n_restored_obs;
  return n_retri + n_restored_track;
}

int run_full_scan_triangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                                const std::vector<Eigen::Vector3d>& poses_C,
                                const std::vector<bool>& registered,
                                const std::vector<camera::Intrinsics>& cameras,
                                const std::vector<int>& image_to_camera_index,
                                double min_tri_angle_deg, double commit_reproj_px) {
  IncrementalRetriangulationOptions o;
  o.scope = RetriangulationScope::kFullScan;
  o.robust.min_tri_angle_deg = min_tri_angle_deg;
  o.robust.ransac_inlier_px = 16.0;
  o.full_scan_commit_reproj_px = 16.0; // use a looser reproj threshold for tracks that had no XYZ
                                       // (see robust_no_xyz in run_retriangulation)
  o.restore_strict_reproj_px = commit_reproj_px;
  return run_retriangulation(store, poses_R, poses_C, registered, cameras, image_to_camera_index,
                             o);
}

int restore_observations_from_cameras(
    TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    const std::unordered_set<int>& changed_cam_model_indices, double strict_reproj_px) {
  if (!store)
    return 0;
  const int n_images = store->num_images();
  if (static_cast<int>(poses_R.size()) != n_images ||
      static_cast<int>(poses_C.size()) != n_images ||
      static_cast<int>(registered.size()) != n_images)
    return 0;

  const bool all_cameras = changed_cam_model_indices.empty();
  int restored = 0;

  // Build list of images that belong to changed camera models.
  // Using the pre-built per-image obs index (image_obs_ids_) avoids scanning all observations.
  std::vector<int> obs_ids;
  for (int im = 0; im < n_images; ++im) {
    if (!registered[static_cast<size_t>(im)])
      continue;
    const int cam_idx = image_to_camera_index[static_cast<size_t>(im)];
    if (!all_cameras && changed_cam_model_indices.find(cam_idx) == changed_cam_model_indices.end())
      continue;

    obs_ids.clear();
    store->get_image_all_obs_ids(im, &obs_ids);
    // get_image_all_obs_ids returns ALL obs for the image (alive + deleted).
    for (int oid : obs_ids) {
      if (!store->is_obs_restorable(oid))
        continue; // alive, or deleted for a non-reproj reason (depth, angle, PnP)

      const int tid = store->obs_track_id(oid);
      if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
        continue; // degenerate track — handled by pending-queue run_retriangulation

      float tx, ty, tz;
      store->get_track_xyz(tid, &tx, &ty, &tz);
      const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                              static_cast<double>(tz));

      Observation obs;
      store->get_obs(oid, &obs);
      const camera::Intrinsics& K =
          cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
      const double e =
          reproj_error_px(X, poses_R[static_cast<size_t>(im)], poses_C[static_cast<size_t>(im)], K,
                          static_cast<double>(obs.u), static_cast<double>(obs.v));
      if (e <= strict_reproj_px) {
        store->mark_observation_restored(oid);
        ++restored;
      }
    }
  }

  VLOG(1) << "[restore_obs] restored=" << restored
          << "  changed_cams=" << changed_cam_model_indices.size()
          << "  strict_px=" << strict_reproj_px;
  return restored;
}

} // namespace sfm
} // namespace insight
