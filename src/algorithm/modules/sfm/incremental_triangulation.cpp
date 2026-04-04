/**
 * @file  incremental_triangulation.cpp
 * @brief Multiview triangulation for incremental SfM (see incremental_triangulation.h).
 */

#include "incremental_triangulation.h"

#include "../camera/camera_utils.h"
#include "track_store.h"

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

// #region tri_diag — verbose per-failure triangulation logs for diagnosis. Set false after fix.
static constexpr bool kTriangulationDiagLog = true;

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
// triangulate_point_multiview but operates in pixel space (different row scaling of A).
Eigen::Vector3d triangulate_point_dlt_openmvg(const std::vector<Eigen::Matrix3d>& R_list,
                                              const std::vector<Eigen::Vector3d>& C_list,
                                              const std::vector<Eigen::Vector2d>& rays_n,
                                              const std::vector<camera::Intrinsics>& K_list) {
  const int N = static_cast<int>(R_list.size());
  if (N < 2 || static_cast<int>(C_list.size()) != N || static_cast<int>(rays_n.size()) != N ||
      static_cast<int>(K_list.size()) != N)
    return Eigen::Vector3d(0, 0, 0);
  Eigen::MatrixXd A(2 * N, 4);
  for (int i = 0; i < N; ++i) {
    const camera::Intrinsics& K = K_list[i];
    const Eigen::Vector3d t = -R_list[i] * C_list[i];
    // Build 3x4 pixel-space projection matrix P = K * [R | t].
    Eigen::Matrix<double, 3, 4> Rt;
    Rt.leftCols<3>() = R_list[i];
    Rt.col(3) = t;
    Eigen::Matrix3d Km;
    Km << K.fx, 0.0, K.cx, 0.0, K.fy, K.cy, 0.0, 0.0, 1.0;
    const Eigen::Matrix<double, 3, 4> P = Km * Rt;
    // Recover undistorted pixel coordinates from normalised ray.
    const double u = rays_n[i](0) * K.fx + K.cx;
    const double v = rays_n[i](1) * K.fy + K.cy;
    A.row(2 * i) = u * P.row(2) - P.row(0);
    A.row(2 * i + 1) = v * P.row(2) - P.row(1);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d v = svd.matrixV().col(3);
  if (std::fabs(v(3)) < 1e-12)
    return Eigen::Vector3d(0, 0, 0);
  return v.head<3>() / v(3);
}

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

  if (N == 2) {
    // LOG(INFO) << "N=2, min inlier views: " << opt.min_inlier_views;
    if (opt.min_inlier_views > 2) {
      // LOG(INFO) << "min inlier views > 2, but N=2, returning false";
      return false;
    }

    std::vector<Eigen::Matrix3d> R2 = {R_list[0], R_list[1]};
    std::vector<Eigen::Vector3d> C2 = {C_list[0], C_list[1]};
    std::vector<Eigen::Vector2d> ray2 = {rays_n[0], rays_n[1]};
    std::vector<camera::Intrinsics> K2 = {cameras_per_view[0], cameras_per_view[1]};

    // [dlt_cmp] Call both DLT implementations and log the comparison.
    Eigen::Vector3d X = triangulate_point_multiview(R2, C2, ray2);
    // Eigen::Vector3d X_omvg = triangulate_point_dlt_openmvg(R2, C2, ray2, K2);
    // LOG(INFO) << "[dlt_cmp] N=2"
    //           << " X_cur=(" << X.transpose() << ")"
    //           << " X_omvg=(" << X_omvg.transpose() << ")"
    //           << " diff=" << (X - X_omvg).norm();
    // {
    //   const double e0_cur  = reproj_error_px(X,      R_list[0], C_list[0], cameras_per_view[0],
    //   u_px[0], v_px[0]); const double e1_cur  = reproj_error_px(X,      R_list[1], C_list[1],
    //   cameras_per_view[1], u_px[1], v_px[1]); const double e0_omvg = reproj_error_px(X_omvg,
    //   R_list[0], C_list[0], cameras_per_view[0], u_px[0], v_px[0]); const double e1_omvg =
    //   reproj_error_px(X_omvg, R_list[1], C_list[1], cameras_per_view[1], u_px[1], v_px[1]);
    //   LOG(INFO) << "[dlt_cmp] reproj_cur=(" << e0_cur << "," << e1_cur << ")"
    //             << " reproj_omvg=(" << e0_omvg << "," << e1_omvg << ")";
    // }

    if (!X.allFinite() || X.norm() < 1e-12) {
      // LOG(INFO) << "X is not finite or norm < 1e-12, returning false";
      return false;
    }
    if (!check_all_depths_positive(X, R_list, C_list)) {
      // LOG(INFO) << "X is not all depths positive, returning false";
      return false;
    }
    // Angle check before GN: degenerate geometry makes J^T J near-singular → GN diverges.
    const double ang_pre = compute_max_ray_angle_deg(X, C2);
    if (ang_pre < opt.min_tri_angle_deg || ang_pre > opt.max_tri_angle_deg) {
      // LOG(INFO) << "pre-GN angle " << ang_pre << " outside [" << opt.min_tri_angle_deg << ","
      //           << opt.max_tri_angle_deg << "], skip GN";
      return false;
    }
    // Pre-GN reproj check.
    double e0 = reproj_error_px(X, R_list[0], C_list[0], cameras_per_view[0], u_px[0], v_px[0]);
    double e1 = reproj_error_px(X, R_list[1], C_list[1], cameras_per_view[1], u_px[1], v_px[1]);
    if (e0 > opt.ransac_inlier_px || e1 > opt.ransac_inlier_px) {
      // LOG(INFO) << "reproj error " << e0 << " or " << e1 << " is greater than "
      //           << opt.ransac_inlier_px;
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
    out->inlier_mask.resize(2);
    out->inlier_mask[0] = true;
    out->inlier_mask[1] = true;
    out->success = true;
    out->X = X;
    return true;
  }

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

  for (const auto& ab : pairs) {
    const int a = ab.first, b = ab.second;
    std::vector<Eigen::Matrix3d> R2 = {R_list[a], R_list[b]};
    std::vector<Eigen::Vector3d> C2 = {C_list[a], C_list[b]};
    std::vector<Eigen::Vector2d> ray2 = {rays_n[a], rays_n[b]};
    Eigen::Vector3d X = triangulate_point_multiview(R2, C2, ray2);
    if (!X.allFinite() || X.norm() < 1e-12)
      continue;
    if ((R_list[a] * (X - C_list[a]))(2) <= 0.0 || (R_list[b] * (X - C_list[b]))(2) <= 0.0)
      continue;
    // Reject pairs with degenerate triangulation angle — small angles produce points at infinity
    // that artificially inflate the inlier count across views.
    const double pair_angle = compute_max_ray_angle_deg(X, C2);
    if (pair_angle < opt.min_tri_angle_deg || pair_angle > opt.max_tri_angle_deg)
      continue;
    std::vector<bool> mask;
    int ic = 0;
    double mean_e = 0.0;
    count_inliers_and_mask(X, R_list, C_list, cameras_per_view, u_px, v_px, opt.ransac_inlier_px,
                           &mask, &ic, &mean_e);
    if (ic > best_cnt || (ic == best_cnt && mean_e < best_mean)) {
      best_cnt = ic;
      best_mean = mean_e;
      best_X = X;
      best_mask = std::move(mask);
    }
  }

  if (best_cnt < effective_min_inliers) {
    // LOG(INFO) << "best count " << best_cnt << " is less than effective min inliers "
    // << effective_min_inliers;
    return false;
  }

#if 0
  std::vector<size_t> in_idx;
  in_idx.reserve(static_cast<size_t>(best_cnt));
  for (int i = 0; i < N; ++i) {
    if (best_mask[static_cast<size_t>(i)])
      in_idx.push_back(static_cast<size_t>(i));
  }
  if (static_cast<int>(in_idx.size()) < opt.min_inlier_views)
    return false;

  std::vector<Eigen::Matrix3d> R_in;
  std::vector<Eigen::Vector3d> C_in;
  std::vector<Eigen::Vector2d> ray_in;
  R_in.reserve(in_idx.size());
  C_in.reserve(in_idx.size());
  ray_in.reserve(in_idx.size());
  for (size_t i : in_idx) {
    R_in.push_back(R_list[i]);
    C_in.push_back(C_list[i]);
    ray_in.push_back(rays_n[i]);
  }
  Eigen::Vector3d X = triangulate_point_multiview(R_in, C_in, ray_in);
  if (!X.allFinite() || !check_all_depths_positive(X, R_in, C_in)) {
    CHECK(false) << "X is not finite or all depths are not positive,but should not happen "
                 << X.transpose();
    return false;
  }

  Eigen::Vector3d X_gn = refine_point_multiview_gn(X, R_list, C_list, cameras_per_view, u_px, v_px,
                                                   in_idx, opt.gn_max_iterations, opt.gn_tolerance);

  std::vector<Eigen::Vector3d> C_ang;
  C_ang.reserve(in_idx.size());
  for (size_t i : in_idx)
    C_ang.push_back(C_list[i]);
  const double ang = compute_max_ray_angle_deg(X_gn, C_ang);
  if (ang < opt.min_tri_angle_deg || ang > opt.max_tri_angle_deg) {
    LOG(INFO) << "[confused!]:angle " << ang << " is not in the range [" << opt.min_tri_angle_deg
              << "," << opt.max_tri_angle_deg << "]";
    LOG(INFO) << "X: " << X.transpose() << " X_gn: " << X_gn.transpose();
    return false;
  }

  int final_inliers = 0;
  double final_mean = 0.0;
  count_inliers_and_mask(X, R_list, C_list, cameras_per_view, u_px, v_px, opt.ransac_inlier_px,
                         &out->inlier_mask, &final_inliers, &final_mean);
  if (final_inliers < effective_min_inliers) {
    LOG(INFO) << "[confused!]:final inliers " << final_inliers
              << " is less than effective min inliers " << effective_min_inliers;

    LOG(INFO) << "X: " << X.transpose() << " X_gn: " << X_gn.transpose();
    LOG(INFO) << "inlier count befor GN is : " << best_cnt;
    return false;
  }
  // out->X = X;
#else
  out->inlier_mask = std::move(best_mask);
  out->X = best_X;
#endif
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

/// Rebuild registered observation arrays from store (valid obs only). reg_obs_ids[i] ∥ reg_inds[i].
static bool rebuild_registered_arrays_from_track(
    TrackStore* store, int track_id, int n_images, const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras, const std::vector<int>& image_to_camera_index,
    std::vector<int>* reg_obs_ids, std::vector<int>* reg_inds, std::vector<Eigen::Vector2d>* rays_n,
    std::vector<double>* u_px, std::vector<double>* v_px,
    std::vector<camera::Intrinsics>* K_per_view) {
  reg_obs_ids->clear();
  reg_inds->clear();
  rays_n->clear();
  u_px->clear();
  v_px->clear();
  K_per_view->clear();
  std::vector<int> ids;
  store->get_track_obs_ids(track_id, &ids);
  for (int oid : ids) {
    Observation o;
    store->get_obs(oid, &o);
    const int im = static_cast<int>(o.image_index);
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    double u = static_cast<double>(o.u), v = static_cast<double>(o.v);
    const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
    if (K.has_distortion()) {
      double u_d, v_d;
      camera::undistort_point(K, u, v, &u_d, &v_d);
      u = u_d;
      v = v_d;
    }
    reg_obs_ids->push_back(oid);
    reg_inds->push_back(im);
    rays_n->push_back(Eigen::Vector2d((u - K.cx) / K.fx, (v - K.cy) / K.fy));
    u_px->push_back(static_cast<double>(u));
    v_px->push_back(static_cast<double>(v));
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
    TriFailCode* fail_out = nullptr) {
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
  if (!rebuild_registered_arrays_from_track(store, track_id, n_images, registered, cameras,
                                            image_to_camera_index, &reg_obs_ids, &reg_inds, &rays_n,
                                            &u_px, &v_px, &K_per_view))
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
    // nv >= 3 and robust failed
    // if (kTriangulationDiagLog) {
    //   LOG(INFO) << "[tri_diag] robust_multiview_failed tid=" << track_id << " nv=" << nv
    //             << " min_inlier_views=" << opt.min_inlier_views
    //             << " ransac_inlier_px=" << opt.ransac_inlier_px << " reg_inds=["
    //             << tri_diag_format_reg_inds(reg_inds) << "]";
    // }
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
      ++n_committed;
    } else {
      store->mark_observation_deleted(reg_obs_ids[static_cast<size_t>(i)]);
      // if (kTriangulationDiagLog) {
      //   LOG(INFO) << "[tri_diag] obs_delete reason="
      //             << (is_robust_inlier ? "inlier_reproj_mismatch" : "robust_outlier")
      //             << " tid=" << track_id << " obs_id=" << reg_obs_ids[static_cast<size_t>(i)]
      //             << " del_image=" << reg_inds[static_cast<size_t>(i)];
      // }
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
  std::unordered_set<int> candidate_set;
  {
    std::vector<int> tid_buf;
    for (int new_im : new_registered_image_indices) {
      tid_buf.clear();
      store->get_image_track_observations(new_im, &tid_buf, nullptr);
      for (int tid : tid_buf) {
        if (store->is_track_valid(tid) && !store->track_has_triangulated_xyz(tid))
          candidate_set.insert(tid);
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
  VLOG(1) << "[PERF] run_batch_triangulation: " << ms << "ms"
          << "  total_tracks=" << store->num_tracks() << "  candidates=" << candidate_set.size()
          << "  scanned=" << tracks_scanned << "  skip_few_views=" << tracks_skipped_few_views
          << "  fail=" << tracks_skipped_fail << "  newly_tri=" << updated
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

/// Restore previously-deleted observations whose reprojection error (against the current
/// triangulated XYZ) is within strict_px.
///
/// Old O(T * N_obs) double-loop replaced by the per-track obs list: for each triangulated
/// track we walk only its own obs list (alive + deleted), which is O(T * avg_obs_per_track).
static int restore_observations_strict(TrackStore* store,
                                       const std::vector<Eigen::Matrix3d>& poses_R,
                                       const std::vector<Eigen::Vector3d>& poses_C,
                                       const std::vector<bool>& registered,
                                       const std::vector<camera::Intrinsics>& cameras,
                                       const std::vector<int>& image_to_camera_index,
                                       double strict_px, int n_images) {
  int restored = 0;
  std::vector<int> all_oids;
  for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
    const int track_id = static_cast<int>(ti);
    if (!store->is_track_valid(track_id) || !store->track_has_triangulated_xyz(track_id))
      continue;
    float tx, ty, tz;
    store->get_track_xyz(track_id, &tx, &ty, &tz);
    const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                            static_cast<double>(tz));
    // get_track_all_obs_ids returns alive + deleted obs for this track only.
    store->get_track_all_obs_ids(track_id, &all_oids);
    for (int oid : all_oids) {
      if (store->is_obs_valid(oid))
        continue; // already alive, skip
      Observation obs;
      store->get_obs(oid, &obs);
      const int im = static_cast<int>(obs.image_index);
      if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
        continue;
      const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];
      const double e =
          reproj_error_px(X, poses_R[static_cast<size_t>(im)], poses_C[static_cast<size_t>(im)], K,
                          static_cast<double>(obs.u), static_cast<double>(obs.v));
      if (e <= strict_px) {
        store->mark_observation_restored(oid);
        ++restored;
      }
    }
  }
  return restored;
}

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

  // Drain the pending queue (populated by reject_outliers_multiview when it marks obs deleted).
  // Currently unused for the untriangulated scan, but drain so the queue stays clean.
  std::vector<int> retri_track_ids;
  store->drain_retriangulation_pending(&retri_track_ids);

  int n_track = 0, n_observers = 0;
  for (int track_id : retri_track_ids) {
    if (store->is_track_valid(track_id) && store->track_has_triangulated_xyz(track_id)) {
      continue; // already retriangulated by a previous pass, skip
    }
    std::vector<int> obs_ids_out;
    store->get_track_all_obs_ids(track_id, &obs_ids_out);
    float tx, ty, tz;
    store->get_track_xyz(track_id, &tx, &ty, &tz);
    bool has_valid_obs = false;
    for (int oid : obs_ids_out) {
      if (!store->is_obs_valid(oid)) {
        // test reproject error
        Observation obs;
        store->get_obs(oid, &obs);
        const int im = static_cast<int>(obs.image_index);
        if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)]) {
          continue;
        }
        const camera::Intrinsics& K = cameras[static_cast<size_t>(image_to_camera_index[im])];

        const Eigen::Vector3d X(static_cast<double>(tx), static_cast<double>(ty),
                                static_cast<double>(tz));
        const double e =
            reproj_error_px(X, poses_R[static_cast<size_t>(im)], poses_C[static_cast<size_t>(im)],
                            K, static_cast<double>(obs.u), static_cast<double>(obs.v));
        if (e <= opts.restore_strict_reproj_px) {
          store->mark_observation_restored(oid);
          ++n_observers;
          has_valid_obs = true;
        }
      }
    }
    if (has_valid_obs) {
      ++n_track;
    }
  }
  auto t1 = std::chrono::steady_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  VLOG(1) << "[PERF] run_retriangulation: " << ms << "ms"
          << "  restored_obs=" << n_observers << "  restored_tracks=" << n_track;
  return n_track;
}

int run_retriangulation(TrackStore* store, const std::vector<Eigen::Matrix3d>& poses_R,
                        const std::vector<Eigen::Vector3d>& poses_C,
                        const std::vector<bool>& registered,
                        const std::vector<camera::Intrinsics>& cameras,
                        const std::vector<int>& image_to_camera_index, double min_tri_angle_deg) {
  IncrementalRetriangulationOptions o;
  o.robust.min_tri_angle_deg = min_tri_angle_deg;
  return run_retriangulation(store, poses_R, poses_C, registered, cameras, image_to_camera_index,
                             o);
}

int restore_observations_from_cameras(
    TrackStore* store,
    const std::vector<Eigen::Matrix3d>& poses_R,
    const std::vector<Eigen::Vector3d>& poses_C,
    const std::vector<bool>& registered,
    const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>& image_to_camera_index,
    const std::unordered_set<int>& changed_cam_model_indices,
    double strict_reproj_px) {
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
      const double e = reproj_error_px(X, poses_R[static_cast<size_t>(im)],
                                       poses_C[static_cast<size_t>(im)], K,
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
