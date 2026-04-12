/**
 * @file  test_sfm_diag2.cpp
 * @brief Post-SfM diagnostic tool – fresh triangulation implementation.
 *
 * Same CLI interface as test_sfm_diagnosis.  All triangulation code is written
 * from scratch in this single file: no dependency on incremental_triangulation.h.
 *
 * Triangulation algorithm (run_triangulation_clean)
 * ──────────────────────────────────────────────────
 *   For each track with ≥ 2 registered observations:
 *   1. Undistort each observation:  pixel-space fixed-point iteration
 *        u_{n+1} = u_n − (distort_px(u_n) − u_obs)
 *      Tolerance 1e-3 px, max 100 iterations.
 *   2. Build projection matrix  P_i = K_i * [R_i | −R_i·C_i]  (3×4)
 *   3. Hartley-normalize the 2D observations for numerical conditioning.
 *   4. DLT: stack 2 rows/view  [u·P3−P1 ; v·P3−P2], solve Ax=0 via SVD → last
 *      right singular vector → inhomogeneous  X = V_col3 / V_col3[3].
 *   5. Depth check: z_i = (R_i·(X−C_i))[2] > 0  for ALL views.
 *   6. Angle check: max triangulation angle across ALL pairs ≥ min_angle_deg.
 *   7. Reprojection check: at least 2 views with  reproj(X) < inlier_px
 *      (uses full Brown-Conrady forward model).
 *   8. Accept: store X via store.set_track_xyz().
 *
 * Usage
 * ─────
 *   test_sfm_diag2 \
 *       -t /path/to/tracks.isat_tracks \
 *       -p /path/to/images_all.json    \
 *       -s /path/to/poses.json         \
 *       [-o report.json]               \
 *       [--save-tracks out.isat_tracks] \
 *       [--save-bundler bundle.out]    \
 *       [--min-3d2d 20]                \
 *       [--min-tri-angle 0.5]          \
 *       [--inlier-px 4.0]              \
 *       [-v | -q]
 */

#include "../../io/track_store_idc.h"
#include "../../tools/project_loader.h"
#include "track_store.h"
#include "view_graph.h"
#include "view_graph_loader.h"

// Only the plain Intrinsics struct – no algorithm implementation pulled in.
#include "../camera/camera_types.h"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <random>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;
using json   = nlohmann::json;

using namespace insight::sfm;
using insight::tools::ProjectData;
using insight::tools::load_project_data;
using CameraIntrinsics = insight::camera::Intrinsics;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// ─────────────────────────────────────────────────────────────────────────────
// Self-contained distortion / undistortion (Brown-Conrady 5-param)
// Same model as ContextCapture / camera_utils.cpp, but inlined here.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

/// Forward distortion: normalized → distorted normalized.
/// u_d = (1 + k1·r² + k2·r⁴ + k3·r⁶)·u + 2·p2·u·v + p1·(r² + 2u²)
/// v_d = (1 + k1·r² + k2·r⁴ + k3·r⁶)·v + 2·p1·u·v + p2·(r² + 2v²)
inline void distort_normalized(const CameraIntrinsics& K,
                                double un, double vn,
                                double* ud, double* vd)
{
    const double r2  = un*un + vn*vn;
    const double r4  = r2 * r2;
    const double r6  = r4 * r2;
    const double rad = 1.0 + K.k1*r2 + K.k2*r4 + K.k3*r6;
    *ud = rad*un + 2.0*K.p2*un*vn + K.p1*(r2 + 2.0*un*un);
    *vd = rad*vn + 2.0*K.p1*un*vn + K.p2*(r2 + 2.0*vn*vn);
}

/// Undistort a single distorted pixel observation to an ideal pixel.
/// Iteration in pixel space:  p_{n+1} = p_n − (distort_px(p_n) − p_obs)
/// Tolerance 1e-3 px; max 100 iterations.
inline void undistort_pixel(const CameraIntrinsics& K,
                             double u_in, double v_in,
                             double* u_out, double* v_out)
{
    constexpr double kTol  = 1e-3;   // 0.001 pixel
    constexpr int    kMaxIt = 100;

    double u = u_in, v = v_in;     // running estimate
    for (int it = 0; it < kMaxIt; ++it) {
        // Current estimate in normalized coords
        const double un = (u - K.cx) / K.fx;
        const double vn = (v - K.cy) / K.fy;
        // What distorted pixel would give us our current guess?
        double ud, vd;
        distort_normalized(K, un, vn, &ud, &vd);
        const double u_pred = K.fx * ud + K.cx;
        const double v_pred = K.fy * vd + K.cy;
        // Correction
        const double du = u_pred - u_in;
        const double dv = v_pred - v_in;
        u -= du;
        v -= dv;
        if (std::abs(du) < kTol && std::abs(dv) < kTol)
            break;
    }
    *u_out = u;
    *v_out = v;
}

// ─────────────────────────────────────────────────────────────────────────────
// Reprojection error with full distortion model
// ─────────────────────────────────────────────────────────────────────────────

/// Project X through camera (R, C, K) → distorted pixel, return L2 error vs (u_obs, v_obs).
inline double reproj_error_px(const Vector3d& X,
                               const Matrix3d& R, const Vector3d& C,
                               const CameraIntrinsics& K,
                               double u_obs, double v_obs)
{
    const Vector3d Xc = R * (X - C);
    if (Xc.z() <= 0.0)
        return 1e30;
    const double xn = Xc.x() / Xc.z();
    const double yn = Xc.y() / Xc.z();
    double xd, yd;
    distort_normalized(K, xn, yn, &xd, &yd);
    const double du = K.fx * xd + K.cx - u_obs;
    const double dv = K.fy * yd + K.cy - v_obs;
    return std::sqrt(du*du + dv*dv);
}

// ─────────────────────────────────────────────────────────────────────────────
// Point-only BA: LM refinement (fixed R, t, K — optimize X only)
// ─────────────────────────────────────────────────────────────────────────────

/// 2D reprojection residual (proj - obs), in pixels.
inline Vector2d reproj_residual_2d(const Vector3d& X,
                                    const Matrix3d& R, const Vector3d& C,
                                    const CameraIntrinsics& K,
                                    double u_obs, double v_obs)
{
    const Vector3d Xc = R * (X - C);
    if (Xc.z() <= 0.0)
        return Vector2d(1e15, 1e15);
    const double xn = Xc.x() / Xc.z();
    const double yn = Xc.y() / Xc.z();
    double xd, yd;
    distort_normalized(K, xn, yn, &xd, &yd);
    return Vector2d(K.fx * xd + K.cx - u_obs,
                    K.fy * yd + K.cy - v_obs);
}

/**
 * Refine a 3D point by Levenberg-Marquardt (structure-only).
 *
 * Minimizes  sum_i || proj(X, R_i, C_i, K_i) - obs_i ||^2
 * All camera parameters (R, C, K) are held fixed; only X (3 DOF) is optimized.
 * Jacobian is computed by central finite differences.
 *
 * @return  Refined point, or X0 unchanged if LM diverges / makes no progress.
 */
static Vector3d refine_point_lm(
    const Vector3d&                    X0,
    const std::vector<Matrix3d>&       R_list,
    const std::vector<Vector3d>&       C_list,
    const std::vector<CameraIntrinsics>& K_list,
    const std::vector<double>&         u_obs,
    const std::vector<double>&         v_obs,
    int    max_iter = 20,
    double lambda   = 1e-3)
{
    const int N  = static_cast<int>(R_list.size());
    const int NR = 2 * N;         // number of residuals
    constexpr double kH     = 1e-4;   // finite-difference step (world units)
    constexpr double kLambdaMax = 1e8;
    constexpr double kConvTol   = 1e-10;

    auto compute_residuals = [&](const Vector3d& X) {
        Eigen::VectorXd r(NR);
        for (int i = 0; i < N; ++i) {
            const Vector2d ri = reproj_residual_2d(X, R_list[i], C_list[i],
                                                    K_list[i], u_obs[i], v_obs[i]);
            r[2*i]   = ri.x();
            r[2*i+1] = ri.y();
        }
        return r;
    };

    Vector3d X = X0;
    Eigen::VectorXd r = compute_residuals(X);
    double cost = r.squaredNorm();

    for (int iter = 0; iter < max_iter; ++iter) {
        // ── Numerical Jacobian J (NR × 3) via central differences ─────────────
        Eigen::MatrixXd J(NR, 3);
        for (int k = 0; k < 3; ++k) {
            Vector3d Xp = X;  Xp[k] += kH;
            Vector3d Xm = X;  Xm[k] -= kH;
            J.col(k) = (compute_residuals(Xp) - compute_residuals(Xm)) / (2.0 * kH);
        }

        // ── Normal equations: (J^T J + lambda I) dX = -J^T r ─────────────────
        const Eigen::Matrix3d JtJ = J.transpose() * J;
        const Eigen::Vector3d Jtr = J.transpose() * r;
        const Eigen::Matrix3d A   = JtJ + lambda * Eigen::Matrix3d::Identity();
        const Eigen::Vector3d dX  = A.ldlt().solve(-Jtr);

        const Vector3d X_new  = X + dX;
        const double   cost_new = compute_residuals(X_new).squaredNorm();

        if (cost_new < cost) {
            X      = X_new;
            r      = compute_residuals(X);   // re-use for next iter
            cost   = cost_new;
            lambda = std::max(lambda * 0.1, 1e-15);
            if (dX.norm() < kConvTol)
                break;
        } else {
            lambda *= 10.0;
            if (lambda > kLambdaMax)
                break;
        }
    }

    // Accept only if cost improved; otherwise return original DLT point.
    const double cost0 = compute_residuals(X0).squaredNorm();
    return (cost < cost0) ? X : X0;
}

// ─────────────────────────────────────────────────────────────────────────────
// DLT triangulation with Hartley normalization
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Triangulate from N ≥ 2 views using the normalized DLT.
 *
 * @param P_list  N projection matrices 3×4, each P = K·[R|t]  (t = −R·C)
 * @param u_u     N undistorted pixel x-coords
 * @param v_u     N undistorted pixel y-coords
 *
 * @return 3-D world point X, or Vector3d::Constant(NaN) on failure.
 */
static Vector3d triangulate_dlt(
    const std::vector<Matrix<double, 3, 4>>& P_list,
    const std::vector<double>& u_u,
    const std::vector<double>& v_u)
{
    const int N = static_cast<int>(P_list.size());
    if (N < 2)
        return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    // ── Hartley normalization of 2D points ────────────────────────────────────
    // Centroid + scale so that RMS distance from centroid ≈ √2.
    double cx = 0.0, cy = 0.0;
    for (int i = 0; i < N; ++i) { cx += u_u[i]; cy += v_u[i]; }
    cx /= N;  cy /= N;
    double rms = 0.0;
    for (int i = 0; i < N; ++i) {
        const double dx = u_u[i] - cx, dy = v_u[i] - cy;
        rms += dx*dx + dy*dy;
    }
    rms = std::sqrt(rms / N);
    const double s = (rms > 1e-9) ? (std::sqrt(2.0) / rms) : 1.0;

    // Normalization transform T (3×3): maps observed coords to normalized coords
    //   [s, 0, -s*cx]
    //   [0, s, -s*cy]
    //   [0, 0,  1   ]
    Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
    T(0,0) = s;  T(0,2) = -s*cx;
    T(1,1) = s;  T(1,2) = -s*cy;
    T(2,2) = 1.0;

    // ── Build system  A·X = 0  (2N × 4) ──────────────────────────────────────
    Eigen::MatrixXd A(2*N, 4);
    for (int i = 0; i < N; ++i) {
        // Normalized projection: P̃ = T · P
        const Matrix<double,3,4> Pn = T * P_list[i];
        // Normalized observation
        const double un = s * u_u[i] - s*cx;   // = s*(u-cx)
        const double vn = s * v_u[i] - s*cy;
        // DLT rows: un·P3 − P1 ,  vn·P3 − P2
        A.row(2*i)   = un * Pn.row(2) - Pn.row(0);
        A.row(2*i+1) = vn * Pn.row(2) - Pn.row(1);
    }

    // ── SVD → null space ─────────────────────────────────────────────────────
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    const Eigen::Vector4d X4 = svd.matrixV().col(3);
    if (std::abs(X4[3]) < 1e-15)
        return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    return X4.head<3>() / X4[3];
}

// ─────────────────────────────────────────────────────────────────────────────
// Triangulation angle helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Maximum triangulation angle (degrees) over all camera-pair combinations.
/// Angle = angle at the 3D point X between rays from C_i and C_j:
///   angle = acos( normalize(C_i−X) · normalize(C_j−X) )
static double max_tri_angle_deg(const Vector3d& X,
                                 const std::vector<Vector3d>& C_list)
{
    const int N = static_cast<int>(C_list.size());
    double min_dot = 2.0;   // will be updated to the actual minimum
    bool found = false;
    for (int i = 0; i < N; ++i) {
        const Vector3d ri = (C_list[i] - X).normalized();
        for (int j = i + 1; j < N; ++j) {
            const Vector3d rj = (C_list[j] - X).normalized();
            const double d = std::max(-1.0, std::min(1.0, ri.dot(rj)));
            if (!found || d < min_dot) { min_dot = d; found = true; }
        }
    }
    if (!found) return 0.0;
    return std::acos(std::max(-1.0, std::min(1.0, min_dot))) * (180.0 / M_PI);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main triangulation pass
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Triangulate all pending tracks in-place.
 *
 * For every track that does NOT yet have a triangulated XYZ (or all tracks when
 * retriangulate_all=true), we:
 *  1. Collect all live observations that belong to registered images.
 *  2. Undistort each observation (pixel-space iteration).
 *  3. Run N-view DLT to get a candidate 3D point X.
 *  4. Accept X only if:
 *     - All views have positive depth.
 *     - Max triangulation angle ≥ min_angle_deg.
 *     - At least 2 views have reprojection error < inlier_px.
 *
 * Returns: number of newly triangulated tracks.
 */
static int run_triangulation_clean(
    TrackStore*                              store,
    const std::vector<Matrix3d>&             poses_R,
    const std::vector<Vector3d>&             poses_C,
    const std::vector<bool>&                 registered,
    const std::vector<CameraIntrinsics>&     cameras,
    const std::vector<int>&                  image_to_camera_idx,
    double                                   min_angle_deg,
    double                                   inlier_px,
    bool                                     retriangulate_all = false)
{
    const int n_tracks = store->num_tracks();
    const int n_images = store->num_images();

    int newly_tri = 0;
    int skip_already_tri = 0;
    int skip_few_views = 0;
    int fail_no_pair = 0;   // no 2-view pair passes chirality + angle
    int fail_reproj  = 0;   // best candidate < 2 reproj inliers

    // Pre-build scratch buffers (reuse per track)
    std::vector<Observation> obs_buf;
    std::vector<Matrix<double, 3, 4>> P_list;
    std::vector<double> u_u_list, v_u_list;
    std::vector<Vector3d>  C_reg;
    std::vector<Matrix3d>  R_reg;
    std::vector<CameraIntrinsics> K_reg;
    std::vector<double> u_obs, v_obs;
    std::vector<std::pair<int,int>> pair_list;

    for (int tid = 0; tid < n_tracks; ++tid) {
        if (!store->is_track_valid(tid))
            continue;
        if (!retriangulate_all && store->track_has_triangulated_xyz(tid)) {
            ++skip_already_tri;
            continue;
        }

        // ── Collect registered observations ───────────────────────────────────
        obs_buf.clear();
        store->get_track_observations(tid, &obs_buf);

        P_list.clear(); u_u_list.clear(); v_u_list.clear();
        C_reg.clear();  R_reg.clear();    K_reg.clear();
        u_obs.clear();  v_obs.clear();

        for (const Observation& o : obs_buf) {
            const int im = static_cast<int>(o.image_index);
            if (im < 0 || im >= n_images)
                continue;
            if (!registered[static_cast<size_t>(im)])
                continue;

            const CameraIntrinsics& K =
                cameras[static_cast<size_t>(image_to_camera_idx[static_cast<size_t>(im)])];
            const Matrix3d& R = poses_R[static_cast<size_t>(im)];
            const Vector3d& C = poses_C[static_cast<size_t>(im)];

            // Build P = K * [R | t]  where t = −R·C
            const Vector3d t = -R * C;
            Matrix<double, 3, 4> P;
            P.leftCols<3>()  = R;
            P.rightCols<1>() = t;
            // Premultiply by K
            Eigen::Matrix3d Km = Eigen::Matrix3d::Zero();
            Km(0,0) = K.fx; Km(0,2) = K.cx;
            Km(1,1) = K.fy; Km(1,2) = K.cy;
            Km(2,2) = 1.0;
            P = Km * P;

            // Undistort observation
            double uu, vu;
            undistort_pixel(K, static_cast<double>(o.u), static_cast<double>(o.v), &uu, &vu);

            P_list.push_back(P);
            u_u_list.push_back(uu);
            v_u_list.push_back(vu);
            C_reg.push_back(C);
            R_reg.push_back(R);
            K_reg.push_back(K);
            u_obs.push_back(static_cast<double>(o.u));
            v_obs.push_back(static_cast<double>(o.v));
        }

        const int N = static_cast<int>(P_list.size());
        if (N < 2) {
            ++skip_few_views;
            continue;
        }

        // ── RANSAC over all 2-view pairs ──────────────────────────────────────
        // For each pair: DLT → chirality → angle → count reproj inliers.
        // Pick the best candidate (max inliers), then LM on the inlier set.
        // Mirrors the pipeline's robust_triangulate_point_multiview strategy:
        // uses outlier-robust pair selection instead of a single N-view DLT.
        constexpr int kMaxPairs = 100;
        pair_list.clear();
        for (int a = 0; a < N; ++a)
            for (int b = a + 1; b < N; ++b)
                pair_list.emplace_back(a, b);
        if ((int)pair_list.size() > kMaxPairs) {
            std::mt19937 rng2(static_cast<uint32_t>(tid));
            std::shuffle(pair_list.begin(), pair_list.end(), rng2);
            pair_list.resize(static_cast<size_t>(kMaxPairs));
        }

        Vector3d best_X  = Vector3d::Zero();
        int      best_n  = 0;
        bool     any_valid = false;

        for (const auto& [pa, pb] : pair_list) {
            const std::vector<Matrix<double,3,4>> P2{P_list[pa], P_list[pb]};
            const std::vector<double> uu2{u_u_list[pa], u_u_list[pb]};
            const std::vector<double> vu2{v_u_list[pa], v_u_list[pb]};
            Vector3d Xc = triangulate_dlt(P2, uu2, vu2);
            if (!Xc.allFinite() || Xc.norm() < 1e-10) continue;
            // Chirality: both seed views must have positive depth
            if ((R_reg[pa] * (Xc - C_reg[pa]))[2] <= 0.0) continue;
            if ((R_reg[pb] * (Xc - C_reg[pb]))[2] <= 0.0) continue;
            // Angle check on the seed pair
            if (max_tri_angle_deg(Xc, {C_reg[pa], C_reg[pb]}) < min_angle_deg) continue;
            any_valid = true;

            // ── Point-only LM on the 2 seed views ────────────────────────────
            // DLT minimizes algebraic error; refine geometrically before
            // counting N-view inliers so best_X is already near-optimal.
            // 5 iterations is enough to escape the algebraic-error neighborhood.
            Xc = refine_point_lm(Xc,
                                  {R_reg[pa], R_reg[pb]},
                                  {C_reg[pa], C_reg[pb]},
                                  {K_reg[pa], K_reg[pb]},
                                  {u_obs[pa], u_obs[pb]},
                                  {v_obs[pa], v_obs[pb]},
                                  /*max_iter=*/5);
            // Re-check chirality after LM (rare but possible to flip)
            if ((R_reg[pa] * (Xc - C_reg[pa]))[2] <= 0.0) continue;
            if ((R_reg[pb] * (Xc - C_reg[pb]))[2] <= 0.0) continue;

            // Count inliers over all N views (reproj_error_px returns 1e30 for z≤0)
            int ninl = 0;
            for (int i = 0; i < N; ++i)
                if (reproj_error_px(Xc, R_reg[i], C_reg[i], K_reg[i],
                                    u_obs[i], v_obs[i]) < inlier_px)
                    ++ninl;
            if (ninl > best_n) { best_n = ninl; best_X = Xc; }
        }

        if (!any_valid) { ++fail_no_pair; continue; }
        if (best_n < 2)  { ++fail_reproj;  continue; }

        // ─────────────────────────────────────────────────────────────────────
        // Two-stage iterative outlier rejection + BA:
        //   Stage 1:  filter at 50 px  →  BA (LM)
        //   Stage 2:  filter at 16 px  →  BA (LM)
        //   Accept if ≥ 2 views remain with reproj < 16 px after stage 2.
        // ─────────────────────────────────────────────────────────────────────
        constexpr double kThresh1 = 50.0;   // loose filter
        constexpr double kThresh2 = 16.0;   // tight filter

        // Inline helper: collect views whose reproj < thresh for point X
        auto collect_inlier_views = [&](const Vector3d& X, double thresh,
                                        std::vector<Matrix3d>&         Ro,
                                        std::vector<Vector3d>&         Co,
                                        std::vector<CameraIntrinsics>& Ko,
                                        std::vector<double>&           uo,
                                        std::vector<double>&           vo)
        {
            Ro.clear(); Co.clear(); Ko.clear(); uo.clear(); vo.clear();
            for (int i = 0; i < N; ++i) {
                if (reproj_error_px(X, R_reg[i], C_reg[i], K_reg[i],
                                    u_obs[i], v_obs[i]) < thresh) {
                    Ro.push_back(R_reg[i]);
                    Co.push_back(C_reg[i]);
                    Ko.push_back(K_reg[i]);
                    uo.push_back(u_obs[i]);
                    vo.push_back(v_obs[i]);
                }
            }
        };

        std::vector<Matrix3d>         R_inl;
        std::vector<Vector3d>         C_inl;
        std::vector<CameraIntrinsics> K_inl;
        std::vector<double>           uo_inl, vo_inl;

        // ── Stage 1: filter at 50 px → BA ────────────────────────────────────
        collect_inlier_views(best_X, kThresh1, R_inl, C_inl, K_inl, uo_inl, vo_inl);
        if (R_inl.size() < 2) { ++fail_reproj; continue; }
        const Vector3d X1 = refine_point_lm(best_X, R_inl, C_inl, K_inl,
                                             uo_inl, vo_inl);

        // ── Stage 2: filter at 16 px → BA ────────────────────────────────────
        collect_inlier_views(X1, kThresh2, R_inl, C_inl, K_inl, uo_inl, vo_inl);
        if (R_inl.size() < 2) { ++fail_reproj; continue; }
        const Vector3d X2 = refine_point_lm(X1, R_inl, C_inl, K_inl,
                                             uo_inl, vo_inl);

        // ── Final count at kThresh2 ───────────────────────────────────────────
        int n_final = 0;
        for (int i = 0; i < (int)R_inl.size(); ++i) {
            if ((R_inl[i] * (X2 - C_inl[i]))[2] > 0.0 &&
                reproj_error_px(X2, R_inl[i], C_inl[i], K_inl[i],
                                uo_inl[i], vo_inl[i]) < kThresh2)
                ++n_final;
        }
        if (n_final < 2) { ++fail_reproj; continue; }
        const Vector3d& X_final = X2;

        // ── Accept ────────────────────────────────────────────────────────────
        store->set_track_xyz(tid,
                             static_cast<float>(X_final.x()),
                             static_cast<float>(X_final.y()),
                             static_cast<float>(X_final.z()));
        ++newly_tri;
    }

    LOG(INFO) << "run_triangulation_clean:"
              << "  total_tracks=" << n_tracks
              << "  skip_already="  << skip_already_tri
              << "  skip_few_views=" << skip_few_views
              << "  fail_no_pair="  << fail_no_pair
              << "  fail_reproj="   << fail_reproj
              << "  newly_tri="     << newly_tri;

    return newly_tri;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Pose loading (identical to test_sfm_diagnosis)
// ─────────────────────────────────────────────────────────────────────────────

struct LoadedPoses {
    int n_images = 0;
    std::vector<bool>             registered;
    std::vector<Matrix3d>         poses_R;
    std::vector<Vector3d>         poses_C;
    std::vector<CameraIntrinsics> cameras;
    std::vector<int>              image_to_camera_index;
    bool                          has_camera_bundle = false;
};

static bool parse_intrinsics_json(const json& j, CameraIntrinsics* K)
{
    if (!K || !j.is_object()) return false;
    K->fx = j.value("fx", 0.0);
    K->fy = j.value("fy", 0.0);
    K->cx = j.value("cx", 0.0);
    K->cy = j.value("cy", 0.0);
    K->width  = j.value("width",  0);
    K->height = j.value("height", 0);
    K->k1 = j.value("k1", 0.0);
    K->k2 = j.value("k2", 0.0);
    K->k3 = j.value("k3", 0.0);
    K->p1 = j.value("p1", 0.0);
    K->p2 = j.value("p2", 0.0);
    return K->fx > 0.0 && K->fy > 0.0;
}

static bool load_poses_json(const std::string& path, int n_images, LoadedPoses* out)
{
    std::ifstream f(path);
    if (!f.is_open()) { LOG(ERROR) << "Cannot open poses file: " << path; return false; }
    json j;
    try { f >> j; } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to parse poses JSON: " << e.what(); return false;
    }

    const json* poses_arr = nullptr;
    if (j.is_array()) {
        poses_arr = &j;
    } else if (j.is_object() && j.contains("poses") && j["poses"].is_array()) {
        poses_arr = &j["poses"];
        if (j.contains("cameras") && j["cameras"].is_array()) {
            out->cameras.clear();
            for (const auto& cj : j["cameras"]) {
                CameraIntrinsics K;
                if (!parse_intrinsics_json(cj, &K)) { out->cameras.clear(); break; }
                out->cameras.push_back(K);
            }
        }
        if (j.contains("image_to_camera_index") && j["image_to_camera_index"].is_array()) {
            try { out->image_to_camera_index = j["image_to_camera_index"].get<std::vector<int>>(); }
            catch (...) { out->image_to_camera_index.clear(); }
        }
        out->has_camera_bundle =
            !out->cameras.empty() &&
            static_cast<int>(out->image_to_camera_index.size()) == n_images;
    } else {
        LOG(ERROR) << "poses.json: expected array or object with poses[] at top level";
        return false;
    }

    out->n_images = n_images;
    out->registered.assign(static_cast<size_t>(n_images), false);
    out->poses_R.resize(static_cast<size_t>(n_images), Matrix3d::Identity());
    out->poses_C.resize(static_cast<size_t>(n_images), Vector3d::Zero());

    int loaded = 0;
    for (const auto& entry : *poses_arr) {
        const int idx = entry.at("image_index").get<int>();
        if (idx < 0 || idx >= n_images) { LOG(WARNING) << "poses.json: index " << idx << " OOB"; continue; }
        const auto rv = entry.at("R").get<std::vector<double>>();
        const auto cv = entry.at("C").get<std::vector<double>>();
        if (rv.size() != 9 || cv.size() != 3) { LOG(WARNING) << "poses.json: bad R/C for " << idx; continue; }
        Matrix3d R;
        R << rv[0],rv[1],rv[2], rv[3],rv[4],rv[5], rv[6],rv[7],rv[8];
        out->poses_R[static_cast<size_t>(idx)] = R;
        out->poses_C[static_cast<size_t>(idx)] = Vector3d(cv[0], cv[1], cv[2]);
        out->registered[static_cast<size_t>(idx)] = true;
        ++loaded;
    }
    LOG(INFO) << "Loaded " << loaded << " registered poses from " << path;
    if (out->has_camera_bundle)
        LOG(INFO) << "Camera bundle loaded from poses.json: " << out->cameras.size() << " cameras";
    return loaded > 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// ViewGraph neighbor summary
// ─────────────────────────────────────────────────────────────────────────────

struct NeighborSummary {
    int  neighbor_image_index = -1;
    bool neighbor_registered  = false;
    int  f_inliers            = 0;
    bool e_ok                 = false;
    bool twoview_ok           = false;
    bool stable               = false;
};

static std::unordered_map<int, std::vector<NeighborSummary>>
build_neighbor_map(const ViewGraph& vg, const std::vector<bool>& registered)
{
    std::unordered_map<int, std::vector<NeighborSummary>> map;
    for (size_t pi = 0; pi < vg.num_pairs(); ++pi) {
        const PairGeoInfo& p = vg.pair_at(pi);
        const int i1 = static_cast<int>(p.image1_index);
        const int i2 = static_cast<int>(p.image2_index);
        const bool r1 = (i1 < static_cast<int>(registered.size())) && registered[i1];
        const bool r2 = (i2 < static_cast<int>(registered.size())) && registered[i2];

        NeighborSummary n1, n2;
        n1.neighbor_image_index = i1; n1.neighbor_registered = r1;
        n1.f_inliers = p.F_inliers; n1.e_ok = p.E_ok; n1.twoview_ok = p.twoview_ok; n1.stable = p.stable;
        n2.neighbor_image_index = i2; n2.neighbor_registered = r2;
        n2.f_inliers = p.F_inliers; n2.e_ok = p.E_ok; n2.twoview_ok = p.twoview_ok; n2.stable = p.stable;

        map[i2].push_back(n1);
        map[i1].push_back(n2);
    }
    return map;
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-image 3D-2D counter
// ─────────────────────────────────────────────────────────────────────────────

static int count_3d2d(const TrackStore& store, int image_index)
{
    std::vector<int> track_ids;
    std::vector<Observation> obs_scratch;
    store.get_image_track_observations(image_index, &track_ids, &obs_scratch);
    int count = 0;
    for (int tid : track_ids)
        if (store.track_has_triangulated_xyz(tid))
            ++count;
    return count;
}

static int count_total_tracks(const TrackStore& store, int image_index)
{
    std::vector<int> track_ids;
    std::vector<Observation> obs_scratch;
    return store.get_image_track_observations(image_index, &track_ids, &obs_scratch);
}

// ─────────────────────────────────────────────────────────────────────────────
// Bundler format export (identical to test_sfm_diagnosis)
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<std::string> load_image_paths(const std::string& project_json_path, int n_images)
{
    std::vector<std::string> paths(static_cast<size_t>(n_images));
    std::ifstream f(project_json_path);
    if (!f.is_open()) return paths;
    nlohmann::json j;
    try { f >> j; } catch (...) { return paths; }
    if (!j.contains("images") || !j["images"].is_array()) return paths;
    const auto& arr = j["images"];
    const size_t n = std::min(arr.size(), static_cast<size_t>(n_images));
    for (size_t i = 0; i < n; ++i)
        paths[i] = arr[i].value("path", "");
    return paths;
}

static bool save_bundler_format(
    const std::string&                   bundle_path,
    const TrackStore&                    store,
    int                                  n_images,
    const std::vector<bool>&             registered,
    const std::vector<Matrix3d>&         poses_R,
    const std::vector<Vector3d>&         poses_C,
    const std::vector<CameraIntrinsics>& cameras,
    const std::vector<int>&              image_to_camera_index,
    const std::vector<std::string>&      image_paths)
{
    struct BundlerObs   { int cam_idx; double bx, by; };
    struct BundlerPoint { double X, Y, Z; std::vector<BundlerObs> obs; };

    std::vector<BundlerPoint> points;
    const int n_tracks = store.num_tracks();
    points.reserve(static_cast<size_t>(n_tracks / 2));

    for (int tid = 0; tid < n_tracks; ++tid) {
        if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
            continue;
        float tx, ty, tz;
        store.get_track_xyz(tid, &tx, &ty, &tz);

        BundlerPoint pt;
        pt.X = static_cast<double>(tx);
        pt.Y = static_cast<double>(ty);
        pt.Z = static_cast<double>(tz);

        std::vector<int> obs_ids;
        store.get_track_obs_ids(tid, &obs_ids);
        for (int oid : obs_ids) {
            if (!store.is_obs_valid(oid)) continue;
            Observation o;
            store.get_obs(oid, &o);
            const int im = static_cast<int>(o.image_index);
            if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
                continue;
            const CameraIntrinsics& K =
                cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
            pt.obs.push_back({im,
                              static_cast<double>(o.u) - K.cx,
                              -(static_cast<double>(o.v) - K.cy)});
        }
        if (pt.obs.size() >= 2)
            points.push_back(std::move(pt));
    }

    std::ofstream f(bundle_path);
    if (!f.is_open()) { LOG(ERROR) << "save_bundler_format: cannot open " << bundle_path; return false; }

    f << "# Bundle file v0.3\n";
    f << n_images << " " << points.size() << "\n";
    f << std::fixed << std::setprecision(10);

    for (int i = 0; i < n_images; ++i) {
        if (!registered[static_cast<size_t>(i)]) {
            f << "0 0 0\n1 0 0\n0 1 0\n0 0 1\n0 0 0\n";
            continue;
        }
        const CameraIntrinsics& K =
            cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(i)])];
        const double focal = (K.fx + K.fy) * 0.5;
        const Matrix3d& R  = poses_R[static_cast<size_t>(i)];
        const Vector3d  t  = -R * poses_C[static_cast<size_t>(i)];
        f << focal << " " << K.k1 << " " << K.k2 << "\n";
        for (int r = 0; r < 3; ++r)
            f << R(r,0) << " " << R(r,1) << " " << R(r,2) << "\n";
        f << t(0) << " " << t(1) << " " << t(2) << "\n";
    }

    f << std::fixed << std::setprecision(6);
    for (const auto& pt : points) {
        f << pt.X << " " << pt.Y << " " << pt.Z << "\n";
        f << "255 255 255\n";
        f << pt.obs.size();
        for (const auto& o : pt.obs)
            f << " " << o.cam_idx << " 0 " << o.bx << " " << o.by;
        f << "\n";
    }

    LOG(INFO) << "Saved Bundler: " << bundle_path
              << "  cameras=" << n_images << "  points=" << points.size();

    if (!image_paths.empty()) {
        const fs::path list_path = fs::path(bundle_path).parent_path() / "list.txt";
        std::ofstream lf(list_path);
        if (lf.is_open()) {
            for (const auto& p : image_paths) lf << p << "\n";
            LOG(INFO) << "Saved list.txt: " << list_path.string();
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO);

    // ── Argument parsing ──────────────────────────────────────────────────────
    std::string tracks_path, project_path, poses_path, output_path;
    std::string save_tracks_path, save_bundler_path;
    int    min_3d2d        = 20;
    double min_tri_angle   = 0.5;   // degrees
    double inlier_px       = 40.0;   // pixels
    bool   verbose         = false;
    bool   quiet           = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { std::cerr << "Missing value for " << a << "\n"; exit(2); }
            return argv[++i];
        };
        if      (a == "-t" || a == "--tracks")       tracks_path       = next();
        else if (a == "-p" || a == "--project")      project_path      = next();
        else if (a == "-s" || a == "--poses")        poses_path        = next();
        else if (a == "-o" || a == "--output")       output_path       = next();
        else if (a == "--save-tracks")               save_tracks_path  = next();
        else if (a == "--save-bundler")              save_bundler_path = next();
        else if (a == "--min-3d2d")                  min_3d2d          = std::stoi(next());
        else if (a == "--min-tri-angle")             min_tri_angle     = std::stod(next());
        else if (a == "--inlier-px")                 inlier_px         = std::stod(next());
        else if (a == "-v" || a == "--verbose")      verbose = true;
        else if (a == "-q" || a == "--quiet")        quiet   = true;
        else if (a == "-h" || a == "--help") {
            std::cout <<
                "test_sfm_diag2: post-SfM diagnostic – fresh triangulation\n\n"
                "  -t / --tracks      <path>   .isat_tracks IDC (required)\n"
                "  -p / --project     <path>   images_all.json (required)\n"
                "  -s / --poses       <path>   poses.json from SfM run (required)\n"
                "  -o / --output      <path>   JSON report (default: stdout)\n"
                "  --save-tracks      <path>   save triangulated TrackStore IDC\n"
                "  --save-bundler     <path>   export Bundler v0.3 bundle.out + list.txt\n"
                "  --min-3d2d         <N>      min 3D-2D threshold (default: 20)\n"
                "  --min-tri-angle    <deg>    min triangulation angle (default: 0.5)\n"
                "  --inlier-px        <px>     reprojection inlier threshold (default: 4.0)\n"
                "  -v                 verbose log\n"
                "  -q                 quiet (error only)\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << a << "\n";
            return 2;
        }
    }

    if (tracks_path.empty() || project_path.empty() || poses_path.empty()) {
        std::cerr << "Error: -t, -p, -s are required\n";
        return 2;
    }

    if (quiet)        google::SetStderrLogging(google::ERROR);
    else if (verbose) google::SetStderrLogging(google::INFO);

    // ── 1. Load TrackStore + ViewGraph ────────────────────────────────────────
    TrackStore store;
    std::vector<uint32_t> image_indices_from_idc;
    ViewGraph vg;
    LOG(INFO) << "Loading tracks: " << tracks_path;
    if (!load_track_store_from_idc(tracks_path, &store, &image_indices_from_idc, &vg)) {
        LOG(ERROR) << "Failed to load tracks";
        return 1;
    }
    const int n_images = store.num_images();
    const size_t n_tracks = store.num_tracks();
    LOG(INFO) << "TrackStore: " << n_images << " images, " << n_tracks
              << " tracks, ViewGraph pairs: " << vg.num_pairs();

    // ── 2. Load ProjectData (cameras + image_to_camera_index) ────────────────
    ProjectData project;
    LOG(INFO) << "Loading project: " << project_path;
    if (!load_project_data(project_path, &project)) {
        LOG(ERROR) << "Failed to load project: " << project_path;
        return 1;
    }
    if (project.num_images() != n_images) {
        LOG(ERROR) << "Image count mismatch: project=" << project.num_images()
                   << " tracks=" << n_images;
        return 1;
    }

    // ── 3. Load poses ─────────────────────────────────────────────────────────
    LoadedPoses poses;
    LOG(INFO) << "Loading poses: " << poses_path;
    if (!load_poses_json(poses_path, n_images, &poses)) {
        LOG(ERROR) << "Failed to load poses: " << poses_path;
        return 1;
    }
    if (poses.has_camera_bundle) {
        LOG(INFO) << "Overriding project cameras with poses.json bundle";
        project.cameras               = poses.cameras;
        project.image_to_camera_index = poses.image_to_camera_index;
    }
    const int n_registered   = static_cast<int>(
        std::count(poses.registered.begin(), poses.registered.end(), true));
    const int n_unregistered = n_images - n_registered;
    LOG(INFO) << "Registered: " << n_registered << "/" << n_images
              << "  Unregistered: " << n_unregistered;

    // ── 4. Fresh triangulation (from-scratch DLT, no existing algo code) ─────
    LOG(INFO) << "Running clean triangulation  (min_angle=" << min_tri_angle
              << " deg,  inlier_px=" << inlier_px << ") ...";
    const int newly_tri = run_triangulation_clean(
        &store,
        poses.poses_R, poses.poses_C, poses.registered,
        project.cameras, project.image_to_camera_index,
        min_tri_angle, inlier_px,
        /*retriangulate_all=*/true);   // re-do everything for a clean slate

    int total_tri = 0;
    for (size_t tid = 0; tid < n_tracks; ++tid)
        if (store.track_has_triangulated_xyz(static_cast<int>(tid)))
            ++total_tri;
    LOG(INFO) << "Triangulation done: newly=" << newly_tri << "  total=" << total_tri;

    // ── 5. Analyze unregistered images ────────────────────────────────────────
    auto neighbor_map = build_neighbor_map(vg, poses.registered);

    struct ImageAnalysis {
        int  image_index;
        int  n_tracks_total;
        int  n_3d2d;
        int  n_registered_neighbors;
        int  max_f_inliers_with_registered;
        int  total_f_inliers_with_registered;
        std::vector<NeighborSummary> top_neighbors;
    };

    std::vector<ImageAnalysis> unregistered_analysis;
    unregistered_analysis.reserve(static_cast<size_t>(n_unregistered));

    for (int i = 0; i < n_images; ++i) {
        if (poses.registered[i]) continue;

        ImageAnalysis a;
        a.image_index    = i;
        a.n_tracks_total = count_total_tracks(store, i);
        a.n_3d2d         = count_3d2d(store, i);

        auto& neighbors = neighbor_map[i];
        int n_reg_nb = 0, max_f = 0, total_f = 0;
        for (const auto& nb : neighbors) {
            if (nb.neighbor_registered) {
                ++n_reg_nb;
                total_f += nb.f_inliers;
                if (nb.f_inliers > max_f) max_f = nb.f_inliers;
            }
        }
        a.n_registered_neighbors          = n_reg_nb;
        a.max_f_inliers_with_registered   = max_f;
        a.total_f_inliers_with_registered = total_f;

        std::vector<NeighborSummary> reg_nbs;
        for (const auto& nb : neighbors)
            if (nb.neighbor_registered) reg_nbs.push_back(nb);
        std::sort(reg_nbs.begin(), reg_nbs.end(),
                  [](const NeighborSummary& x, const NeighborSummary& y) {
                      return x.f_inliers > y.f_inliers;
                  });
        a.top_neighbors.assign(reg_nbs.begin(),
                               reg_nbs.begin() + std::min(reg_nbs.size(), size_t(5)));
        unregistered_analysis.push_back(std::move(a));
    }

    std::sort(unregistered_analysis.begin(), unregistered_analysis.end(),
              [](const ImageAnalysis& x, const ImageAnalysis& y) {
                  return x.n_3d2d > y.n_3d2d;
              });

    // Count images with enough 3D-2D for registration
    int raw_resectable = 0;
    for (const auto& a : unregistered_analysis)
        if (a.n_3d2d >= min_3d2d)
            ++raw_resectable;

    // ── 6. Console summary ────────────────────────────────────────────────────
    std::cout << "\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  SfM Diag2: " << n_unregistered << " unregistered / " << n_images << " total\n";
    std::cout << "  After clean triangulation: total_tri=" << total_tri
              << " (newly=" << newly_tri << ")\n";
    std::cout << "  min_tri_angle=" << min_tri_angle << " deg"
              << "  inlier_px=" << inlier_px << "\n";
    std::cout << "  Images with >= " << min_3d2d << " 3D-2D matches: " << raw_resectable << "\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    int cat_a = 0, cat_b = 0, cat_c = 0;
    for (const auto& a : unregistered_analysis) {
        if      (a.n_3d2d >= min_3d2d)              ++cat_a;
        else if (a.n_registered_neighbors > 0)      ++cat_b;
        else                                         ++cat_c;
    }
    std::cout << "  Category A (enough 3D-2D, fixable by better tri):  " << cat_a << "\n";
    std::cout << "  Category B (reg neighbors but <" << min_3d2d << " 3D-2D, weak match): " << cat_b << "\n";
    std::cout << "  Category C (no ViewGraph edge to any registered):   " << cat_c << "\n\n";

    std::cout << " idx |  3D-2D | tracks | reg_nb | max_F | Category\n";
    std::cout << "-----|--------|--------|--------|-------|----------\n";
    for (const auto& a : unregistered_analysis) {
        const char cat = (a.n_3d2d >= min_3d2d) ? 'A'
                       : (a.n_registered_neighbors > 0) ? 'B' : 'C';
        std::printf(" %3d | %6d | %6d | %6d | %5d | %c\n",
                    a.image_index, a.n_3d2d, a.n_tracks_total,
                    a.n_registered_neighbors, a.max_f_inliers_with_registered, cat);
    }
    std::cout << "\n";

    // ── 8. JSON report ────────────────────────────────────────────────────────
    json report;
    report["summary"] = {
        {"n_images",                         n_images},
        {"n_registered",                     n_registered},
        {"n_unregistered",                   n_unregistered},
        {"n_tracks",                         n_tracks},
        {"n_triangulated",                   total_tri},
        {"n_newly_triangulated",             newly_tri},
        {"n_with_enough_3d2d",               raw_resectable},
        {"min_3d2d_threshold",               min_3d2d},
        {"min_tri_angle_deg",                min_tri_angle},
        {"inlier_px",                        inlier_px},
        {"category_a_enough_3d2d",           cat_a},
        {"category_b_weak_matches",          cat_b},
        {"category_c_no_registered_edge",    cat_c}
    };

    json unreg_arr = json::array();
    for (const auto& a : unregistered_analysis) {
        const char cat = (a.n_3d2d >= min_3d2d) ? 'A'
                       : (a.n_registered_neighbors > 0) ? 'B' : 'C';
        json entry = {
            {"image_index",                       a.image_index},
            {"n_3d2d",                            a.n_3d2d},
            {"n_tracks_total",                    a.n_tracks_total},
            {"n_registered_neighbors",            a.n_registered_neighbors},
            {"max_f_inliers_with_registered",     a.max_f_inliers_with_registered},
            {"total_f_inliers_with_registered",   a.total_f_inliers_with_registered},
            {"category",                          std::string(1, cat)}
        };
        json top_nb = json::array();
        for (const auto& nb : a.top_neighbors)
            top_nb.push_back({{"image_index", nb.neighbor_image_index},
                              {"f_inliers",   nb.f_inliers},
                              {"e_ok",        nb.e_ok},
                              {"twoview_ok",  nb.twoview_ok},
                              {"stable",      nb.stable}});
        entry["top_registered_neighbors"] = top_nb;
        unreg_arr.push_back(std::move(entry));
    }
    report["unregistered_images"] = std::move(unreg_arr);

    const std::string report_str = report.dump(2);
    if (output_path.empty()) {
        std::cout << report_str << "\n";
    } else {
        std::ofstream of(output_path);
        if (!of.is_open()) { LOG(ERROR) << "Cannot write report to " << output_path; }
        else { of << report_str; LOG(INFO) << "Wrote report to " << output_path; }
    }

    // ── 9. Save triangulated TrackStore ──────────────────────────────────────
    if (!save_tracks_path.empty()) {
        std::vector<uint32_t> idx_out(static_cast<size_t>(n_images));
        for (int i = 0; i < n_images; ++i)
            idx_out[i] = static_cast<uint32_t>(i);
        if (save_track_store_to_idc(store, idx_out, save_tracks_path, &vg))
            LOG(INFO) << "Saved triangulated tracks to " << save_tracks_path;
        else
            LOG(ERROR) << "Failed to save tracks to " << save_tracks_path;
    }

    // ── 10. Export Bundler ────────────────────────────────────────────────────
    if (!save_bundler_path.empty()) {
        const auto image_paths = load_image_paths(project_path, n_images);
        if (!save_bundler_format(
                save_bundler_path, store, n_images,
                poses.registered, poses.poses_R, poses.poses_C,
                project.cameras, project.image_to_camera_index,
                image_paths))
            LOG(ERROR) << "Failed to export Bundler to " << save_bundler_path;
    }

    return 0;
}
