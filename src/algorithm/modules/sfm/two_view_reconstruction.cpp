/**
 * two_view_reconstruction.cpp
 * InsightAT – Two-View Geometric Reconstruction  (CPU / Eigen)
 */
#include "two_view_reconstruction.h"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace insight {
namespace sfm {

// ─────────────────────────────────────────────────────────────────────────────
// Helper: essential-matrix residual r(f) = |σ₁(E) - σ₂(E)|
// ─────────────────────────────────────────────────────────────────────────────

static double EssentialResidual(const Eigen::Matrix3d& F,
                                 double cx, double cy, double f)
{
    // K = diag(f, f, 1) with pp at (cx, cy)
    Eigen::Matrix3d K;
    K << f,   0.0, cx,
         0.0, f,   cy,
         0.0, 0.0, 1.0;
    Eigen::Matrix3d E = K.transpose() * F * K;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E);
    const auto& sv = svd.singularValues();   // descending
    return std::abs(sv[0] - sv[1]);
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. FocalFromFundamental
// ─────────────────────────────────────────────────────────────────────────────

double FocalFromFundamental(const Eigen::Matrix3d& F,
                             double cx, double cy,
                             double f_min, double f_max)
{
    // Phase 1: coarse grid search (log-spaced, 200 steps)
    constexpr int N_GRID = 200;
    double best_f = (f_min + f_max) * 0.5;
    double best_r = std::numeric_limits<double>::max();
    const double log_lo = std::log(f_min), log_hi = std::log(f_max);
    for (int i = 0; i < N_GRID; ++i) {
        double f = std::exp(log_lo + (log_hi - log_lo) * i / (N_GRID - 1));
        double r = EssentialResidual(F, cx, cy, f);
        if (r < best_r) { best_r = r; best_f = f; }
    }

    // Phase 2: golden-section refinement in [best_f/1.1, best_f*1.1]
    const double phi = (std::sqrt(5.0) - 1.0) * 0.5; // ≈ 0.618
    double lo = best_f / 1.1, hi = best_f * 1.1;
    double x1 = hi - phi * (hi - lo);
    double x2 = lo + phi * (hi - lo);
    double f1  = EssentialResidual(F, cx, cy, x1);
    double f2  = EssentialResidual(F, cx, cy, x2);
    for (int iter = 0; iter < 60; ++iter) {
        if (f1 < f2) {
            hi = x2; x2 = x1; f2 = f1;
            x1 = hi - phi * (hi - lo);
            f1 = EssentialResidual(F, cx, cy, x1);
        } else {
            lo = x1; x1 = x2; f1 = f2;
            x2 = lo + phi * (hi - lo);
            f2 = EssentialResidual(F, cx, cy, x2);
        }
        if ((hi - lo) < 0.01) break;
    }
    best_f = (lo + hi) * 0.5;
    best_r = EssentialResidual(F, cx, cy, best_f);

    // Sanity: if residual is very large, F might be degenerate
    if (best_r > 1e3) return -1.0;
    return best_f;
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. EnforceEssential
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Matrix3d EnforceEssential(const Eigen::Matrix3d& M) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d s = svd.singularValues();
    double avg = (s[0] + s[1]) * 0.5;
    Eigen::Vector3d s_enforced(avg, avg, 0.0);
    return svd.matrixU() * s_enforced.asDiagonal() * svd.matrixV().transpose();
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. DecomposeEssential
// ─────────────────────────────────────────────────────────────────────────────

// Triangulate a single point (returns cam1-frame depth of the point).
// Returns (X, depth1, depth2); depth < 0 means behind camera.
static std::tuple<Eigen::Vector3d, double, double>
TriangulateForCheirality(const Eigen::Vector2d& x1n,
                          const Eigen::Vector2d& x2n,
                          const Eigen::Matrix3d& R,
                          const Eigen::Vector3d& t)
{
    // P1 = [I | 0], P2 = [R | t]
    // Build the 4×4 system  A x = 0
    Eigen::Matrix4d A;
    A.row(0) = x1n[0] * Eigen::RowVector4d(0,0,1,0) - Eigen::RowVector4d(1,0,0,0);
    A.row(1) = x1n[1] * Eigen::RowVector4d(0,0,1,0) - Eigen::RowVector4d(0,1,0,0);

    Eigen::RowVector4d P2r0(R(0,0), R(0,1), R(0,2), t[0]);
    Eigen::RowVector4d P2r1(R(1,0), R(1,1), R(1,2), t[1]);
    Eigen::RowVector4d P2r2(R(2,0), R(2,1), R(2,2), t[2]);

    A.row(2) = x2n[0] * P2r2 - P2r0;
    A.row(3) = x2n[1] * P2r2 - P2r1;

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d X4 = svd.matrixV().col(3);
    if (std::abs(X4[3]) < 1e-10) return {Eigen::Vector3d::Zero(), -1.0, -1.0};
    Eigen::Vector3d X = X4.head<3>() / X4[3];

    double depth1 = X[2]; // z in cam1
    Eigen::Vector3d Xc2 = R * X + t;
    double depth2 = Xc2[2]; // z in cam2
    return {X, depth1, depth2};
}

int DecomposeEssential(const Eigen::Matrix3d& E,
                        const std::vector<Eigen::Vector2d>& pts1_n,
                        const std::vector<Eigen::Vector2d>& pts2_n,
                        Eigen::Matrix3d& R_out,
                        Eigen::Vector3d& t_out)
{
    assert(pts1_n.size() == pts2_n.size());
    const int N = (int)pts1_n.size();

    // Standard E decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Ensure proper rotation (det = +1)
    if (U.determinant() < 0) U = -U;
    if (V.determinant() < 0) V = -V;

    Eigen::Matrix3d W;
    W << 0,-1, 0,
         1, 0, 0,
         0, 0, 1;

    const Eigen::Matrix3d R1 = U * W   * V.transpose();
    const Eigen::Matrix3d R2 = U * W.transpose() * V.transpose();
    const Eigen::Vector3d t1 =  U.col(2);
    const Eigen::Vector3d t2 = -U.col(2);

    // 4 candidates
    const std::array<std::pair<Eigen::Matrix3d, Eigen::Vector3d>, 4> candidates = {{
        {R1, t1}, {R1, t2}, {R2, t1}, {R2, t2}
    }};

    // Pick the candidate with the most cheirality-consistent triangulated points.
    // Use only first min(N, 50) points for speed.
    const int N_check = std::min(N, 50);
    int best_count = -1;
    int best_idx   = 0;

    for (int ci = 0; ci < 4; ++ci) {
        const auto& [R, t] = candidates[ci];
        int count = 0;
        for (int i = 0; i < N_check; ++i) {
            auto [X, d1, d2] = TriangulateForCheirality(pts1_n[i], pts2_n[i], R, t);
            if (d1 > 0.0 && d2 > 0.0) ++count;
        }
        if (count > best_count) { best_count = count; best_idx = ci; }
    }

    R_out = candidates[best_idx].first;
    t_out = candidates[best_idx].second;
    return best_count;
}

// ─────────────────────────────────────────────────────────────────────────────
// 4a. Degeneracy detection
// ─────────────────────────────────────────────────────────────────────────────

DegeneracyResult DetectDegeneracy(int F_inliers, int H_inliers, int num_matches)
{
    DegeneracyResult out;
    out.is_degenerate = false;
    out.model_preferred = 0;
    out.h_over_f_ratio = 0.0;

    if (num_matches < 8 || F_inliers < 8) {
        out.is_degenerate = true;  // too few points, can't trust any model
        return out;
    }

    if (H_inliers <= 0) {
        // H not estimated – assume general scene (F valid)
        return out;
    }

    const double h_over_f = static_cast<double>(H_inliers) / static_cast<double>(F_inliers);
    out.h_over_f_ratio = h_over_f;

    // COLMAP-style: if H explains data as well as F (similar or more inliers),
    // and H inlier ratio is high, likely planar – F is degenerate
    // F has 7 DOF, H has 8 DOF; AIC = 2*k + n*log(RSS).
    // Simpler heuristic: if H_inliers >= F_inliers * 1.15 and H has enough inliers
    constexpr double kPlanarThreshold = 1.15;
    constexpr int    kMinHInliers = 10;

    if (H_inliers >= kMinHInliers && h_over_f >= kPlanarThreshold) {
        out.is_degenerate = true;
        out.model_preferred = 1;
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// 4b. Stability metrics
// ─────────────────────────────────────────────────────────────────────────────

static double RadToDeg(double rad) {
    static constexpr double kPi = 3.14159265358979323846;
    return rad * 180.0 / kPi;
}

StabilityMetrics ComputeStabilityMetrics(
    const std::vector<Eigen::Vector3d>& points3d,
    const std::vector<Eigen::Vector2d>& pts1_n,
    const std::vector<Eigen::Vector2d>& pts2_n,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    double min_parallax_deg,
    double min_depth_baseline,
    double max_depth_baseline)
{
    StabilityMetrics out;
    out.median_parallax_deg = 0.0;
    out.median_depth_baseline = 0.0;
    out.num_valid = 0;
    out.is_stable = false;

    const size_t N = points3d.size();
    if (N != pts1_n.size() || N != pts2_n.size() || N == 0) return out;

    const double baseline = t.norm();
    if (baseline < 1e-12) return out;  // pure rotation, no baseline

    std::vector<double> parallax_deg, depth_baseline;
    parallax_deg.reserve(N);
    depth_baseline.reserve(N);

    const Eigen::Vector3d C2 = -R.transpose() * t;  // cam2 center in cam1 frame

    for (size_t i = 0; i < N; i++) {
        const Eigen::Vector3d& X = points3d[i];
        if (std::isnan(X.x()) || std::isnan(X.y()) || std::isnan(X.z())) continue;
        if (X.z() <= 1e-9) continue;  // behind cam1

        Eigen::Vector3d Xc2 = R * X + t;
        if (Xc2.z() <= 1e-9) continue;  // behind cam2

        // Parallax: angle between ray1 (from cam1 to X) and ray2 (from cam2 to X)
        const Eigen::Vector3d ray1 = X.normalized();
        const Eigen::Vector3d ray2 = (X - C2).normalized();
        const double cos_angle = std::max(-1.0, std::min(1.0, ray1.dot(ray2)));
        const double angle_rad = std::acos(cos_angle);
        parallax_deg.push_back(RadToDeg(angle_rad));

        // Depth / baseline: use cam1 depth
        const double depth = X.z();
        depth_baseline.push_back(depth / baseline);
    }

    out.num_valid = static_cast<int>(parallax_deg.size());
    if (out.num_valid < 3) return out;

    auto mid = out.num_valid / 2;
    std::nth_element(parallax_deg.begin(), parallax_deg.begin() + mid, parallax_deg.end());
    out.median_parallax_deg = parallax_deg[mid];
    if (out.num_valid % 2 == 0) {
        auto mid2 = out.num_valid / 2 - 1;
        std::nth_element(parallax_deg.begin(), parallax_deg.begin() + mid2, parallax_deg.end());
        out.median_parallax_deg = 0.5 * (parallax_deg[mid] + parallax_deg[mid2]);
    }

    std::nth_element(depth_baseline.begin(), depth_baseline.begin() + mid, depth_baseline.end());
    out.median_depth_baseline = depth_baseline[mid];
    if (out.num_valid % 2 == 0) {
        auto mid2 = out.num_valid / 2 - 1;
        std::nth_element(depth_baseline.begin(), depth_baseline.begin() + mid2, depth_baseline.end());
        out.median_depth_baseline = 0.5 * (depth_baseline[mid] + depth_baseline[mid2]);
    }

    out.is_stable = (out.median_parallax_deg >= min_parallax_deg &&
                     out.median_depth_baseline >= min_depth_baseline &&
                     out.median_depth_baseline <= max_depth_baseline);
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Triangulation
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d TriangulatePoint(const Eigen::Vector2d& x1,
                                  const Eigen::Vector2d& x2,
                                  const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& t)
{
    auto [X, d1, d2] = TriangulateForCheirality(x1, x2, R, t);
    (void)d1; (void)d2;
    return X;
}

std::vector<Eigen::Vector3d> TriangulatePoints(
        const std::vector<Eigen::Vector2d>& pts1_n,
        const std::vector<Eigen::Vector2d>& pts2_n,
        const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t)
{
    const int N = (int)pts1_n.size();
    std::vector<Eigen::Vector3d> result(N);
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector3d nan3(kNaN, kNaN, kNaN);

    for (int i = 0; i < N; ++i) {
        auto [X, d1, d2] = TriangulateForCheirality(pts1_n[i], pts2_n[i], R, t);
        if (d1 > 0.0 && d2 > 0.0)
            result[i] = X;
        else
            result[i] = nan3;
    }
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// End of two_view_reconstruction.cpp
// ─────────────────────────────────────────────────────────────────────────────

}  // namespace sfm
}  // namespace insight
