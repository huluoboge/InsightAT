/**
 * focal_from_view_graph.cpp
 * GLOMAP-inspired focal estimation from a Fundamental-matrix view graph.
 */
#include "focal_from_view_graph.h"

#include "two_view_reconstruction.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace insight {
namespace sfm {
namespace {

double essential_residual(const Eigen::Matrix3d& F, double cx, double cy, double f) {
  Eigen::Matrix3d K;
  K << f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0;
  Eigen::Matrix3d E = K.transpose() * F * K;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(E);
  const auto& sv = svd.singularValues();
  return std::abs(sv[0] - sv[1]);
}

/// Weighted median of samples with integer weights (replicated conceptually).
double weighted_median(std::vector<std::pair<double, int>> samples) {
  if (samples.empty())
    return -1.0;
  std::sort(samples.begin(), samples.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  int total = 0;
  for (const auto& s : samples)
    total += std::max(1, s.second);
  const int half = total / 2;
  int acc = 0;
  for (const auto& s : samples) {
    acc += std::max(1, s.second);
    if (acc > half)
      return s.first;
  }
  return samples.back().first;
}

struct EssentialResidualCost {
  EssentialResidualCost(const Eigen::Matrix3d& F, double cx, double cy, double weight)
      : F_(F), cx_(cx), cy_(cy), weight_(std::sqrt(std::max(1.0, weight))) {}

  template <typename T>
  bool operator()(const T* const f, T* residual) const {
    // Autodiff over f via finite residual on the SVD path is awkward; use
    // numeric residual with Jets through a cast-free double path is not
    // possible.  Instead evaluate residual in double and scale (Ceres
    // NumericDiff).  This functor is only used with NumericDiffCostFunction.
    (void)f;
    (void)residual;
    return false;
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const {
    const double f = parameters[0][0];
    if (!(f > 1e-3)) {
      residuals[0] = 1e3 * weight_;
      if (jacobians && jacobians[0])
        jacobians[0][0] = 0.0;
      return true;
    }
    residuals[0] = weight_ * essential_residual(F_, cx_, cy_, f);
    if (jacobians && jacobians[0]) {
      const double eps = std::max(1e-3, 1e-4 * f);
      const double r_plus = weight_ * essential_residual(F_, cx_, cy_, f + eps);
      const double r_minus = weight_ * essential_residual(F_, cx_, cy_, f - eps);
      jacobians[0][0] = (r_plus - r_minus) / (2.0 * eps);
    }
    return true;
  }

  Eigen::Matrix3d F_;
  double cx_, cy_, weight_;
};

class EssentialResidualCostFunction : public ceres::SizedCostFunction<1, 1> {
public:
  EssentialResidualCostFunction(const Eigen::Matrix3d& F, double cx, double cy, double weight)
      : impl_(F, cx, cy, weight) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    return impl_.Evaluate(parameters, residuals, jacobians);
  }

private:
  EssentialResidualCost impl_;
};

bool refine_shared_focal_ceres(const std::vector<FocalPairConstraint>& pairs,
                               const std::vector<int>& pair_indices, double* f_inout,
                               const FocalFromViewGraphOptions& opts) {
  if (!f_inout || pair_indices.empty())
    return false;

  ceres::Problem problem;
  problem.AddParameterBlock(f_inout, 1);
  problem.SetParameterLowerBound(f_inout, 0, opts.f_min);
  problem.SetParameterUpperBound(f_inout, 0, opts.f_max);

  ceres::LossFunction* loss = new ceres::CauchyLoss(opts.ceres_cauchy_scale);
  for (int idx : pair_indices) {
    const auto& p = pairs[static_cast<size_t>(idx)];
    const double cx = 0.5 * (p.cx1 + p.cx2);
    const double cy = 0.5 * (p.cy1 + p.cy2);
    problem.AddResidualBlock(
        new EssentialResidualCostFunction(p.F, cx, cy, static_cast<double>(p.weight)), loss,
        f_inout);
  }

  ceres::Solver::Options so;
  so.max_num_iterations = opts.max_ceres_iterations;
  so.linear_solver_type = ceres::DENSE_QR;
  so.minimizer_progress_to_stdout = false;
  so.num_threads = std::max(1, opts.ceres_num_threads);
  so.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(so, &problem, &summary);
  VLOG(1) << "focal_from_view_graph Ceres: " << summary.BriefReport();
  return summary.IsSolutionUsable() && *f_inout > opts.f_min && *f_inout < opts.f_max;
}

} // namespace

FocalFromViewGraphResult estimate_focals_from_view_graph(
    const std::vector<FocalPairConstraint>& pairs, const FocalFromViewGraphOptions& opts) {
  FocalFromViewGraphResult result;
  result.method = "robust_median";

  // camera_id → list of (pair_index, per-pair focal, weight)
  struct Sample {
    int pair_index;
    double focal;
    int weight;
  };
  std::unordered_map<int, std::vector<Sample>> samples_by_cam;

  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto& p = pairs[i];
    if (p.is_degenerate || p.weight < opts.min_inliers) {
      ++result.num_pairs_skipped;
      continue;
    }
    if (p.camera_id1 != p.camera_id2) {
      // v1: shared-camera only (ETH3D / single DSLR group).
      ++result.num_pairs_skipped;
      continue;
    }

    const double cx = 0.5 * (p.cx1 + p.cx2);
    const double cy = 0.5 * (p.cy1 + p.cy2);
    const double f = focal_from_fundamental(p.F, cx, cy, opts.f_min, opts.f_max);
    if (!(f > 0.0)) {
      ++result.num_pairs_skipped;
      continue;
    }

    const double prior =
        opts.prior_focal > 0.0
            ? opts.prior_focal
            : (opts.soft_prior_focal > 0.0 ? opts.soft_prior_focal : 0.0);
    if (prior > 0.0) {
      const double ratio = f / prior;
      if (ratio < opts.prior_ratio_lo || ratio > opts.prior_ratio_hi) {
        ++result.num_pairs_skipped;
        continue;
      }
    }

    samples_by_cam[p.camera_id1].push_back(
        Sample{static_cast<int>(i), f, std::max(1, p.weight)});
    ++result.num_pairs_used;
  }

  if (samples_by_cam.empty()) {
    LOG(WARNING) << "focal_from_view_graph: no usable shared-camera F constraints";
    return result;
  }

  for (auto& [camera_id, samples] : samples_by_cam) {
    CameraFocalEstimate est;
    est.camera_id = camera_id;
    est.num_pairs = static_cast<int>(samples.size());
    if (est.num_pairs < opts.min_pairs_per_camera) {
      est.ok = false;
      est.num_rejected = est.num_pairs;
      result.cameras.push_back(est);
      continue;
    }

    std::vector<std::pair<double, int>> wm;
    wm.reserve(samples.size());
    for (const auto& s : samples)
      wm.emplace_back(s.focal, s.weight);
    est.median_focal = weighted_median(std::move(wm));
    est.focal = est.median_focal;

    if (opts.refine_with_ceres) {
      std::vector<int> idxs;
      idxs.reserve(samples.size());
      for (const auto& s : samples)
        idxs.push_back(s.pair_index);
      double f = est.focal;
      if (refine_shared_focal_ceres(pairs, idxs, &f, opts)) {
        est.focal = f;
        result.method = "robust_median+ceres";
      }
    }

    est.ok = (est.focal > opts.f_min && est.focal < opts.f_max);
    result.cameras.push_back(est);
    LOG(INFO) << "focal_from_view_graph cam=" << camera_id << " pairs=" << est.num_pairs
              << " median=" << est.median_focal << " final=" << est.focal
              << " ok=" << est.ok;
  }

  std::sort(result.cameras.begin(), result.cameras.end(),
            [](const CameraFocalEstimate& a, const CameraFocalEstimate& b) {
              return a.camera_id < b.camera_id;
            });

  for (const auto& c : result.cameras) {
    if (c.ok) {
      result.ok = true;
      result.shared_focal = c.focal;
      break;
    }
  }
  // If multiple cameras, shared_focal is the first ok one (caller should use cameras[]).
  return result;
}

} // namespace sfm
} // namespace insight
