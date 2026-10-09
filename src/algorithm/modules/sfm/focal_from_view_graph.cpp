/**
 * focal_from_view_graph.cpp
 * GLOMAP-inspired focal estimation from a Fundamental-matrix view graph.
 */
#include "focal_from_view_graph.h"

#include "two_view_reconstruction.h"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace insight {
namespace sfm {
namespace {

Eigen::Matrix3d normalize_fundamental(const Eigen::Matrix3d& F) {
  const double norm = F.norm();
  if (!(norm > 1e-15) || !std::isfinite(norm))
    return Eigen::Matrix3d::Zero();
  return F / norm;
}

double essential_residual_same(const Eigen::Matrix3d& F, double cx, double cy, double f) {
  if (!(f > 0.0) || !std::isfinite(f))
    return 1.0;
  Eigen::Matrix3d K;
  K << f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0;
  Eigen::Matrix3d E = K.transpose() * normalize_fundamental(F) * K;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(E);
  const auto& sv = svd.singularValues();
  const double scale = sv[0] + sv[1];
  if (!(scale > 1e-15) || !std::isfinite(scale))
    return 1.0;
  return std::abs(sv[0] - sv[1]) / scale;
}

double essential_residual_two(const Eigen::Matrix3d& F, double cx1, double cy1, double f1,
                              double cx2, double cy2, double f2) {
  if (!(f1 > 0.0) || !(f2 > 0.0) || !std::isfinite(f1) || !std::isfinite(f2))
    return 1.0;
  Eigen::Matrix3d K1, K2;
  K1 << f1, 0.0, cx1, 0.0, f1, cy1, 0.0, 0.0, 1.0;
  K2 << f2, 0.0, cx2, 0.0, f2, cy2, 0.0, 0.0, 1.0;
  Eigen::Matrix3d E = K2.transpose() * normalize_fundamental(F) * K1;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(E);
  const auto& sv = svd.singularValues();
  const double scale = sv[0] + sv[1];
  if (!(scale > 1e-15) || !std::isfinite(scale))
    return 1.0;
  return std::abs(sv[0] - sv[1]) / scale;
}

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

void resolve_camera_bounds(const FocalFromViewGraphOptions& opts, int camera_id, double* f_min,
                           double* f_max, double* soft_prior) {
  double lo = opts.f_min, hi = opts.f_max, prior = opts.soft_prior_focal;
  if (opts.prior_focal > 0.0)
    prior = opts.prior_focal;

  if (camera_id >= 0 && camera_id < static_cast<int>(opts.camera_priors.size())) {
    const auto& p = opts.camera_priors[static_cast<size_t>(camera_id)];
    if (p.f_min > 0.0 && p.f_max > p.f_min) {
      lo = p.f_min;
      hi = p.f_max;
    } else if (p.width > 0.0 && p.height > 0.0) {
      focal_bounds_from_image_size(p.width, p.height, &lo, &hi, nullptr);
    }
    if (p.soft_prior > 0.0)
      prior = p.soft_prior;
    else if (p.width > 0.0 && p.height > 0.0)
      prior = 0.7 * std::max(p.width, p.height);
  }
  if (f_min)
    *f_min = lo;
  if (f_max)
    *f_max = hi;
  if (soft_prior)
    *soft_prior = prior;
}

class SameFocalCostFunction : public ceres::SizedCostFunction<1, 1> {
public:
  SameFocalCostFunction(const Eigen::Matrix3d& F, double cx, double cy, double weight)
      : F_(F), cx_(cx), cy_(cy), weight_(std::sqrt(std::max(1.0, weight))) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double log_f = parameters[0][0];
    const double f = std::exp(log_f);
    if (!(f > 1e-3) || !std::isfinite(f)) {
      residuals[0] = 1e3 * weight_;
      if (jacobians && jacobians[0])
        jacobians[0][0] = 0.0;
      return true;
    }
    residuals[0] = weight_ * essential_residual_same(F_, cx_, cy_, f);
    if (jacobians && jacobians[0]) {
      constexpr double eps = 1e-5;
      const double r_plus = weight_ * essential_residual_same(F_, cx_, cy_,
                                                               std::exp(log_f + eps));
      const double r_minus = weight_ * essential_residual_same(F_, cx_, cy_,
                                                                std::exp(log_f - eps));
      jacobians[0][0] = (r_plus - r_minus) / (2.0 * eps);
    }
    return true;
  }

private:
  Eigen::Matrix3d F_;
  double cx_, cy_, weight_;
};

class TwoFocalCostFunction : public ceres::SizedCostFunction<1, 1, 1> {
public:
  TwoFocalCostFunction(const Eigen::Matrix3d& F, double cx1, double cy1, double cx2, double cy2,
                       double weight)
      : F_(F), cx1_(cx1), cy1_(cy1), cx2_(cx2), cy2_(cy2),
        weight_(std::sqrt(std::max(1.0, weight))) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double log_f1 = parameters[0][0];
    const double log_f2 = parameters[1][0];
    const double f1 = std::exp(log_f1);
    const double f2 = std::exp(log_f2);
    if (!(f1 > 1e-3) || !(f2 > 1e-3) || !std::isfinite(f1) || !std::isfinite(f2)) {
      residuals[0] = 1e3 * weight_;
      if (jacobians) {
        if (jacobians[0])
          jacobians[0][0] = 0.0;
        if (jacobians[1])
          jacobians[1][0] = 0.0;
      }
      return true;
    }
    residuals[0] = weight_ * essential_residual_two(F_, cx1_, cy1_, f1, cx2_, cy2_, f2);
    if (jacobians) {
      constexpr double eps = 1e-5;
      if (jacobians[0]) {
        const double r_plus = weight_ * essential_residual_two(
                                             F_, cx1_, cy1_, std::exp(log_f1 + eps), cx2_, cy2_,
                                             f2);
        const double r_minus = weight_ * essential_residual_two(
                                              F_, cx1_, cy1_, std::exp(log_f1 - eps), cx2_, cy2_,
                                              f2);
        jacobians[0][0] = (r_plus - r_minus) / (2.0 * eps);
      }
      if (jacobians[1]) {
        const double r_plus = weight_ * essential_residual_two(
                                             F_, cx1_, cy1_, f1, cx2_, cy2_, std::exp(log_f2 + eps));
        const double r_minus = weight_ * essential_residual_two(
                                              F_, cx1_, cy1_, f1, cx2_, cy2_, std::exp(log_f2 - eps));
        jacobians[1][0] = (r_plus - r_minus) / (2.0 * eps);
      }
    }
    return true;
  }

private:
  Eigen::Matrix3d F_;
  double cx1_, cy1_, cx2_, cy2_, weight_;
};

class LogSoftPriorCostFunction : public ceres::SizedCostFunction<1, 1> {
public:
  LogSoftPriorCostFunction(double prior, double sigma)
      : prior_(std::log(std::max(1e-3, prior))),
        inv_sigma_(1.0 / std::max(1e-3, sigma)) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    residuals[0] = (parameters[0][0] - prior_) * inv_sigma_;
    if (jacobians && jacobians[0])
      jacobians[0][0] = inv_sigma_;
    return true;
  }

private:
  double prior_, inv_sigma_;
};

bool refine_focals_ceres(const std::vector<FocalPairConstraint>& pairs,
                         const std::vector<int>& pair_indices,
                         const FocalFromViewGraphOptions& opts,
                         std::unordered_map<int, double>* f_by_cam) {
  if (!f_by_cam || f_by_cam->empty() || pair_indices.empty())
    return false;

  ceres::Problem problem;
  std::unordered_map<int, double> log_f_by_cam;
  std::unordered_map<int, double*> ptrs;
  log_f_by_cam.reserve(f_by_cam->size());
  ptrs.reserve(f_by_cam->size());
  for (const auto& kv : *f_by_cam) {
    double lo = opts.f_min, hi = opts.f_max, prior = opts.soft_prior_focal;
    resolve_camera_bounds(opts, kv.first, &lo, &hi, &prior);
    if (!(lo > 0.0) || !(hi > lo))
      return false;
    double initial = kv.second;
    if (!(initial > 0.0) || !std::isfinite(initial))
      initial = prior > 0.0 ? prior : 0.5 * (lo + hi);
    initial = std::max(lo, std::min(hi, initial));
    log_f_by_cam.emplace(kv.first, std::log(initial));
  }
  for (auto& kv : log_f_by_cam) {
    problem.AddParameterBlock(&kv.second, 1);
    double lo = opts.f_min, hi = opts.f_max, prior = opts.soft_prior_focal;
    resolve_camera_bounds(opts, kv.first, &lo, &hi, &prior);
    problem.SetParameterLowerBound(&kv.second, 0, std::log(lo));
    problem.SetParameterUpperBound(&kv.second, 0, std::log(hi));
    ptrs[kv.first] = &kv.second;

    // Optional weak anchor in the same log-focal space as the joint variables.
    if (opts.ceres_prior_sigma_frac > 0.0 && prior > 0.0) {
      problem.AddResidualBlock(
          new LogSoftPriorCostFunction(prior, opts.ceres_prior_sigma_frac), nullptr, &kv.second);
    }
  }

  ceres::LossFunction* loss = new ceres::CauchyLoss(opts.ceres_cauchy_scale);
  int n_same = 0, n_cross = 0;
  for (int idx : pair_indices) {
    const auto& p = pairs[static_cast<size_t>(idx)];
    auto it1 = ptrs.find(p.camera_id1);
    auto it2 = ptrs.find(p.camera_id2);
    if (it1 == ptrs.end() || it2 == ptrs.end())
      continue;
    if (p.camera_id1 == p.camera_id2) {
      const double cx = 0.5 * (p.cx1 + p.cx2);
      const double cy = 0.5 * (p.cy1 + p.cy2);
      problem.AddResidualBlock(
          new SameFocalCostFunction(p.F, cx, cy, static_cast<double>(p.weight)), loss, it1->second);
      ++n_same;
    } else if (opts.use_cross_camera_pairs) {
      problem.AddResidualBlock(new TwoFocalCostFunction(p.F, p.cx1, p.cy1, p.cx2, p.cy2,
                                                        static_cast<double>(p.weight)),
                               loss, it1->second, it2->second);
      ++n_cross;
    }
  }
  if (problem.NumResidualBlocks() == 0)
    return false;

  ceres::Solver::Options so;
  so.max_num_iterations = opts.max_ceres_iterations;
  so.linear_solver_type = (f_by_cam->size() <= 2) ? ceres::DENSE_QR : ceres::DENSE_SCHUR;
  so.minimizer_progress_to_stdout = false;
  so.num_threads = std::max(1, opts.ceres_num_threads);
  so.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(so, &problem, &summary);
  VLOG(1) << "focal_from_view_graph Ceres: same=" << n_same << " cross=" << n_cross << " "
          << summary.BriefReport();
  if (!summary.IsSolutionUsable())
    return false;

  for (auto& kv : *f_by_cam) {
    const auto it = log_f_by_cam.find(kv.first);
    if (it == log_f_by_cam.end())
      return false;
    kv.second = std::exp(it->second);
    if (!(kv.second > 0.0) || !std::isfinite(kv.second))
      return false;
  }
  return true;
}

static double focal_constraint_residual(const FocalPairConstraint& p, double log_f1,
                                        double log_f2) {
  const double f1 = std::exp(log_f1);
  const double f2 = std::exp(log_f2);
  if (p.camera_id1 == p.camera_id2) {
    return essential_residual_same(p.F, 0.5 * (p.cx1 + p.cx2), 0.5 * (p.cy1 + p.cy2), f1);
  }
  return essential_residual_two(p.F, p.cx1, p.cy1, f1, p.cx2, p.cy2, f2);
}

static double log_focal_residual_sensitivity(const FocalPairConstraint& p, double log_f1,
                                             double log_f2, bool first_parameter) {
  constexpr double eps = 1e-4;
  const double r0 = focal_constraint_residual(p, log_f1, log_f2);
  const double r_plus = first_parameter
                            ? focal_constraint_residual(p, log_f1 + eps, log_f2)
                            : focal_constraint_residual(p, log_f1, log_f2 + eps);
  const double r_minus = first_parameter
                             ? focal_constraint_residual(p, log_f1 - eps, log_f2)
                             : focal_constraint_residual(p, log_f1, log_f2 - eps);
  return std::max(std::abs(r_plus - r0), std::abs(r_minus - r0)) / eps;
}

static void assess_joint_observability(
    const std::vector<FocalPairConstraint>& pairs, const std::vector<int>& pair_indices,
    const std::unordered_map<int, double>& f_by_cam,
    std::unordered_map<int, CameraFocalEstimate>* estimates) {
  if (!estimates)
    return;

  std::unordered_map<int, std::vector<int>> incident;
  incident.reserve(f_by_cam.size());
  for (const auto& kv : f_by_cam)
    incident.emplace(kv.first, std::vector<int>());

  for (int idx : pair_indices) {
    if (idx < 0 || idx >= static_cast<int>(pairs.size()))
      continue;
    const auto& p = pairs[static_cast<size_t>(idx)];
    if (incident.find(p.camera_id1) == incident.end() ||
        incident.find(p.camera_id2) == incident.end())
      continue;
    incident[p.camera_id1].push_back(idx);
    if (p.camera_id2 != p.camera_id1)
      incident[p.camera_id2].push_back(idx);
  }

  for (auto& kv : *estimates) {
    const auto it = incident.find(kv.first);
    kv.second.num_constraints = it == incident.end() ? 0 : static_cast<int>(it->second.size());
    kv.second.observable_rank = 0;
    kv.second.observable = false;
  }

  std::unordered_set<int> visited;
  visited.reserve(f_by_cam.size());
  for (const auto& root : f_by_cam) {
    if (!visited.insert(root.first).second)
      continue;

    std::vector<int> stack{root.first};
    std::vector<int> cameras;
    std::vector<int> component_pairs;
    std::unordered_set<int> component_pair_set;
    while (!stack.empty()) {
      const int camera_id = stack.back();
      stack.pop_back();
      cameras.push_back(camera_id);
      const auto incident_it = incident.find(camera_id);
      if (incident_it == incident.end())
        continue;
      for (const int pair_index : incident_it->second) {
        if (component_pair_set.insert(pair_index).second)
          component_pairs.push_back(pair_index);
        const auto& p = pairs[static_cast<size_t>(pair_index)];
        const int other = p.camera_id1 == camera_id ? p.camera_id2 : p.camera_id1;
        if (other != camera_id && visited.insert(other).second)
          stack.push_back(other);
      }
    }

    std::unordered_map<int, int> column;
    column.reserve(cameras.size());
    for (int i = 0; i < static_cast<int>(cameras.size()); ++i)
      column[cameras[static_cast<size_t>(i)]] = i;

    Eigen::MatrixXd jacobian =
        Eigen::MatrixXd::Zero(static_cast<int>(component_pairs.size()),
                              static_cast<int>(cameras.size()));
    for (int row = 0; row < static_cast<int>(component_pairs.size()); ++row) {
      const auto& p = pairs[static_cast<size_t>(component_pairs[static_cast<size_t>(row)])];
      const double log_f1 = std::log(f_by_cam.at(p.camera_id1));
      const double log_f2 = std::log(f_by_cam.at(p.camera_id2));
      jacobian(row, column.at(p.camera_id1)) =
          log_focal_residual_sensitivity(p, log_f1, log_f2, true);
      if (p.camera_id2 != p.camera_id1) {
        jacobian(row, column.at(p.camera_id2)) =
            log_focal_residual_sensitivity(p, log_f1, log_f2, false);
      }
    }

    if (component_pairs.empty()) {
      for (const int camera_id : cameras) {
        auto& estimate = estimates->at(camera_id);
        estimate.observable_rank = 0;
        estimate.observable = false;
      }
      continue;
    }

    for (int col = 0; col < jacobian.cols(); ++col) {
      const double norm = jacobian.col(col).norm();
      if (norm > 1e-12)
        jacobian.col(col) /= norm;
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
    const double max_sv = svd.singularValues().size() > 0 ? svd.singularValues()[0] : 0.0;
    const double rank_threshold = std::max(1e-8, max_sv * 1e-6);
    int rank = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
      if (svd.singularValues()[i] > rank_threshold)
        ++rank;
    }
    const bool observable = rank == static_cast<int>(cameras.size());
    for (const int camera_id : cameras) {
      auto& estimate = estimates->at(camera_id);
      estimate.observable_rank = rank;
      estimate.observable = observable;
    }
  }
}

} // namespace

FocalFromViewGraphResult estimate_focals_from_view_graph(
    const std::vector<FocalPairConstraint>& pairs, const FocalFromViewGraphOptions& opts) {
  FocalFromViewGraphResult result;
  result.method = "robust_median";

  struct Sample {
    int pair_index;
    double focal;
    int weight;
  };

  struct PairSample {
    int camera_id = -1;
    double focal = -1.0;
    int weight = 0;
    bool ok = false;
  };
  std::vector<PairSample> pair_samples(pairs.size());
  int skipped = 0;
  std::vector<int> cross_pair_indices;
  cross_pair_indices.reserve(pairs.size() / 8);

#ifdef _OPENMP
  const int nthreads = std::max(1, opts.hartley_num_threads);
  omp_set_num_threads(nthreads);
#pragma omp parallel for schedule(dynamic, 64) reduction(+ : skipped)
#endif
  for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
    const auto& p = pairs[static_cast<size_t>(i)];
    if (p.is_degenerate || p.weight < opts.min_inliers) {
      ++skipped;
      continue;
    }

    // Cross-camera: skip Hartley (needs two focals); keep for joint Ceres.
    if (p.camera_id1 != p.camera_id2) {
#ifdef _OPENMP
#pragma omp critical(focal_cross_list)
#endif
      cross_pair_indices.push_back(i);
      continue;
    }

    double f_min = opts.f_min, f_max = opts.f_max, prior = opts.soft_prior_focal;
    resolve_camera_bounds(opts, p.camera_id1, &f_min, &f_max, &prior);
    if (opts.prior_focal > 0.0)
      prior = opts.prior_focal;

    const double cx = 0.5 * (p.cx1 + p.cx2);
    const double cy = 0.5 * (p.cy1 + p.cy2);
    const double f = focal_from_fundamental(p.F, cx, cy, f_min, f_max);
    if (!(f > 0.0)) {
      ++skipped;
      continue;
    }

    if (prior > 0.0) {
      const double ratio = f / prior;
      if (ratio < opts.prior_ratio_lo || ratio > opts.prior_ratio_hi) {
        ++skipped;
        continue;
      }
    }

    PairSample& ps = pair_samples[static_cast<size_t>(i)];
    ps.camera_id = p.camera_id1;
    ps.focal = f;
    ps.weight = std::max(1, p.weight);
    ps.ok = true;
  }
  result.num_pairs_skipped = skipped;

  std::unordered_map<int, std::vector<Sample>> samples_by_cam;
  std::vector<int> same_pair_indices;
  same_pair_indices.reserve(pairs.size());
  for (size_t i = 0; i < pair_samples.size(); ++i) {
    const auto& ps = pair_samples[i];
    if (!ps.ok)
      continue;
    samples_by_cam[ps.camera_id].push_back(
        Sample{static_cast<int>(i), ps.focal, ps.weight});
    same_pair_indices.push_back(static_cast<int>(i));
    ++result.num_pairs_used;
  }

  // Cameras that only appear in cross pairs still need an init for Ceres.
  std::unordered_set<int> all_cams;
  for (const auto& p : pairs) {
    if (p.is_degenerate || p.weight < opts.min_inliers)
      continue;
    all_cams.insert(p.camera_id1);
    all_cams.insert(p.camera_id2);
  }

  if (samples_by_cam.empty() && !(opts.refine_with_ceres && opts.use_cross_camera_pairs &&
                                  !cross_pair_indices.empty())) {
    LOG(WARNING) << "focal_from_view_graph: no usable F constraints";
    return result;
  }

  std::unordered_map<int, double> f_by_cam;
  std::unordered_map<int, CameraFocalEstimate> est_by_cam;

  for (int cam_id : all_cams) {
    CameraFocalEstimate est;
    est.camera_id = cam_id;
    double f_min = opts.f_min, f_max = opts.f_max, prior = opts.soft_prior_focal;
    resolve_camera_bounds(opts, cam_id, &f_min, &f_max, &prior);

    auto it = samples_by_cam.find(cam_id);
    if (it != samples_by_cam.end() &&
        static_cast<int>(it->second.size()) >= opts.min_pairs_per_camera) {
      std::vector<std::pair<double, int>> wm;
      wm.reserve(it->second.size());
      for (const auto& s : it->second)
        wm.emplace_back(s.focal, s.weight);
      est.median_focal = weighted_median(std::move(wm));
      est.focal = est.median_focal;
      est.num_pairs = static_cast<int>(it->second.size());
      est.ok = (est.focal > f_min && est.focal < f_max);
    } else {
      est.num_pairs = it != samples_by_cam.end() ? static_cast<int>(it->second.size()) : 0;
      est.num_rejected = est.num_pairs;
      est.median_focal = prior > 0.0 ? prior : 0.5 * (f_min + f_max);
      est.focal = est.median_focal;
      est.ok = false; // may become ok after cross-camera Ceres
    }
    f_by_cam[cam_id] = est.focal;
    est_by_cam[cam_id] = est;
  }

  std::vector<int> joint_pairs = same_pair_indices;
  if (opts.use_cross_camera_pairs) {
    joint_pairs.insert(joint_pairs.end(), cross_pair_indices.begin(), cross_pair_indices.end());
    result.num_cross_pairs_used = static_cast<int>(cross_pair_indices.size());
  }

  bool joint_solved = false;
  if (opts.refine_with_ceres && !f_by_cam.empty())
    joint_solved = refine_focals_ceres(pairs, joint_pairs, opts, &f_by_cam);

  if (joint_solved)
    result.method = "joint_ceres";

  for (auto& kv : est_by_cam) {
    const int cam_id = kv.first;
    double f_min = opts.f_min, f_max = opts.f_max;
    resolve_camera_bounds(opts, cam_id, &f_min, &f_max, nullptr);
    kv.second.focal = f_by_cam[cam_id];
    int n_cross = 0;
    for (int idx : cross_pair_indices) {
      const auto& p = pairs[static_cast<size_t>(idx)];
      if (p.camera_id1 == cam_id || p.camera_id2 == cam_id)
        ++n_cross;
    }
    kv.second.num_cross_pairs = n_cross;
    kv.second.ok = !opts.refine_with_ceres && kv.second.num_pairs >= opts.min_pairs_per_camera &&
                  (kv.second.focal > f_min && kv.second.focal < f_max);
  }

  assess_joint_observability(pairs, joint_pairs, f_by_cam, &est_by_cam);
  for (auto& kv : est_by_cam) {
    double f_min = opts.f_min, f_max = opts.f_max;
    resolve_camera_bounds(opts, kv.first, &f_min, &f_max, nullptr);
    kv.second.ok = kv.second.observable && kv.second.focal > f_min &&
                   kv.second.focal < f_max && (!opts.refine_with_ceres || joint_solved);
  }

  result.cameras.clear();
  result.cameras.reserve(est_by_cam.size());
  for (auto& kv : est_by_cam) {
    result.cameras.push_back(kv.second);
    LOG(INFO) << "focal_from_view_graph cam=" << kv.second.camera_id
              << " same_pairs=" << kv.second.num_pairs
              << " cross_pairs=" << kv.second.num_cross_pairs
              << " constraints=" << kv.second.num_constraints
              << " rank=" << kv.second.observable_rank
              << " median=" << kv.second.median_focal << " final=" << kv.second.focal
              << " ok=" << kv.second.ok;
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
  return result;
}

} // namespace sfm
} // namespace insight
