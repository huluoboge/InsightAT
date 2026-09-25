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
#include <unordered_set>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace insight {
namespace sfm {
namespace {

double essential_residual_same(const Eigen::Matrix3d& F, double cx, double cy, double f) {
  Eigen::Matrix3d K;
  K << f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0;
  Eigen::Matrix3d E = K.transpose() * F * K;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(E);
  const auto& sv = svd.singularValues();
  return std::abs(sv[0] - sv[1]);
}

double essential_residual_two(const Eigen::Matrix3d& F, double cx1, double cy1, double f1,
                              double cx2, double cy2, double f2) {
  Eigen::Matrix3d K1, K2;
  K1 << f1, 0.0, cx1, 0.0, f1, cy1, 0.0, 0.0, 1.0;
  K2 << f2, 0.0, cx2, 0.0, f2, cy2, 0.0, 0.0, 1.0;
  Eigen::Matrix3d E = K2.transpose() * F * K1;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(E);
  const auto& sv = svd.singularValues();
  return std::abs(sv[0] - sv[1]);
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
    const double f = parameters[0][0];
    if (!(f > 1e-3)) {
      residuals[0] = 1e3 * weight_;
      if (jacobians && jacobians[0])
        jacobians[0][0] = 0.0;
      return true;
    }
    residuals[0] = weight_ * essential_residual_same(F_, cx_, cy_, f);
    if (jacobians && jacobians[0]) {
      const double eps = std::max(1e-3, 1e-4 * f);
      const double r_plus = weight_ * essential_residual_same(F_, cx_, cy_, f + eps);
      const double r_minus = weight_ * essential_residual_same(F_, cx_, cy_, f - eps);
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
    const double f1 = parameters[0][0];
    const double f2 = parameters[1][0];
    if (!(f1 > 1e-3) || !(f2 > 1e-3)) {
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
      const double eps1 = std::max(1e-3, 1e-4 * f1);
      const double eps2 = std::max(1e-3, 1e-4 * f2);
      if (jacobians[0]) {
        const double r_plus =
            weight_ * essential_residual_two(F_, cx1_, cy1_, f1 + eps1, cx2_, cy2_, f2);
        const double r_minus =
            weight_ * essential_residual_two(F_, cx1_, cy1_, f1 - eps1, cx2_, cy2_, f2);
        jacobians[0][0] = (r_plus - r_minus) / (2.0 * eps1);
      }
      if (jacobians[1]) {
        const double r_plus =
            weight_ * essential_residual_two(F_, cx1_, cy1_, f1, cx2_, cy2_, f2 + eps2);
        const double r_minus =
            weight_ * essential_residual_two(F_, cx1_, cy1_, f1, cx2_, cy2_, f2 - eps2);
        jacobians[1][0] = (r_plus - r_minus) / (2.0 * eps2);
      }
    }
    return true;
  }

private:
  Eigen::Matrix3d F_;
  double cx1_, cy1_, cx2_, cy2_, weight_;
};

class SoftPriorCostFunction : public ceres::SizedCostFunction<1, 1> {
public:
  SoftPriorCostFunction(double prior, double sigma)
      : prior_(prior), inv_sigma_(1.0 / std::max(1e-3, sigma)) {}

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
  std::unordered_map<int, double*> ptrs;
  for (auto& kv : *f_by_cam) {
    problem.AddParameterBlock(&kv.second, 1);
    double lo = opts.f_min, hi = opts.f_max, prior = opts.soft_prior_focal;
    resolve_camera_bounds(opts, kv.first, &lo, &hi, &prior);
    problem.SetParameterLowerBound(&kv.second, 0, lo);
    problem.SetParameterUpperBound(&kv.second, 0, hi);
    ptrs[kv.first] = &kv.second;

    // Weak anchor toward image-size soft prior so F-noise cannot yank f to the bound.
    if (opts.ceres_prior_sigma_frac > 0.0 && prior > 0.0) {
      const double sigma = opts.ceres_prior_sigma_frac * prior;
      problem.AddResidualBlock(new SoftPriorCostFunction(prior, sigma), nullptr, &kv.second);
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
  return summary.IsSolutionUsable();
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

  if (opts.refine_with_ceres && !f_by_cam.empty()) {
    std::vector<int> ceres_pairs = same_pair_indices;
    if (opts.use_cross_camera_pairs) {
      ceres_pairs.insert(ceres_pairs.end(), cross_pair_indices.begin(), cross_pair_indices.end());
      result.num_cross_pairs_used = static_cast<int>(cross_pair_indices.size());
    }
    if (refine_focals_ceres(pairs, ceres_pairs, opts, &f_by_cam)) {
      result.method = samples_by_cam.empty() ? "cross_ceres" : "robust_median+ceres";
      for (auto& kv : est_by_cam) {
        const int cam_id = kv.first;
        double f_min = opts.f_min, f_max = opts.f_max;
        resolve_camera_bounds(opts, cam_id, &f_min, &f_max, nullptr);
        double f_new = f_by_cam[cam_id];
        // Guard: if Ceres drifts too far from robust median, keep the median.
        if (kv.second.median_focal > 0.0 && opts.ceres_max_median_rel_delta > 0.0) {
          const double rel = std::abs(f_new - kv.second.median_focal) / kv.second.median_focal;
          if (rel > opts.ceres_max_median_rel_delta) {
            LOG(WARNING) << "focal_from_view_graph cam=" << cam_id
                         << " Ceres f=" << f_new << " drifts " << (100.0 * rel)
                         << "% from median=" << kv.second.median_focal << "; keeping median";
            f_new = kv.second.median_focal;
            f_by_cam[cam_id] = f_new;
          }
        }
        kv.second.focal = f_new;
        // Count cross pairs touching this camera.
        int n_cross = 0;
        for (int idx : cross_pair_indices) {
          const auto& p = pairs[static_cast<size_t>(idx)];
          if (p.camera_id1 == cam_id || p.camera_id2 == cam_id)
            ++n_cross;
        }
        kv.second.num_cross_pairs = n_cross;
        const bool enough =
            kv.second.num_pairs >= opts.min_pairs_per_camera ||
            (opts.use_cross_camera_pairs && n_cross >= opts.min_pairs_per_camera);
        kv.second.ok = enough && (kv.second.focal > f_min && kv.second.focal < f_max);
      }
    }
  }

  result.cameras.clear();
  result.cameras.reserve(est_by_cam.size());
  for (auto& kv : est_by_cam) {
    result.cameras.push_back(kv.second);
    LOG(INFO) << "focal_from_view_graph cam=" << kv.second.camera_id
              << " same_pairs=" << kv.second.num_pairs
              << " cross_pairs=" << kv.second.num_cross_pairs
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
