#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace insight {
namespace tools {

struct SeedStrategyProfile {
  std::string id;
  std::string name;
  int init_min_inliers = 100;
  double init_max_forward_motion = 0.95;
  double init_min_angle_deg = 2.0;
  double init_min_median_angle_deg = 30.0;
};

struct SeedEvalMetrics {
  int registered_images = 0;
  int triangulated_points = 0;
  double runtime_sec = 0.0;
  double points_per_image = 0.0;
  double score = 0.0;
};

inline std::vector<SeedStrategyProfile> default_seed_profiles() {
  return {
      {"00_balanced", "balanced", 100, 0.95, 2.0, 20.0},
      {"01_wide_baseline", "wide_baseline", 110, 0.80, 3.0, 30.0},
      {"02_support_first", "support_first", 80, 0.95, 1.5, 10.0},
      {"03_conservative", "conservative", 120, 0.75, 2.5, 15.0},
  };
}

inline double compute_seed_eval_score(int registered_images, int triangulated_points,
                                      double runtime_sec) {
  const double reg_term = 20.0 * static_cast<double>(registered_images);
  const double tri_term = 4.0 * std::log1p(std::max(0, triangulated_points));
  const double time_penalty = 0.2 * std::max(0.0, runtime_sec);
  return reg_term + tri_term - time_penalty;
}

} // namespace tools
} // namespace insight
