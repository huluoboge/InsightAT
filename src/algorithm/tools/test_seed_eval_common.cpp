#include "seed_eval_common.h"

#include <cmath>
#include <iostream>
#include <string>

using insight::tools::SeedStrategyProfile;

namespace {

int fail(const std::string& msg) {
  std::cerr << "FAIL: " << msg << "\n";
  return 1;
}

int test_default_profiles() {
  const auto profiles = insight::tools::default_seed_profiles();
  if (profiles.size() != 4)
    return fail("default profiles must contain 4 strategies");

  const SeedStrategyProfile& balanced = profiles[0];
  if (balanced.name != "balanced")
    return fail("first profile must be balanced");
  if (balanced.init_min_inliers <= 0)
    return fail("balanced min_inliers must be positive");

  const SeedStrategyProfile& conservative = profiles[3];
  if (conservative.init_max_forward_motion >= balanced.init_max_forward_motion)
    return fail("conservative should have stricter forward-motion gate than balanced");

  return 0;
}

int test_scoring_monotonicity() {
  const double s_low = insight::tools::compute_seed_eval_score(4, 500, 20.0);
  const double s_high_reg = insight::tools::compute_seed_eval_score(6, 500, 20.0);
  const double s_high_pts = insight::tools::compute_seed_eval_score(4, 1500, 20.0);
  const double s_slow = insight::tools::compute_seed_eval_score(4, 500, 60.0);

  if (!(s_high_reg > s_low))
    return fail("score must increase with registered images");
  if (!(s_high_pts > s_low))
    return fail("score must increase with triangulated points");
  if (!(s_slow < s_low))
    return fail("score must decrease with runtime penalty");

  return 0;
}

} // namespace

int main() {
  if (int rc = test_default_profiles(); rc != 0)
    return rc;
  if (int rc = test_scoring_monotonicity(); rc != 0)
    return rc;

  std::cout << "PASS: test_seed_eval_common\n";
  return 0;
}
