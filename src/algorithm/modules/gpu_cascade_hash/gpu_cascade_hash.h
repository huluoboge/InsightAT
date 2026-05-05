#pragma once

#include "algorithm/modules/matching/match_types.h"
#include "algorithm/modules/cpu_cascade_hash/cpu_cascade_hash.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace insight {
namespace algorithm {
namespace gpu_cascade_hash {

struct GpuCascadeHashOptions {
  std::vector<float> mean_descriptor;  // length 128
  int cuda_device_id = 0;
};

class GpuCascadeHashBlockMatcher {
public:
  explicit GpuCascadeHashBlockMatcher(const GpuCascadeHashOptions& options);
  ~GpuCascadeHashBlockMatcher();

  GpuCascadeHashBlockMatcher(const GpuCascadeHashBlockMatcher&) = delete;
  GpuCascadeHashBlockMatcher& operator=(const GpuCascadeHashBlockMatcher&) = delete;

  bool add_image(uint32_t image_index, const matching::FeatureData* features);
  bool add_image_with_index(uint32_t image_index, const matching::FeatureData* features,
                            const cpu_cascade_hash::ImageFeatures* hash_index);
  bool finalize();
  matching::MatchResult match_pair(uint32_t image1_index, uint32_t image2_index);
  std::vector<matching::MatchResult> match_pairs(
      const std::vector<std::pair<uint32_t, uint32_t>>& image_pairs);

private:
  struct Impl;
  Impl* impl_;
};

}  // namespace gpu_cascade_hash
}  // namespace algorithm
}  // namespace insight

