/**
 * @file  cpu_cascade_hash_test.cpp
 * @brief Unit tests for CPU cascade hashing matcher and grouping utility.
 *
 * Usage
 * ─────
 *   ./test_cpu_cascade_hash
 */

#include "cpu_cascade_hash.h"

#include <glog/logging.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <random>
#include <set>
#include <vector>

namespace insight {
namespace algorithm {
namespace cpu_cascade_hash {
namespace {

constexpr int kDescriptorDim = 128;

matching::FeatureData make_random_feature_data(int num_features, uint32_t seed) {
  matching::FeatureData data(static_cast<size_t>(num_features), matching::DescriptorType::kUInt8);
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dist_u8(0, 255);
  std::uniform_real_distribution<float> dist_xy(0.0f, 4000.0f);
  std::uniform_real_distribution<float> dist_scale(1.0f, 8.0f);
  std::uniform_real_distribution<float> dist_angle(-3.1415926f, 3.1415926f);

  for (int i = 0; i < num_features; ++i) {
    data.keypoints[static_cast<size_t>(i)] =
        Eigen::Vector4f(dist_xy(rng), dist_xy(rng), dist_scale(rng), dist_angle(rng));
    for (int d = 0; d < kDescriptorDim; ++d) {
      data.descriptors_uint8[static_cast<size_t>(i) * kDescriptorDim + d] =
          static_cast<uint8_t>(dist_u8(rng));
    }
  }
  data.num_features = static_cast<size_t>(num_features);
  return data;
}

matching::FeatureData make_perturbed_matching_data(const matching::FeatureData& src, int extra_features,
                                                   int noise_bound, uint32_t seed) {
  const int src_count = static_cast<int>(src.num_features);
  matching::FeatureData dst(static_cast<size_t>(src_count + extra_features),
                            matching::DescriptorType::kUInt8);
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dist_noise(-noise_bound, noise_bound);
  std::uniform_int_distribution<int> dist_u8(0, 255);
  std::uniform_real_distribution<float> dist_xy(0.0f, 4000.0f);
  std::uniform_real_distribution<float> dist_scale(1.0f, 8.0f);
  std::uniform_real_distribution<float> dist_angle(-3.1415926f, 3.1415926f);

  // Keep first src_count descriptors as one-to-one noisy counterparts.
  for (int i = 0; i < src_count; ++i) {
    dst.keypoints[static_cast<size_t>(i)] = src.keypoints[static_cast<size_t>(i)];
    for (int d = 0; d < kDescriptorDim; ++d) {
      const int base = static_cast<int>(src.descriptors_uint8[static_cast<size_t>(i) * kDescriptorDim + d]);
      const int noise = dist_noise(rng);
      const int value = std::clamp(base + noise, 0, 255);
      dst.descriptors_uint8[static_cast<size_t>(i) * kDescriptorDim + d] = static_cast<uint8_t>(value);
    }
  }

  // Append distractor descriptors.
  for (int i = src_count; i < src_count + extra_features; ++i) {
    dst.keypoints[static_cast<size_t>(i)] =
        Eigen::Vector4f(dist_xy(rng), dist_xy(rng), dist_scale(rng), dist_angle(rng));
    for (int d = 0; d < kDescriptorDim; ++d) {
      dst.descriptors_uint8[static_cast<size_t>(i) * kDescriptorDim + d] =
          static_cast<uint8_t>(dist_u8(rng));
    }
  }
  dst.num_features = static_cast<size_t>(src_count + extra_features);
  return dst;
}

int test_match_cascade_hash_with_synthetic_descriptors() {
  std::cout << "[Test 1] Synthetic SIFT descriptors with perturbation\n";
  const int n_source = 200;
  const int n_distractor = 120;
  const matching::FeatureData source = make_random_feature_data(n_source, 11);
  const matching::FeatureData target =
      make_perturbed_matching_data(source, n_distractor, /*noise_bound=*/2, 29);

  CascadeHashOptions options;
  options.ratio_test = 0.85f;
  options.mutual_best = true;
  options.random_seed = 2026;

  std::vector<const matching::FeatureData*> samples = {&source, &target};
  const CascadeHashSampleModel model = build_sample_model(samples, options);
  const ImageFeatures source_features = compute_image_features(source, model);
  const ImageFeatures target_features = compute_image_features(target, model);
  const matching::MatchResult result =
      match_cascade_hash(source, source_features, target, target_features, model);
  std::set<std::pair<int, int>> match_pairs;
  for (const auto& p : result.indices) {
    match_pairs.insert({static_cast<int>(p.first), static_cast<int>(p.second)});
  }

  int correct = 0;
  for (int i = 0; i < n_source; ++i) {
    if (match_pairs.find({i, i}) != match_pairs.end()) {
      ++correct;
    }
  }

  const double recall = static_cast<double>(correct) / static_cast<double>(n_source);
  std::cout << "  matched=" << result.num_matches << ", correct=" << correct
            << ", recall=" << recall << "\n";

  if (recall < 0.75) {
    std::cerr << "  FAIL: recall too low (< 0.75)\n";
    return 1;
  }
  std::cout << "  PASS\n";
  return 0;
}

int test_group_images_for_pairs() {
  std::cout << "[Test 2] Group image pairing layout\n";
  std::set<uint32_t> images;
  for (uint32_t i = 0; i < 10; ++i) {
    images.insert(i);
  }

  const GroupedPairs grouped = group_images_for_pairs(images, 4);
  if (grouped.group_images.size() != 3) {
    std::cerr << "  FAIL: unexpected group count\n";
    return 1;
  }

  // group sizes should be 4, 4, 2.
  const std::array<size_t, 3> expected = {4, 4, 2};
  for (uint32_t i = 0; i < expected.size(); ++i) {
    if (grouped.group_images.at(i).size() != expected.at(i)) {
      std::cerr << "  FAIL: unexpected size for group " << i << "\n";
      return 1;
    }
  }

  // in-group pairs: C(4,2)+C(4,2)+C(2,2)=6+6+1=13
  size_t in_group_pair_total = 0;
  for (const auto& [_, pairs] : grouped.pairs_in_group) {
    in_group_pair_total += pairs.size();
  }
  if (in_group_pair_total != 13) {
    std::cerr << "  FAIL: unexpected in-group pair count\n";
    return 1;
  }

  // between-group pairs: 4*4 + 4*2 + 4*2 = 32
  size_t between_pair_total = 0;
  for (const auto& [_, pairs] : grouped.pairs_between_groups) {
    between_pair_total += pairs.size();
  }
  if (between_pair_total != 32) {
    std::cerr << "  FAIL: unexpected between-group pair count\n";
    return 1;
  }

  std::cout << "  PASS\n";
  return 0;
}

}  // namespace

}  // namespace cpu_cascade_hash
}  // namespace algorithm
}  // namespace insight

int main() {
  google::InitGoogleLogging("test_cpu_cascade_hash");
  FLAGS_logtostderr = 1;
  FLAGS_minloglevel = 2;

  int failures = 0;
  failures += insight::algorithm::cpu_cascade_hash::test_match_cascade_hash_with_synthetic_descriptors();
  failures += insight::algorithm::cpu_cascade_hash::test_group_images_for_pairs();

  if (failures == 0) {
    std::cout << "\nAll tests PASSED.\n";
    return 0;
  }
  std::cerr << "\n" << failures << " test(s) FAILED.\n";
  return 1;
}
