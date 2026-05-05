/**
 * @file  cpu_cascade_hash.h
 * @brief CPU implementation of cascade hashing matcher and image-pair grouping.
 *
 * This module provides:
 * 1) CPU cascade hashing matcher over `matching::FeatureData`.
 * 2) Group-based image pairing utility for memory-constrained matching stages.
 */

#pragma once

#include "algorithm/modules/matching/match_types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace insight {
namespace algorithm {
namespace cpu_cascade_hash {

using ImagePair = std::pair<uint32_t, uint32_t>;
using ImagePairSet = std::set<ImagePair>;

struct CascadeHashOptions {
  int hash_bits = 128;
  int bucket_groups = 6;
  int bucket_bits = 8;
  int candidate_top_min = 6;
  int candidate_top_max = 10;
  int min_match_list_len = 16;
  float ratio_test = 0.8f;
  bool mutual_best = true;
  bool compute_distances = false;
  bool use_bucket_secondary_hash = true;   ///< Use secondary projection for bucket IDs (paper design).
  bool use_legacy_rng = true;             ///< Use legacy rand()-based Gaussian (for bit-exact compat only).
  bool use_legacy_numeric = false;        ///< Use int16 mean-centering and int accumulation (legacy compat only).
  uint32_t random_seed = 1337;
};

struct ImageFeatures {
  // SoA layout for better cache locality:
  // - compressed_hashes[i] stores descriptor i hash code (2 x uint64)
  // - bucket_ids_flat[i * bucket_groups + g] stores descriptor i bucket id for group g
  // - bucket_counts[group * bucket_count + bucket] stores bucket length
  // - bucket_offsets[group * (bucket_count + 1) + bucket] stores begin offset in bucket_indices
  // - bucket_indices stores all descriptor indices for all (group,bucket), contiguous by bucket
  std::vector<std::array<uint64_t, 2>> compressed_hashes;
  std::vector<uint16_t> bucket_ids_flat;
  std::vector<int> bucket_counts;
  std::vector<int> bucket_offsets;
  std::vector<int> bucket_indices;
};

struct CascadeHashSampleModel {
  CascadeHashOptions options;
  std::vector<float> mean_descriptor;
  std::vector<std::vector<float>> primary_projection_matrix;
  std::vector<std::vector<std::vector<float>>> secondary_projection_matrix;
  std::vector<std::vector<int>> bucket_bit_indices;
};

struct GroupedPairs {
  std::map<uint32_t, std::set<uint32_t>> group_images;
  std::map<uint32_t, ImagePairSet> pairs_in_group;
  std::map<ImagePair, ImagePairSet> pairs_between_groups;
};

CascadeHashSampleModel build_sample_model(
    const std::vector<const matching::FeatureData*>& sample_features,
    const CascadeHashOptions& options = CascadeHashOptions());

CascadeHashSampleModel build_sample_model_from_mean_descriptor(
    const std::vector<float>& mean_descriptor,
    const CascadeHashOptions& options = CascadeHashOptions());

ImageFeatures compute_image_features(const matching::FeatureData& features,
                                     const CascadeHashSampleModel& model);

matching::MatchResult match_cascade_hash(const matching::FeatureData& query_features,
                                         const ImageFeatures& query_image_features,
                                         const matching::FeatureData& train_features,
                                         const ImageFeatures& train_image_features,
                                         const CascadeHashSampleModel& model);

matching::MatchResult match_cascade_hash(const matching::FeatureData& features1,
                                         const matching::FeatureData& features2,
                                         const CascadeHashOptions& options = CascadeHashOptions());

GroupedPairs group_images_for_pairs(const std::set<uint32_t>& images, int group_size);

}  // namespace cpu_cascade_hash
}  // namespace algorithm
}  // namespace insight
