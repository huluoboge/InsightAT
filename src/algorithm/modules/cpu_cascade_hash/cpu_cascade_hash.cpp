/**
 * @file  cpu_cascade_hash.cpp
 * @brief CPU cascade hashing matcher implementation.
 */

#include "algorithm/modules/cpu_cascade_hash/cpu_cascade_hash.h"
#include "algorithm/modules/matching/match_postprocess.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <unordered_map>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#if defined(__AVX2__)
#include <immintrin.h>
#elif defined(__SSE2__)
#include <emmintrin.h>
#endif

namespace insight {
namespace algorithm {
namespace cpu_cascade_hash {
namespace {

constexpr int kDescriptorDim = 128;
constexpr int kCompressedWords = 2;

#if defined(__SSE2__)
inline int horizontal_sum_epi32(__m128i v) {
  alignas(16) int values[4];
  _mm_storeu_si128(reinterpret_cast<__m128i*>(values), v);
  return values[0] + values[1] + values[2] + values[3];
}
#endif

#if defined(__AVX2__)
inline int horizontal_sum_epi32(__m256i v) {
  alignas(32) int values[8];
  _mm256_storeu_si256(reinterpret_cast<__m256i*>(values), v);
  return values[0] + values[1] + values[2] + values[3] +
         values[4] + values[5] + values[6] + values[7];
}
#endif

inline std::pair<int, int> normalize_pair(uint32_t a, uint32_t b) {
  if (a < b) {
    return {static_cast<int>(a), static_cast<int>(b)};
  }
  return {static_cast<int>(b), static_cast<int>(a)};
}

inline float descriptor_value(const matching::FeatureData& f, int desc_idx, int dim) {
  if (f.descriptor_type == matching::DescriptorType::kFloat32) {
    return f.descriptors_float[static_cast<size_t>(desc_idx) * kDescriptorDim + dim];
  }
  return static_cast<float>(f.descriptors_uint8[static_cast<size_t>(desc_idx) * kDescriptorDim + dim]);
}

inline int popcount_u64(uint64_t value) {
#if defined(_MSC_VER)
  return static_cast<int>(__popcnt64(value));
#else
  return __builtin_popcountll(value);
#endif
}

inline int hamming_distance(const std::array<uint64_t, kCompressedWords>& lhs,
                            const std::array<uint64_t, kCompressedWords>& rhs) {
  return popcount_u64(lhs[0] ^ rhs[0]) + popcount_u64(lhs[1] ^ rhs[1]);
}

inline size_t bucket_index(int group, int bucket, int bucket_count) {
  return static_cast<size_t>(group) * static_cast<size_t>(bucket_count) + static_cast<size_t>(bucket);
}

inline size_t bucket_offset_index(int group, int bucket, int bucket_count) {
  return static_cast<size_t>(group) * static_cast<size_t>(bucket_count + 1) + static_cast<size_t>(bucket);
}

inline uint16_t get_bucket_id(const ImageFeatures& features, size_t point_idx, int group, int bucket_groups) {
  return features.bucket_ids_flat[point_idx * static_cast<size_t>(bucket_groups) + static_cast<size_t>(group)];
}

std::vector<float> compute_mean_descriptor(const std::vector<const matching::FeatureData*>& samples) {
  std::vector<float> mean(kDescriptorDim, 0.0f);
  size_t total = 0;
  for (const auto* sample : samples) {
    if (sample == nullptr) {
      continue;
    }
    total += sample->num_features;
  }
  if (total == 0) {
    return mean;
  }

  for (const auto* sample : samples) {
    if (sample == nullptr) {
      continue;
    }
    for (size_t i = 0; i < sample->num_features; ++i) {
      for (int d = 0; d < kDescriptorDim; ++d) {
        mean[d] += descriptor_value(*sample, static_cast<int>(i), d);
      }
    }
  }
  for (float& v : mean) {
    v /= static_cast<float>(total);
  }
  return mean;
}

double legacy_normal_random(std::mt19937* rng) {
  if (rng != nullptr) {
    std::uniform_int_distribution<int> dist(1, 1000);
    const double u1 = static_cast<double>(dist(*rng)) / 1000.0;
    const double u2 = static_cast<double>(dist(*rng)) / 1000.0;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::acos(-1.0) * u2);
  }
  const double u1 = (std::rand() % 1000 + 1) / 1000.0;
  const double u2 = (std::rand() % 1000 + 1) / 1000.0;
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::acos(-1.0) * u2);
}

std::vector<std::vector<float>> build_projection_matrix(int rows, uint32_t seed, bool use_legacy_rng) {
  if (use_legacy_rng) {
    std::mt19937 rng(seed);
    std::vector<std::vector<float>> matrix(rows, std::vector<float>(kDescriptorDim, 0.0f));
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < kDescriptorDim; ++c) {
        matrix[r][c] = static_cast<float>(legacy_normal_random(&rng) * 1000.0);
      }
    }
    return matrix;
  }
  std::mt19937 rng(seed);
  std::normal_distribution<float> normal(0.0f, 1.0f);
  std::vector<std::vector<float>> matrix(rows, std::vector<float>(kDescriptorDim, 0.0f));
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < kDescriptorDim; ++c) {
      matrix[r][c] = normal(rng);
    }
  }
  return matrix;
}

std::vector<std::vector<std::vector<float>>> build_secondary_projection_matrix(
    int groups, int bucket_bits, uint32_t seed, bool use_legacy_rng) {
  std::vector<std::vector<std::vector<float>>> matrix;
  matrix.reserve(groups);
  for (int g = 0; g < groups; ++g) {
    matrix.push_back(build_projection_matrix(bucket_bits, seed + static_cast<uint32_t>(131 * g),
                                             use_legacy_rng));
  }
  return matrix;
}

std::vector<std::vector<int>> build_bucket_bit_indices(int groups, int bucket_bits, int hash_bits,
                                                       uint32_t seed) {
  std::mt19937 rng(seed);
  std::vector<std::vector<int>> bit_indices(groups, std::vector<int>(bucket_bits, 0));
  for (int g = 0; g < groups; ++g) {
    std::vector<int> indices(hash_bits);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    for (int b = 0; b < bucket_bits; ++b) {
      bit_indices[g][b] = indices[b];
    }
  }
  return bit_indices;
}

ImageFeatures build_hash_index(const matching::FeatureData& f, const std::vector<float>& mean_descriptor,
                               const std::vector<std::vector<float>>& primary_projection,
                               const std::vector<std::vector<std::vector<float>>>& secondary_projection,
                               const std::vector<std::vector<int>>& bucket_bit_indices,
                               const CascadeHashOptions& options) {
  const int bucket_count = 1 << options.bucket_bits;
  ImageFeatures index;
  const size_t num_points = f.num_features;
  index.compressed_hashes.resize(num_points);
  index.bucket_ids_flat.resize(num_points * static_cast<size_t>(options.bucket_groups), 0);
  index.bucket_counts.assign(static_cast<size_t>(options.bucket_groups) * static_cast<size_t>(bucket_count), 0);
  index.bucket_offsets.assign(static_cast<size_t>(options.bucket_groups) * static_cast<size_t>(bucket_count + 1), 0);
  index.bucket_indices.resize(num_points * static_cast<size_t>(options.bucket_groups), 0);

  std::vector<float> centered(kDescriptorDim, 0.0f);
  std::vector<int16_t> centered_i16(kDescriptorDim, 0);
  for (size_t i = 0; i < num_points; ++i) {
    std::array<uint8_t, 128> hash_bits{};
    for (int d = 0; d < kDescriptorDim; ++d) {
      const float centered_value = descriptor_value(f, static_cast<int>(i), d) - mean_descriptor[d];
      centered[d] = centered_value;
      centered_i16[d] = static_cast<int16_t>(centered_value);
    }

    for (int h = 0; h < options.hash_bits; ++h) {
      if (options.use_legacy_numeric) {
        int sum = 0;
        for (int d = 0; d < kDescriptorDim; ++d) {
          if (centered_i16[d] != 0) {
            sum += static_cast<int>(centered_i16[d]) * static_cast<int>(primary_projection[h][d]);
          }
        }
        hash_bits[h] = sum > 0 ? 1 : 0;
      } else {
        float dot = 0.0f;
        for (int d = 0; d < kDescriptorDim; ++d) {
          dot += primary_projection[h][d] * centered[d];
        }
        hash_bits[h] = dot > 0.0f ? 1 : 0;
      }
    }

    for (int w = 0; w < kCompressedWords; ++w) {
      uint64_t packed = 0;
      for (int b = 0; b < 64; ++b) {
        packed = (packed << 1) | hash_bits[w * 64 + b];
      }
      index.compressed_hashes[i][w] = packed;
    }

    for (int g = 0; g < options.bucket_groups; ++g) {
      uint16_t bucket_id = 0;
      if (options.use_bucket_secondary_hash) {
        for (int b = 0; b < options.bucket_bits; ++b) {
          if (options.use_legacy_numeric) {
            int sum = 0;
            for (int d = 0; d < kDescriptorDim; ++d) {
              if (centered_i16[d] != 0) {
                sum += static_cast<int>(centered_i16[d]) *
                       static_cast<int>(secondary_projection[g][b][d]);
              }
            }
            bucket_id = static_cast<uint16_t>((bucket_id << 1) | (sum > 0 ? 1 : 0));
          } else {
            float dot = 0.0f;
            for (int d = 0; d < kDescriptorDim; ++d) {
              dot += secondary_projection[g][b][d] * centered[d];
            }
            bucket_id = static_cast<uint16_t>((bucket_id << 1) | (dot > 0.0f ? 1 : 0));
          }
        }
      } else {
        for (int b = 0; b < options.bucket_bits; ++b) {
          bucket_id = static_cast<uint16_t>((bucket_id << 1) | hash_bits[bucket_bit_indices[g][b]]);
        }
      }
      index.bucket_ids_flat[i * static_cast<size_t>(options.bucket_groups) + static_cast<size_t>(g)] = bucket_id;
      ++index.bucket_counts[bucket_index(g, static_cast<int>(bucket_id), bucket_count)];
    }
  }

  // Prefix-sum per group to build contiguous bucket spans.
  for (int g = 0; g < options.bucket_groups; ++g) {
    int running = 0;
    for (int bucket = 0; bucket < bucket_count; ++bucket) {
      const size_t off_idx = bucket_offset_index(g, bucket, bucket_count);
      index.bucket_offsets[off_idx] = running;
      running += index.bucket_counts[bucket_index(g, bucket, bucket_count)];
    }
    index.bucket_offsets[bucket_offset_index(g, bucket_count, bucket_count)] = running;
  }

  // Current write cursors copied from offsets.
  std::vector<int> write_cursor = index.bucket_offsets;
  for (size_t i = 0; i < num_points; ++i) {
    for (int g = 0; g < options.bucket_groups; ++g) {
      const int bucket = static_cast<int>(get_bucket_id(index, i, g, options.bucket_groups));
      const size_t cursor_idx = bucket_offset_index(g, bucket, bucket_count);
      const int pos = write_cursor[cursor_idx]++;
      const size_t base = static_cast<size_t>(g) * num_points;
      index.bucket_indices[base + static_cast<size_t>(pos)] = static_cast<int>(i);
    }
  }

  return index;
}

float descriptor_distance_squared(const matching::FeatureData& f1, int idx1,
                                  const matching::FeatureData& f2, int idx2) {
  const size_t base1 = static_cast<size_t>(idx1) * kDescriptorDim;
  const size_t base2 = static_cast<size_t>(idx2) * kDescriptorDim;
  float sum = 0.0f;
  if (f1.descriptor_type == matching::DescriptorType::kUInt8 &&
      f2.descriptor_type == matching::DescriptorType::kUInt8) {
    const uint8_t* ptr1 = f1.descriptors_uint8.data() + base1;
    const uint8_t* ptr2 = f2.descriptors_uint8.data() + base2;
#if defined(__AVX2__)
    __m256i acc0 = _mm256_setzero_si256();
    __m256i acc1 = _mm256_setzero_si256();
    for (int d = 0; d < kDescriptorDim; d += 32) {
      const __m128i a_u8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr1 + d));
      const __m128i b_u8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr2 + d));
      const __m256i a_u16 = _mm256_cvtepu8_epi16(a_u8);
      const __m256i b_u16 = _mm256_cvtepu8_epi16(b_u8);
      const __m256i diff = _mm256_sub_epi16(a_u16, b_u16);
      acc0 = _mm256_add_epi32(acc0, _mm256_madd_epi16(diff, diff));

      const __m128i a_u8_hi = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr1 + d + 16));
      const __m128i b_u8_hi = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr2 + d + 16));
      const __m256i a_u16_hi = _mm256_cvtepu8_epi16(a_u8_hi);
      const __m256i b_u16_hi = _mm256_cvtepu8_epi16(b_u8_hi);
      const __m256i diff_hi = _mm256_sub_epi16(a_u16_hi, b_u16_hi);
      acc1 = _mm256_add_epi32(acc1, _mm256_madd_epi16(diff_hi, diff_hi));
    }
    return static_cast<float>(horizontal_sum_epi32(_mm256_add_epi32(acc0, acc1)));
#elif defined(__SSE2__)
    __m128i acc = _mm_setzero_si128();
    const __m128i zero = _mm_setzero_si128();
    for (int d = 0; d < kDescriptorDim; d += 16) {
      const __m128i a_u8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr1 + d));
      const __m128i b_u8 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(ptr2 + d));

      const __m128i a_lo = _mm_unpacklo_epi8(a_u8, zero);
      const __m128i b_lo = _mm_unpacklo_epi8(b_u8, zero);
      const __m128i diff_lo = _mm_sub_epi16(a_lo, b_lo);
      acc = _mm_add_epi32(acc, _mm_madd_epi16(diff_lo, diff_lo));

      const __m128i a_hi = _mm_unpackhi_epi8(a_u8, zero);
      const __m128i b_hi = _mm_unpackhi_epi8(b_u8, zero);
      const __m128i diff_hi = _mm_sub_epi16(a_hi, b_hi);
      acc = _mm_add_epi32(acc, _mm_madd_epi16(diff_hi, diff_hi));
    }
    return static_cast<float>(horizontal_sum_epi32(acc));
#else
    int d = 0;
    for (; d + 7 < kDescriptorDim; d += 8) {
      const float diff0 = static_cast<float>(ptr1[d + 0]) - static_cast<float>(ptr2[d + 0]);
      const float diff1 = static_cast<float>(ptr1[d + 1]) - static_cast<float>(ptr2[d + 1]);
      const float diff2 = static_cast<float>(ptr1[d + 2]) - static_cast<float>(ptr2[d + 2]);
      const float diff3 = static_cast<float>(ptr1[d + 3]) - static_cast<float>(ptr2[d + 3]);
      const float diff4 = static_cast<float>(ptr1[d + 4]) - static_cast<float>(ptr2[d + 4]);
      const float diff5 = static_cast<float>(ptr1[d + 5]) - static_cast<float>(ptr2[d + 5]);
      const float diff6 = static_cast<float>(ptr1[d + 6]) - static_cast<float>(ptr2[d + 6]);
      const float diff7 = static_cast<float>(ptr1[d + 7]) - static_cast<float>(ptr2[d + 7]);
      sum += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
      sum += diff4 * diff4 + diff5 * diff5 + diff6 * diff6 + diff7 * diff7;
    }
    for (; d < kDescriptorDim; ++d) {
      const float diff = static_cast<float>(ptr1[d]) - static_cast<float>(ptr2[d]);
      sum += diff * diff;
    }
    return sum;
#endif
  }
  if (f1.descriptor_type == matching::DescriptorType::kFloat32 &&
      f2.descriptor_type == matching::DescriptorType::kFloat32) {
    const float* ptr1 = f1.descriptors_float.data() + base1;
    const float* ptr2 = f2.descriptors_float.data() + base2;
#if defined(__SSE__)
    __m128 accum = _mm_setzero_ps();
    int d = 0;
    for (; d + 3 < kDescriptorDim; d += 4) {
      const __m128 v1 = _mm_loadu_ps(ptr1 + d);
      const __m128 v2 = _mm_loadu_ps(ptr2 + d);
      const __m128 diff = _mm_sub_ps(v1, v2);
      accum = _mm_add_ps(accum, _mm_mul_ps(diff, diff));
    }
    alignas(16) float lane_sum[4];
    _mm_storeu_ps(lane_sum, accum);
    sum = lane_sum[0] + lane_sum[1] + lane_sum[2] + lane_sum[3];
    for (; d < kDescriptorDim; ++d) {
      const float diff = ptr1[d] - ptr2[d];
      sum += diff * diff;
    }
#else
    for (int d = 0; d < kDescriptorDim; ++d) {
      const float diff = ptr1[d] - ptr2[d];
      sum += diff * diff;
    }
#endif
    return sum;
  }
  for (int d = 0; d < kDescriptorDim; ++d) {
    const float lhs = (f1.descriptor_type == matching::DescriptorType::kFloat32)
                          ? f1.descriptors_float[base1 + static_cast<size_t>(d)]
                          : static_cast<float>(f1.descriptors_uint8[base1 + static_cast<size_t>(d)]);
    const float rhs = (f2.descriptor_type == matching::DescriptorType::kFloat32)
                          ? f2.descriptors_float[base2 + static_cast<size_t>(d)]
                          : static_cast<float>(f2.descriptors_uint8[base2 + static_cast<size_t>(d)]);
    const float diff = lhs - rhs;
    sum += diff * diff;
  }
  return sum;
}

std::vector<std::pair<int, int>> match_one_direction(const matching::FeatureData& query,
                                                     const matching::FeatureData& train,
                                                     const ImageFeatures& query_index,
                                                     const ImageFeatures& train_index,
                                                     const CascadeHashOptions& options) {
  std::vector<std::pair<int, int>> matches;
  matches.reserve(query.num_features);
  const float ratio_sq = options.ratio_test * options.ratio_test;
  const size_t max_count = std::max(query.num_features, train.num_features);
  std::vector<int> candidate_indices;
  candidate_indices.reserve(max_count);
  std::vector<unsigned char> candidate_hamming;
  candidate_hamming.reserve(max_count);
  std::array<std::vector<int>, kDescriptorDim + 1> hamming_buckets;
  std::vector<int> selected_stamp(train.num_features, 0);
  int stamp = 1;
  std::vector<int> top_candidates;
  top_candidates.reserve(static_cast<size_t>(options.candidate_top_max));
  const int bucket_count = 1 << options.bucket_bits;

  for (size_t q = 0; q < query.num_features; ++q) {
    candidate_indices.clear();
    top_candidates.clear();
    if (stamp == std::numeric_limits<int>::max()) {
      std::fill(selected_stamp.begin(), selected_stamp.end(), 0);
      stamp = 1;
    }
    const int current_stamp = stamp++;

    for (auto& bucket : hamming_buckets) {
      bucket.clear();
    }

    // Gather candidates from all bucket groups.
    for (int g = 0; g < options.bucket_groups; ++g) {
      const uint16_t bucket_id = get_bucket_id(query_index, q, g, options.bucket_groups);
      const size_t off_idx = bucket_offset_index(g, static_cast<int>(bucket_id), bucket_count);
      const int begin = train_index.bucket_offsets[off_idx];
      const int end = train_index.bucket_offsets[off_idx + 1];
      const size_t train_group_base = static_cast<size_t>(g) * train.num_features;
      for (int p = begin; p < end; ++p) {
        const int idx = train_index.bucket_indices[train_group_base + static_cast<size_t>(p)];
        candidate_indices.push_back(idx);
      }
    }
    if (candidate_indices.size() < 2) {
      continue;
    }

    candidate_hamming.resize(candidate_indices.size());
    // Build a fixed hamming-distance bucket list [0..128] so we avoid O(n log n) sort.
    for (size_t i = 0; i < candidate_indices.size(); ++i) {
      const int idx = candidate_indices[i];
      const int dist = hamming_distance(query_index.compressed_hashes[q],
                                        train_index.compressed_hashes[static_cast<size_t>(idx)]);
      candidate_hamming[i] = static_cast<unsigned char>(dist);
      hamming_buckets[static_cast<size_t>(dist)].push_back(idx);
    }

    int selected_count = 0;
    for (int dist = 0; dist <= kDescriptorDim && selected_count < options.candidate_top_min; ++dist) {
      auto& bucket = hamming_buckets[static_cast<size_t>(dist)];
      if (bucket.empty()) {
        continue;
      }
      for (int idx : bucket) {
        if (selected_stamp[static_cast<size_t>(idx)] == current_stamp) {
          continue;
        }
        selected_stamp[static_cast<size_t>(idx)] = current_stamp;
        top_candidates.push_back(idx);
        ++selected_count;
        if (selected_count >= options.candidate_top_max) {
          break;
        }
      }
    }
    if (top_candidates.size() < 2) {
      continue;
    }

    float best = std::numeric_limits<float>::max();
    float second = std::numeric_limits<float>::max();
    int best_idx = -1;
    for (int idx : top_candidates) {
      const float d2 = descriptor_distance_squared(query, static_cast<int>(q), train, idx);
      if (d2 < best) {
        second = best;
        best = d2;
        best_idx = idx;
      } else if (d2 < second) {
        second = d2;
      }
    }

    if (best_idx >= 0 && second < std::numeric_limits<float>::max() && best < ratio_sq * second) {
      matches.emplace_back(static_cast<int>(q), best_idx);
    }
  }
  if (static_cast<int>(matches.size()) < options.min_match_list_len) {
    matches.clear();
  }
  return matches;
}

}  // namespace

CascadeHashSampleModel build_sample_model_from_mean_descriptor(
    const std::vector<float>& mean_descriptor, const CascadeHashOptions& options) {
  CascadeHashSampleModel model;
  model.options = options;
  model.mean_descriptor = mean_descriptor;
  if (model.mean_descriptor.size() != static_cast<size_t>(kDescriptorDim)) {
    model.mean_descriptor.assign(kDescriptorDim, 0.0f);
  }
  if (options.use_legacy_numeric) {
    for (float& value : model.mean_descriptor) {
      value = static_cast<float>(static_cast<int16_t>(value));
    }
  }
  model.primary_projection_matrix =
      build_projection_matrix(options.hash_bits, options.random_seed, options.use_legacy_rng);
  if (options.use_bucket_secondary_hash) {
    model.secondary_projection_matrix =
        build_secondary_projection_matrix(options.bucket_groups, options.bucket_bits,
                                          options.random_seed + 10007, options.use_legacy_rng);
  }
  model.bucket_bit_indices = build_bucket_bit_indices(options.bucket_groups, options.bucket_bits,
                                                      options.hash_bits, options.random_seed + 17);
  return model;
}

CascadeHashSampleModel build_sample_model(
    const std::vector<const matching::FeatureData*>& sample_features, const CascadeHashOptions& options) {
  return build_sample_model_from_mean_descriptor(compute_mean_descriptor(sample_features), options);
}

ImageFeatures compute_image_features(const matching::FeatureData& features,
                                     const CascadeHashSampleModel& model) {
  return build_hash_index(features, model.mean_descriptor, model.primary_projection_matrix,
                          model.secondary_projection_matrix,
                          model.bucket_bit_indices, model.options);
}

matching::MatchResult match_cascade_hash(const matching::FeatureData& query_features,
                                         const ImageFeatures& query_image_features,
                                         const matching::FeatureData& train_features,
                                         const ImageFeatures& train_image_features,
                                         const CascadeHashSampleModel& model) {
  matching::MatchResult result;
  if (query_features.num_features == 0 || train_features.num_features == 0) {
    return result;
  }

  const CascadeHashOptions& options = model.options;
  const auto forward = match_one_direction(query_features, train_features, query_image_features,
                                           train_image_features, options);
  if (!options.mutual_best) {
    result.reserve(forward.size());
    for (const auto& m : forward) {
      append_match(&result, query_features, train_features,
                   static_cast<uint16_t>(m.first), static_cast<uint16_t>(m.second),
                   options.compute_distances);
    }
    deduplicate_match_indices(&result);
    result.num_matches = result.indices.size();
    return result;
  }

  const auto backward = match_one_direction(train_features, query_features, train_image_features,
                                            query_image_features, options);
  std::vector<std::pair<int, int>> forward_sorted = forward;
  std::vector<std::pair<int, int>> backward_flipped;
  backward_flipped.reserve(backward.size());
  for (const auto& m : backward) {
    backward_flipped.emplace_back(m.second, m.first);
  }
  std::sort(forward_sorted.begin(), forward_sorted.end());
  std::sort(backward_flipped.begin(), backward_flipped.end());

  result.reserve(std::min(forward_sorted.size(), backward_flipped.size()));
  size_t i = 0;
  size_t j = 0;
  while (i < forward_sorted.size() && j < backward_flipped.size()) {
    if (forward_sorted[i] < backward_flipped[j]) {
      ++i;
      continue;
    }
    if (backward_flipped[j] < forward_sorted[i]) {
      ++j;
      continue;
    }
    const auto& m = forward_sorted[i];
    append_match(&result, query_features, train_features,
                 static_cast<uint16_t>(m.first), static_cast<uint16_t>(m.second),
                 options.compute_distances);
    ++i;
    ++j;
  }
  deduplicate_match_indices(&result);
  result.num_matches = result.indices.size();
  return result;
}

matching::MatchResult match_cascade_hash(const matching::FeatureData& features1,
                                         const matching::FeatureData& features2,
                                         const CascadeHashOptions& options) {
  std::vector<const matching::FeatureData*> samples = {&features1, &features2};
  const CascadeHashSampleModel model = build_sample_model(samples, options);
  const ImageFeatures image_features1 = compute_image_features(features1, model);
  const ImageFeatures image_features2 = compute_image_features(features2, model);
  return match_cascade_hash(features1, image_features1, features2, image_features2, model);
}

GroupedPairs group_images_for_pairs(const std::set<uint32_t>& images, int group_size) {
  GroupedPairs grouped;
  if (group_size <= 0 || images.empty()) {
    return grouped;
  }

  uint32_t group_id = 0;
  for (auto it = images.begin(); it != images.end();) {
    auto& bucket = grouped.group_images[group_id++];
    for (int i = 0; i < group_size && it != images.end(); ++i, ++it) {
      bucket.insert(*it);
    }
  }

  for (const auto& [id, ids] : grouped.group_images) {
    ImagePairSet in_group_pairs;
    for (auto i = ids.begin(); i != ids.end(); ++i) {
      for (auto j = std::next(i); j != ids.end(); ++j) {
        in_group_pairs.insert({*i, *j});
      }
    }
    grouped.pairs_in_group[id] = std::move(in_group_pairs);
  }

  for (auto i = grouped.group_images.begin(); i != grouped.group_images.end(); ++i) {
    auto j = i;
    ++j;
    for (; j != grouped.group_images.end(); ++j) {
      ImagePairSet between;
      for (uint32_t left : i->second) {
        for (uint32_t right : j->second) {
          if (left == right) {
            continue;
          }
          const auto normalized = normalize_pair(left, right);
          between.insert({static_cast<uint32_t>(normalized.first),
                          static_cast<uint32_t>(normalized.second)});
        }
      }
      grouped.pairs_between_groups[{i->first, j->first}] = std::move(between);
    }
  }
  return grouped;
}

}  // namespace cpu_cascade_hash
}  // namespace algorithm
}  // namespace insight
