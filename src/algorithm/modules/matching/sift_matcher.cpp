/**
 * @file  sift_matcher.cpp
 * @brief SiftMatcher 实现；第三方 SiftMatchGPU 调用（SetLanguage / CreateContextGL /
 * SetDescriptors / GetSiftMatch / GetGuidedSiftMatch 等）保持原样。
 */

#include "sift_matcher.h"
#include "match_postprocess.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <cstring>

#include <SiftGPU/SiftGPU.h>
#include <glog/logging.h>
#ifndef INSIGHTAT_ENABLE_SIFTGPU
#define INSIGHTAT_ENABLE_SIFTGPU 0
#endif
#if defined(HAVE_CUDA) && __has_include(<popsift/features.h>)
#define INSIGHTAT_HAS_POPSIFT_MATCH 1
#include <popsift/features.h>
#else
#define INSIGHTAT_HAS_POPSIFT_MATCH 0
#endif

namespace insight {
namespace algorithm {
namespace matching {

SiftMatcher::SiftMatcher(int max_features, bool use_cuda)
    : SiftMatcher(SiftMatcherParams{max_features, use_cuda, true, false}) {}

SiftMatcher::SiftMatcher(const SiftMatcherParams& params)
    : max_features_(params.max_features), params_(params) {
  if (params_.use_sift_gpu == params_.use_pop_sift) {
    LOG(ERROR) << "Exactly one matching backend must be enabled: use_sift_gpu xor use_pop_sift";
    return;
  }

  if (params_.use_pop_sift) {
#if INSIGHTAT_HAS_POPSIFT_MATCH
    backend_ = Backend::kPopSift;
    LOG(INFO) << "SiftMatcher initialized with PopSift backend (max_features=" << max_features_
              << ")";
    return;
#else
    LOG(ERROR) << "PopSift matcher requested but not available in this build";
    return;
#endif
  }
  backend_ = Backend::kSiftGPU;

#if INSIGHTAT_ENABLE_SIFTGPU
  matcher_ = std::make_unique<SiftMatchGPU>(max_features_);

  if (params_.use_cuda) {
    // CUDA backend: no GL context needed. SetLanguage(CUDA) then VerifyContextGL() only;
    // SiftMatchGPU creates SiftMatchCU and sets _GoodOpenGL in InitSiftMatch (no CreateContextGL).
    matcher_->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA);
    if (matcher_->VerifyContextGL() == 0) {
      LOG(ERROR) << "SiftMatchGPU CUDA backend failed (VerifyContextGL)";
      matcher_.reset();
    } else {
      LOG(INFO) << "SiftMatchGPU initialized with max_features=" << max_features_
                << " (CUDA backend)";
    }
    return;
  }

  // GLSL backend: create GL/EGL context then verify
  int support = matcher_->CreateContextGL();
  if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
    LOG(ERROR) << "SiftGPU not fully supported. Support level: " << support;
    matcher_.reset();
    return;
  }
  if (matcher_->VerifyContextGL() == 0) {
    LOG(ERROR) << "Failed to create/verify context for SiftMatchGPU";
    matcher_.reset();
  } else {
    LOG(INFO) << "SiftMatchGPU initialized with max_features=" << max_features_
              << " (GLSL backend)";
  }
#else
  LOG(ERROR) << "SiftGPU matcher requested but disabled at compile time "
             << "(INSIGHTAT_ENABLE_SIFTGPU=OFF)";
#endif
}

SiftMatcher::~SiftMatcher() {
  // Unique_ptr handles cleanup
}

bool SiftMatcher::verify_context() const {
  if (backend_ == Backend::kPopSift) {
#if INSIGHTAT_HAS_POPSIFT_MATCH
    return true;
#else
    return false;
#endif
  }
#if INSIGHTAT_ENABLE_SIFTGPU
  return matcher_ && (matcher_->CreateContextGL() != 0) && (matcher_->VerifyContextGL() != 0);
#else
  return false;
#endif
}

MatchResult SiftMatcher::match(const FeatureData& features1, const FeatureData& features2,
                               const MatchOptions& options) {
  if (backend_ == Backend::kPopSift) {
    return match_popsift(features1, features2, options);
  }

  if (!matcher_) {
    LOG(ERROR) << "SiftMatchGPU not initialized";
    return MatchResult();
  }

  if (features1.num_features == 0 || features2.num_features == 0) {
    LOG(WARNING) << "Empty feature set: " << features1.num_features << " vs "
                 << features2.num_features;
    return MatchResult();
  }

  auto upload_descriptors = [&](int slot, const FeatureData& f, const std::vector<int>& idx) {
    int cnt = static_cast<int>(idx.size());
    if (f.descriptor_type == DescriptorType::kFloat32) {
      if (idx.size() == f.num_features) {
        // No subsampling - upload directly
        matcher_->SetDescriptors(slot, cnt, reinterpret_cast<const float*>(f.getDescriptorData()));
      } else {
        std::vector<float> buf(cnt * 128);
        for (int i = 0; i < cnt; ++i)
          std::copy_n(f.descriptors_float.data() + idx[i] * 128, 128, buf.data() + i * 128);
        matcher_->SetDescriptors(slot, cnt, buf.data());
      }
    } else {
      if (idx.size() == f.num_features) {
        matcher_->SetDescriptors(slot, cnt,
                                 reinterpret_cast<const unsigned char*>(f.getDescriptorData()));
      } else {
        std::vector<unsigned char> buf(cnt * 128);
        for (int i = 0; i < cnt; ++i)
          std::copy_n(f.descriptors_uint8.data() + idx[i] * 128, 128, buf.data() + i * 128);
        matcher_->SetDescriptors(slot, cnt, buf.data());
      }
    }
  };

  int cap = options.max_features_per_image;
  int grow = options.spatial_grid_rows > 0 ? options.spatial_grid_rows : 4;
  int gcol = options.spatial_grid_cols > 0 ? options.spatial_grid_cols : 4;
  auto idx1 = select_top_n_spatial_indices(features1, cap, grow, gcol);
  auto idx2 = select_top_n_spatial_indices(features2, cap, grow, gcol);
  int n1 = static_cast<int>(idx1.size());
  int n2 = static_cast<int>(idx2.size());

  VLOG(1) << "Uploading descriptors (spatial " << grow << "x" << gcol << " grid): " << n1 << "/"
          << features1.num_features << " and " << n2 << "/" << features2.num_features
          << " (type=" << (features1.descriptor_type == DescriptorType::kFloat32 ? "f32" : "u8")
          << ")";

  upload_descriptors(0, features1, idx1);
  upload_descriptors(1, features2, idx2);

  // Allocate match buffer
  int max_match = options.max_matches > 0 ? options.max_matches : std::min(n1, n2);
  std::vector<uint32_t> match_buffer_flat(max_match * 2);
  uint32_t (*match_buffer)[2] = reinterpret_cast<uint32_t (*)[2]>(match_buffer_flat.data());

  // Execute matching
  VLOG(1) << "GetSiftMatch: max=" << max_match << ", dist_max=" << options.distance_max
          << ", ratio=" << options.ratio_test << ", mutual=" << options.mutual_best_match;

  int num_matches = matcher_->GetSiftMatch(max_match, match_buffer, options.distance_max,
                                           options.ratio_test, options.mutual_best_match ? 1 : 0);

  VLOG(1) << "GetSiftMatch returned: " << num_matches;

  if (num_matches < 0) {
    LOG(ERROR) << "SiftMatchGPU::GetSiftMatch failed";
    return MatchResult();
  }

  VLOG(1) << "Matched " << num_matches << " features (" << n1 << " vs " << n2 << ")";

  // Convert to MatchResult.
  // match_buffer holds indices into the compact (subsampled) arrays;
  // remap back to original feature indices via idx1/idx2.
  MatchResult result;
  result.reserve(num_matches);
  for (int i = 0; i < num_matches; ++i) {
    int orig1 = idx1[match_buffer[i][0]];
    int orig2 = idx2[match_buffer[i][1]];
    append_match(&result, features1, features2,
                 static_cast<uint16_t>(orig1), static_cast<uint16_t>(orig2),
                 options.compute_distances);
  }
  deduplicate_match_indices(&result);
  result.num_matches = result.indices.size();
  return result;
}

MatchResult SiftMatcher::match_popsift(const FeatureData& features1, const FeatureData& features2,
                                       const MatchOptions& options) {
#if INSIGHTAT_HAS_POPSIFT_MATCH
  if (features1.num_features == 0 || features2.num_features == 0) {
    return MatchResult();
  }

  auto pack_to_float = [](const FeatureData& f, const std::vector<int>& idx) {
    std::vector<float> buf(static_cast<size_t>(idx.size()) * 128, 0.0f);
    if (f.descriptor_type == DescriptorType::kFloat32) {
      for (size_t i = 0; i < idx.size(); ++i) {
        std::memcpy(buf.data() + i * 128, f.descriptors_float.data() + idx[i] * 128, 128 * sizeof(float));
      }
    } else {
      for (size_t i = 0; i < idx.size(); ++i) {
        const uint8_t* src = f.descriptors_uint8.data() + idx[i] * 128;
        float* dst = buf.data() + i * 128;
        for (int k = 0; k < 128; ++k) dst[k] = static_cast<float>(src[k]);
      }
    }
    return buf;
  };

  int cap = options.max_features_per_image;
  int grow = options.spatial_grid_rows > 0 ? options.spatial_grid_rows : 4;
  int gcol = options.spatial_grid_cols > 0 ? options.spatial_grid_cols : 4;
  auto idx1 = select_top_n_spatial_indices(features1, cap, grow, gcol);
  auto idx2 = select_top_n_spatial_indices(features2, cap, grow, gcol);

  auto desc1 = pack_to_float(features1, idx1);
  auto desc2 = pack_to_float(features2, idx2);
  auto rows = popsift::match_descriptors_bruteforce(
      desc1.data(), static_cast<int>(idx1.size()), desc2.data(), static_cast<int>(idx2.size()),
      options.ratio_test, true);
  if (rows.size() != idx1.size()) {
    LOG(ERROR) << "PopSift brute-force returned unexpected row count";
    return MatchResult();
  }

  std::vector<int> reverse_best;
  if (options.mutual_best_match) {
    auto rows_rev = popsift::match_descriptors_bruteforce(
        desc2.data(), static_cast<int>(idx2.size()), desc1.data(), static_cast<int>(idx1.size()),
        1.0f, false);
    reverse_best.assign(idx2.size(), -1);
    for (size_t i = 0; i < rows_rev.size(); ++i) reverse_best[i] = rows_rev[i].best_train_idx;
  }

  MatchResult result;
  for (size_t qi = 0; qi < rows.size(); ++qi) {
    const auto& r = rows[qi];
    if (!r.accepted || r.best_train_idx < 0 || r.best_train_idx >= static_cast<int>(idx2.size())) continue;
    if (options.mutual_best_match && reverse_best[static_cast<size_t>(r.best_train_idx)] != static_cast<int>(qi)) {
      continue;
    }
    int o1 = idx1[qi];
    int o2 = idx2[static_cast<size_t>(r.best_train_idx)];
    append_match(&result, features1, features2,
                 static_cast<uint16_t>(o1), static_cast<uint16_t>(o2), false);
    if (options.compute_distances) {
      result.distances.push_back(std::sqrt(std::max(0.0f, r.best_dist_sq)));
    }
    if (options.max_matches > 0 && static_cast<int>(result.indices.size()) >= options.max_matches) break;
  }
  deduplicate_match_indices(&result);
  result.num_matches = result.indices.size();
  return result;
#else
  (void)features1;
  (void)features2;
  (void)options;
  LOG(ERROR) << "PopSift matcher backend not compiled";
  return MatchResult();
#endif
}

MatchResult SiftMatcher::match_guided(const FeatureData& features1, const FeatureData& features2,
                                      const Eigen::Matrix3f* F, const Eigen::Matrix3f* H,
                                      const MatchOptions& options) {
#if !INSIGHTAT_ENABLE_SIFTGPU
  (void)features1;
  (void)features2;
  (void)F;
  (void)H;
  (void)options;
  LOG(ERROR) << "Guided matching requires SiftGPU, but SiftGPU is disabled at compile time";
  return MatchResult();
#else
  if (!matcher_) {
    LOG(ERROR) << "SiftMatchGPU not initialized";
    return MatchResult();
  }

  if (features1.num_features == 0 || features2.num_features == 0) {
    return MatchResult();
  }

  // Upload descriptors (auto-detect format)
  if (features1.descriptor_type == DescriptorType::kFloat32) {
    matcher_->SetDescriptors(0, features1.num_features,
                             reinterpret_cast<const float*>(features1.getDescriptorData()));
  } else {
    matcher_->SetDescriptors(0, features1.num_features,
                             reinterpret_cast<const unsigned char*>(features1.getDescriptorData()));
  }

  if (features2.descriptor_type == DescriptorType::kFloat32) {
    matcher_->SetDescriptors(1, features2.num_features,
                             reinterpret_cast<const float*>(features2.getDescriptorData()));
  } else {
    matcher_->SetDescriptors(1, features2.num_features,
                             reinterpret_cast<const unsigned char*>(features2.getDescriptorData()));
  }

  // Convert Eigen matrices to float[3][3]
  float H_array[3][3] = {{0}};
  float F_array[3][3] = {{0}};

  if (H) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        H_array[i][j] = (*H)(i, j);
      }
    }
  }

  if (F) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        F_array[i][j] = (*F)(i, j);
      }
    }
  }

  // Allocate match buffer
  int max_match = options.max_matches > 0
                      ? options.max_matches
                      : std::min(features1.num_features, features2.num_features);
  std::vector<uint32_t> match_buffer_flat(max_match * 2);
  uint32_t (*match_buffer)[2] = reinterpret_cast<uint32_t (*)[2]>(match_buffer_flat.data());

  // Execute guided matching
  int num_matches = matcher_->GetGuidedSiftMatch(
      max_match, match_buffer, H ? reinterpret_cast<float*>(H_array) : nullptr,
      F ? reinterpret_cast<float*>(F_array) : nullptr, options.distance_max, options.ratio_test,
      options.homography_threshold, options.fundamental_threshold,
      options.mutual_best_match ? 1 : 0);

  if (num_matches < 0) {
    LOG(ERROR) << "SiftMatchGPU::GetGuidedSiftMatch failed";
    return MatchResult();
  }

  VLOG(1) << "Guided matching: " << num_matches << " features";

  return convert_match_result(match_buffer_flat, num_matches, features1, features2,
                              options.compute_distances);
#endif
}

MatchResult SiftMatcher::convert_match_result(const std::vector<uint32_t>& match_buffer,
                                              int num_matches, const FeatureData& features1,
                                              const FeatureData& features2,
                                              bool compute_distances) {
  MatchResult result;
  result.reserve(num_matches);

  for (int i = 0; i < num_matches; ++i) {
    uint32_t idx1 = match_buffer[2 * i];
    uint32_t idx2 = match_buffer[2 * i + 1];

    // Validate indices
    if (idx1 >= features1.num_features || idx2 >= features2.num_features) {
      LOG(WARNING) << "Invalid match index: " << idx1 << " vs " << idx2;
      continue;
    }

    append_match(&result, features1, features2,
                 static_cast<uint16_t>(idx1), static_cast<uint16_t>(idx2),
                 compute_distances);
  }
  deduplicate_match_indices(&result);
  result.num_matches = result.indices.size();

  return result;
}

float SiftMatcher::compute_descriptor_distance(const FeatureData& features1,
                                               const FeatureData& features2, size_t idx1,
                                               size_t idx2) const {
  return descriptor_distance_l2(features1, features2, idx1, idx2);
}

} // namespace matching
} // namespace algorithm
} // namespace insight
