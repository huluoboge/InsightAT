/**
 * @file  sift_matcher.cpp
 * @brief SiftMatcher 实现；第三方 SiftMatchGPU 调用（SetLanguage / CreateContextGL /
 * SetDescriptors / GetSiftMatch / GetGuidedSiftMatch 等）保持原样。
 */

#include "sift_matcher.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <SiftGPU/SiftGPU.h>
#include <glog/logging.h>

namespace insight {
namespace algorithm {
namespace matching {

SiftMatcher::SiftMatcher(int max_features, bool use_cuda) : max_features_(max_features) {

  matcher_ = std::make_unique<SiftMatchGPU>(max_features);

  if (use_cuda) {
    // CUDA backend: no GL context needed. SetLanguage(CUDA) then VerifyContextGL() only;
    // SiftMatchGPU creates SiftMatchCU and sets _GoodOpenGL in InitSiftMatch (no CreateContextGL).
    matcher_->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA);
    if (matcher_->VerifyContextGL() == 0) {
      LOG(ERROR) << "SiftMatchGPU CUDA backend failed (VerifyContextGL)";
      matcher_.reset();
    } else {
      LOG(INFO) << "SiftMatchGPU initialized with max_features=" << max_features
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
    LOG(INFO) << "SiftMatchGPU initialized with max_features=" << max_features << " (GLSL backend)";
  }
}

SiftMatcher::~SiftMatcher() {
  // Unique_ptr handles cleanup
}

bool SiftMatcher::verify_context() const {
  return matcher_ && (matcher_->CreateContextGL() != 0) && (matcher_->VerifyContextGL() != 0);
}

MatchResult SiftMatcher::match(const FeatureData& features1, const FeatureData& features2,
                               const MatchOptions& options) {
  if (!matcher_) {
    LOG(ERROR) << "SiftMatchGPU not initialized";
    return MatchResult();
  }

  if (features1.num_features == 0 || features2.num_features == 0) {
    LOG(WARNING) << "Empty feature set: " << features1.num_features << " vs "
                 << features2.num_features;
    return MatchResult();
  }

  auto top_n_by_scale = [](const FeatureData& f, int max_n) -> std::vector<int> {
    int n = static_cast<int>(f.num_features);
    std::vector<int> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    if (max_n > 0 && n > max_n) {
      std::partial_sort(idx.begin(), idx.begin() + max_n, idx.end(),
                        [&f](int a, int b) { return f.keypoints[a][2] > f.keypoints[b][2]; });
      idx.resize(max_n);
    }
    return idx;
  };

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
  auto idx1 = top_n_by_scale(features1, cap);
  auto idx2 = top_n_by_scale(features2, cap);
  int n1 = static_cast<int>(idx1.size());
  int n2 = static_cast<int>(idx2.size());

  VLOG(1) << "Uploading descriptors: " << n1 << "/" << features1.num_features << " and " << n2
          << "/" << features2.num_features
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
    result.indices.emplace_back(static_cast<uint16_t>(orig1), static_cast<uint16_t>(orig2));
    const auto& kp1 = features1.keypoints[orig1];
    const auto& kp2 = features2.keypoints[orig2];
    result.coords_pixel.emplace_back(kp1[0], kp1[1], kp2[0], kp2[1]);
  }
  result.num_matches = num_matches;
  return result;
}

MatchResult SiftMatcher::match_guided(const FeatureData& features1, const FeatureData& features2,
                                      const Eigen::Matrix3f* F, const Eigen::Matrix3f* H,
                                      const MatchOptions& options) {
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

  return convert_match_result(match_buffer_flat, num_matches, features1, features2);
}

MatchResult SiftMatcher::convert_match_result(const std::vector<uint32_t>& match_buffer,
                                              int num_matches, const FeatureData& features1,
                                              const FeatureData& features2) {
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

    // Store index pair (convert to uint16_t)
    result.indices.push_back({static_cast<uint16_t>(idx1), static_cast<uint16_t>(idx2)});

    // Extract pixel coordinates
    const auto& kp1 = features1.keypoints[idx1];
    const auto& kp2 = features2.keypoints[idx2];

    Eigen::Vector4f coords;
    coords << kp1(0), kp1(1), kp2(0), kp2(1); // [x1, y1, x2, y2]
    result.coords_pixel.push_back(coords);

    // Compute descriptor distance (support both uint8 and float32)
    float distance = compute_descriptor_distance(features1, features2, idx1, idx2);
    result.distances.push_back(distance);
  }

  result.num_matches = result.indices.size();

  return result;
}

float SiftMatcher::compute_descriptor_distance(const FeatureData& features1,
                                               const FeatureData& features2, size_t idx1,
                                               size_t idx2) const {
  // Both features must have same descriptor type for valid comparison
  if (features1.descriptor_type != features2.descriptor_type) {
    LOG(WARNING) << "Descriptor type mismatch in distance computation";
    return 1e6f; // Large distance
  }

  float sum = 0.0f;

  if (features1.descriptor_type == DescriptorType::kUInt8) {
    const uint8_t* desc1 = &features1.descriptors_uint8[idx1 * 128];
    const uint8_t* desc2 = &features2.descriptors_uint8[idx2 * 128];

    for (int i = 0; i < 128; ++i) {
      float diff = static_cast<float>(desc1[i]) - static_cast<float>(desc2[i]);
      sum += diff * diff;
    }
  } else {
    const float* desc1 = &features1.descriptors_float[idx1 * 128];
    const float* desc2 = &features2.descriptors_float[idx2 * 128];

    for (int i = 0; i < 128; ++i) {
      float diff = desc1[i] - desc2[i];
      sum += diff * diff;
    }
  }

  return std::sqrt(sum);
}

} // namespace matching
} // namespace algorithm
} // namespace insight
