/**
 * @file  sift_matcher.cpp
 * @brief SiftMatcher 实现；第三方 SiftMatchGPU 调用（SetLanguage / CreateContextGL /
 * SetDescriptors / GetSiftMatch / GetGuidedSiftMatch 等）保持原样。
 */

#include "sift_matcher.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <cstring>

#include <SiftGPU/SiftGPU.h>
#include <glog/logging.h>
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
  return matcher_ && (matcher_->CreateContextGL() != 0) && (matcher_->VerifyContextGL() != 0);
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

  // Spatially stratified feature selection.
  // Divides the keypoint bounding box into (grid_rows × grid_cols) cells, sorts each
  // cell by scale descending, then picks features round-robin across cells until max_n
  // is reached.  This prevents large-scale features (e.g. tree canopy) from monopolising
  // the GPU upload budget and ensures building corners / other small-scale structures are
  // represented.
  auto top_n_spatial = [](const FeatureData& f, int max_n,
                          int grid_rows, int grid_cols) -> std::vector<int> {
    int n = static_cast<int>(f.num_features);
    if (n == 0) return {};

    // If no cap is requested, return all in original order.
    if (max_n <= 0 || n <= max_n) {
      std::vector<int> idx(n);
      std::iota(idx.begin(), idx.end(), 0);
      return idx;
    }

    // Compute bounding box of keypoints.
    float xmin = f.keypoints[0][0], xmax = xmin;
    float ymin = f.keypoints[0][1], ymax = ymin;
    for (int i = 1; i < n; ++i) {
      xmin = std::min(xmin, f.keypoints[i][0]);
      xmax = std::max(xmax, f.keypoints[i][0]);
      ymin = std::min(ymin, f.keypoints[i][1]);
      ymax = std::max(ymax, f.keypoints[i][1]);
    }
    float cell_w = (xmax - xmin + 1.0f) / grid_cols;
    float cell_h = (ymax - ymin + 1.0f) / grid_rows;

    // Assign each feature to its cell.
    int n_cells = grid_rows * grid_cols;
    std::vector<std::vector<int>> cells(n_cells);
    for (int i = 0; i < n; ++i) {
      int cx = static_cast<int>((f.keypoints[i][0] - xmin) / cell_w);
      int cy = static_cast<int>((f.keypoints[i][1] - ymin) / cell_h);
      cx = std::min(cx, grid_cols - 1);
      cy = std::min(cy, grid_rows - 1);
      cells[cy * grid_cols + cx].push_back(i);
    }

    // Sort each cell by scale descending.
    for (auto& cell : cells) {
      std::sort(cell.begin(), cell.end(),
                [&f](int a, int b) { return f.keypoints[a][2] > f.keypoints[b][2]; });
    }

    // Round-robin pick across cells until max_n is reached.
    std::vector<int> result;
    result.reserve(max_n);
    std::vector<int> cell_pos(n_cells, 0);
    int picked = 0;
    while (picked < max_n) {
      bool any = false;
      for (int c = 0; c < n_cells && picked < max_n; ++c) {
        if (cell_pos[c] < static_cast<int>(cells[c].size())) {
          result.push_back(cells[c][cell_pos[c]++]);
          ++picked;
          any = true;
        }
      }
      if (!any) break; // all cells exhausted
    }
    return result;
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
  int grow = options.spatial_grid_rows > 0 ? options.spatial_grid_rows : 4;
  int gcol = options.spatial_grid_cols > 0 ? options.spatial_grid_cols : 4;
  auto idx1 = top_n_spatial(features1, cap, grow, gcol);
  auto idx2 = top_n_spatial(features2, cap, grow, gcol);
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
    result.indices.emplace_back(static_cast<uint16_t>(orig1), static_cast<uint16_t>(orig2));
    const auto& kp1 = features1.keypoints[orig1];
    const auto& kp2 = features2.keypoints[orig2];
    result.coords_pixel.emplace_back(kp1[0], kp1[1], kp2[0], kp2[1]);
  }
  result.num_matches = num_matches;
  return result;
}

MatchResult SiftMatcher::match_popsift(const FeatureData& features1, const FeatureData& features2,
                                       const MatchOptions& options) {
#if INSIGHTAT_HAS_POPSIFT_MATCH
  if (features1.num_features == 0 || features2.num_features == 0) {
    return MatchResult();
  }

  auto top_n_spatial = [](const FeatureData& f, int max_n,
                          int grid_rows, int grid_cols) -> std::vector<int> {
    int n = static_cast<int>(f.num_features);
    if (n == 0) return {};
    if (max_n <= 0 || n <= max_n) {
      std::vector<int> idx(n);
      std::iota(idx.begin(), idx.end(), 0);
      return idx;
    }
    float xmin = f.keypoints[0][0], xmax = xmin;
    float ymin = f.keypoints[0][1], ymax = ymin;
    for (int i = 1; i < n; ++i) {
      xmin = std::min(xmin, f.keypoints[i][0]);
      xmax = std::max(xmax, f.keypoints[i][0]);
      ymin = std::min(ymin, f.keypoints[i][1]);
      ymax = std::max(ymax, f.keypoints[i][1]);
    }
    float cell_w = (xmax - xmin + 1.0f) / grid_cols;
    float cell_h = (ymax - ymin + 1.0f) / grid_rows;
    int n_cells = grid_rows * grid_cols;
    std::vector<std::vector<int>> cells(n_cells);
    for (int i = 0; i < n; ++i) {
      int cx = std::min(static_cast<int>((f.keypoints[i][0] - xmin) / cell_w), grid_cols - 1);
      int cy = std::min(static_cast<int>((f.keypoints[i][1] - ymin) / cell_h), grid_rows - 1);
      cells[cy * grid_cols + cx].push_back(i);
    }
    for (auto& cell : cells) {
      std::sort(cell.begin(), cell.end(),
                [&f](int a, int b) { return f.keypoints[a][2] > f.keypoints[b][2]; });
    }
    std::vector<int> out;
    out.reserve(max_n);
    std::vector<int> pos(n_cells, 0);
    while (static_cast<int>(out.size()) < max_n) {
      bool any = false;
      for (int c = 0; c < n_cells && static_cast<int>(out.size()) < max_n; ++c) {
        if (pos[c] < static_cast<int>(cells[c].size())) {
          out.push_back(cells[c][pos[c]++]);
          any = true;
        }
      }
      if (!any) break;
    }
    return out;
  };

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
  auto idx1 = top_n_spatial(features1, cap, grow, gcol);
  auto idx2 = top_n_spatial(features2, cap, grow, gcol);

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
    result.indices.emplace_back(static_cast<uint16_t>(o1), static_cast<uint16_t>(o2));
    const auto& kp1 = features1.keypoints[o1];
    const auto& kp2 = features2.keypoints[o2];
    result.coords_pixel.emplace_back(kp1[0], kp1[1], kp2[0], kp2[1]);
    result.distances.push_back(std::sqrt(std::max(0.0f, r.best_dist_sq)));
    if (options.max_matches > 0 && static_cast<int>(result.indices.size()) >= options.max_matches) break;
  }
  result.num_matches = static_cast<int>(result.indices.size());
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
