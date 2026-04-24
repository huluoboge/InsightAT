/**
 * @file  sift_matcher.h
 * @brief SiftMatcher：封装 SiftMatchGPU，提供 GPU 特征匹配与 guided matching 接口。
 *
 * 第三方调用（勿改）：matcher_->SetLanguage / CreateContextGL / SetDescriptors /
 * GetSiftMatch / GetGuidedSiftMatch 等 SiftMatchGPU API。
 */

#pragma once

#include "match_types.h"

#include <memory>
#include <string>

#include <Eigen/Core>

class SiftMatchGPU;

namespace insight {
namespace algorithm {
namespace matching {

struct SiftMatcherParams {
  int max_features = 10000;
  bool use_cuda = true;
  bool use_sift_gpu = false;
  bool use_pop_sift = true;
};

/**
 * SiftMatcher - Wrapper around SiftMatchGPU
 *
 * Provides high-level interface for feature matching using GPU acceleration.
 * Manages OpenGL context and GPU resources.
 *
 * Usage:
 *   SiftMatcher matcher(10000);
 *   auto result = matcher.match(features1, features2, options);
 */
class SiftMatcher {
public:
  /**
   * Constructor
   * @param params matcher backend and capacity options
   */
  explicit SiftMatcher(const SiftMatcherParams& params = SiftMatcherParams());

  // Backward-compatible constructor
  explicit SiftMatcher(int max_features, bool use_cuda);

  /**
   * Destructor - releases GPU resources
   */
  ~SiftMatcher();

  // Disable copy (GPU resources not copyable)
  SiftMatcher(const SiftMatcher&) = delete;
  SiftMatcher& operator=(const SiftMatcher&) = delete;

  /**
   * Match two sets of features
   *
   * @param features1 Features from first image
   * @param features2 Features from second image
   * @param options Matching parameters
   * @return Match result containing indices, coordinates, and distances
   */
  MatchResult match(const FeatureData& features1, const FeatureData& features2,
                    const MatchOptions& options = MatchOptions());

  /**
   * Guided matching using pre-computed geometry
   *
   * @param features1 Features from first image
   * @param features2 Features from second image
   * @param F Fundamental matrix (3x3, can be nullptr)
   * @param H Homography matrix (3x3, can be nullptr)
   * @param options Matching parameters
   * @return Filtered match result based on geometric constraints
   */
  MatchResult match_guided(const FeatureData& features1, const FeatureData& features2,
                          const Eigen::Matrix3f* F = nullptr, const Eigen::Matrix3f* H = nullptr,
                          const MatchOptions& options = MatchOptions());

  /** @return true 表示 GPU 上下文已正确初始化。 */
  bool verify_context() const;

  int get_max_features() const { return max_features_; }

private:
  enum class Backend { kSiftGPU, kPopSift };
  int max_features_;
  SiftMatcherParams params_;
  Backend backend_ = Backend::kSiftGPU;
  std::unique_ptr<SiftMatchGPU> matcher_;

  MatchResult convert_match_result(const std::vector<uint32_t>& match_buffer, int num_matches,
                                  const FeatureData& features1, const FeatureData& features2);
  float compute_descriptor_distance(const FeatureData& features1, const FeatureData& features2,
                                   size_t idx1, size_t idx2) const;
  MatchResult match_popsift(const FeatureData& features1, const FeatureData& features2,
                            const MatchOptions& options);
};

}  // namespace matching
}  // namespace algorithm
}  // namespace insight
