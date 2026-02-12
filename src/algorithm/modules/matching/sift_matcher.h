#pragma once

#include "match_types.h"
#include <memory>
#include <string>
#include <Eigen/Core>

// Forward declaration
class SiftMatchGPU;

namespace insight {
namespace algorithm {
namespace matching {

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
     * @param max_features Maximum number of features per image
     */
    explicit SiftMatcher(int max_features = 10000);
    
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
    MatchResult match(
        const FeatureData& features1,
        const FeatureData& features2,
        const MatchOptions& options = MatchOptions()
    );
    
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
    MatchResult matchGuided(
        const FeatureData& features1,
        const FeatureData& features2,
        const Eigen::Matrix3f* F = nullptr,
        const Eigen::Matrix3f* H = nullptr,
        const MatchOptions& options = MatchOptions()
    );
    
    /**
     * Verify OpenGL context
     * @return true if GPU is properly initialized
     */
    bool verifyContext() const;
    
    /**
     * Get maximum number of features
     */
    int getMaxFeatures() const { return max_features_; }

private:
    int max_features_;
    std::unique_ptr<SiftMatchGPU> matcher_;
    
    // Convert match buffer to MatchResult
    MatchResult convertMatchResult(
        const std::vector<uint32_t>& match_buffer,
        int num_matches,
        const FeatureData& features1,
        const FeatureData& features2
    );
    
    // Compute descriptor distance (L2 norm, supports both uint8 and float32)
    float computeDescriptorDistance(
        const FeatureData& features1,
        const FeatureData& features2,
        size_t idx1,
        size_t idx2
    ) const;
};

}  // namespace matching
}  // namespace algorithm
}  // namespace insight
