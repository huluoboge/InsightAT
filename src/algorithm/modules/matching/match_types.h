#pragma once

#include <vector>
#include <cstdint>
#include <Eigen/Core>

namespace insight {
namespace algorithm {
namespace matching {

/**
 * Descriptor type enumeration
 */
enum class DescriptorType {
    kUInt8,    // 8-bit unsigned integer (0-255)
    kFloat32   // 32-bit float (RootSIFT normalized)
};

/**
 * Feature data structure for matching
 * Supports both uint8 and float32 descriptor formats
 */
struct FeatureData {
    std::vector<Eigen::Vector4f> keypoints;  // [x, y, scale, orientation]
    
    // Descriptor storage (only one is used based on descriptor_type)
    std::vector<uint8_t> descriptors_uint8;  // For uint8 format
    std::vector<float> descriptors_float;    // For float32 format
    
    DescriptorType descriptor_type = DescriptorType::kUInt8;
    size_t num_features = 0;
    
    FeatureData() = default;
    
    FeatureData(size_t n, DescriptorType dtype = DescriptorType::kUInt8) 
        : descriptor_type(dtype), num_features(n) {
        keypoints.resize(n);
        if (dtype == DescriptorType::kUInt8) {
            descriptors_uint8.resize(n * 128);
        } else {
            descriptors_float.resize(n * 128);
        }
    }
    
    // Get descriptor data pointer (for GPU upload)
    const void* getDescriptorData() const {
        if (descriptor_type == DescriptorType::kUInt8) {
            return descriptors_uint8.data();
        } else {
            return descriptors_float.data();
        }
    }
    
    void clear() {
        keypoints.clear();
        descriptors_uint8.clear();
        descriptors_float.clear();
        num_features = 0;
        keypoints.shrink_to_fit();
        descriptors_uint8.shrink_to_fit();
        descriptors_float.shrink_to_fit();
    }
};

/**
 * Match result structure
 */
struct MatchResult {
    // Index pairs (feature indices in both images)
    std::vector<std::pair<uint16_t, uint16_t>> indices;
    
    // Pixel coordinates [x1, y1, x2, y2] for each match
    std::vector<Eigen::Vector4f> coords_pixel;
    
    // Descriptor distances (optional, for quality assessment)
    std::vector<float> distances;
    
    size_t num_matches = 0;
    
    void clear() {
        indices.clear();
        coords_pixel.clear();
        distances.clear();
        num_matches = 0;
    }
    
    void reserve(size_t n) {
        indices.reserve(n);
        coords_pixel.reserve(n);
        distances.reserve(n);
    }
};

/**
 * Matching options
 */
struct MatchOptions {
    // Lowe's ratio test threshold
    float ratio_test = 0.8f;
    
    // Maximum descriptor distance
    float distance_max = 0.7f;
    
    // Maximum number of matches (-1 = unlimited)
    int max_matches = -1;
    
    // Require mutual best match (bidirectional consistency)
    bool mutual_best_match = true;
    
    // Guided matching parameters (optional)
    bool use_guided_matching = false;
    float homography_threshold = 32.0f;     // H matrix error threshold
    float fundamental_threshold = 16.0f;     // F matrix Sampson distance threshold
};

}  // namespace matching
}  // namespace algorithm
}  // namespace insight
