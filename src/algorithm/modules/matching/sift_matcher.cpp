#include "sift_matcher.h"
#include <SiftGPU/SiftGPU.h>
#include <glog/logging.h>
#include <cmath>

namespace insight {
namespace algorithm {
namespace matching {

SiftMatcher::SiftMatcher(int max_features) 
    : max_features_(max_features) {
    
    // Create SiftMatchGPU instance
    matcher_ = std::make_unique<SiftMatchGPU>(max_features);
    
    // Create OpenGL context (CRITICAL: must be called before VerifyContextGL)
    int support = matcher_->CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        LOG(ERROR) << "SiftGPU not fully supported. Support level: " << support;
        matcher_.reset();
        return;
    }
    
    // Verify OpenGL context
    if (matcher_->VerifyContextGL() == 0) {
        LOG(ERROR) << "Failed to create OpenGL context for SiftMatchGPU";
        matcher_.reset();
    } else {
        LOG(INFO) << "SiftMatchGPU initialized with max_features=" << max_features;
    }
}

SiftMatcher::~SiftMatcher() {
    // Unique_ptr handles cleanup
}

bool SiftMatcher::verifyContext() const {
    return matcher_ && (matcher_->VerifyContextGL() != 0);
}

MatchResult SiftMatcher::match(
    const FeatureData& features1,
    const FeatureData& features2,
    const MatchOptions& options
) {
    if (!matcher_) {
        LOG(ERROR) << "SiftMatchGPU not initialized";
        return MatchResult();
    }
    
    if (features1.num_features == 0 || features2.num_features == 0) {
        LOG(WARNING) << "Empty feature set: " << features1.num_features 
                    << " vs " << features2.num_features;
        return MatchResult();
    }
    
    // Upload descriptors to GPU (auto-detect format)
    VLOG(1) << "Uploading descriptors: " << features1.num_features << " and " << features2.num_features;
    VLOG(1) << "Descriptor types: " << (features1.descriptor_type == DescriptorType::kFloat32 ? "float32" : "uint8")
            << " and " << (features2.descriptor_type == DescriptorType::kFloat32 ? "float32" : "uint8");
    
    // SiftGPU supports both uint8 and float descriptors
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
    
    // Allocate match buffer
    int max_match = options.max_matches > 0 ? options.max_matches : 
                    std::min(features1.num_features, features2.num_features);
    std::vector<uint32_t> match_buffer_flat(max_match * 2);
    uint32_t (*match_buffer)[2] = reinterpret_cast<uint32_t (*)[2]>(match_buffer_flat.data());
    
    // Execute matching
    LOG(INFO) << "Calling GetSiftMatch with: max=" << max_match 
              << ", dist_max=" << options.distance_max 
              << ", ratio=" << options.ratio_test 
              << ", mutual=" << options.mutual_best_match;
    
    int num_matches = matcher_->GetSiftMatch(
        max_match,
        match_buffer,
        options.distance_max,
        options.ratio_test,
        options.mutual_best_match ? 1 : 0
    );
    
    LOG(INFO) << "GetSiftMatch returned: " << num_matches;
    
    if (num_matches < 0) {
        LOG(ERROR) << "SiftMatchGPU::GetSiftMatch failed";
        return MatchResult();
    }
    
    VLOG(1) << "Matched " << num_matches << " features ("
            << features1.num_features << " vs " << features2.num_features << ")";
    
    // Convert to MatchResult
    return convertMatchResult(match_buffer_flat, num_matches, features1, features2);
}

MatchResult SiftMatcher::matchGuided(
    const FeatureData& features1,
    const FeatureData& features2,
    const Eigen::Matrix3f* F,
    const Eigen::Matrix3f* H,
    const MatchOptions& options
) {
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
    int max_match = options.max_matches > 0 ? options.max_matches : 
                    std::min(features1.num_features, features2.num_features);
    std::vector<uint32_t> match_buffer_flat(max_match * 2);
    uint32_t (*match_buffer)[2] = reinterpret_cast<uint32_t (*)[2]>(match_buffer_flat.data());
    
    // Execute guided matching
    int num_matches = matcher_->GetGuidedSiftMatch(
        max_match,
        match_buffer,
        H ? reinterpret_cast<float*>(H_array) : nullptr,
        F ? reinterpret_cast<float*>(F_array) : nullptr,
        options.distance_max,
        options.ratio_test,
        options.homography_threshold,
        options.fundamental_threshold,
        options.mutual_best_match ? 1 : 0
    );
    
    if (num_matches < 0) {
        LOG(ERROR) << "SiftMatchGPU::GetGuidedSiftMatch failed";
        return MatchResult();
    }
    
    VLOG(1) << "Guided matching: " << num_matches << " features";
    
    return convertMatchResult(match_buffer_flat, num_matches, features1, features2);
}

MatchResult SiftMatcher::convertMatchResult(
    const std::vector<uint32_t>& match_buffer,
    int num_matches,
    const FeatureData& features1,
    const FeatureData& features2
) {
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
        result.indices.push_back({static_cast<uint16_t>(idx1), 
                                  static_cast<uint16_t>(idx2)});
        
        // Extract pixel coordinates
        const auto& kp1 = features1.keypoints[idx1];
        const auto& kp2 = features2.keypoints[idx2];
        
        Eigen::Vector4f coords;
        coords << kp1(0), kp1(1), kp2(0), kp2(1);  // [x1, y1, x2, y2]
        result.coords_pixel.push_back(coords);
        
        // Compute descriptor distance (support both uint8 and float32)
        float distance = computeDescriptorDistance(features1, features2, idx1, idx2);
        result.distances.push_back(distance);
    }
    
    result.num_matches = result.indices.size();
    
    return result;
}

float SiftMatcher::computeDescriptorDistance(
    const FeatureData& features1,
    const FeatureData& features2,
    size_t idx1,
    size_t idx2
) const {
    // Both features must have same descriptor type for valid comparison
    if (features1.descriptor_type != features2.descriptor_type) {
        LOG(WARNING) << "Descriptor type mismatch in distance computation";
        return 1e6f;  // Large distance
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

}  // namespace matching
}  // namespace algorithm
}  // namespace insight
