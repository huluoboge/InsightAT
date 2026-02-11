#pragma once

#include <memory>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <glog/logging.h>
#include "SiftGPU/SiftGPU.h"

namespace insight {
namespace modules {

/**
 * SIFT GPU Parameters
 */
struct SiftGPUParams {
    int nOctiveFrom = -1;        // Starting octave (-1 = auto)
    int nOctives = -1;           // Number of octaves (-1 = auto)
    int nLevel = 3;              // Levels per octave
    double dPeak = 0.04;         // Peak threshold (divided by nLevel)
    int nMaxFeatures = 8000;     // Maximum features to extract
    bool adaptDarkness = true;   // Adapt to dark images
};

/**
 * SIFT GPU Feature Extractor
 * 
 * Encapsulates SiftGPU initialization and feature extraction.
 * Thread-safe: each instance owns its own GPU context.
 */
class SiftGPUExtractor {
public:
    using SiftGPUPtr = std::shared_ptr<SiftGPU>;
    
    SiftGPUExtractor(const SiftGPUParams& params);
    ~SiftGPUExtractor() = default;
    
    // Initialize GPU context (must be called before extract)
    bool initialize();
    
    // Extract features from image
    // Returns number of features extracted
    int extract(const cv::Mat& image,
                std::vector<SiftGPU::SiftKeypoint>& keypoints,
                std::vector<float>& descriptors);
    
    // Check if GPU is available
    bool isInitialized() const { return initialized_; }
    
private:
    SiftGPUParams params_;
    SiftGPUPtr sift_gpu_;
    bool initialized_ = false;
    
    // Create SiftGPU instance with parameters
    SiftGPUPtr createSiftGPU(const SiftGPUParams& param);
};

} // namespace modules
} // namespace insight
