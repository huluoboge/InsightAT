#pragma once

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <opencv2/core.hpp>
#include <glog/logging.h>
#include "SiftGPU/SiftGPU.h"

namespace insight {
namespace modules {

/**
 * Descriptor normalization type
 * 
 * L2: Standard L2 normalization (default in original SIFT)
 * L1_ROOT: L1 normalize + element-wise square root (RootSIFT)
 *          Better matching performance as shown in:
 *          "Three things everyone should know to improve object retrieval"
 *          Arandjelovic & Zisserman, CVPR 2012
 */
enum class DescriptorNormalization {
    L2,       // L2 normalization: d = d / ||d||_2
    L1_ROOT   // L1-Root normalization: d = sqrt(d / ||d||_1)
};

/**
 * SIFT GPU Parameters (pure extraction only)
 */
struct SiftGPUParams {
    int nOctiveFrom = 0;        // Starting octave 
    int nOctives = -1;           // Number of octaves (-1 = auto)
    int nLevel = 3;              // Levels per octave
    double dPeak = 0.02;         // Peak threshold (divided by nLevel)
    int nMaxFeatures = 8000;     // Maximum features to extract
    bool adaptDarkness = true;   // Adapt to dark images
    // Feature truncation method:
    //   0 = -tc  (keep highest levels: large-scale stable features, delete after extraction)
    //   1 = -tc2 (keep highest levels: large-scale stable features, faster - stop during extraction)
    //   2 = -tc3 (keep lowest levels: small-scale dense features)
    int truncateMethod = 0;
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
    
    // Reconfigure SIFT parameters (for dual-output mode)
    // WARNING: SiftGPU uses global state, so reconfiguration affects the instance
    bool reconfigure(const SiftGPUParams& new_params);
    
    // Extract features from image (returns float descriptors only)
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

/**
 * Helper Functions - Exposed for CPU Multi-threading
 * These functions can be called independently from the extraction pipeline
 */

/**
 * Apply feature distribution to keypoints and descriptors
 * This function can be called from CPU threads without GPU dependency
 * 
 * @param keypoints Input/output keypoints
 * @param descriptors Input/output float descriptors (128-dim per keypoint)
 * @param image_width Image width
 * @param image_height Image height
 * @param grid_size Grid cell size (default: 32 pixels)
 * @param max_per_cell Max features per cell (default: 2)
 * @param keep_orientation Keep multi-orientation features (default: true)
 */
void ApplyFeatureDistribution(
    std::vector<SiftGPU::SiftKeypoint>& keypoints,
    std::vector<float>& descriptors,
    int image_width,
    int image_height,
    int grid_size = 32,
    int max_per_cell = 2,
    bool keep_orientation = true);

/**
 * Apply feature distribution to keypoints and uint8 descriptors
 */
void ApplyFeatureDistribution(
    std::vector<SiftGPU::SiftKeypoint>& keypoints,
    std::vector<unsigned char>& descriptors,
    int image_width,
    int image_height,
    int grid_size = 32,
    int max_per_cell = 2,
    bool keep_orientation = true);

/**
 * L2 normalize descriptors (exposed for external use)
 */
void L2NormalizeDescriptors(std::vector<float>& descriptors, int dim = 128);

/**
 * L1-Root normalize descriptors (RootSIFT, exposed for external use)
 */
void L1RootNormalizeDescriptors(std::vector<float>& descriptors, int dim = 128);

/**
 * Convert float descriptors to uint8 (exposed for external use)
 */
std::vector<unsigned char> ConvertDescriptorsToUChar(
    const std::vector<float>& descriptors_float, int dim = 128);

} // namespace modules
} // namespace insight
