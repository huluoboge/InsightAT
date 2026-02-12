#include "sift_gpu_extractor.h"
#include "feature_distribution.h"
#include <glog/logging.h>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <GL/gl.h>

namespace insight {
namespace modules {

// ============================================================================
// Generic descriptor normalization and conversion utilities
// ============================================================================

/**
 * L2 normalize descriptors (in-place)
 * Each descriptor is normalized to unit length: d = d / ||d||_2
 * 
 * Note: SIFT descriptors should be non-negative (gradient histograms).
 * If negative values appear (SiftGPU bug or precision error), use abs().
 */
void L2NormalizeDescriptors(std::vector<float>& descriptors, int dim) {
    const size_t num_features = descriptors.size() / dim;
    for (size_t i = 0; i < num_features; ++i) {
        float* desc = descriptors.data() + i * dim;
        
        // Compute L2 norm (use abs to handle potential negative values from SiftGPU)
        double sum_sq = 0.0;
        for (int k = 0; k < dim; ++k) {
            const float val = std::fabs(desc[k]);  // Ensure non-negative
            sum_sq += val * val;
        }
        const double norm = std::sqrt(sum_sq);
        
        // Normalize
        if (norm > 1e-10) {  // Avoid division by zero
            for (int k = 0; k < dim; ++k) {
                desc[k] = static_cast<float>(std::fabs(desc[k]) / norm);
            }
        }
    }
}

/**
 * L1-Root normalize descriptors (RootSIFT, in-place)
 * Each descriptor: d = sqrt(d / ||d||_1)
 * 
 * Reference: "Three things everyone should know to improve object retrieval"
 * Arandjelovic & Zisserman, CVPR 2012
 */
void L1RootNormalizeDescriptors(std::vector<float>& descriptors, int dim) {
    const size_t num_features = descriptors.size() / dim;
    for (size_t i = 0; i < num_features; ++i) {
        float* desc = descriptors.data() + i * dim;
        
        // Compute L1 norm
        double sum = 0.0;
        for (int k = 0; k < dim; ++k) {
            sum += std::fabs(desc[k]);
        }
        
        // L1 normalize and square root
        if (sum > 1e-10) {  // Avoid division by zero
            for (int k = 0; k < dim; ++k) {
                desc[k] = static_cast<float>(std::sqrt(std::fabs(desc[k]) / sum));
            }
        }
    }
}

/**
 * Convert float descriptors to unsigned char (uint8)
 * 
 * Scale factor is 512 (not 256) for better quantization precision:
 * - uint8 has only 256 discrete values
 * - Scaling by 512 gives quantization step of ~0.002 (1/512)
 * - Scaling by 256 gives quantization step of ~0.004 (1/256)
 * - 2x better precision with 512
 * 
 * After L2 normalization and optional clipping, descriptor values are in [0, 1].
 * We scale by 512 and clamp to [0, 255].
 */
std::vector<unsigned char> ConvertDescriptorsToUChar(
    const std::vector<float>& descriptors_float, int dim) {
    
    std::vector<unsigned char> descriptors_uchar(descriptors_float.size());
    
    for (size_t i = 0; i < descriptors_float.size(); ++i) {
        const float scaled_value = std::round(512.0f * descriptors_float[i]);
        // Clamp to [0, 255]
        descriptors_uchar[i] = static_cast<unsigned char>(
            std::min(255.0f, std::max(0.0f, scaled_value)));
    }
    
    return descriptors_uchar;
}

// ============================================================================
// SiftGPUExtractor implementation
// ============================================================================

SiftGPUExtractor::SiftGPUExtractor(const SiftGPUParams& params)
    : params_(params)
{
}

bool SiftGPUExtractor::initialize()
{
    if (initialized_) {
        LOG(WARNING) << "SiftGPU already initialized";
        return true;
    }
    
    sift_gpu_ = createSiftGPU(params_);
    if (!sift_gpu_) {
        LOG(ERROR) << "Failed to create SiftGPU instance";
        return false;
    }
    
    initialized_ = true;
    LOG(INFO) << "SiftGPU initialized successfully";
    return true;
}

int SiftGPUExtractor::extract(const cv::Mat& image,
                               std::vector<SiftGPU::SiftKeypoint>& keypoints,
                               std::vector<float>& descriptors)
{
    if (!initialized_) {
        LOG(ERROR) << "SiftGPU not initialized";
        return 0;
    }
    
    // Convert to grayscale if needed
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }
    
    // Run SIFT GPU
    LOG(INFO) << "Running SIFT on image: " << gray.cols << "x" << gray.rows << " (grayscale)";
    int result = sift_gpu_->RunSIFT(gray.cols, gray.rows, gray.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    if (result == 0) {
        LOG(ERROR) << "RunSIFT failed for image " << gray.cols << "x" << gray.rows 
                   << " - check GPU/OpenGL availability";
        return 0;
    }
    
    // Get number of features
    int num_features = sift_gpu_->GetFeatureNum();
    if (num_features == 0) {
        LOG(WARNING) << "RunSIFT succeeded but found 0 features - try adjusting threshold (-t) or nfeatures (-n)";
    } else {
        LOG(INFO) << "Extracted " << num_features << " features";
    }
    
    // Allocate memory
    keypoints.resize(num_features);
    descriptors.resize(num_features * 128); // SIFT descriptor is 128-dimensional
    
    // Download features from GPU
    sift_gpu_->GetFeatureVector(keypoints.data(), descriptors.data());
    
    // Note: Normalization and post-processing (L2/L1Root, uint8, distribution)
    // are now handled externally using CPU helper functions.
    // This keeps GPU extraction pure and allows flexible pipeline composition.
    
    return num_features;
}

SiftGPUExtractor::SiftGPUPtr SiftGPUExtractor::createSiftGPU(const SiftGPUParams& param)
{
    SiftGPUPtr siftGPUPtr(new SiftGPU);
    
    char strOcFrom[10];
    char strNOctave[10];
    char strNLevel[10];
    char strPeak[10];
    char strMaxFeatures[10];
    
    sprintf(strOcFrom, "%d", param.nOctiveFrom);
    sprintf(strNOctave, "%d", param.nOctives);
    sprintf(strNLevel, "%d", param.nLevel);
    sprintf(strPeak, "%f", param.dPeak / param.nLevel);
    sprintf(strMaxFeatures, "%d", param.nMaxFeatures);
    
    const char* argv[100] = { 0 };
    int ii = 0;
    argv[ii++] = "-v";
    argv[ii++] = "0";
    argv[ii++] = "-fo";
    argv[ii++] = strOcFrom;
    argv[ii++] = "-t";
    argv[ii++] = strPeak;
    argv[ii++] = "-d";
    argv[ii++] = strNLevel;
    argv[ii++] = "-w";
    argv[ii++] = "3";
    
    if (param.nOctives != -1) {
        argv[ii++] = "-no";
        argv[ii++] = strNOctave;
    }
    
    if (param.nMaxFeatures != -1) {
        argv[ii++] = "-tc2";
        argv[ii++] = strMaxFeatures;
    }
    
    if (param.adaptDarkness) {
        argv[ii++] = "-da";
    }
    
    siftGPUPtr->ParseParam(ii, argv);
    
    // Create OpenGL context for computation
    int support = siftGPUPtr->CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        LOG(ERROR) << "SiftGPU not fully supported";
        return SiftGPUPtr();
    }
    
    if (siftGPUPtr->VerifyContextGL() == 0) {
        LOG(ERROR) << "VerifyContextGL failed";
        return SiftGPUPtr();
    }
    
    return siftGPUPtr;
}

// ============================================================================
// Helper Functions - Exposed for CPU Multi-threading
// ============================================================================

void ApplyFeatureDistribution(
    std::vector<SiftGPU::SiftKeypoint>& keypoints,
    std::vector<float>& descriptors,
    int image_width,
    int image_height,
    int grid_size,
    int max_per_cell,
    bool keep_orientation) {
    
    if (keypoints.empty()) return;
    
    // Setup grid distribution parameters
    GridDistributionParams params;
    params.grid_size = grid_size;
    params.max_per_cell = max_per_cell;
    params.keep_orientation = keep_orientation;
    
    // Get indices of features to keep
    auto kept_indices = DistributeKeypointsGrid(keypoints, image_width, image_height, params);
    
    LOG(INFO) << "Grid distribution: kept " << kept_indices.size() << "/" << keypoints.size()
              << " features (grid=" << grid_size << "px, max_per_cell=" << max_per_cell << ")";
    
    // Re-pack keypoints and descriptors
    std::vector<SiftGPU::SiftKeypoint> kpts_filtered;
    std::vector<float> desc_filtered;
    kpts_filtered.reserve(kept_indices.size());
    desc_filtered.reserve(kept_indices.size() * 128);
    
    for (size_t idx : kept_indices) {
        kpts_filtered.push_back(keypoints[idx]);
        const float* src = descriptors.data() + idx * 128;
        desc_filtered.insert(desc_filtered.end(), src, src + 128);
    }
    
    keypoints = std::move(kpts_filtered);
    descriptors = std::move(desc_filtered);
}

void ApplyFeatureDistribution(
    std::vector<SiftGPU::SiftKeypoint>& keypoints,
    std::vector<unsigned char>& descriptors,
    int image_width,
    int image_height,
    int grid_size,
    int max_per_cell,
    bool keep_orientation) {
    
    if (keypoints.empty()) return;
    
    // Setup grid distribution parameters
    GridDistributionParams params;
    params.grid_size = grid_size;
    params.max_per_cell = max_per_cell;
    params.keep_orientation = keep_orientation;
    
    // Get indices of features to keep
    auto kept_indices = DistributeKeypointsGrid(keypoints, image_width, image_height, params);
    
    LOG(INFO) << "Grid distribution: kept " << kept_indices.size() << "/" << keypoints.size()
              << " features (grid=" << grid_size << "px, max_per_cell=" << max_per_cell << ")";
    
    // Re-pack keypoints and descriptors
    std::vector<SiftGPU::SiftKeypoint> kpts_filtered;
    std::vector<unsigned char> desc_filtered;
    kpts_filtered.reserve(kept_indices.size());
    desc_filtered.reserve(kept_indices.size() * 128);
    
    for (size_t idx : kept_indices) {
        kpts_filtered.push_back(keypoints[idx]);
        const unsigned char* src = descriptors.data() + idx * 128;
        desc_filtered.insert(desc_filtered.end(), src, src + 128);
    }
    
    keypoints = std::move(kpts_filtered);
    descriptors = std::move(desc_filtered);
}

} // namespace modules
} // namespace insight
