#include "sift_gpu_extractor.h"
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <GL/gl.h>

namespace insight {
namespace modules {

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
    int result = sift_gpu_->RunSIFT(gray.cols, gray.rows, gray.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    if (result == 0) {
        LOG(ERROR) << "RunSIFT failed";
        return 0;
    }
    
    // Get number of features
    int num_features = sift_gpu_->GetFeatureNum();
    LOG(INFO) << "Extracted " << num_features << " features";
    
    // Allocate memory
    keypoints.resize(num_features);
    descriptors.resize(num_features * 128); // SIFT descriptor is 128-dimensional
    
    // Download features
    sift_gpu_->GetFeatureVector(keypoints.data(), descriptors.data());
    
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

} // namespace modules
} // namespace insight
