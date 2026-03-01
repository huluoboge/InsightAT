#include "superpoint_extractor.h"
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <fstream>

#ifdef INSIGHTAT_HAS_SUPERPOINT
#include <onnxruntime_cxx_api.h>
#endif

namespace insight {
namespace modules {

bool SuperPointConfig::validate(std::string* error_msg) const {
    // Check model file exists
    std::ifstream f(model_path);
    if (!f.good()) {
        if (error_msg) {
            *error_msg = "Model file not found: " + model_path;
        }
        return false;
    }
    
    // Check provider
    if (provider != "cpu" && provider != "cuda") {
        if (error_msg) {
            *error_msg = "Invalid provider: " + provider + " (must be 'cpu' or 'cuda')";
        }
        return false;
    }
    
    // Check threshold
    if (threshold <= 0.0f || threshold >= 1.0f) {
        if (error_msg) {
            *error_msg = "Invalid threshold: must be in (0, 1)";
        }
        return false;
    }
    
    return true;
}

#define INSIGHTAT_HAS_SUPERPOINT
#ifdef INSIGHTAT_HAS_SUPERPOINT

// PIMPL implementation
class SuperPointExtractor::Impl {
public:
    Impl(const SuperPointConfig& config) : config_(config) {
        // Initialize ONNX Runtime environment
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "SuperPoint");
        
        // Configure session options
        session_options_.SetIntraOpNumThreads(4);
        session_options_.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_BASIC);
        
        // Configure provider (CPU or CUDA)
        if (config_.provider == "cuda") {
            try {
                OrtCUDAProviderOptions cuda_options;
                cuda_options.device_id = 0;
                cuda_options.arena_extend_strategy = 1;  // kSameAsRequested
                cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;
                cuda_options.do_copy_in_default_stream = 1;
                
                session_options_.AppendExecutionProvider_CUDA(cuda_options);
                LOG(INFO) << "SuperPoint: Using CUDA provider";
            } catch (const std::exception& e) {
                LOG(ERROR) << "Failed to initialize CUDA provider: " << e.what();
                throw std::runtime_error("CUDA provider initialization failed");
            }
        } else {
            LOG(INFO) << "SuperPoint: Using CPU provider";
        }
        
        // Create session
        session_ = std::make_unique<Ort::Session>(
            env_, config_.model_path.c_str(), session_options_);
        
        // Get input/output names
        Ort::AllocatorWithDefaultOptions allocator;
        
        size_t num_input = session_->GetInputCount();
        size_t num_output = session_->GetOutputCount();
        
        for (size_t i = 0; i < num_input; ++i) {
            auto name = session_->GetInputNameAllocated(i, allocator);
            input_names_str_.push_back(std::string(name.get()));
        }
        
        for (size_t i = 0; i < num_output; ++i) {
            auto name = session_->GetOutputNameAllocated(i, allocator);
            output_names_str_.push_back(std::string(name.get()));
        }
        
        // Convert to const char*
        for (const auto& name : input_names_str_) {
            input_names_.push_back(name.c_str());
        }
        for (const auto& name : output_names_str_) {
            output_names_.push_back(name.c_str());
        }
        
        LOG(INFO) << "SuperPoint model loaded: " << config_.model_path
                  << ", inputs: " << num_input << ", outputs: " << num_output;
    }
    
    ~Impl() = default;
    
    bool extract(const cv::Mat& image,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors,
                 std::vector<float>* scores) {
        // Preprocess: convert to grayscale if needed
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }
        
        // Convert to float32 and normalize to [0, 1]
        cv::Mat float_gray;
        gray.convertTo(float_gray, CV_32F, 1.0 / 255.0);
        
        // Prepare input tensor
        std::vector<int64_t> input_shape = {1, 1, gray.rows, gray.cols};
        size_t input_size = gray.rows * gray.cols;
        std::vector<float> input_data(input_size);
        std::memcpy(input_data.data(), float_gray.data, input_size * sizeof(float));
        
        auto memory_info = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);
        
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_data.data(), input_size,
            input_shape.data(), input_shape.size());
        
        // Run inference
        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr},
            input_names_.data(), &input_tensor, 1,
            output_names_.data(), output_names_.size());
        
        // Parse outputs
        // scores: [1, H, W]
        // descriptors: [1, 256, H/8, W/8]
        float* scores_data = output_tensors[0].GetTensorMutableData<float>();
        float* desc_data = output_tensors[1].GetTensorMutableData<float>();
        
        auto scores_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        auto desc_shape = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();
        
        int height = scores_shape[1];
        int width = scores_shape[2];
        int desc_h = desc_shape[2];
        int desc_w = desc_shape[3];
        
        // Apply NMS and threshold
        cv::Mat scores_mat(height, width, CV_32F, scores_data);
        cv::Mat threshold_mask = scores_mat > config_.threshold;
        
        cv::Mat nms_mask;
        if (config_.nms_radius > 0) {
            nms_mask = applyNMS(scores_mat, config_.nms_radius);
        } else {
            nms_mask = cv::Mat::ones(height, width, CV_8U);
        }
        
        cv::Mat final_mask;
        cv::bitwise_and(threshold_mask, nms_mask, final_mask);
        
        // Extract keypoints and descriptors
        std::vector<cv::KeyPoint> kpts;
        std::vector<std::vector<float>> descs;
        std::vector<float> confs;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (final_mask.at<uchar>(y, x)) {
                    float score = scores_data[y * width + x];
                    
                    cv::KeyPoint kp;
                    kp.pt = cv::Point2f(x, y);
                    kp.size = 1.0f;  // Scale
                    kp.angle = 0.0f;  // Angle (SuperPoint doesn't compute orientation)
                    kp.response = score;
                    kpts.push_back(kp);
                    confs.push_back(score);
                    
                    // Extract descriptor
                    int desc_y = y / 8;
                    int desc_x = x / 8;
                    if (desc_y < desc_h && desc_x < desc_w) {
                        std::vector<float> descriptor(256);
                        for (int c = 0; c < 256; ++c) {
                            descriptor[c] = desc_data[c * desc_h * desc_w + 
                                                      desc_y * desc_w + desc_x];
                        }
                        
                        // L2 normalize
                        float norm = 0.0f;
                        for (float v : descriptor) {
                            norm += v * v;
                        }
                        norm = std::sqrt(norm);
                        if (norm > 1e-6f) {
                            for (float& v : descriptor) {
                                v /= norm;
                            }
                        }
                        
                        descs.push_back(descriptor);
                    }
                }
            }
        }
        
        // Apply max_keypoints limit if specified
        if (config_.max_keypoints > 0 && kpts.size() > static_cast<size_t>(config_.max_keypoints)) {
            // Sort by score (descending)
            std::vector<size_t> indices(kpts.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), 
                      [&confs](size_t a, size_t b) { return confs[a] > confs[b]; });
            
            // Keep only top-k
            std::vector<cv::KeyPoint> kpts_filtered;
            std::vector<std::vector<float>> descs_filtered;
            std::vector<float> confs_filtered;
            
            for (int i = 0; i < config_.max_keypoints; ++i) {
                size_t idx = indices[i];
                kpts_filtered.push_back(kpts[idx]);
                descs_filtered.push_back(descs[idx]);
                confs_filtered.push_back(confs[idx]);
            }
            
            kpts = std::move(kpts_filtered);
            descs = std::move(descs_filtered);
            confs = std::move(confs_filtered);
        }
        
        // Convert to output format
        keypoints = kpts;
        
        if (!descs.empty()) {
            descriptors = cv::Mat(descs.size(), 256, CV_32F);
            for (size_t i = 0; i < descs.size(); ++i) {
                std::memcpy(descriptors.ptr<float>(i), descs[i].data(), 256 * sizeof(float));
            }
        } else {
            descriptors = cv::Mat();
        }
        
        if (scores) {
            *scores = confs;
        }
        
        VLOG(1) << "SuperPoint extracted " << keypoints.size() << " keypoints";
        
        return true;
    }
    
private:
    cv::Mat applyNMS(const cv::Mat& scores_mat, int nms_radius) {
        int height = scores_mat.rows;
        int width = scores_mat.cols;
        
        cv::Mat nms_mask = cv::Mat::ones(height, width, CV_8U);
        
        // Use dilation for local maximum pooling
        cv::Mat dilated;
        int kernel_size = 2 * nms_radius + 1;
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        cv::dilate(scores_mat, dilated, kernel);
        
        // Keep only local maxima
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float score = scores_mat.at<float>(y, x);
                float max_score = dilated.at<float>(y, x);
                
                if (score < max_score) {
                    nms_mask.at<uchar>(y, x) = 0;
                }
            }
        }
        
        return nms_mask;
    }
    
    SuperPointConfig config_;
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    
    std::vector<std::string> input_names_str_;
    std::vector<std::string> output_names_str_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
};

SuperPointExtractor::SuperPointExtractor(const SuperPointConfig& config) {
    std::string error_msg;
    if (!config.validate(&error_msg)) {
        throw std::runtime_error("Invalid SuperPoint config: " + error_msg);
    }
    
    impl_ = std::make_unique<Impl>(config);
}

SuperPointExtractor::~SuperPointExtractor() = default;

bool SuperPointExtractor::extract(const cv::Mat& image,
                                   std::vector<cv::KeyPoint>& keypoints,
                                   cv::Mat& descriptors,
                                   std::vector<float>* scores) {
    return impl_->extract(image, keypoints, descriptors, scores);
}

bool SuperPointExtractor::isCudaAvailable() {
    // Check if CUDA provider is available in ONNX Runtime
    try {
        Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "test");
        Ort::SessionOptions options;
        OrtCUDAProviderOptions cuda_options;
        cuda_options.device_id = 0;
        options.AppendExecutionProvider_CUDA(cuda_options);
        return true;
    } catch (...) {
        return false;
    }
}

#else  // !INSIGHTAT_HAS_SUPERPOINT

// Stub implementation when SuperPoint not compiled in
class SuperPointExtractor::Impl {
public:
    Impl(const SuperPointConfig&) {
        throw std::runtime_error("SuperPoint not compiled in. Rebuild with INSIGHTAT_ENABLE_SUPERPOINT=ON and ONNXRuntime.");
    }
};

SuperPointExtractor::SuperPointExtractor(const SuperPointConfig& config) {
    impl_ = std::make_unique<Impl>(config);  // Will throw
}

SuperPointExtractor::~SuperPointExtractor() = default;

bool SuperPointExtractor::extract(const cv::Mat&,
                                   std::vector<cv::KeyPoint>&,
                                   cv::Mat&,
                                   std::vector<float>*) {
    return false;
}

bool SuperPointExtractor::isCudaAvailable() {
    return false;
}

#endif  // INSIGHTAT_HAS_SUPERPOINT

} // namespace modules
} // namespace insight
