#pragma once

#include <string>
#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace insight {
namespace modules {

/**
 * SuperPoint Configuration
 */
struct SuperPointConfig {
    std::string model_path;          // Path to ONNX model file
    std::string provider = "cpu";    // "cpu" or "cuda"
    float threshold = 0.005f;        // Detection confidence threshold
    int nms_radius = 4;              // NMS radius (0 to disable)
    int max_keypoints = -1;          // Max keypoints to keep (-1 for unlimited)
    
    // Validate configuration
    bool validate(std::string* error_msg = nullptr) const;
};

/**
 * SuperPoint Feature Extractor
 * 
 * Thread Safety: Each instance owns its ONNX Runtime session.
 * Safe for single-threaded use (e.g., StageCurrent in pipeline).
 * For multi-threaded extraction, create one instance per thread.
 */
class SuperPointExtractor {
public:
    /**
     * Constructor
     * @param config SuperPoint configuration
     * @throws std::runtime_error if initialization fails
     */
    explicit SuperPointExtractor(const SuperPointConfig& config);
    
    /**
     * Destructor
     */
    ~SuperPointExtractor();
    
    // Non-copyable (owns ONNX session resources)
    SuperPointExtractor(const SuperPointExtractor&) = delete;
    SuperPointExtractor& operator=(const SuperPointExtractor&) = delete;
    
    /**
     * Extract keypoints and descriptors from image
     * 
     * @param image Input image (grayscale or BGR)
     * @param keypoints Output keypoints (x, y, scale=1.0, angle=0)
     * @param descriptors Output descriptors (N x 256, float32, L2-normalized)
     * @param scores Output confidence scores (N x 1, optional)
     * @return true on success, false on failure
     */
    bool extract(const cv::Mat& image,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors,
                 std::vector<float>* scores = nullptr);
    
    /**
     * Get descriptor dimension (always 256 for SuperPoint)
     */
    static constexpr int getDescriptorDim() { return 256; }
    
    /**
     * Get descriptor type (always float32 for SuperPoint)
     */
    static std::string getDescriptorType() { return "float32"; }
    
    /**
     * Get feature type name
     */
    static std::string getFeatureType() { return "superpoint"; }
    
    /**
     * Check if CUDA provider is available
     */
    static bool isCudaAvailable();

private:
    class Impl;  // Forward declaration for PIMPL
    std::unique_ptr<Impl> impl_;
};

} // namespace modules
} // namespace insight
