/**
 * @file  sift_gpu_extractor.h
 * @brief SIFT GPU 特征提取：封装 SiftGPU 初始化与提取，支持 L2/RootSIFT 与网格分布。
 */

#pragma once

#include <cstddef>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "SiftGPU/SiftGPU.h"

namespace insight {
namespace modules {

/** 描述子归一化类型：L2 或 L1-Root (RootSIFT)。 */
enum class DescriptorNormalization {
  L2,     ///< L2 归一化
  L1_ROOT ///< L1 + 逐元开方 (RootSIFT)
};

/** SIFT GPU 参数（仅提取）。 */
struct SiftGPUParams {
  int n_octave_from = 0;      ///< 起始 octave
  int n_octaves = -1;         ///< octave 数（-1 自动）
  int n_level = 3;            ///< 每 octave 层数
  double d_peak = 0.02;       ///< 峰值阈值（会除以 n_level）
  int n_max_features = 8000;  ///< 最大特征数
  bool adapt_darkness = true; ///< 适应暗图
  bool use_cuda = false;      ///< 使用 CUDA 后端
  int truncate_method = 0;    ///< 0=-tc, 1=-tc2, 2=-tc3
};

/**
 * SIFT GPU 特征提取器，每实例独占 GPU 上下文，可多线程各建实例。
 */
class SiftGPUExtractor {
public:
  using SiftGPUPtr = std::shared_ptr<SiftGPU>;

  explicit SiftGPUExtractor(const SiftGPUParams& params);
  ~SiftGPUExtractor() = default;

  bool initialize();
  bool reconfigure(const SiftGPUParams& new_params);

  /** 从图像提取特征，返回提取数量。 */
  int extract(const cv::Mat& image, std::vector<SiftGPU::SiftKeypoint>& keypoints,
              std::vector<float>& descriptors);

  bool is_initialized() const { return initialized_; }

private:
  SiftGPUParams params_;
  SiftGPUPtr sift_gpu_;
  bool initialized_ = false;

  SiftGPUPtr create_sift_gpu(const SiftGPUParams& param);
};

/** 对关键点与描述子做网格分布（可在 CPU 线程中调用）。 */
void apply_feature_distribution(std::vector<SiftGPU::SiftKeypoint>& keypoints,
                                std::vector<float>& descriptors, int image_width, int image_height,
                                int grid_size = 32, int max_per_cell = 2,
                                bool keep_orientation = true);

void apply_feature_distribution(std::vector<SiftGPU::SiftKeypoint>& keypoints,
                                std::vector<unsigned char>& descriptors, int image_width,
                                int image_height, int grid_size = 32, int max_per_cell = 2,
                                bool keep_orientation = true);

void l2_normalize_descriptors(std::vector<float>& descriptors, int dim = 128);
void l1_root_normalize_descriptors(std::vector<float>& descriptors, int dim = 128);
std::vector<unsigned char> convert_descriptors_to_uchar(const std::vector<float>& descriptors_float,
                                                        int dim = 128);

} // namespace modules
} // namespace insight
