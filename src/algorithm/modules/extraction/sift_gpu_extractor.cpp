/**
 * @file  sift_gpu_extractor.cpp
 * @brief SIFT GPU 提取实现、描述子归一化与网格分布封装。
 */

#include "sift_gpu_extractor.h"
#include "feature_distribution.h"

#ifndef INSIGHTAT_ENABLE_SIFTGPU
#define INSIGHTAT_ENABLE_SIFTGPU 0
#endif

#if INSIGHTAT_ENABLE_SIFTGPU
#include <GL/gl.h>
#endif
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#if defined(HAVE_CUDA) && __has_include(<popsift/features.h>) && __has_include(<popsift/popsift.h>) && \
    __has_include(<popsift/sift_conf.h>)
#define INSIGHTAT_HAS_POPSIFT 1
#include <popsift/features.h>
#include <popsift/popsift.h>
#include <popsift/sift_conf.h>
#else
#define INSIGHTAT_HAS_POPSIFT 0
#endif

namespace insight {
namespace modules {

struct SiftGPUExtractor::PopSiftImpl {
#if INSIGHTAT_HAS_POPSIFT
  std::unique_ptr<PopSift> popsift;
#endif
};

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
void l2_normalize_descriptors(std::vector<float>& descriptors, int dim) {
  const size_t num_features = descriptors.size() / dim;
  for (size_t i = 0; i < num_features; ++i) {
    float* desc = descriptors.data() + i * dim;

    // Compute L2 norm (use abs to handle potential negative values from SiftGPU)
    double sum_sq = 0.0;
    for (int k = 0; k < dim; ++k) {
      const float val = std::fabs(desc[k]); // Ensure non-negative
      sum_sq += val * val;
    }
    const double norm = std::sqrt(sum_sq);

    // Normalize
    if (norm > 1e-10) { // Avoid division by zero
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
void l1_root_normalize_descriptors(std::vector<float>& descriptors, int dim) {
  const size_t num_features = descriptors.size() / dim;
  for (size_t i = 0; i < num_features; ++i) {
    float* desc = descriptors.data() + i * dim;

    // Compute L1 norm
    double sum = 0.0;
    for (int k = 0; k < dim; ++k) {
      sum += std::fabs(desc[k]);
    }

    // L1 normalize and square root
    if (sum > 1e-10) { // Avoid division by zero
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
std::vector<unsigned char> convert_descriptors_to_uchar(const std::vector<float>& descriptors_float,
                                                        int dim) {

  std::vector<unsigned char> descriptors_uchar(descriptors_float.size());

  for (size_t i = 0; i < descriptors_float.size(); ++i) {
    const float scaled_value = std::round(512.0f * descriptors_float[i]);
    // Clamp to [0, 255]
    descriptors_uchar[i] =
        static_cast<unsigned char>(std::min(255.0f, std::max(0.0f, scaled_value)));
  }

  return descriptors_uchar;
}

// ============================================================================
// SiftGPUExtractor implementation
// ============================================================================

SiftGPUExtractor::SiftGPUExtractor(const SiftGPUParams& params) : params_(params) {}

bool SiftGPUExtractor::initialize() {
  if (initialized_) {
    LOG(WARNING) << "SiftGPU already initialized";
    return true;
  }

  if (params_.use_sift_gpu == params_.use_pop_sift) {
    LOG(ERROR) << "Exactly one backend must be enabled: use_sift_gpu xor use_pop_sift";
    return false;
  }

  if (params_.use_pop_sift) {
    if (!initialize_popsift(params_)) {
      return false;
    }
  } else {
#if !INSIGHTAT_ENABLE_SIFTGPU
    LOG(ERROR) << "SiftGPU extractor requested but disabled at compile time "
               << "(INSIGHTAT_ENABLE_SIFTGPU=OFF)";
    return false;
#else
    sift_gpu_ = create_sift_gpu(params_);
    if (!sift_gpu_) {
      LOG(ERROR) << "Failed to create SiftGPU instance";
      return false;
    }
#endif
  }

  initialized_ = true;
  LOG(INFO) << "SiftGPU initialized successfully";
  return true;
}

bool SiftGPUExtractor::reconfigure(const SiftGPUParams& new_params) {
  if (!initialized_) {
    LOG(ERROR) << "Cannot reconfigure: SiftGPU not initialized";
    return false;
  }

  if (new_params == params_) {
    LOG(INFO) << "SiftGPU params unchanged, skipping reconfigure";
    return true;
  }
  if (new_params.use_sift_gpu == new_params.use_pop_sift) {
    LOG(ERROR) << "Exactly one backend must be enabled: use_sift_gpu xor use_pop_sift";
    return false;
  }

  // destroy existing backend instance before creating a new one
  SiftGPUPtr().swap(sift_gpu_);
  popsift_impl_.reset();
  params_ = new_params;
  if (params_.use_pop_sift) {
    if (!initialize_popsift(params_)) {
      return false;
    }
  } else {
#if !INSIGHTAT_ENABLE_SIFTGPU
    LOG(ERROR) << "SiftGPU extractor requested but disabled at compile time "
               << "(INSIGHTAT_ENABLE_SIFTGPU=OFF)";
    return false;
#else
    sift_gpu_ = create_sift_gpu(new_params);
    if (!sift_gpu_) {
      LOG(ERROR) << "Failed to create SiftGPU instance";
      return false;
    }
#endif
  }

  initialized_ = true;
  LOG(INFO) << "SiftGPU initialized successfully";
  return true;
}

int SiftGPUExtractor::extract(const cv::Mat& image, std::vector<SiftGPU::SiftKeypoint>& keypoints,
                              std::vector<float>& descriptors) {
  if (params_.use_pop_sift) {
    return extract_popsift(image, keypoints, descriptors);
  }

#if !INSIGHTAT_ENABLE_SIFTGPU
  (void)image;
  (void)keypoints;
  (void)descriptors;
  LOG(ERROR) << "SiftGPU extraction requested but disabled at compile time";
  return 0;
#else
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
    LOG(WARNING) << "RunSIFT succeeded but found 0 features - try adjusting threshold (-t) or "
                    "nfeatures (-n)";
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
#endif
}

SiftGPUExtractor::SiftGPUPtr SiftGPUExtractor::create_sift_gpu(const SiftGPUParams& param) {
#if !INSIGHTAT_ENABLE_SIFTGPU
  (void)param;
  LOG(ERROR) << "create_sift_gpu called but SiftGPU is disabled at compile time";
  return SiftGPUPtr();
#else
  if (param.use_pop_sift) {
    LOG(ERROR) << "create_sift_gpu called while use_pop_sift=true";
    return SiftGPUPtr();
  }
  SiftGPUPtr sift_gpu_ptr(new SiftGPU);

  char strOcFrom[10];
  char strNOctave[10];
  char strNLevel[10];
  char strPeak[10];
  char strMaxFeatures[10];
  char strImageMaxDimension[10];

  sprintf(strOcFrom, "%d", param.n_octave_from);
  sprintf(strNOctave, "%d", param.n_octaves);
  sprintf(strNLevel, "%d", param.n_level);
  sprintf(strPeak, "%f", static_cast<double>(param.d_peak / param.n_level));
  sprintf(strMaxFeatures, "%d", param.n_max_features);
  sprintf(strImageMaxDimension, "%d", param.image_max_dimension);

  const char* argv[100] = {0};
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
  argv[ii++] = "-maxd";
  argv[ii++] = strImageMaxDimension;

  if (param.n_octaves != -1) {
    argv[ii++] = "-no";
    argv[ii++] = strNOctave;
  }

  if (param.n_max_features != -1) {
    switch (param.truncate_method) {
    case 1:
      argv[ii++] = "-tc2";
      break;
    case 2:
      argv[ii++] = "-tc3";
      break;
    default:
      argv[ii++] = "-tc";
      break; // Method 0 (default)
    }
    argv[ii++] = strMaxFeatures;
  }

  if (param.adapt_darkness) {
    argv[ii++] = "-da";
  }
  if (param.use_cuda) {
    argv[ii++] = "-cuda";
    argv[ii++] = "0"; // device 0
  }

  if (VLOG_IS_ON(1)) {
    for (int i = 0; i < ii; i++) {
      LOG(INFO) << "argv[" << i << "] = " << argv[i];
    }
  }
  sift_gpu_ptr->ParseParam(ii, argv);

  // sift_gpu_ptr->AllocatePyramid(param.image_max_dimension, param.image_max_dimension);
  int support = sift_gpu_ptr->CreateContextGL();
  if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
    LOG(ERROR) << "SiftGPU not fully supported";
    return SiftGPUPtr();
  }

  if (sift_gpu_ptr->VerifyContextGL() == 0) {
    LOG(ERROR) << "VerifyContextGL failed";
    return SiftGPUPtr();
  }

  return sift_gpu_ptr;
#endif
}

bool SiftGPUExtractor::initialize_popsift(const SiftGPUParams& param) {
#if INSIGHTAT_HAS_POPSIFT
  if (param.use_sift_gpu && param.use_pop_sift) {
    LOG(ERROR) << "Both use_sift_gpu and use_pop_sift are enabled; choose only one";
    return false;
  }
  if (!param.use_pop_sift) {
    LOG(ERROR) << "initialize_popsift called while use_pop_sift=false";
    return false;
  }

  popsift_impl_ = std::make_shared<PopSiftImpl>();
  try {
    popsift::Config ps_config;
    ps_config.setThreshold(static_cast<float>(param.d_peak));
    ps_config.setEdgeLimit(10.0f);
    ps_config.setFilterMaxExtrema(param.n_max_features);
    ps_config.setFilterGridSize(4);
    ps_config.setFilterSorting(popsift::Config::LargestScaleFirst);

    popsift_impl_->popsift = std::make_unique<PopSift>(
        ps_config, popsift::Config::ExtractingMode, PopSift::ByteImages, param.popsift_gpu_device);
  } catch (const std::exception& e) {
    LOG(ERROR) << "PopSift init failed: " << e.what();
    popsift_impl_.reset();
    return false;
  } catch (...) {
    LOG(ERROR) << "PopSift init failed: unknown exception";
    popsift_impl_.reset();
    return false;
  }

  LOG(INFO) << "PopSift initialized successfully on device " << param.popsift_gpu_device;
  return true;
#else
  (void)param;
  LOG(ERROR) << "PopSift requested but not available in this build (missing CUDA/PopSift headers)";
  return false;
#endif
}

int SiftGPUExtractor::extract_popsift(const cv::Mat& image, std::vector<SiftGPU::SiftKeypoint>& keypoints,
                                      std::vector<float>& descriptors) {
#if INSIGHTAT_HAS_POPSIFT
  try {
    if (!popsift_impl_ || !popsift_impl_->popsift) {
      LOG(ERROR) << "PopSift backend not initialized";
      return 0;
    }

    cv::Mat gray;
    if (image.channels() == 1 && image.depth() == CV_8U) {
      gray = image;
    } else if (image.channels() == 3) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      if (gray.depth() != CV_8U) {
        gray.convertTo(gray, CV_8U);
      }
    } else if (image.channels() == 4) {
      cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
      if (gray.depth() != CV_8U) {
        gray.convertTo(gray, CV_8U);
      }
    } else {
      image.convertTo(gray, CV_8U);
    }

    // Internal fallback: if caller did not pre-resize, guard PopSift memory by resizing here.
    // When caller already resized to <= image_max_dimension, this branch is a no-op.
    double coord_scale_back = 1.0;
    const int max_dim = std::max(gray.cols, gray.rows);
    if (params_.image_max_dimension > 0 && max_dim > params_.image_max_dimension) {
      const double scale = static_cast<double>(params_.image_max_dimension) / max_dim;
      cv::Mat resized;
      cv::resize(gray, resized, cv::Size(), scale, scale, cv::INTER_AREA);
      gray = std::move(resized);
      coord_scale_back = 1.0 / scale;
      LOG(WARNING) << "PopSift internal resize from " << image.cols << "x" << image.rows << " to "
                   << gray.cols << "x" << gray.rows
                   << " (image_max_dimension=" << params_.image_max_dimension << ")";
    }

    if (!gray.isContinuous()) {
      gray = gray.clone();
    }

    SiftJob* job = nullptr;
    try {
      job = popsift_impl_->popsift->enqueue(gray.cols, gray.rows, gray.data);
    } catch (const std::exception& e) {
      LOG(ERROR) << "PopSift enqueue failed: " << e.what();
      return 0;
    }

    if (!job) {
      LOG(ERROR) << "PopSift enqueue returned null job";
      return 0;
    }

    popsift::FeaturesHost* features = nullptr;
    try {
      features = job->getHost();
    } catch (const std::exception& e) {
      LOG(ERROR) << "PopSift getHost failed: " << e.what();
      delete job;
      return 0;
    }

    keypoints.clear();
    descriptors.clear();

    if (features) {
      const int num_features = features->getFeatureCount();
      int num_desc = 0;
      for (int i = 0; i < num_features; ++i) {
        num_desc += features->begin()[i].num_ori;
      }

      keypoints.reserve(static_cast<size_t>(num_desc));
      descriptors.reserve(static_cast<size_t>(num_desc) * 128);

      for (int i = 0; i < num_features; ++i) {
        const popsift::Feature& feat = features->begin()[i];
        for (int o = 0; o < feat.num_ori; ++o) {
          SiftGPU::SiftKeypoint kp;
          kp.x = static_cast<float>(feat.xpos * coord_scale_back);
          kp.y = static_cast<float>(feat.ypos * coord_scale_back);
          kp.s = static_cast<float>(feat.sigma * coord_scale_back);
          kp.o = feat.orientation[o];
          keypoints.push_back(kp);

          const popsift::Descriptor* desc = feat.desc[o];
          if (desc) {
            descriptors.insert(descriptors.end(), desc->features, desc->features + 128);
          } else {
            descriptors.insert(descriptors.end(), 128, 0.0f);
          }
        }
      }
      delete features;
    }

    delete job;
    return static_cast<int>(keypoints.size());
  } catch (const std::exception& e) {
    LOG(ERROR) << "PopSift extract failed with exception: " << e.what();
    return 0;
  } catch (...) {
    LOG(ERROR) << "PopSift extract failed with unknown exception";
    return 0;
  }
#else
  (void)image;
  (void)keypoints;
  (void)descriptors;
  LOG(ERROR) << "PopSift backend is not compiled in this build";
  return 0;
#endif
}

// ============================================================================
// Helper Functions - Exposed for CPU Multi-threading
// ============================================================================

void apply_feature_distribution(std::vector<SiftGPU::SiftKeypoint>& keypoints,
                                std::vector<float>& descriptors, int image_width, int image_height,
                                int grid_size, int max_per_cell, bool keep_orientation) {

  if (keypoints.empty())
    return;

  GridDistributionParams params;
  params.grid_size = grid_size;
  params.max_per_cell = max_per_cell;
  params.keep_orientation = keep_orientation;

  auto kept_indices = distribute_keypoints_grid(keypoints, image_width, image_height, params);

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

void apply_feature_distribution(std::vector<SiftGPU::SiftKeypoint>& keypoints,
                                std::vector<unsigned char>& descriptors, int image_width,
                                int image_height, int grid_size, int max_per_cell,
                                bool keep_orientation) {

  if (keypoints.empty())
    return;

  GridDistributionParams params;
  params.grid_size = grid_size;
  params.max_per_cell = max_per_cell;
  params.keep_orientation = keep_orientation;

  auto kept_indices = distribute_keypoints_grid(keypoints, image_width, image_height, params);

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
