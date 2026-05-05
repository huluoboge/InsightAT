/**
 * @file  bundler_loader.h
 * @brief 读取 Noah Snavely Bundler v0.3 导出（list.txt + bundle.out），并填充 RenderTracks。
 *
 * COLMAP：宽高与主点来自 cameras.txt；纯 Bundler 无尺寸时可 GDAL 读图，失败则用主点整数估 w=2*cx、h=2*cy。
 */
#pragma once
#ifndef INSIGHT_RENDER_BUNDLER_LOADER_H
#define INSIGHT_RENDER_BUNDLER_LOADER_H

#include "render_global.h"
#include "render_tracks.h"

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace insight {
namespace render {

/// 单相机 Bundler 参数（world→camera: Xc = R * Xw + t）。
struct BundlerCamera {
  double focal = 0;
  double k1 = 0;
  double k2 = 0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  /// COLMAP cameras.txt 中的宽高；>0 时跳过 GDAL。
  int image_width = 0;
  int image_height = 0;
  /// 像素主点 (cx, cy)；COLMAP 由内参填写；纯 Bundler 常为 0，GDAL 失败估尺寸时用 round(focal) 作整数主点近似。
  double principal_cx = 0;
  double principal_cy = 0;
};

struct BundlerObservation {
  int cam_idx = 0;
  int key_idx = 0;
  float u = 0;
  float v = 0;
};

struct BundlerPoint {
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d rgb = Eigen::Vector3d::Zero(); // 0–255
  std::vector<BundlerObservation> observations;
};

struct BundlerScene {
  std::vector<std::string> image_paths; // 与 bundle 中相机顺序一致
  std::vector<BundlerCamera> cameras;
  std::vector<BundlerPoint> points;
};

/// 磁盘解析阶段进度（读 bundle.out / COLMAP 文本，尚未 GDAL）。`current` ∈ [0, total]；
/// `total < 0` 表示总数未知（仅更新 current，适合不定进度条）。`stage`："bundle" | "images" | "points3D"。
using ReconstructionLoadProgress =
    std::function<void(int64_t current, int64_t total, const char* stage)>;

/// 在 bundle 目录下查找 list.txt 与 bundle.out（或 bundle.r.out），解析为 BundlerScene。
/// 成功时调用方需已链接 GDAL；内部会 GdalUtils::InitGDAL()。
RENDER_EXPORT bool load_bundler_directory(const std::string& bundle_dir, BundlerScene* scene,
                                          std::string* error_message,
                                          ReconstructionLoadProgress progress = {});

/// 自动识别目录类型：若存在 COLMAP text 模型（cameras.txt + images.txt + points3D.txt）则按
/// COLMAP 解析；否则按 Bundler（list.txt + bundle.out）解析。
RENDER_EXPORT bool load_reconstruction_directory(const std::string& dir, BundlerScene* scene,
                                                 std::string* error_message,
                                                 ReconstructionLoadProgress progress = {});

/// 加载进度：current ∈ [1,total]，stage 为简短英文阶段名（如 "dimensions"）。
using BundlerFillProgress = std::function<void(int current, int total, const char* stage)>;

/// 根据平均观测深度与像素尺度估计初始 photoScale，使视锥在场景中的尺度约为 avg_depth / 10
///（与 render_tracks 中 1/50 缩放一致）。mean_max_pixel_extent 为各相机 max(w,h,focal) 的平均值。
RENDER_EXPORT float compute_initial_photo_scale_from_scene(double avg_observation_depth,
                                                          float mean_max_pixel_extent);

/// 将 Bundler 场景写入 RenderTracks（平移到场景质心）。逐相机 GDAL 读尺寸时可传 progress。
/// 会根据场景尺度设置初始 RenderOptions（photoScale、点大小等）。
RENDER_EXPORT void fill_render_tracks_from_bundler(RenderTracks* tracks, const BundlerScene& scene,
                                                   BundlerFillProgress progress = {});

} // namespace render
} // namespace insight

#endif // INSIGHT_RENDER_BUNDLER_LOADER_H
