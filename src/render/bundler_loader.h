/**
 * @file  bundler_loader.h
 * @brief 读取 Noah Snavely Bundler v0.3 导出（list.txt + bundle.out），并填充 RenderTracks。
 *
 * 图像尺寸通过 insight::GdalUtils::GetWidthHeightPixel 读取，避免解码整幅图像。
 */
#pragma once
#ifndef INSIGHT_RENDER_BUNDLER_LOADER_H
#define INSIGHT_RENDER_BUNDLER_LOADER_H

#include "render_global.h"
#include "render_tracks.h"

#include <Eigen/Core>

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
};

struct BundlerPoint {
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d rgb = Eigen::Vector3d::Zero(); // 0–255
};

struct BundlerScene {
  std::vector<std::string> image_paths; // 与 bundle 中相机顺序一致
  std::vector<BundlerCamera> cameras;
  std::vector<BundlerPoint> points;
};

/// 在 bundle 目录下查找 list.txt 与 bundle.out（或 bundle.r.out），解析为 BundlerScene。
/// 成功时调用方需已链接 GDAL；内部会 GdalUtils::InitGDAL()。
RENDER_EXPORT bool load_bundler_directory(const std::string& bundle_dir, BundlerScene* scene,
                                          std::string* error_message);

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
