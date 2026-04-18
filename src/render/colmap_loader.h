/**
 * @file  colmap_loader.h
 * @brief 读取 COLMAP text 稀疏模型（cameras.txt + images.txt + points3D.txt）到 BundlerScene。
 */
#pragma once
#ifndef INSIGHT_RENDER_COLMAP_LOADER_H
#define INSIGHT_RENDER_COLMAP_LOADER_H

#include "render_global.h"

#include <string>

namespace insight {
namespace render {

struct BundlerScene;

/// 目录下需同时存在 cameras.txt、images.txt、points3D.txt（与 COLMAP sparse/0 一致）。
/// 图像路径按 COLMAP 惯例解析：相对路径相对 sparse 目录，并尝试 ../images/<basename>。
RENDER_EXPORT bool load_colmap_text_directory(const std::string& colmap_sparse_dir,
                                              BundlerScene* scene, std::string* error_message);

} // namespace render
} // namespace insight

#endif
