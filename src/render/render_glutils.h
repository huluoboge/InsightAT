/**
 * @file  render_glutils.h
 * @brief OpenGL 屏幕坐标与模型/世界坐标互转、纹理上限查询等工具。
 */
#pragma once
#ifndef RENDER_GLUTILS_H
#define RENDER_GLUTILS_H

#include "render_camera.h"
#include "render_global.h"

#include <Eigen/Core>

namespace insight {

namespace render {

class RENDER_EXPORT GLUtils {
public:
  static Eigen::Vector3f screen_to_world(float screen_x, float screen_y, float deep /* 0--1 */);
  static Eigen::Vector3d screen_to_world(Eigen::Matrix<double, 4, 4> model_view, float screen_x,
                                         float screen_y, float deep /* 0--1 */);
  static int max_texture_size();
};

} // namespace render

} // namespace insight

#endif // RENDER_GLUTILS_H
