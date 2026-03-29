/**
 * @file  opengl_mat.h
 * @brief OpenGL 风格 4×4 变换（Eigen Mat4），用于 Qt/OpenGL 渲染路径的投影与模型矩阵。
 *
 * Architecture
 * ─────────────
 *  本头文件仅提供无状态的 `inline` 函数，依赖 `insight::Mat4` / `Vec3`（见 `util/numeric.h`），
 *  与旧版自实现 vmath 在数值上等价，便于与算法层 Eigen 类型互通。
 *
 * Usage
 * ─────
 *   Mat4 V = look_at_matrix(eye, center, up);
 *   Mat4 P = frustum_matrix(l, r, b, t, n, f);
 *   Mat4 R = rotation_matrix_deg(angle_deg, axis);
 */

#pragma once
#ifndef OPENGL_MAT_H
#define OPENGL_MAT_H

#include <cmath>

#include "render_global.h"
#include "render_types.h"

#ifdef minor
#undef minor
#endif

namespace insight {

namespace render {

/// 刚体变换（正交旋转 + 平移）的快速逆，与旧 vmath::fast_inverse 等价。
inline Mat4 fast_inverse(const Mat4& mat) {
  const Vec3 t = mat.col(3).head<3>();
  const double tx = -mat.col(0).head<3>().dot(t);
  const double ty = -mat.col(1).head<3>().dot(t);
  const double tz = -mat.col(2).head<3>().dot(t);

  Mat4 inverse_mat;
  inverse_mat << mat(0, 0), mat(1, 0), mat(2, 0), tx, mat(0, 1), mat(1, 1), mat(2, 1), ty,
      mat(0, 2), mat(1, 2), mat(2, 2), tz, 0, 0, 0, 1;
  return inverse_mat;
}

/// OpenGL 风格 look-at 视图矩阵（与旧 vmath::lookat_matrix 一致）。
inline Mat4 look_at_matrix(const Vec3& eye, const Vec3& center, const Vec3& up) {
  const Vec3 forward = (center - eye).normalized();
  const Vec3 side = forward.cross(up).normalized();
  const Vec3 up2 = side.cross(forward);

  Mat4 m = Mat4::Identity();
  m(0, 0) = side.x();
  m(0, 1) = side.y();
  m(0, 2) = side.z();
  m(1, 0) = up2.x();
  m(1, 1) = up2.y();
  m(1, 2) = up2.z();
  m(2, 0) = -forward.x();
  m(2, 1) = -forward.y();
  m(2, 2) = -forward.z();

  Mat4 tr = Mat4::Identity();
  tr(0, 3) = -eye.x();
  tr(1, 3) = -eye.y();
  tr(2, 3) = -eye.z();
  return m * tr;
}

inline Mat4 frustum_matrix(double l, double r, double b, double t, double n, double f) {
  Mat4 m;
  m << (2 * n) / (r - l), 0, (r + l) / (r - l), 0, 0, (2 * n) / (t - b), (t + b) / (t - b), 0, 0, 0,
      -(f + n) / (f - n), -(2 * f * n) / (f - n), 0, 0, -1, 0;
  return m;
}

inline Mat4 ortho_matrix(double l, double r, double b, double t, double n, double f) {
  Mat4 m;
  m << 2 / (r - l), 0, 0, -(r + l) / (r - l), 0, 2 / (t - b), 0, -(t + b) / (t - b), 0, 0,
      -2 / (f - n), -(f + n) / (f - n), 0, 0, 0, 1;
  return m;
}

/// 角度制，轴可为非单位向量（零向量时返回单位矩阵，避免 NaN）。
inline Mat4 rotation_matrix_deg(double angle_deg, const Vec3& axis) {
  const double len = axis.norm();
  if (len < 1e-20) {
    return Mat4::Identity();
  }
  const Vec3 u = axis / len;
  const double a = D2R(angle_deg);
  Eigen::Matrix3d S;
  S << 0, -u.z(), u.y(), u.z(), 0, -u.x(), -u.y(), u.x(), 0;
  const Eigen::Matrix3d uut = u * u.transpose();
  const Eigen::Matrix3d R =
      uut + std::cos(a) * (Eigen::Matrix3d::Identity() - uut) + std::sin(a) * S;
  Mat4 m = Mat4::Identity();
  m.block<3, 3>(0, 0) = R;
  return m;
}

inline Mat4 rotation_matrix_deg(double angle_deg, double x, double y, double z) {
  return rotation_matrix_deg(angle_deg, Vec3(x, y, z));
}

inline Mat4 scaling_matrix(double x, double y, double z) {
  Mat4 m = Mat4::Zero();
  m(0, 0) = x;
  m(1, 1) = y;
  m(2, 2) = z;
  m(3, 3) = 1;
  return m;
}

/// 绕点缩放（与旧 scaling_pos_matrix 一致）。
inline Mat4 scaling_pos_matrix(double posx, double posy, double posz, double x, double y,
                               double z) {
  Mat4 m = Mat4::Zero();
  m(0, 0) = x;
  m(1, 1) = y;
  m(2, 2) = z;
  m(0, 3) = posx * (1 - x);
  m(1, 3) = posy * (1 - y);
  m(2, 3) = posz * (1 - z);
  m(3, 3) = 1;
  return m;
}

inline Mat4 translation_matrix(double x, double y, double z) {
  Mat4 m = Mat4::Identity();
  m(0, 3) = x;
  m(1, 3) = y;
  m(2, 3) = z;
  return m;
}

} // namespace render

} // namespace insight

#endif // OPENGL_MAT_H
