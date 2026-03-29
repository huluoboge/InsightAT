// Feature: render-lib-refactor, Property 2: opengl_mat.h 数学运算（Eigen 实现）
//
// 属性测试：验证 lookAt / frustum / ortho / rotation / fast_inverse 与已知数值性质一致。

#define RENDER_GLOBAL_H
#include <cmath>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>

#include "opengl_mat.h"

namespace {

bool approx_equal(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b, double eps = 1e-10) {
  return (a - b).cwiseAbs().maxCoeff() < eps;
}

} // namespace

using insight::render::fast_inverse;
using insight::render::frustum_matrix;
using insight::render::look_at_matrix;
using insight::render::ortho_matrix;
using insight::render::rotation_matrix_deg;
using insight::render::scaling_matrix;
using insight::render::translation_matrix;

// ── Property 测试 ─────────────────────────────────────────────────────────────

RC_GTEST_PROP(OpenGLMatProperty, RotationMatrixIsOrthogonal, ()) {
  const double angle = *rc::gen::inRange(-3600, 3600);
  const Eigen::Matrix4d m_x = rotation_matrix_deg(static_cast<double>(angle), 1.0, 0.0, 0.0);
  const Eigen::Matrix3d R_x = m_x.topLeftCorner<3, 3>();
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  RC_ASSERT((R_x * R_x.transpose() - I3).cwiseAbs().maxCoeff() < 1e-10);

  const Eigen::Matrix3d R_y =
      rotation_matrix_deg(static_cast<double>(angle), 0.0, 1.0, 0.0).topLeftCorner<3, 3>();
  RC_ASSERT((R_y * R_y.transpose() - I3).cwiseAbs().maxCoeff() < 1e-10);

  const Eigen::Matrix3d R_z =
      rotation_matrix_deg(static_cast<double>(angle), 0.0, 0.0, 1.0).topLeftCorner<3, 3>();
  RC_ASSERT((R_z * R_z.transpose() - I3).cwiseAbs().maxCoeff() < 1e-10);
}

RC_GTEST_PROP(OpenGLMatProperty, RotationMatrixArbitraryAxisIsOrthogonal, ()) {
  const double angle = *rc::gen::inRange(-3600, 3600);
  const int ax = *rc::gen::inRange(-100, 100);
  const int ay = *rc::gen::inRange(-100, 100);
  const int az = *rc::gen::inRange(-100, 100);
  RC_PRE(ax != 0 || ay != 0 || az != 0);

  const Eigen::Matrix3d R = rotation_matrix_deg(static_cast<double>(angle), static_cast<double>(ax),
                                                static_cast<double>(ay), static_cast<double>(az))
                                .topLeftCorner<3, 3>();
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  RC_ASSERT((R * R.transpose() - I3).cwiseAbs().maxCoeff() < 1e-10);
}

// ── 单元测试 ────────────────────────────────────────────────────────────────

TEST(OpenGLMatUnit, FastInverseOfIdentityIsIdentity) {
  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d result = fast_inverse(identity);
  EXPECT_TRUE(approx_equal(result, Eigen::Matrix4d::Identity(), 1e-10));
}

TEST(OpenGLMatUnit, FastInverseOfTranslation) {
  const Eigen::Matrix4d t = translation_matrix(1.0, 2.0, 3.0);
  const Eigen::Matrix4d t_inv = fast_inverse(t);
  EXPECT_TRUE(approx_equal(t * t_inv, Eigen::Matrix4d::Identity(), 1e-10));
}

TEST(OpenGLMatUnit, LookatMatrixKnownInput) {
  const Eigen::Vector3d eye(0.0, 0.0, 5.0);
  const Eigen::Vector3d center(0.0, 0.0, 0.0);
  const Eigen::Vector3d up(0.0, 1.0, 0.0);

  const Eigen::Matrix4d mat = look_at_matrix(eye, center, up);

  EXPECT_NEAR(mat(3, 0), 0.0, 1e-10);
  EXPECT_NEAR(mat(3, 1), 0.0, 1e-10);
  EXPECT_NEAR(mat(3, 2), 0.0, 1e-10);
  EXPECT_NEAR(mat(3, 3), 1.0, 1e-10);

  const Eigen::Matrix3d R = mat.topLeftCorner<3, 3>();
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  EXPECT_LT((R * R.transpose() - I3).cwiseAbs().maxCoeff(), 1e-10);

  EXPECT_NEAR(mat(2, 0), 0.0, 1e-10);
  EXPECT_NEAR(mat(2, 1), 0.0, 1e-10);
  EXPECT_NEAR(mat(2, 2), 1.0, 1e-10);
}

TEST(OpenGLMatUnit, FrustumMatrixKnownInput) {
  const double l = -1.0, r = 1.0, b = -1.0, t = 1.0, n = 1.0, f = 100.0;
  const Eigen::Matrix4d mat = frustum_matrix(l, r, b, t, n, f);

  EXPECT_NEAR(mat(0, 0), 2.0 * n / (r - l), 1e-10);
  EXPECT_NEAR(mat(1, 1), 2.0 * n / (t - b), 1e-10);
  EXPECT_NEAR(mat(2, 2), -(f + n) / (f - n), 1e-10);
  EXPECT_NEAR(mat(2, 3), -2.0 * f * n / (f - n), 1e-10);
  EXPECT_NEAR(mat(3, 2), -1.0, 1e-10);
  EXPECT_NEAR(mat(3, 3), 0.0, 1e-10);
}

TEST(OpenGLMatUnit, OrthoMatrixKnownInput) {
  const double l = -10.0, r = 10.0, b = -10.0, t = 10.0, n = 0.1, f = 1000.0;
  const Eigen::Matrix4d mat = ortho_matrix(l, r, b, t, n, f);

  EXPECT_NEAR(mat(0, 0), 2.0 / (r - l), 1e-10);
  EXPECT_NEAR(mat(1, 1), 2.0 / (t - b), 1e-10);
  EXPECT_NEAR(mat(2, 2), -2.0 / (f - n), 1e-10);
  EXPECT_NEAR(mat(3, 3), 1.0, 1e-10);
  EXPECT_NEAR(mat(0, 3), -(r + l) / (r - l), 1e-10);
  EXPECT_NEAR(mat(1, 3), -(t + b) / (t - b), 1e-10);
  EXPECT_NEAR(mat(2, 3), -(f + n) / (f - n), 1e-10);
}

TEST(OpenGLMatUnit, RotationMatrixZeroAngleIsIdentity) {
  const Eigen::Matrix4d m = rotation_matrix_deg(0.0, 1.0, 0.0, 0.0);
  EXPECT_TRUE(approx_equal(m, Eigen::Matrix4d::Identity(), 1e-10));
}
