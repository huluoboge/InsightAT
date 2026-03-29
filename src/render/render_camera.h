/**
 * @file  render_camera.h
 * @brief ??????????????? OpenGL ???????????
 */

#pragma once

#include "render_global.h"

#include "opengl_mat.h"
#include "render_node.h"
#include "render_pivot.h"
#include "render_types.h"

namespace insight {

namespace render {

class RENDER_EXPORT RenderCamera : public RenderNode {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RenderCamera();
  ~RenderCamera() override;

  /// ???????????????
  inline Mat4 view_matrix() const { return fast_inverse(ref_model_matrix()); }

  void update_gl_matrix();

  inline Vec3 u() const { return view_matrix().row(0).head<3>(); }
  inline Vec3 v() const { return view_matrix().row(1).head<3>(); }
  inline Vec3 n() const { return view_matrix().row(2).head<3>(); }

  inline Mat4 project_matrix() const { return frustum_matrix_; }

  inline Mat4& ref_project_matrix() { return frustum_matrix_; }

  inline const Mat4& ref_project_matrix() const { return frustum_matrix_; }

  /**
   * ???????eye / target / up?????????????????
   */
  void look_at(const Vec3& eye, const Vec3& target, const Vec3& up);

  void move(const Vec3& v) {
    Mat4 mat = view_matrix();
    mat(0, 3) -= v.x();
    mat(1, 3) -= v.y();
    mat(2, 3) -= v.z();
  }

  void frustum(double left, double right, double bottom, double top, double near_plane,
               double far_plane);

  void get_frustum(double& left, double& right, double& bottom, double& top, double& near_plane,
                   double& far_plane);

  double focus_length() { return near_plane_; }

  void render(RenderContext* rc);

protected:
  Mat4 frustum_matrix_;
  double left_;
  double right_;
  double bottom_;
  double top_;
  double near_plane_;
  double far_plane_;
};

class RENDER_EXPORT RenderCamera2d : public RenderCamera {
public:
  RenderCamera2d() = default;
  ~RenderCamera2d() override = default;

  void ortho(double left, double right, double bottom, double top, double near_plane,
             double far_plane);
  void get_ortho(double& left, double& right, double& bottom, double& top, double& near_plane,
                 double& far_plane);

  void scale_ortho(double left, double right, double bottom, double top, double near_plane,
                   double far_plane);

  double scale_ratio() const { return scale_ratio_; }
  void set_scale_ratio(double ratio) { scale_ratio_ = ratio; }

  void zoom(double ratio) { scale_ratio_ *= ratio; }

protected:
  double scale_ratio_ = 1.0;
};

} // namespace render

} // namespace insight
