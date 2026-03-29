#include "render_camera.h"

namespace insight {

namespace render {

RenderCamera::RenderCamera() { frustum_matrix_ = Mat4::Identity(); }

RenderCamera::~RenderCamera() = default;

void RenderCamera::look_at(const Vec3& eye, const Vec3& target, const Vec3& up) {
  const Mat4 mat = look_at_matrix(eye, target, up);
  ref_model_matrix() = fast_inverse(mat);
}

void RenderCamera::update_gl_matrix() {
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixd(frustum_matrix_.data());

  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixd(view_matrix().data());
}

void RenderCamera::frustum(double left, double right, double bottom, double top, double near_plane,
                           double far_plane) {
  left_ = left;
  right_ = right;
  bottom_ = bottom;
  top_ = top;
  near_plane_ = near_plane;
  far_plane_ = far_plane;
  ref_project_matrix() = frustum_matrix(left, right, bottom, top, near_plane, far_plane);
}

void RenderCamera::get_frustum(double& left, double& right, double& bottom, double& top,
                               double& near_plane, double& far_plane) {
  left = left_;
  right = right_;
  bottom = bottom_;
  top = top_;
  near_plane = near_plane_;
  far_plane = far_plane_;
}

void RenderCamera::render(RenderContext* rc) {
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  draw(rc);
  for (int i = 0; i < (int)child_nodes_.size(); ++i) {
    child_nodes_[i]->render(rc);
  }
  glPopAttrib();
}

void RenderCamera2d::ortho(double left, double right, double bottom, double top, double near_plane,
                           double far_plane) {
  left_ = left;
  right_ = right;
  bottom_ = bottom;
  top_ = top;
  near_plane_ = near_plane;
  far_plane_ = far_plane;
  ref_project_matrix() = ortho_matrix(left, right, bottom, top, near_plane, far_plane);
}

void RenderCamera2d::scale_ortho(double left, double right, double bottom, double top,
                                 double near_plane, double far_plane) {
  left_ = left * scale_ratio_;
  right_ = right * scale_ratio_;
  bottom_ = bottom * scale_ratio_;
  top_ = top * scale_ratio_;
  near_plane_ = near_plane;
  far_plane_ = far_plane;
  ref_project_matrix() = ortho_matrix(left_, right_, bottom_, top_, near_plane_, far_plane_);
}

void RenderCamera2d::get_ortho(double& left, double& right, double& bottom, double& top,
                               double& near_plane, double& far_plane) {
  left = left_;
  right = right_;
  bottom = bottom_;
  top = top_;
  near_plane = near_plane_;
  far_plane = far_plane_;
}

} // namespace render

} // namespace insight
