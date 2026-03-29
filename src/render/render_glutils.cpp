/**
 * @file  render_glutils.cpp
 * @brief GLUtils: gluUnProject, gluProjectEx, and max texture size query.
 */

#include "render_glutils.h"

#include "render_global.h"

namespace insight {

namespace render {

Eigen::Vector3f GLUtils::screen_to_world(float screen_x, float screen_y, float deep /* 0--1 */) {
  GLdouble model_view[16];
  GLint viewport[4];
  GLdouble projection[16];
  glMatrixMode(GL_MODELVIEW);
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);

  GLdouble winx, winy, winz;
  GLdouble objx, objy, objz;
  winx = screen_x;
  winy = screen_y;
  winz = deep;
  gluUnProject(winx, winy, winz, model_view, projection, viewport, &objx, &objy, &objz);
  return Eigen::Vector3f(static_cast<float>(objx), static_cast<float>(objy),
                         static_cast<float>(objz));
}

/*
 * Transform a point (column vector) by a 4x4 matrix: out = m * in.
 * Input: m -- 4x4 matrix; in -- 4x1 vector.
 * Output: out -- resulting 4x1 vector.
 */
static void transform_point(double out[4], const double m[16], const double in[4]) {
#define M(row, col) m[col * 4 + row]
  out[0] = M(0, 0) * in[0] + M(0, 1) * in[1] + M(0, 2) * in[2] + M(0, 3) * in[3];
  out[1] = M(1, 0) * in[0] + M(1, 1) * in[1] + M(1, 2) * in[2] + M(1, 3) * in[3];
  out[2] = M(2, 0) * in[0] + M(2, 1) * in[1] + M(2, 2) * in[2] + M(2, 3) * in[3];
  out[3] = M(3, 0) * in[0] + M(3, 1) * in[1] + M(3, 2) * in[2] + M(3, 3) * in[3];
#undef M
}

GLint gluProjectEx(double objx, double objy, double objz, const double model[16],
                   const double proj[16], const GLint viewport[4], double* winx, double* winy,
                   double* winz) {
  double obj_coor[4];
  double obj_proj[4], obj_model[4];

  obj_coor[0] = objx;
  obj_coor[1] = objy;
  obj_coor[2] = objz;
  obj_coor[3] = 1.0;

  transform_point(obj_model, model, obj_coor);
  transform_point(obj_proj, proj, obj_model);

  if (obj_proj[3] == 0.0)
    return GL_FALSE;

  obj_proj[0] /= obj_proj[3];
  obj_proj[1] /= obj_proj[3];
  obj_proj[2] /= obj_proj[3];

  // NDC [-1,1] to [0,1], then scale by viewport and add offset.
#define SCALE_FROM_0_TO_1(_pt) (((_pt) + 1) / 2)
  obj_proj[0] = SCALE_FROM_0_TO_1(obj_proj[0]);
  obj_proj[1] = SCALE_FROM_0_TO_1(obj_proj[1]);
  obj_proj[2] = SCALE_FROM_0_TO_1(obj_proj[2]);
#undef SCALE_FROM_0_TO_1

  *winx = viewport[0] + obj_proj[0] * viewport[2];
  *winy = viewport[1] + obj_proj[1] * viewport[3];
  *winz = obj_proj[2];
  return GL_TRUE;
}

Eigen::Vector3d GLUtils::screen_to_world(Eigen::Matrix<double, 4, 4> model_view, float screen_x,
                                         float screen_y, float deep /* 0--1 */) {
  GLdouble* mv = model_view.data();
  GLint viewport[4];
  GLdouble projection[16];
  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);

  GLdouble x = mv[3];
  GLdouble y = mv[7];
  GLdouble z = mv[11];
  mv[3] = 0;
  mv[7] = 0;
  mv[11] = 0;
  GLdouble winx, winy, winz;
  GLdouble objx, objy, objz;
  winx = screen_x;
  winy = screen_y;
  winz = deep;
  gluUnProject(winx, winy, winz, mv, projection, viewport, &objx, &objy, &objz);
  objx -= x;
  objy -= y;
  objz -= z;
  return Eigen::Vector3d(objx, objy, objz);
}

int GLUtils::max_texture_size() {
  GLint max_texture = -1;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture);
  return max_texture;
}

} // namespace render

} // namespace insight
