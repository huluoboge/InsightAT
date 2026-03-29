
#include "render_rotation_tool.h"

#include <QBitmap>
#include <QCursor>
#include <QDebug>
#include <QEvent>
#include <QGLWidget>
#include <QMouseEvent>
#include <iostream>

#include "opengl_mat.h"
#include "render_camera.h"
#include "render_context.h"

namespace insight {

namespace render {
RenderRotationTool::RenderRotationTool()
    : m_isRotating(false), m_lastCursor(Qt::ArrowCursor), m_rotateCursor(Qt::ArrowCursor) {}

RenderRotationTool::~RenderRotationTool() {}

void RenderRotationTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    m_isRotating = false;
    render_context()->widget->setCursor(m_lastCursor);
    render_context()->widget->update();
  }
}

void RenderRotationTool::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    m_rotatePos = event->pos();
    m_isRotating = true;
    bool exceed = false;
    m_lastMouseOrientation = convert_mouse_position_to_orientation(event->x(), event->y());
  }
}

void RenderRotationTool::mouseMoveEvent(QMouseEvent* event) {
  if (m_isRotating) {
    render_context()->widget->setCursor(m_rotateCursor);
    render_context()->widget->update();
    // m_pivot->show();
    if (event->x() > render_context()->w - 2 || event->y() > render_context()->h - 2 ||
        event->x() < 0 || event->y() < 0) {
      // qDebug() <<"x y exceed view extent "<< event->x() << " " << event->y();
      return;
    }
    bool exceed = false;
    m_currentMouseOrientation = convert_mouse_position_to_orientation(event->x(), event->y());
    if (m_currentMouseOrientation == m_lastMouseOrientation) {
      // qDebug() << "find same";
      return;
    }
    Mat4 rotateMat =
        generate_gl_rotation_matrix_from_vectors(m_lastMouseOrientation, m_currentMouseOrientation);
    m_lastMouseOrientation = m_currentMouseOrientation;
    if (exceed) {
      return;
    }
    //??pivots???????????targetobject?????????
    std::vector<RenderNode*> parents = m_target->parent_nodes();
    Mat4 mat = Mat4::Identity();
    for (RenderNode* o : parents) {
      mat *= o->ref_model_matrix();
    }
    Mat4 matObj = mat * m_target->ref_model_matrix();
    Mat4 invMat = matObj.inverse();

    //???X????
    // A1A2A3...AX = R(A1A2A3...A) -> X=(A1A2A3...)-1 R(A1A2A3...A)
    Mat4 xMat = invMat * rotateMat * matObj;
    m_target->ref_model_matrix() = m_target->ref_model_matrix() * xMat;
    //?????????????
    mat *= m_target->ref_model_matrix();

    m_pivot->update_matrix(mat);
    render_context()->widget->update();
  }
}

Vec3 RenderRotationTool::convert_mouse_position_to_orientation(int x, int y) {
  render_context()->widget->makeCurrent();
  GLdouble modelview[16];
  GLdouble projection[16];
  GLint viewport[4];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);

  Mat4 mat = Eigen::Map<Mat4>(modelview);
  Vec4 pos = mat.col(3);
  GLdouble xp, yp, zp;
  gluProject(pos.x(), pos.y(), pos.z(), modelview, projection, viewport, &xp, &yp, &zp);

  // invert y
  y = render_context()->h - 1 - y;

  Vec3 v(x - xp, y - yp, 0);

  double h = render_context()->h;
  double w = render_context()->w;

  // 		xp = std::min<GLdouble>(xp, 3 * w / 4);
  // 		xp = std::max<GLdouble>(xp, w / 4);
  //
  // 		yp = std::min<GLdouble>(yp, 3 * h / 4);
  // 		yp = std::max<GLdouble>(yp, h / 4);

  // double r = 0.75 * std::min(w, h) / 2.0;
  double r = std::max(w, h) / 2.0;
  double d1 = sqrt(v.x() * v.x() + v.y() * v.y());

  // double r =  std::max(w, h) / 2.0;
  // double xc = static_cast<double>(w / 2 );
  // double yc = static_cast<double>(h / 2 );

  v.x() /= r;
  v.y() /= r;
  // v.x() = std::max<double>(std::min<double>(v.x() / r, 1), -1);
  // v.y() = std::max<double>(std::min<double>(v.y() / r, 1), -1);

  // 	if (m_verticalRotationLocked || m_bubbleViewModeEnabled)
  // 	{
  // 		v.y = 0;
  // 	}

  // square 'radius'
  double d2 = v.x() * v.x() + v.y() * v.y();

  // projection on the unit sphere
  if (d2 > 1) {
    double d = sqrt(d2);
    v.x() /= d;
    v.y() /= d;
  } else {
    v.z() = sqrt(1.0 - d2);
  }

  if (d1 > r) {
    v.z() *= -1;
  }
  // qDebug() << v.x << " " << v.y << " " << v.z;
  return v;
}

Mat4 RenderRotationTool::generate_gl_rotation_matrix_from_vectors(const Vec3& sourceVec,
                                                                  const Vec3& destVec) {
  // we compute scalar prod between the two vectors
  double ps = sourceVec.dot(destVec) / sourceVec.norm() * destVec.norm();

  //	double ps = dot(sourceVec, destVec) / length(sourceVec) * length(destVec);

  // we bound result (in case vecors are not exactly unit)
  if (ps > 1.0)
    ps = 1.0;
  else if (ps < -1.0)
    ps = -1.0;

  // we deduce angle from scalar prod
  double angle_deg = R2D(acos(ps));

  // we compute rotation axis with scalar prod

  Vec3 axis = sourceVec.cross(destVec);

  // we eventually compute the rotation matrix with axis and angle
  return rotation_matrix_deg(angle_deg, axis);
}

} // namespace render

} // namespace insight
