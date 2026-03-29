#include "render_pan_tool.h"

#include <QApplication>
#include <QDebug>
#include <QEvent>
#include <QGLWidget>
#include <QWheelEvent>
#include <cmath>

namespace insight {

namespace render {

RenderPanTool::RenderPanTool() : m_isMoving(false), m_lastCursor(Qt::ArrowCursor) {}

RenderPanTool::~RenderPanTool() {}

void RenderPanTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::RightButton || event->button() == Qt::MidButton) {
    m_isMoving = false;
    render_context()->widget->setCursor(m_lastCursor);
  }
}

void RenderPanTool::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::RightButton || event->button() == Qt::MidButton) {
    m_isMoving = true;
    m_movePos = event->pos();
    m_lastCursor = render_context()->widget->cursor();
  }
}

void RenderPanTool::mouseMoveEvent(QMouseEvent* event) {
  if (m_isMoving) {
    QCursor cusor(Qt::SizeAllCursor);
    render_context()->widget->setCursor(cusor);

    QPoint pt = event->pos();

    QPoint dis = m_movePos - pt;
    dis.setY(-dis.y());

    Vec3 v = render_context()->camera->v();
    Vec3 u = render_context()->camera->u();

    Vec3 camPos = render_context()->camera->position();
    float w = render_context()->w;
    float h = render_context()->h;
    double l, r, b, t, n, f;
    render_context()->camera->get_frustum(l, r, b, t, n, f);
    //???????????
    double dx = dis.x() / w * (r - l) / render_context()->camera->focus_length();
    double dy = dis.y() / h * (t - b) / render_context()->camera->focus_length();

    Vec3 zero(0, 0, 0);
    Vec3 targetPos = m_target->local_to_world(zero);
    targetPos = (render_context()->camera->view_matrix() *
                 Vec4(targetPos.x(), targetPos.y(), targetPos.z(), 1.0))
                    .head<3>();

    double targetX = targetPos.x() / targetPos.z();
    double targetY = targetPos.y() / targetPos.z();
    if (targetPos.z() > 0) {
      targetX -= dx;
      targetY -= dy;
    } else {
      targetX += dx;
      targetY += dy;
    }

    targetPos.x() = targetX * targetPos.z();
    targetPos.y() = targetY * targetPos.z();
    // std::cout << "targetPos.z " << targetPos.z << std::endl;
    targetPos = (render_context()->camera->model_matrix() *
                 Vec4(targetPos.x(), targetPos.y(), targetPos.z(), 1.0))
                    .head<3>();
    RenderNode* p = m_target->parent_node();
    if (p != NULL) {
      targetPos = p->world_to_local(targetPos);
    }
    m_target->set_position(targetPos);
    m_movePos = pt;
    render_context()->widget->update();
  }
}

void RenderPanTool2::wheelEvent(QWheelEvent* event) {
  double ratio = event->delta() > 0 ? 0.9 : 1.1;

  zoom(ratio, event->x(), event->y());
}

void RenderPanTool2::mouseReleaseEvent(QMouseEvent* event) {
  m_bIsMoving = false;
  render_context()->widget->setCursor(Qt::CrossCursor);
}

void RenderPanTool2::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::RightButton || event->button() == Qt::MidButton) {
    m_posX = event->x();
    m_posY = event->y();
    m_bIsMoving = true;
  }
}

void RenderPanTool2::mouseMoveEvent(QMouseEvent* event) {
  if (m_bIsMoving || event->button() == Qt::RightButton) {
    render_context()->widget->setCursor(Qt::SizeAllCursor);
    RenderCamera2d* camera = (RenderCamera2d*)render_context()->camera;
    //????????????????Y???????????
    Vec3 dis = Vec3(m_posX - event->x(), event->y() - m_posY, 0.0);
    double ratio = camera->scale_ratio();
    camera->translate(dis * ratio);
    render_context()->widget->update();
    m_posX = event->x();
    m_posY = event->y();

    int w = render_context()->w;
    int h = render_context()->h;

    double scale = camera->scale_ratio();
    double x = camera->position().x() - w / 2.0 * scale;
    double y = camera->position().y() - h / 2.0 * scale;
    double width = w * scale;
    double height = h * scale;

    emit view_changed(QRectF(x, y, width, height), scale);
  }
}

void RenderPanTool2::resizeEvent(QResizeEvent* event) {
  RenderCamera2d* camera = (RenderCamera2d*)render_context()->camera;

  int w = event->size().width();
  int h = event->size().height();
  render_context()->widget->makeCurrent();
  glViewport(0, 0, w, h);
  camera->scale_ortho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -10000, 10000);

  double scale = camera->scale_ratio();
  double x = camera->position().x() - w / 2.0 * scale;
  double y = camera->position().y() - h / 2.0 * scale;
  double width = w * scale;
  double height = h * scale;

  emit view_changed(QRectF(x, y, width, height), scale);
}

void RenderPanTool2::zoom_to_extent(double leftBottomX, double leftBottomY, double rightTopX,
                                    double rightTopY) {
  RenderCamera2d* camera = (RenderCamera2d*)render_context()->camera;
  double x = (leftBottomX + rightTopX) / 2.0;
  double y = (leftBottomY + rightTopY) / 2.0;

  // move to x,y
  double _x, _y, _z;
  camera->position(_x, _y, _z);
  camera->set_position(x, y, _z);

  int w = render_context()->widget->width();
  int h = render_context()->widget->height();

  double extentX = rightTopX - leftBottomX;
  double extentY = rightTopY - leftBottomY;
  double scale1 = extentX / w;
  double scale2 = extentY / h;
  double scale_ratio = std::max(scale1, scale2);

  camera->set_scale_ratio(scale_ratio);
  camera->scale_ortho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -1000, 1000);
  render_context()->widget->update();

  double scale = camera->scale_ratio();
  x = camera->position().x() - w / 2.0 * scale;
  y = camera->position().y() - h / 2.0 * scale;
  double width = w * scale;
  double height = h * scale;
  emit view_changed(QRectF(x, y, width, height), scale);
}

void RenderPanTool2::zoom_in() {
  int w = render_context()->w;
  int h = render_context()->h;

  zoom(0.9, w * 0.5, h * 0.5);
}

void RenderPanTool2::zoom_out() {
  int w = render_context()->w;
  int h = render_context()->h;
  zoom(1.1, w * 0.5, h * 0.5);
}

void RenderPanTool2::zoom(float ratio, float cx, float cy) {
  RenderCamera2d* camera = (RenderCamera2d*)render_context()->camera;
  double beforeScaleRatio = camera->scale_ratio();
  double afterScaleRatio = beforeScaleRatio * ratio;

  float realRatio = 1.0 / afterScaleRatio;
  if (realRatio > m_maxScaleRatio) {
    return;
  }
  int w = render_context()->w;
  int h = render_context()->h;

  camera->set_scale_ratio(afterScaleRatio);
  double transX = (cx - w / 2.0) * (afterScaleRatio - beforeScaleRatio);
  double transY = (h / 2.0 - cy) * (afterScaleRatio - beforeScaleRatio);

  camera->scale_ortho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -10000, 10000);
  camera->translate(-transX, -transY, 0);

  render_context()->widget->update();

  double scale = camera->scale_ratio();
  double x = camera->position().x() - w / 2.0 * scale;
  double y = camera->position().y() - h / 2.0 * scale;
  double width = w * scale;
  double height = h * scale;
  emit view_changed(QRectF(x, y, width, height), scale);
}

void RenderPanTool2::zoom(float ratio) {
  int w = render_context()->w;
  int h = render_context()->h;
  zoom(ratio, w * 0.5, h * 0.5);
}

void RenderPanTool2::resize_window(int w, int h) {
  RenderCamera2d* camera = (RenderCamera2d*)render_context()->camera;

  render_context()->widget->makeCurrent();
  glViewport(0, 0, w, h);
  camera->scale_ortho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -10000, 10000);

  double scale = camera->scale_ratio();
  double x = camera->position().x() - w / 2.0 * scale;
  double y = camera->position().y() - h / 2.0 * scale;
  double width = w * scale;
  double height = h * scale;

  emit view_changed(QRectF(x, y, width, height), scale);
}

} // namespace render

} // namespace insight
