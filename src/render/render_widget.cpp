#include "render_widget.h"

#include <QBrush>
#include <QColor>
#include <QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>

#include <iostream>

#include "render_camera.h"
#include "render_node.h"
#include "render_object.h"
#include "render_tracks.h"

#include <algorithm>
//#include "common/base_log.h"

namespace insight {

namespace render {
RenderWidget::RenderWidget(QWidget* parent)
    : QGLWidget(parent), m_camera(NULL), render_context_(NULL) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  init_scene();
}

RenderWidget::~RenderWidget() {
  makeCurrent();
  delete m_camera;
  delete render_context_;
  delete m_rotationTool;
}

void RenderWidget::init_scene() {
  m_camera = new RenderCamera;
  render_context_ = new RenderContext;
  render_context_->camera = m_camera;
  render_context_->widget = this;
  m_pivot = new RenderPivot;
  m_root = new RenderNode;
  data_root_ = new RenderNode(m_root);
  m_rotationTool = new RenderRotationTool;
  m_rotationTool->set_render_context(render_context_);
  m_rotationTool->set_cursor(QCursor(Qt::DragMoveCursor));
  m_rotationTool->set_target(m_root);
  m_rotationTool->set_pivot(m_pivot);

  m_panTool = new RenderPanTool;
  m_panTool->set_target(data_root_);
  m_panTool->set_render_context(render_context_);

  m_zoomTool = new RenderZoomTool;
  m_zoomTool->set_target(m_root);
  m_zoomTool->set_render_context(render_context_);
}

void RenderWidget::initializeGL() {
  GLenum code = glewInit();
  if (GLEW_OK != (int)code) {
    LOG(ERROR) << "Init glew failed, error information: " << glewGetErrorString(code);
    return;
  }
  glClearColor(0, 0, 0, 1.0);
  glClearDepth(1.0);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_SMOOTH);
  glHint(GL_POINT_SMOOTH, GL_NICEST);
  glHint(GL_LINE_SMOOTH, GL_NICEST);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  Vec3 camInitPos = Vec3(0, 0, 1000);

  m_camera->look_at(camInitPos, Vec3(0, 0, 0), Vec3(0, 1, 0));
}

void RenderWidget::paintGL() {
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  m_camera->update_gl_matrix();
  render_context_->clear();
  render_context_->modelview = m_camera->view_matrix();
  if (m_pivot->isVisible()) {
    m_pivot->draw(render_context_);
  }
  m_root->render(render_context_);
}

void RenderWidget::resizeGL(int w, int h) {
  m_camera->frustum(-double(w) / h, double(w) / h, -1, 1, 5, 100000);
  glViewport(0, 0, w, h);
  render_context_->w = w;
  render_context_->h = h;
}

void RenderWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (!lock_pan_) {
    m_rotationTool->mouseReleaseEvent(event);
    m_panTool->mouseReleaseEvent(event);
  }
  // if (m_tool != nullptr) {
  //     m_tool->mouseReleaseEvent(event);

  // }
  update();
}

void RenderWidget::mousePressEvent(QMouseEvent* event) {
  if (!lock_pan_) {
    m_rotationTool->mousePressEvent(event);
    m_panTool->mousePressEvent(event);
  }
  // if (m_tool != nullptr) {
  //     m_tool->mousePressEvent(event);
  // }
  update();
}

void RenderWidget::mouseMoveEvent(QMouseEvent* event) {
  if (!lock_pan_) {
    m_rotationTool->mouseMoveEvent(event);
    m_panTool->mouseMoveEvent(event);
  }
  // if (m_tool != nullptr) {
  //     m_tool->mouseMoveEvent(event);
  // }
  update();
}

void RenderWidget::wheelEvent(QWheelEvent* event) {
  if (!lock_pan_) {
    m_zoomTool->wheelEvent(event);
  }
  // if (m_tool != nullptr) {
  //     m_tool->wheelEvent(event);
  // }
  update();
}

bool RenderWidget::unproject_ray(int px, int py, Vec3* ray_origin, Vec3* ray_dir) const {
  // Delegate to RenderTracks which captures GL state during draw().
  for (RenderObject* o : data_root_->render_objects()) {
    auto* tracks = dynamic_cast<RenderTracks*>(o);
    if (tracks)
      return tracks->unproject_ray(px, py, ray_origin, ray_dir);
  }
  return false;
}

void RenderWidget::fit_scene_to_view() {
  makeCurrent();
  const int w = std::max(1, width());
  const int h = std::max(1, height());
  resizeGL(w, h);

  m_root->identity_all();

  RenderTracks* tracks = nullptr;
  for (RenderObject* o : data_root_->render_objects()) {
    tracks = dynamic_cast<RenderTracks*>(o);
    if (tracks)
      break;
  }

  double left = 0, right = 0, bottom = 0, top = 0, near_plane = 0, far_plane = 0;
  m_camera->get_frustum(left, right, bottom, top, near_plane, far_plane);
  const double tan_half_y = ((top - bottom) * 0.5) / near_plane;
  const double tan_half_x = ((right - left) * 0.5) / near_plane;

  auto default_camera = [&]() {
    m_camera->look_at(Vec3(0, 0, 1000), Vec3(0, 0, 0), Vec3(0, 1, 0));
  };

  if (!tracks) {
    default_camera();
    update();
    return;
  }

  Vec3 bmin, bmax;
  if (!tracks->world_axis_aligned_bounds(&bmin, &bmax)) {
    default_camera();
    update();
    return;
  }

  const Vec3 center = (bmin + bmax) * 0.5;
  double R = 0.5 * (bmax - bmin).norm();
  R = std::max(R, 1.0);

  constexpr double kMargin = 1.25;
  const double dist =
      std::max(kMargin * R / tan_half_x, kMargin * R / tan_half_y);
  const double dist_clamped = std::max(dist, 10.0);

  const Vec3 eye = center + Vec3(0, 0, dist_clamped);
  m_camera->look_at(eye, center, Vec3(0, 1, 0));
  update();
}

} // namespace render

} // namespace insight
