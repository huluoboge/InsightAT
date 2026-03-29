#include "render_map2.h"

#include <QBrush>
#include <QColor>
#include <QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>

#include <iostream>

#include "render_camera.h"
#include "render_global.h"
#include "render_glutils.h"
#include "render_node.h"
#include "render_object.h"

//#include "common/base_log.h"

namespace insight {

namespace render {

RenderMap2::RenderMap2(QWidget* parent) : QGLWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  init_scene();
}

RenderMap2::~RenderMap2() {
  // makeCurrent();
  // for (int i = 0; i < m_layers.size(); ++i) {
  // 	// delete m_layers[i];
  // }
  // m_layers.clear();
  delete m_camera;
  delete render_context_;
  delete m_panTool;
}

void RenderMap2::init_scene() {
  m_camera = new RenderCamera2d;
  render_context_ = new RenderContext;
  render_context_->camera = m_camera;
  render_context_->widget = this;
  m_panTool = new RenderPanTool2;
  m_panTool->set_render_context(render_context_);
  connect(m_panTool, SIGNAL(view_changed(const QRectF&, double)), this,
          SLOT(repaint_all_layers(const QRectF&)));
  // m_mapOrigin.setX(0);
  // m_mapOrigin.setY(0);
}

void RenderMap2::initializeGL() {
  GLenum code = glewInit();
  if (GLEW_OK != (int)code) {
    LOG(ERROR) << "Init glew failed, error information: " << glewGetErrorString(code);
    return;
  }
  glClearColor(0, 0, 0, 1.0);
  // glClearDepth(1.0);
  // glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_SMOOTH);
  glHint(GL_POINT_SMOOTH, GL_NICEST);
  glHint(GL_LINE_SMOOTH, GL_NICEST);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  Vec3 camInitPos = Vec3(0, 0, 1);
  m_camera->look_at(camInitPos, Vec3(0, 0, 0), Vec3(0, 1, 0));
}

void RenderMap2::paintGL() {
  makeCurrent();
  // glEnable(GL_DEPTH_TEST);// �����ڻ���֮ǰָ������������Ч
  // glDepthFunc(GL_LESS);
  glClear(GL_COLOR_BUFFER_BIT);
  ////glMatrixMode(GL_PROJECTION);
  glLoadMatrixd(m_camera->ref_project_matrix().data());
  m_camera->update_gl_matrix();

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  render_context_->clear();
  render_context_->modelview = m_camera->view_matrix();

  glLoadMatrixd(render_context_->modelview.data());
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  for (int i = 0; i < m_layers.size(); ++i) {
    m_layers[i]->render(render_context_);
  }
  glPopAttrib();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  // glMatrixMode(GL_MODELVIEW);
  // glLoadMatrixd(view_matrix().data());
  // 		m_camera->update_gl_matrix();
  // 		glMatrixMode(GL_MODELVIEW);
  // glTranslatef(m_mapOrigin.x(), m_mapOrigin.y(), 0.f);
  // 		glBegin(GL_LINE_LOOP);
  // 		glVertex3d(0, 0, 0);
  // 		glVertex3d(0, 100, 0);
  // 		glVertex3d(100, 100, 0);
  // 		glVertex3d(100, 0, 0);
  // 		glEnd();
  // 		for (int i = 0; i < m_layers.size(); ++i)
  // 		{
  // 			glMatrixMode(GL_MODELVIEW);
  // 			glPushMatrix();
  // 			glPushAttrib(GL_ALL_ATTRIB_BITS);
  // 			m_layers[i]->render(render_context_);
  // 			glPopAttrib();
  // 			glMatrixMode(GL_MODELVIEW);
  // 			glPopMatrix();
  // 		}
}

void RenderMap2::resizeGL(int w, int h) {
  render_context_->w = w;
  render_context_->h = h;
  glViewport(0, 0, w, h);
  m_panTool->resize_window(w, h);
}

void RenderMap2::mouseReleaseEvent(QMouseEvent* event) { m_panTool->mouseReleaseEvent(event); }

void RenderMap2::mousePressEvent(QMouseEvent* event) { m_panTool->mousePressEvent(event); }

void RenderMap2::mouseMoveEvent(QMouseEvent* event) {
  m_panTool->mouseMoveEvent(event);
  makeCurrent();
  render_context_->camera->update_gl_matrix();
  float y = render_context_->h - event->y();
  float x = event->x();
  Eigen::Vector3d pt3d =
      GLUtils::screen_to_world(render_context_->camera->view_matrix(), x, y, 0.f);
  QPointF pt(pt3d.x(), pt3d.y());
  emit world_mouse_moved(pt);
}

void RenderMap2::wheelEvent(QWheelEvent* event) { m_panTool->wheelEvent(event); }

void RenderMap2::repaint_all_layers(const QRectF& camExtent) {
  // printf("repaint\n");
  m_lastCamExtent = camExtent;

  double x = camExtent.x();
  double y = camExtent.y();
  double w = camExtent.width();
  double h = camExtent.height();
  QRectF worldExtent(x, y, w, h);
  for (int i = 0; i < m_layers.size(); ++i) {
    m_layers[i]->repaint(render_context_, worldExtent);
  }
  update();
}

void RenderMap2::zoom_to_extent(const QRectF& extent) {
  if (!extent.isNull()) {
    m_panTool->zoom_to_extent(extent.x(), extent.y(), extent.x() + extent.width(),
                              extent.y() + extent.height());
  }
  update();
}

void RenderMap2::repaint() { repaint_all_layers(m_lastCamExtent); }

void RenderMap2::clear_layers() {
  for (int i = 0; i < m_layers.size(); ++i) {
    delete m_layers[i];
  }
  m_layers.clear();
}

} // namespace render

} // namespace insight
