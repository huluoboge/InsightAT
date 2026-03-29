#pragma once

/*
@author: Jones
@date: 2019-04-27
@descriptions:


*/
#include "render_camera.h"
#include "render_context.h"
#include "render_global.h"
#include "render_layer.h"
#include "render_pan_tool.h"

#include <QGLWidget>

namespace insight {

namespace render {
class RenderLayer;
class RENDER_EXPORT RenderMap2 : public QGLWidget {
  Q_OBJECT
signals:
  void world_mouse_moved(const QPointF& world_point);

public:
  RenderMap2(QWidget* parent = nullptr);
  ~RenderMap2();
  void init_scene();

  void add_layer(RenderLayer* layer) { m_layers.push_back(layer); }

  void zoom_to_extent(const QRectF& extent);

  QList<RenderLayer*>& layers() { return m_layers; }

public slots:
  void repaint_all_layers(const QRectF& cam_extent);
  void repaint();
  void clear_layers();

protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int w, int h);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);

protected:
  RenderContext* render_context_;
  RenderCamera2d* m_camera;
  RenderPanTool2* m_panTool;
  QList<RenderLayer*> m_layers;
  QRectF m_lastCamExtent;
};

} // namespace render

} // namespace insight
