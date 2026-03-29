#pragma once

#include "render_global.h"

#include "render_camera.h"
#include "render_context.h"
#include "render_pan_tool.h"
#include "render_pivot.h"
#include "render_rotation_tool.h"
#include "render_zoom_tool.h"
#include <QGLWidget>

namespace insight {

namespace render {
class RenderContext;
class RenderCamera;

class RENDER_EXPORT RenderWidget : public QGLWidget {
  Q_OBJECT
public:
  explicit RenderWidget(QWidget* parent = NULL);
  ~RenderWidget();

public:
  RenderNode* root() { return m_root; }

  RenderNode* data_root() { return data_root_; }

  void set_pivot_visible(bool vis) { m_pivot->setVisible(vis); }

  /// 重置场景节点变换，并将相机对准当前数据包围盒（Bundler / 点云等）。
  void fit_scene_to_view();

protected:
  // virtual void paintEvent(QPaintEvent *event);
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int w, int h);
  virtual void mouseReleaseEvent(QMouseEvent* event); //�õ�������Ҽ�
  virtual void mousePressEvent(QMouseEvent* event);   //�õ���������
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);

private:
  void init_scene();

  RenderCamera* m_camera;
  RenderContext* render_context_;
  RenderRotationTool* m_rotationTool;
  RenderPanTool* m_panTool;
  RenderZoomTool* m_zoomTool;
  // RenderTool *m_tool;
  RenderPivot* m_pivot;
  RenderNode* m_root;
  RenderNode* data_root_;
  bool lock_pan_ = false;
};

} // namespace render

} // namespace insight
