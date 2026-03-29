#pragma once

#include "render_global.h"
#include <QCursor>
#include <QObject>
#include <QPoint>
#include <QRectF>
#include <set>

#include "render_camera.h"
#include "render_context.h"
#include "render_tool.h"
#include "render_types.h"

class QWheelEvent;
class QMouseEvent;
class QEvent;
class QResizeEvent;

namespace insight {

namespace render {

class RENDER_EXPORT RenderPanTool : public RenderTool {
  Q_OBJECT
public:
  explicit RenderPanTool();
  virtual ~RenderPanTool();

  void set_target(RenderNode* node) { m_target = node; }
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);

private:
  bool m_isMoving;
  QPoint m_movePos;
  QCursor m_lastCursor;

  RenderNode* m_target;
};

class RENDER_EXPORT RenderPanTool2 : public RenderTool {
  Q_OBJECT
signals:
  /**
   * @brief view_changed
   * @param worldRect
   * @param scaleRatio
   */
  void view_changed(const QRectF& worldRect, double scaleRatio);

public:
  explicit RenderPanTool2() {}
  virtual ~RenderPanTool2() {}

  void zoom_to_extent(double leftBottomX, double leftBottomY, double rightTopX, double rightTopY);
  float max_scale_ratio() const { return m_maxScaleRatio; }
  void set_max_scale_ratio(float ratio) { m_maxScaleRatio = ratio; }

  void zoom_in();
  void zoom_out();
  void zoom(float ratio, float x, float y);
  void zoom(float ratio);
  virtual void resize_window(int w, int h);
  virtual void wheelEvent(QWheelEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void resizeEvent(QResizeEvent* event);

private:
  int m_posX;
  int m_posY;
  bool m_bIsMoving = false;
  float m_maxScaleRatio = 10000000; //????????
};
} // namespace render

} // namespace insight
