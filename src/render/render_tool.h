#pragma once

#include "render_global.h"

#include "render_context.h"

#include <QObject>

namespace insight {

namespace render {
class RENDER_EXPORT RenderTool : public QObject {
  Q_OBJECT
public:
  RenderTool() {}
  virtual ~RenderTool() {}

  virtual void mouseReleaseEvent(QMouseEvent* event) {}
  virtual void mousePressEvent(QMouseEvent* event) {}
  virtual void mouseMoveEvent(QMouseEvent* event) {}
  virtual void wheelEvent(QWheelEvent* event) {}
  virtual void resizeEvent(QResizeEvent* event) {}
  void set_render_context(RenderContext* rc) { render_context_ = rc; }
  RenderContext* render_context() { return render_context_; }

protected:
  RenderContext* render_context_;
};
} // namespace render

} // namespace insight
