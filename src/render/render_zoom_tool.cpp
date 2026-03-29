#include "render_zoom_tool.h"

#include <QGLWidget>
#include <QWheelEvent>

namespace insight {

namespace render {
void RenderZoomTool::wheelEvent(QWheelEvent* event) {
  double delta = event->delta();
  if (delta > 0)
    m_target->scale(1.1);
  else
    m_target->scale(0.9);

  render_context()->widget->update();
}

RenderZoomTool::RenderZoomTool() : m_target(NULL) {}

RenderZoomTool::~RenderZoomTool() {}

} // namespace render

} // namespace insight
