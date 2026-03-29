#pragma once

#include "render_global.h"
#include "render_node.h"
#include "render_tool.h"

#include <QObject>

class QWheelEvent;

namespace insight {

namespace render {
class RENDER_EXPORT RenderZoomTool : public RenderTool {
  Q_OBJECT

public:
  explicit RenderZoomTool();
  virtual ~RenderZoomTool();

  RenderNode* target() { return m_target; }
  void set_target(RenderNode* obj) { m_target = obj; }

  virtual void wheelEvent(QWheelEvent* event);

private:
  RenderNode* m_target;
};
} // namespace render

} // namespace insight
