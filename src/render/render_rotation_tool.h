#ifndef WRotateObjectTool_h__
#define WRotateObjectTool_h__

#include "render_camera.h"
#include "render_global.h"
#include "render_pivot.h"
#include "render_tool.h"
#include "render_types.h"
#include <QCursor>
#include <QObject>
#include <QPoint>

class QWheelEvent;
class QMouseEvent;
class QEvent;

namespace insight {

namespace render {
class RENDER_EXPORT RenderRotationTool : public RenderTool {
  Q_OBJECT
public:
  explicit RenderRotationTool();
  virtual ~RenderRotationTool();

  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  void set_cursor(const QCursor& cursor) { m_rotateCursor = cursor; }

  void set_target(RenderNode* node) { m_target = node; }

  void set_pivot(RenderPivot* pivot) { m_pivot = pivot; }

private:
  Vec3 convert_mouse_position_to_orientation(int x, int y);
  Mat4 generate_gl_rotation_matrix_from_vectors(const Vec3& sourceVec, const Vec3& destVec);

private:
  bool m_isRotating;
  QPoint m_rotatePos;
  QPoint m_movePos;
  QCursor m_lastCursor;
  QCursor m_rotateCursor;
  Vec3 m_currentMouseOrientation;
  Vec3 m_lastMouseOrientation;
  RenderNode* m_target;
  RenderPivot* m_pivot;
};
} // namespace render

} // namespace insight

// class WCube;

#endif // WRotateObjectTool_h__
