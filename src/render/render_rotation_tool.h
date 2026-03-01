#ifndef WRotateObjectTool_h__
#define WRotateObjectTool_h__

#include <QObject>
#include <QPoint>
#include <QCursor>
#include "render_camera.h"
#include "render_global.h"
#include "render_tool.h"
#include "render_types.h"
#include "render_pivot.h"

class QWheelEvent;
class QMouseEvent;
class QEvent;

namespace insight{

namespace render
{
	class  RenderRotationTool : public RenderTool
	{
		Q_OBJECT
	public:
		explicit RenderRotationTool();
		virtual ~RenderRotationTool();
		
		virtual void mouseReleaseEvent(QMouseEvent * event);
		virtual void mousePressEvent(QMouseEvent * event);
		virtual void mouseMoveEvent(QMouseEvent * event);
		void setCusor(const QCursor &cursor) { m_rotateCursor = cursor; }

		void setTarget(RenderNode *node) {
			m_target = node;
		}

		void setPivot(RenderPivot *pivot) { m_pivot = pivot; }
	private:
		Vec3 convertMousePositionToOrientation(int x, int y);
		Mat4 generateGLRotationMatrixFromVectors(const Vec3& sourceVec, const Vec3& destVec);
	private:
		bool m_isRotating;
		QPoint m_rotatePos;
		QPoint m_movePos;
		QCursor m_lastCursor;
		QCursor m_rotateCursor;
		Vec3 m_currentMouseOrientation;
		Vec3 m_lastMouseOrientation;
		RenderNode *m_target;
		RenderPivot *m_pivot;
	};
}

}//name space insight

//class WCube;


#endif // WRotateObjectTool_h__
