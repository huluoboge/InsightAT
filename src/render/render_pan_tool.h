#pragma once

#include <QObject>
#include <QPoint>
#include <QCursor>
#include <QRectF>
#include <set>
#include "render_global.h"

#include "render_camera.h"
#include "render_types.h"
#include "render_context.h"
#include "render_tool.h"

class QWheelEvent;
class QMouseEvent;
class QEvent;
class QResizeEvent;

namespace insight{

namespace render
{

	class  RenderPanTool : public RenderTool
	{
		Q_OBJECT
	public:
		explicit RenderPanTool();
		virtual ~RenderPanTool();

		void setTarget(RenderNode *node) { m_target = node; }
		virtual void mouseReleaseEvent(QMouseEvent * event);
		virtual void mousePressEvent(QMouseEvent * event);
		virtual void mouseMoveEvent(QMouseEvent * event);
	private:
		bool m_isMoving;
		QPoint m_movePos;
		QCursor m_lastCursor;

		RenderNode *m_target;
	};


	class  RenderPanTool2 : public RenderTool
	{
		Q_OBJECT
	signals:
		/**
		 * @brief viewChanged
		 * @param worldRect
		 * @param scaleRatio
		 */
		void viewChanged(const QRectF&worldRect, double scaleRatio);
	public:
		explicit RenderPanTool2(){}
		virtual ~RenderPanTool2(){}

		void zoomToExtent(double leftBottomX, double leftBottomY, double rightTopX, double rightTopY);
		float maxScaleRatio() const { return m_maxScaleRatio; }
		void setMaxScaleRatio(float ratio) { m_maxScaleRatio = ratio; }

		void zoomIn();
		void zoomOut();
		void zoom(float ratio, float x, float y);
		void zoom(float ratio);
		virtual void resizeWindow(int w, int h);
		virtual void wheelEvent(QWheelEvent *event);
		virtual void mouseReleaseEvent(QMouseEvent * event);
		virtual void mousePressEvent(QMouseEvent * event);
		virtual void mouseMoveEvent(QMouseEvent * event);
		virtual void resizeEvent(QResizeEvent *event);
		
	private:
		int m_posX;
		int m_posY;
		bool m_bIsMoving = false;
		float m_maxScaleRatio = 10000000;//���Ŵ����
	};
}

}//name space insight

