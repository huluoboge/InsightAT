#pragma once

#include "render_global.h"

#include "render_context.h"

#include <QObject>

namespace insight{

namespace render
{
	class  RenderTool : public QObject
	{
		Q_OBJECT
	public:
		RenderTool() {}
		virtual ~RenderTool() {}

		virtual void mouseReleaseEvent(QMouseEvent * event) {}
		virtual void mousePressEvent(QMouseEvent * event) {}
		virtual void mouseMoveEvent(QMouseEvent * event) {}
		virtual void wheelEvent(QWheelEvent *event){}
		virtual void resizeEvent(QResizeEvent *event){}
		void setRenderContext(RenderContext *rc) { m_renderContext = rc; }
		RenderContext *renderContext() { return m_renderContext; }
	protected:
		RenderContext *m_renderContext;
	};
}

}//name space insight
