#include "render_zoom_tool.h"

#include <QWheelEvent>
#include <QGLWidget>

namespace insight{

namespace render
{
	void RenderZoomTool::wheelEvent(QWheelEvent *event)
	{
		double delta = event->delta();
		if (delta > 0)
			m_target->scale(1.1);
		else
			m_target->scale(0.9);

		renderContext()->widget->update();
	}

	RenderZoomTool::RenderZoomTool()
		: m_target(NULL)
	{
	}

	RenderZoomTool::~RenderZoomTool()
	{

	}

}

}//name space insight
