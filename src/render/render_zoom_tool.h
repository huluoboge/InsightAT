#pragma once

#include "render_global.h"
#include "render_tool.h"
#include "render_node.h"

#include <QObject>

class QWheelEvent;

namespace insight{

namespace render
{
	class  RenderZoomTool : public RenderTool
	{
		Q_OBJECT

	public:
		explicit RenderZoomTool();
		virtual ~RenderZoomTool();

		RenderNode *target() { return m_target; }
		void setTarget(RenderNode *obj) { m_target = obj; }

		virtual void wheelEvent(QWheelEvent *event);
	private:
		RenderNode *m_target;
	};
}

}//name space insight

