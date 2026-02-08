#pragma once

#include "render_global.h"

#include <QGLWidget>
#include "render_camera.h"
#include "render_context.h"
#include "render_rotation_tool.h"
#include "render_pivot.h"
#include "render_pan_tool.h"
#include "render_zoom_tool.h"

namespace insight{

namespace render
{
	class RenderContext;
	class RenderCamera;

	class  RenderWidget : public QGLWidget
	{
		Q_OBJECT
	public:
		explicit RenderWidget(QWidget *parent = NULL);
		~RenderWidget();

	public:
		RenderNode *root()
		{
			return m_root;
		}
		
		RenderNode *dataRoot()
		{
			return m_dataRoot;
		}

		void setPivotVisible(bool vis) { m_pivot->setVisible(vis); }

	protected:
		//virtual void paintEvent(QPaintEvent *event);
		virtual void initializeGL();
		virtual void paintGL();
		virtual void resizeGL(int w, int h);
		virtual void mouseReleaseEvent(QMouseEvent *event); //�õ�������Ҽ�
		virtual void mousePressEvent(QMouseEvent *event);	//�õ���������
		virtual void mouseMoveEvent(QMouseEvent *event);
		virtual void wheelEvent(QWheelEvent *event);

	private:
		void initScene();

		RenderCamera *m_camera;
		RenderContext *m_renderContext;
		RenderRotationTool *m_rotationTool;
		RenderPanTool *m_panTool;
		RenderZoomTool *m_zoomTool;
		// RenderTool *m_tool;
		RenderPivot *m_pivot;
		RenderNode *m_root;
		RenderNode *m_dataRoot;
		bool m_lockPan = false;
	};

} // namespace render

}//name space insight
