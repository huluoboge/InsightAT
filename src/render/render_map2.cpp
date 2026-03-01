#include "render_map2.h"

#include <QPainter>
#include <QColor>
#include <QBrush>
#include <QMouseEvent>
#include <QPixmap>
#include <QDebug>

#include <iostream>

#include "render_global.h"
#include "render_camera.h"
#include "render_object.h"
#include "render_node.h"
#include "render_glutils.h"

//#include "common/base_log.h"

namespace insight{

namespace render
{

	RenderMap2::RenderMap2(QWidget *parent)
		:QGLWidget(parent)
	{
		setFocusPolicy(Qt::StrongFocus);
		setMouseTracking(true);
		initScene();
	}

	RenderMap2::~RenderMap2()
	{
		// makeCurrent();
		// for (int i = 0; i < m_layers.size(); ++i) {
		// 	// delete m_layers[i];
		// }
		// m_layers.clear();
		delete m_camera;
		delete m_renderContext;
		delete m_panTool;
	}

	void RenderMap2::initScene()
	{
		m_camera = new RenderCamera2d;
		m_renderContext = new RenderContext;
		m_renderContext->camera = m_camera;
		m_renderContext->widget = this;
		m_panTool = new RenderPanTool2;
		m_panTool->setRenderContext(m_renderContext);
		connect(m_panTool, SIGNAL(viewChanged(const QRectF&, double)), this, SLOT(repaintAllLayers(const QRectF&)));
		//m_mapOrigin.setX(0);
		//m_mapOrigin.setY(0);
	}

	void RenderMap2::initializeGL()
	{
		GLenum code = glewInit();
		if (GLEW_OK != (int)code)
		{
			LOG(ERROR) << "Init glew failed, error information: " << glewGetErrorString(code);
			return;
		}
		glClearColor(0, 0, 0, 1.0);
		//glClearDepth(1.0);
		//glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glEnable(GL_SMOOTH);
		glHint(GL_POINT_SMOOTH, GL_NICEST);
		glHint(GL_LINE_SMOOTH, GL_NICEST);
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_LINE_SMOOTH);
		Vec3 camInitPos = Vec3(0, 0, 1);
		m_camera->lookAt(camInitPos, Vec3(0, 0, 0), Vec3(0, 1, 0));
	}

	void RenderMap2::paintGL()
	{
		makeCurrent();
		//glEnable(GL_DEPTH_TEST);// �����ڻ���֮ǰָ������������Ч
		//glDepthFunc(GL_LESS);
		glClear(GL_COLOR_BUFFER_BIT);
		////glMatrixMode(GL_PROJECTION);
		glLoadMatrixd(m_camera->refProjectMatrix().data());
		m_camera->updateGLMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		m_renderContext->clear();
		m_renderContext->modelview = m_camera->viewMatrix();
		
		glLoadMatrixd(m_renderContext->modelview.data());
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		for (int i = 0; i < m_layers.size(); ++i)
		{
			m_layers[i]->render(m_renderContext);
		}
		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		//glMatrixMode(GL_MODELVIEW);
		//glLoadMatrixd(viewMatrix().data());
// 		m_camera->updateGLMatrix();
// 		glMatrixMode(GL_MODELVIEW);
		//glTranslatef(m_mapOrigin.x(), m_mapOrigin.y(), 0.f);
// 		glBegin(GL_LINE_LOOP);
// 		glVertex3d(0, 0, 0);
// 		glVertex3d(0, 100, 0);
// 		glVertex3d(100, 100, 0);
// 		glVertex3d(100, 0, 0);
// 		glEnd();
// 		for (int i = 0; i < m_layers.size(); ++i)
// 		{
// 			glMatrixMode(GL_MODELVIEW);
// 			glPushMatrix();
// 			glPushAttrib(GL_ALL_ATTRIB_BITS);
// 			m_layers[i]->render(m_renderContext);
// 			glPopAttrib();
// 			glMatrixMode(GL_MODELVIEW);
// 			glPopMatrix();
// 		}
	}

	void RenderMap2::resizeGL(int w, int h)
	{
		m_renderContext->w = w;
		m_renderContext->h = h;
		glViewport(0, 0, w, h);
		m_panTool->resizeWindow(w, h);
	}

	void RenderMap2::mouseReleaseEvent(QMouseEvent *event)
	{
		m_panTool->mouseReleaseEvent(event);
	}

	void RenderMap2::mousePressEvent(QMouseEvent *event)
	{
		m_panTool->mousePressEvent(event);
	}

	void RenderMap2::mouseMoveEvent(QMouseEvent *event)
	{
		m_panTool->mouseMoveEvent(event);
		makeCurrent();
		m_renderContext->camera->updateGLMatrix();
		float y = m_renderContext->h - event->y();
		float x = event->x();
		Eigen::Vector3d pt3d = GLUtils::screenToWorld(m_renderContext->camera->viewMatrix(), x, y, 0.f);
		QPointF pt(pt3d.x(), pt3d.y());
		emit mouseMove(pt);
	}

	void RenderMap2::wheelEvent(QWheelEvent *event)
	{
		m_panTool->wheelEvent(event);
	}

	void RenderMap2::repaintAllLayers(const QRectF &camExtent)
	{
		//printf("repaint\n");
		m_lastCamExtent = camExtent;
		
		double x = camExtent.x();
		double y = camExtent.y();
		double w = camExtent.width();
		double h = camExtent.height();
		QRectF worldExtent(x, y, w, h);
		for (int i = 0; i < m_layers.size(); ++i) {
			m_layers[i]->repaint(m_renderContext, worldExtent);
		}
		update();
	}

	void RenderMap2::zoomToExtent(const QRectF &extent)
	{
		if (!extent.isNull()) {
			m_panTool->zoomToExtent(extent.x(), extent.y(), extent.x() + extent.width(), extent.y() + extent.height());
		}
		update();
	}

	void RenderMap2::repaint()
	{
		repaintAllLayers(m_lastCamExtent);
	}

	void RenderMap2::clearLayers()
	{
		for (int i = 0; i < m_layers.size(); ++i)
		{
			delete m_layers[i];
		}
		m_layers.clear();
	}


}

}//name space insight
