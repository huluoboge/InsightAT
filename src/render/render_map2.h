#pragma once

/*
@author: Jones
@date: 2019-04-27
@descriptions:


*/
#include "render_global.h"
#include "render_context.h"
#include "render_camera.h"
#include "render_pan_tool.h"
#include "render_layer.h"

#include <QGLWidget>


namespace insight{

namespace render
{
	class RenderLayer;
	class  RenderMap2 : public QGLWidget
	{
		Q_OBJECT
	signals:
		void mouseMove(const QPointF &worldPoint);
	public:
		RenderMap2(QWidget *parent  = nullptr);
		~RenderMap2();
		void initScene();

		void addLayer(RenderLayer *layer) { m_layers.push_back(layer); }

		void zoomToExtent(const QRectF &extent);

		QList<RenderLayer *> &layers() { return m_layers; }

	public slots:
		void repaintAllLayers(const QRectF &camExtent);
		void repaint();
		void clearLayers();
	protected:
		virtual void initializeGL();
		virtual void paintGL();
		virtual void resizeGL(int w, int h);
        virtual void mouseReleaseEvent(QMouseEvent *event);
        virtual void mousePressEvent(QMouseEvent *event);
		virtual void mouseMoveEvent(QMouseEvent *event);
		virtual void wheelEvent(QWheelEvent *event);

	protected:
		RenderContext *m_renderContext;
		RenderCamera2d *m_camera;
		RenderPanTool2 *m_panTool;
		QList<RenderLayer *> m_layers;
		QRectF m_lastCamExtent;
	};
}

}//name space insight
