#include "render_pan_tool.h"


#include <QEvent>
#include <QGLWidget>
#include <QWheelEvent>
#include <cmath>
#include <QDebug>
#include <QApplication>

// Qt版本兼容性处理
#if QT_VERSION >= QT_VERSION_CHECK(5, 3, 0)
    #define MIDDLE_BUTTON Qt::MiddleButton
#else
    #define MIDDLE_BUTTON Qt::MidButton
#endif
namespace insight{

namespace render
{

	RenderPanTool::RenderPanTool()
		: m_isMoving(false)
		, m_lastCursor(Qt::ArrowCursor)
	{
	}

	RenderPanTool::~RenderPanTool()
	{
	}

	void RenderPanTool::mouseReleaseEvent(QMouseEvent * event)
	{
		//Qt::MidButton or Qt MiddleButton
		//检查Qt的版本， 兼容Qt::MidButton or Qt MiddleButton
		if (event->button() == Qt::RightButton || event->button() == MIDDLE_BUTTON)
		{
			m_isMoving = false;
			renderContext()->widget->setCursor(m_lastCursor);
		}
	}

	void RenderPanTool::mousePressEvent(QMouseEvent * event)
	{
		if (event->button() == Qt::RightButton || event->button() == Qt::MidButton)
		{
			m_isMoving = true;
			m_movePos = event->pos();
			m_lastCursor = renderContext()->widget->cursor();
		}
	}

	void RenderPanTool::mouseMoveEvent(QMouseEvent * event)
	{
		if (m_isMoving)
		{
			QCursor cusor(Qt::SizeAllCursor);
			renderContext()->widget->setCursor(cusor);

			QPoint pt = event->pos();

			QPoint dis = m_movePos - pt;
			dis.setY(-dis.y());
			
			Vec3 v = renderContext()->camera->v();
			Vec3 u = renderContext()->camera->u();

			Vec3 camPos = renderContext()->camera->position();
			float w = renderContext()->w;
			float h = renderContext()->h;
			double l, r, b, t, n, f;
			renderContext()->camera->getFrustum(l, r, b, t, n, f);
			//��һ��������
			double dx = dis.x() / w * (r - l) / renderContext()->camera->focusLength();
			double dy = dis.y() / h * (t - b) / renderContext()->camera->focusLength();

			Vec3 zero(0, 0, 0);
			Vec3 targetPos = m_target->localToWorld(zero);
			targetPos = (renderContext()->camera->viewMatrix() * Vec4(targetPos.x(), targetPos.y(), targetPos.z(), 1.0)).head<3>();

			double targetX = targetPos.x() / targetPos.z();
			double targetY = targetPos.y() / targetPos.z();
			if (targetPos.z() > 0)
			{
				targetX -= dx;
				targetY -= dy;
			}
			else
			{
				targetX += dx;
				targetY += dy;
			}

			targetPos.x() = targetX * targetPos.z();
			targetPos.y() = targetY * targetPos.z();
			//std::cout << "targetPos.z " << targetPos.z << std::endl;
			targetPos = (renderContext()->camera->modelMatrix() * Vec4(targetPos.x(), targetPos.y(), targetPos.z(), 1.0)).head<3>();
			RenderNode *p = m_target->parentNode();
			if (p != NULL)
			{
				targetPos = p->worldToLocal(targetPos);
			}
			m_target->setPosition(targetPos);
			m_movePos = pt;
			renderContext()->widget->update();
		}
	}

	void RenderPanTool2::wheelEvent(QWheelEvent * event)
	{
		double ratio = event->delta() > 0 ? 0.9 : 1.1;

		zoom(ratio, event->x(), event->y());
	}

	void RenderPanTool2::mouseReleaseEvent(QMouseEvent * event)
	{
		m_bIsMoving = false;
		renderContext()->widget->setCursor(Qt::CrossCursor);
	}

	void RenderPanTool2::mousePressEvent(QMouseEvent * event)
	{
		if (event->button() == Qt::RightButton || event->button() == Qt::MidButton)
		{
			m_posX = event->x();
			m_posY = event->y();
			m_bIsMoving = true;
		}
	}

	void RenderPanTool2::mouseMoveEvent(QMouseEvent * event)
	{
		if (m_bIsMoving || event->button() == Qt::RightButton)
		{
			renderContext()->widget->setCursor(Qt::SizeAllCursor);
			RenderCamera2d *camera = (RenderCamera2d *)renderContext()->camera;
			//��Ļ�������������Y�ķ������෴��
			Vec3 dis = Vec3(m_posX - event->x(), event->y() - m_posY, 0.0);
			double ratio = camera->scaleRatio();
			camera->translate(dis * ratio);
			renderContext()->widget->update();
			m_posX = event->x();
			m_posY = event->y();

			
			int w = renderContext()->w;
			int h = renderContext()->h;

			double scale = camera->scaleRatio();
			double x = camera->position().x() - w / 2.0 * scale;
			double y = camera->position().y() - h / 2.0 * scale;
			double width = w * scale;
			double height = h * scale;

			emit viewChanged(QRectF(x, y, width, height), scale);
		}
	}

	void RenderPanTool2::resizeEvent(QResizeEvent *event)
	{
		RenderCamera2d *camera = (RenderCamera2d *)renderContext()->camera;

		int w = event->size().width();
		int h = event->size().height();
		renderContext()->widget->makeCurrent();
		glViewport(0, 0, w, h);
		camera->scaleOrtho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -10000, 10000);

		double scale = camera->scaleRatio();
		double x = camera->position().x() - w / 2.0 * scale;
		double y = camera->position().y() - h / 2.0 * scale;
		double width = w * scale;
		double height = h * scale;

		emit viewChanged(QRectF(x, y, width, height), scale);
	}

	void RenderPanTool2::zoomToExtent(
		double leftBottomX, double leftBottomY, double rightTopX, double rightTopY)
	{
		RenderCamera2d *camera = (RenderCamera2d *)renderContext()->camera;
		double x = (leftBottomX + rightTopX) / 2.0;
		double y = (leftBottomY + rightTopY) / 2.0;

		//move to x,y
		double _x, _y, _z;
		camera->position(_x, _y, _z);
		camera->setPosition(x, y, _z);

 		int w = renderContext()->widget->width();
		int h = renderContext()->widget->height();
		
		double extentX = rightTopX - leftBottomX;
		double extentY = rightTopY - leftBottomY;
		double scale1 = extentX / w;
		double scale2 = extentY / h;
		double scaleRatio = std::max(scale1, scale2);


		camera->setScaleRatio(scaleRatio);
		camera->scaleOrtho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -1000, 1000);
		renderContext()->widget->update();

		double scale = camera->scaleRatio();
		x = camera->position().x() - w / 2.0 * scale;
		y = camera->position().y() - h / 2.0 * scale;
		double width = w * scale;
		double height = h * scale;
		emit viewChanged(QRectF(x, y, width, height), scale);
	}

	void RenderPanTool2::zoomIn()
	{
		int w = renderContext()->w;
		int h = renderContext()->h;

		zoom(0.9, w * 0.5, h * 0.5);
	}

	void RenderPanTool2::zoomOut()
	{
		int w = renderContext()->w;
		int h = renderContext()->h;
		zoom(1.1, w * 0.5, h * 0.5);
	}

	void RenderPanTool2::zoom(float ratio, float cx, float cy)
	{
		RenderCamera2d *camera = (RenderCamera2d *)renderContext()->camera;
		double beforeScaleRatio = camera->scaleRatio();
		double afterScaleRatio = beforeScaleRatio * ratio;

		float realRatio = 1.0 / afterScaleRatio;
		if (realRatio > m_maxScaleRatio)
		{
			return;
		}
		int w = renderContext()->w;
		int h = renderContext()->h;

		camera->setScaleRatio(afterScaleRatio);
		double transX = (cx - w / 2.0) * (afterScaleRatio - beforeScaleRatio);
		double transY = (h / 2.0 - cy) * (afterScaleRatio - beforeScaleRatio);

		camera->scaleOrtho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -10000, 10000);
		camera->translate(-transX, -transY, 0);

		renderContext()->widget->update();

		double scale = camera->scaleRatio();
		double x = camera->position().x() - w / 2.0 * scale;
		double y = camera->position().y() - h / 2.0 * scale;
		double width = w * scale;
		double height = h * scale;
		emit viewChanged(QRectF(x, y, width, height), scale);
	}

	void RenderPanTool2::zoom(float ratio)
	{
		int w = renderContext()->w;
		int h = renderContext()->h;
		zoom(ratio, w * 0.5, h * 0.5);
	}

	void RenderPanTool2::resizeWindow(int w, int h)
	{
		RenderCamera2d *camera = (RenderCamera2d *)renderContext()->camera;

		renderContext()->widget->makeCurrent();
		glViewport(0, 0, w, h);
		camera->scaleOrtho(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, -10000, 10000);

		double scale = camera->scaleRatio();
		double x = camera->position().x() - w / 2.0 * scale;
		double y = camera->position().y() - h / 2.0 * scale;
		double width = w * scale;
		double height = h * scale;

		emit viewChanged(QRectF(x, y, width, height), scale);
	}

}

}//name space insight
