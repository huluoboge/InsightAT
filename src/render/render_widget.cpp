#include "render_widget.h"

#include <QPainter>
#include <QColor>
#include <QBrush>
#include <QMouseEvent>
#include <QPixmap>
#include <QDebug>

#include <iostream>


#include "render_camera.h"
#include "render_object.h"
#include "render_node.h"
//#include "common/base_log.h"

namespace insight{

namespace render
{
    RenderWidget::RenderWidget(QWidget *parent)
        :QGLWidget(parent)
        , m_camera(NULL)
        , m_renderContext(NULL)
    {
        setFocusPolicy(Qt::StrongFocus);
        setMouseTracking(true);
        initScene();
    }

    RenderWidget::~RenderWidget()
    {
        makeCurrent();
        delete m_camera;
        delete m_renderContext;
        delete m_rotationTool;
    }

    void RenderWidget::initScene()
    {
        m_camera = new RenderCamera;
        m_renderContext = new RenderContext;
        m_renderContext->camera = m_camera;
        m_renderContext->widget = this;
        m_pivot = new RenderPivot;
        m_root = new RenderNode;
        m_dataRoot = new RenderNode(m_root);
        m_rotationTool = new RenderRotationTool;
        m_rotationTool->setRenderContext(m_renderContext);
        m_rotationTool->setCusor(QCursor(Qt::DragMoveCursor));
        m_rotationTool->setTarget(m_root);
        m_rotationTool->setPivot(m_pivot);

        m_panTool = new RenderPanTool;
        m_panTool->setTarget(m_dataRoot);
        m_panTool->setRenderContext(m_renderContext);

        m_zoomTool = new RenderZoomTool;
        m_zoomTool->setTarget(m_root);
        m_zoomTool->setRenderContext(m_renderContext);
    }

    void RenderWidget::initializeGL()
    {
        GLenum code = glewInit();
        if (GLEW_OK != (int)code)
        {
            LOG(ERROR) << "Init glew failed, error information: " <<glewGetErrorString(code);
            return;
        }
        glClearColor(0, 0, 0, 1.0);
        glClearDepth(1.0);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glEnable(GL_SMOOTH);
        glHint(GL_POINT_SMOOTH, GL_NICEST);
        glHint(GL_LINE_SMOOTH, GL_NICEST);
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);
        Vec3 camInitPos = Vec3(0, 0, 1000);

        m_camera->lookAt(camInitPos, Vec3(0, 0, 0), Vec3(0, 1, 0));
    }

    void RenderWidget::paintGL()
    {
        glEnable(GL_DEPTH_TEST);// �����ڻ���֮ǰָ������������Ч
        //glDepthFunc(GL_LESS);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        m_camera->updateGLMatrix();
        m_renderContext->clear();
        m_renderContext->modelview = m_camera->viewMatrix();
		if (m_pivot->isVisible()) {
			m_pivot->draw(m_renderContext);
		}
        m_root->render(m_renderContext);
    }

    void RenderWidget::resizeGL(int w, int h)
    {
        m_camera->frustum(-double(w) / h, double(w) / h, -1, 1, 5, 100000);
        glViewport(0, 0, w, h);
        m_renderContext->w = w;
        m_renderContext->h = h;
    }

    void RenderWidget::mouseReleaseEvent(QMouseEvent *event)
    {
        if (!m_lockPan) {
            m_rotationTool->mouseReleaseEvent(event);
            m_panTool->mouseReleaseEvent(event);
        }
        // if (m_tool != nullptr) {
        //     m_tool->mouseReleaseEvent(event);

        // }
        update();
    }

    void RenderWidget::mousePressEvent(QMouseEvent *event)
    {
        if (!m_lockPan) {
            m_rotationTool->mousePressEvent(event);
            m_panTool->mousePressEvent(event);
        }
        // if (m_tool != nullptr) {
        //     m_tool->mousePressEvent(event);
        // }
        update();
    }

    void RenderWidget::mouseMoveEvent(QMouseEvent * event)
    {
        if (!m_lockPan) {
            m_rotationTool->mouseMoveEvent(event);
            m_panTool->mouseMoveEvent(event);
        }
        // if (m_tool != nullptr) {
        //     m_tool->mouseMoveEvent(event);
        // }
        update();
    }

    void RenderWidget::wheelEvent(QWheelEvent *event)
    {
        if (!m_lockPan) {
            m_zoomTool->wheelEvent(event);
        }
        // if (m_tool != nullptr) {
        //     m_tool->wheelEvent(event);
        // }
        update();
    }

}

}//name space insight
