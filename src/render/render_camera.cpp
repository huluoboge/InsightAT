#include "render_camera.h"
#include "opengl_mat.h"
#include "render_types.h"

namespace insight {
namespace render {

    RenderCamera::RenderCamera()
    {
        m_frustumMatrix = Mat4::Identity();
    }

    RenderCamera::~RenderCamera()
    {
    }
    void RenderCamera::lookAt(const Vec3& eye, const Vec3& target, const Vec3& up)
    {
        vmath::vec3<double> vEye, vTarget, vUp;
        vEye.x = eye.x();
        vEye.y = eye.y();
        vEye.z = eye.z();
        vTarget.x = target.x();
        vTarget.y = target.y();
        vTarget.z = target.z();
        vUp.x = up.x();
        vUp.y = up.y();
        vUp.z = up.z();

        Mat4 mat = toMat4(vmath::lookat_matrix(vEye, vTarget, vUp));
        refModelMatrix() = fastInverse(mat);
    }

    void RenderCamera::updateGLMatrix()
    {
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixd(m_frustumMatrix.data());

        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixd(viewMatrix().data());
    }

    void RenderCamera::frustum(double left, double right, double bottom, double top, double nearPlane, double farPlane)
    {
        m_left = left;
        m_right = right;
        m_bottom = bottom;
        m_top = top;
        m_nearPlane = nearPlane;
        m_farPlane = farPlane;
        refProjectMatrix() = toMat4(vmath::frustum_matrix(left, right, bottom, top, nearPlane, farPlane));
    }

    void RenderCamera::getFrustum(double& left, double& right, double& bottom, double& top, double& nearPlane, double& farPlane)
    {
        left = m_left;
        right = m_right;
        bottom = m_bottom;
        top = m_top;
        nearPlane = m_nearPlane;
        farPlane = m_farPlane;
    }

    void RenderCamera::render(RenderContext* rc)
    {
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        draw(rc);
        for (int i = 0; i < (int)m_childNodes.size(); ++i) {
            m_childNodes[i]->render(rc);
        }
        glPopAttrib();
    }

    void RenderCamera2d::ortho(double left, double right, double bottom, double top, double nearPlane, double farPlane)
    {
        m_left = left, m_right = right, m_bottom = bottom, m_top = top, m_nearPlane = nearPlane, m_farPlane = farPlane;
        refProjectMatrix() = toMat4(vmath::ortho_matrix(left, right, bottom, top, nearPlane, farPlane));
    }

    void RenderCamera2d::scaleOrtho(double left, double right, double bottom, double top, double nearPlane, double farPlane)
    {
        m_left = left * m_scaleRatio, m_right = right * m_scaleRatio, m_bottom = bottom * m_scaleRatio;
        m_top = top * m_scaleRatio, m_nearPlane = nearPlane, m_farPlane = farPlane;
        refProjectMatrix() = toMat4(vmath::ortho_matrix(m_left, m_right, m_bottom, m_top, m_nearPlane, m_farPlane));
    }

    void RenderCamera2d::getOrtho(double& left, double& right, double& bottom, double& top, double& nearPlane, double& farPlane)
    {
        left = m_left, right = m_right, bottom = m_bottom;
        top = m_top, nearPlane = m_nearPlane, farPlane = m_farPlane;
    }

}

} // name space insight
