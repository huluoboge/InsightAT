#pragma once
#include "render_global.h"

#include "render_types.h"
#include "render_node.h"
#include "opengl_mat.h"
#include "render_pivot.h"


namespace insight{

namespace render
{
    class  RenderCamera : public RenderNode
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                RenderCamera();
        virtual ~RenderCamera();
    public:

        /**
         * @brief view
         * @return
         */
        inline Mat4 viewMatrix() const { return fastInverse(refModelMatrix()); }

        void updateGLMatrix();


        inline Vec3 u() const { return viewMatrix().row(0).head<3>(); }
        inline Vec3 v() const { return viewMatrix().row(1).head<3>(); }
        inline Vec3 n() const { return viewMatrix().row(2).head<3>(); }

        inline Mat4 projectMatrix() const { return m_frustumMatrix; }

        inline Mat4 &refProjectMatrix() { return m_frustumMatrix; }

        inline const Mat4 &refProjectMatrix() const { return m_frustumMatrix; }

        /**
         * @param eye
         * @param target
         * @param up
         */
        void lookAt(const Vec3 &eye, const Vec3 &target, const Vec3 &up);

        //�ƶ����
        void move(const Vec3 &v)
        {
            Mat4 mat = viewMatrix();
            mat(0, 3) -= v.x();
            mat(1, 3) -= v.y();
            mat(2, 3) -= v.z();
        }

        void frustum(double left, double right, double bottom, double top, double nearPlane, double farPlane);

        void getFrustum(double &left, double &right, double &bottom, double &top, double &nearPlane, double &farPlane);


        double focusLength() { return m_nearPlane; }

        void render(RenderContext *rc);

    protected:
        Mat4 m_frustumMatrix;
        double m_left, m_right, m_bottom, m_top, m_nearPlane, m_farPlane;
    };

    class  RenderCamera2d : public RenderCamera
    {
    public:
        RenderCamera2d() {}
        virtual ~RenderCamera2d() {}
    public:
        /**
            * @param left
            * @param right
            * @param bottom
            * @param top
            * @param nearPlane
            * @param farPlane
            */
        void ortho(double left, double right, double bottom, double top, double nearPlane, double farPlane);
        void getOrtho(double &left, double &right, double &bottom, double &top, double &nearPlane, double &farPlane);


        void scaleOrtho(double left, double right, double bottom, double top, double nearPlane, double farPlane);


        double scaleRatio() const { return m_scaleRatio; }
        void setScaleRatio(double ratio) {
            m_scaleRatio = ratio;
        }

        /**
        * @brief scaeratio
        * @param ratio
        */
        void zoom(double ratio) { m_scaleRatio *= ratio; }
    protected:
        double m_scaleRatio = 1.0;
    };
}

}//name space insight
