#include "render_pivot.h"

#include <QGLWidget>

#include "opengl_mat.h"
#include "render_context.h"
#include "render_camera.h"

namespace insight{

namespace render
{
	void RenderPivot::glDrawUnitCircle(int dim, unsigned steps /*= 64*/)
	{
		double thetaStep = 2.0 * M_PI / static_cast<double>(steps);
		unsigned char dimX = (dim < 2 ? dim + 1 : 0);
		unsigned char dimY = (dimX < 2 ? dimX + 1 : 0);

		Vec3 P(0, 0, 0);

		glBegin(GL_LINE_LOOP);
		for (unsigned i = 0; i < steps; ++i)
		{
			double theta = thetaStep * i;
			P[dimX] = cos(theta);
			P[dimY] = sin(theta);
			glVertex3dv(P.data());
		}
		glEnd();
	}

	RenderPivot::RenderPivot()
		: m_noScale(true)
	{
		_modelMatrix = Mat4::Identity();
	}

	//����cloud compare
	void RenderPivot::draw(RenderContext *rc)
	{
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		//glMultMatrixd(_modelMatrix.data());
		rc->push();
		rc->modelview *= _modelMatrix;
		glLoadMatrixd(rc->modelview.data());
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		if (m_noScale) //����
		{
			//���ְ뾶Ϊ������̵�һ���3/4
			double halfW = rc->w / 2;
			double halfH = rc->h / 2;
			RenderCamera *cam = rc->camera;
			double l, r, t, b, n, f;
			cam->getFrustum(l, r, b, t, n, f);
			double radius = 0.75 * std::min(r, t);
			double fov = r / t;

			GLdouble modelView[16];
			glMatrixMode(GL_MODELVIEW);
			glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
			Mat4 mat = Eigen::Map<Mat4>(modelView);
			Vec3 x = mat.col(0).head<3>();
			Vec3 y = mat.col(1).head<3>();
			Vec3 z = mat.col(2).head<3>();
			double sx = x.norm();
			double sy = y.norm();
			double sz = z.norm();
			Vec4 pos = mat.col(3);
			double ratio = fabs(pos.z() / n);
			radius *= ratio;
			glPushMatrix();
			glScaled(radius / sx, radius / sy, radius / sz);
		}

		//draw 3 circles
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_BLEND);
		const float c_alpha = 0.6f;
		glLineWidth(1.0f);

		//RenderPivot symbol: 3 circles
		glColor4f(1.0f, 0.0f, 0.0f, c_alpha);
		glDrawUnitCircle(0, 128);

		glColor4f(0.0f, 1.0f, 0.0f, c_alpha);
		glDrawUnitCircle(1, 128);

		glColor4f(0.0f, 0.7f, 1.0f, c_alpha);
		glDrawUnitCircle(2, 128);

		glPointSize(10);
		glColor4f(0.0f,1.0f,0.0f, 1.0f);
		glBegin(GL_POINTS);
			glVertex3d(0,0,0);
		glEnd();
		if (m_noScale)
		{
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		}

		glPopAttrib();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		rc->pop();
	}

	void RenderPivot::updateMatrix(const Mat4 &mat)
	{
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				_modelMatrix(i, j) = mat(i, j);
			}
		}
	}

} // namespace render

}//name space insight
