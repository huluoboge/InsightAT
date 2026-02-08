#include "render_global.h"
#include "render_glutils.h"

namespace insight{

namespace render
{
	Eigen::Vector3f GLUtils::screenToWorld(float screenX, float screenY, float deep/*0--1*/)
	{
		GLdouble modelView[16];
		GLint viewport[4];
		GLdouble projection[16];
		glMatrixMode(GL_MODELVIEW);
		glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
		glGetIntegerv(GL_VIEWPORT, viewport);
		glGetDoublev(GL_PROJECTION_MATRIX, projection);

		GLdouble winx, winy, winz;
		GLdouble objx, objy, objz;
		winx = screenX;
		winy = screenY;
		winz = deep;
		gluUnProject(winx, winy, winz, modelView, projection, viewport, &objx, &objy, &objz);
		return Eigen::Vector3f(objx, objy, objz);
	}
	
	/*
		Transform a point(column vector) by a 4x4 matrix.I.e.out = m * in
		* Input: m - the 4x4 matrix
		* in - the 4x1 vector
		* Output : out - the resulting 4x1 vector.
	*/
	static void transform_point(double out[4], const double m[16], const double in[4])
	{
#define M(row,col) m[col*4+row]  
		out[0] = M(0, 0) * in[0] + M(0, 1) * in[1] + M(0, 2) * in[2] + M(0, 3) * in[3];
		out[1] = M(1, 0) * in[0] + M(1, 1) * in[1] + M(1, 2) * in[2] + M(1, 3) * in[3];
		out[2] = M(2, 0) * in[0] + M(2, 1) * in[1] + M(2, 2) * in[2] + M(2, 3) * in[3];
		out[3] = M(3, 0) * in[0] + M(3, 1) * in[1] + M(3, 2) * in[2] + M(3, 3) * in[3];
#undef M  
	}

	GLint gluProjectEx(double objx, double objy, double objz
		, const double model[16], const double proj[16], const GLint viewport[4]
		, double * winx, double * winy, double * winz)
	{
		/* transformation matrix */
		double objCoor[4];
		double objProj[4], objModel[4];

		/* initilise matrix and vector transform */
		// 4x4 matrix must be multi to a 4 dimension vector( it a 1 x 4 matrix)  
		// so we need to put the original vertex to a 4D vector  
		objCoor[0] = objx;
		objCoor[1] = objy;
		objCoor[2] = objz;
		objCoor[3] = 1.0;

		// ����ԭ��������λ�ڱ�׼������(1, 0, 0), (0, 1, 0), (0, 0, 1)�У�������Ҫ��ת������ǰ��ģ�;�����  
		transform_point(objModel, model, objCoor);

		// Ȼ��ģ�;����еĶ���ת����ͶӰ������������ϵ�ľ�����  
		transform_point(objProj, proj, objModel);

		// scale matrix  
		/*
		GLdouble scaleMat[4][4] =
		{
			{0.5, 0, 0, objPr0j[3]},
			{0, 0.5, 0, objProj[3]},
			{0, 0, 0.5, objProj[3]},
			{1, 1, 1,   1}
		};

		GLdouble objProjTemp[4];
		memcpy(objProjTemp, objProj, sizeof(objProjTemp);
		transfrom_point(objProj, scaleMat, objProjTemp);
		*/

		/* or the result of normalized between -1 and 1 */
		if (objProj[3] == 0.0)
			return GL_FALSE;

		objProj[0] /= objProj[3];
		objProj[1] /= objProj[3];
		objProj[2] /= objProj[3];

		/* in screen coordinates */
		// ����ͶӰ����ͶӰ��[-1, 1]֮�䣬������Ҫ��ת�����ͶӰ������õ�[0, 1]֮��  
		// �������һ��offset ������ת��Ϊ��Ļ����Ϳ����ˣ�viewport[4]���Լ򵥵���Ϊһ��offset���Σ�  

#define SCALE_FROM_0_TO_1(_pt)  (((_pt) + 1)/2)  
		objProj[0] = SCALE_FROM_0_TO_1(objProj[0]);
		objProj[1] = SCALE_FROM_0_TO_1(objProj[1]);
		objProj[2] = SCALE_FROM_0_TO_1(objProj[2]);
#undef SCALE_FROM_0_TO_1  

		*winx = viewport[0] + objProj[0] * viewport[2];
		*winy = viewport[1] + objProj[1] * viewport[3];

		/* between 0 and 1 */
		*winz = objProj[2];
		return GL_TRUE;
	}

	Eigen::Vector3d GLUtils::screenToWorld(Eigen::Matrix<double, 4, 4> ModelView, float screenX, float screenY, float deep/*0--1*/)
	{
		GLdouble *modelView = ModelView.data();
		GLint viewport[4];
		GLdouble projection[16];
		glGetIntegerv(GL_VIEWPORT, viewport);
		glGetDoublev(GL_PROJECTION_MATRIX, projection);

		GLdouble x = modelView[3];
		GLdouble y = modelView[7];
		GLdouble z = modelView[11];
		modelView[3] = 0;
		modelView[7] = 0;
		modelView[11] = 0;
		GLdouble winx, winy, winz;
		GLdouble objx, objy, objz;
		winx = screenX;
		winy = screenY;
		winz = deep;
		gluUnProject(winx, winy, winz, modelView, projection, viewport, &objx, &objy, &objz);
		objx -= x;
		objy -= y;
		objz -= z;
		return Eigen::Vector3d(objx, objy, objz);
	}

	int GLUtils::maxTextureSize()
	{
		GLint maxTexture = -1;
		glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexture);
		return maxTexture;
	}

}

}//name space insight
