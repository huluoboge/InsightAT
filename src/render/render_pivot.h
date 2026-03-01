#pragma once

#include "render_global.h"
#include "render_object.h"
#include "render_types.h"
#include "render_node.h"

/************************************************************************/
/* 
��ά��ת��
*/
/************************************************************************/

namespace insight{

namespace render
{
	class  RenderPivot : public RenderObject
	{
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		RenderPivot();

		void setNoScale(bool noScale) { m_noScale = noScale; }
		bool isNoScale() const { return m_noScale; }

		virtual void draw(RenderContext *rc);
		
		Mat4 &refModelMatrix() { return _modelMatrix; }

		void updateMatrix(const Mat4 &mat);
	private:
		//draw a unit circle in a given plane (0=YZ, 1 = XZ, 2=XY) 
		void glDrawUnitCircle(int dim, unsigned steps = 64);

		bool m_noScale;
		Mat4 _modelMatrix;
	};

}

}//name space insight

