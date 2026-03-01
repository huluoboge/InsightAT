#pragma once
#include "render_global.h"
#include "render_types.h"

#include <QGLWidget>

namespace insight{

namespace render
{
	class RenderCamera;
	class RenderContext
	{
	public:
		int w = 0;
		int h = 0;
        RenderCamera *camera = nullptr;//
        QGLWidget *widget = nullptr;//

        //opengl modelview
        Mat4 modelview;

		void push() { modelviewStack.push_back(modelview); }
		void pop() { modelview = modelviewStack.back(); modelviewStack.pop_back(); }
		void clear() { modelviewStack.clear(); }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	protected:
		QList<Mat4> modelviewStack;
	};
}

}//name space insight
