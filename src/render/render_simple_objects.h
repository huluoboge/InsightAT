#pragma once


#include "render_global.h"
#include "render_object.h"

#include "render_types.h"

namespace insight{

namespace render
{
	class  RenderSimpleRectange : public RenderObject
	{
	public:
		RenderSimpleRectange();
		Vec3 color;
		Vec3 data[4];

		virtual void draw(RenderContext *rc) {
			glBegin(GL_LINE_LOOP);
			glVertex3dv(data[0].data());
			glVertex3dv(data[1].data());
			glVertex3dv(data[2].data());
			glVertex3dv(data[3].data());
			glEnd();
		}

	};
}

}//name space insight
