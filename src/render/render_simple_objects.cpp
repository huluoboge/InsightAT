#include "render_simple_objects.h"

namespace insight{

namespace render
{

	RenderSimpleRectange::RenderSimpleRectange() 
		:color(1.0, 1.0, 1.0)
	{
		data[0] = Vec3(-100., -100., 0);
		data[1] = Vec3(100, -100, 0);
		data[2] = Vec3(100, 100, 0);
		data[3] = Vec3(-100, 100, 0);
	}

}

}//name space insight

