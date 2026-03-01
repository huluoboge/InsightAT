#include "render_object.h"
namespace insight{

namespace render
{
	bool g_exit_render = false;
	 void exitRender()
	{
		g_exit_render = true;
	}

	 void startRender()
	{
		g_exit_render = false;
	}
}

}//name space insight
