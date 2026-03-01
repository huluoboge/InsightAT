#pragma once

#include "render_global.h"
#include "render_camera.h"

#include "Eigen/Core"

namespace insight{

namespace render
{
	class  GLUtils
	{
	public:
		static Eigen::Vector3f screenToWorld(float screenX, float screenY, float deep/*0--1*/);
		static Eigen::Vector3d screenToWorld(Eigen::Matrix<double, 4, 4> modelView, float screenX, float screenY, float deep/*0--1*/);
		static int  maxTextureSize();
	};
}

}//name space insight
