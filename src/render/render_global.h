#pragma once

#include <QtCore/qglobal.h>
#pragma warning(disable:4251)


#if defined (WIN32) || (WIN64) || (_WIN32) || (_WIN64)
#include "gl/glew.h"
#else
#include "GL/glew.h"
#endif




