#ifndef opengl_utils_h__
#define opengl_utils_h__

#include "common.h"
#include "numeric.h"

namespace insight{

template<class T>
Eigen::Matrix<T, 4, 4> lookAt
(
Eigen::Matrix<T, 3, 1> const & eye,
Eigen::Matrix<T, 3, 1> const & center,
Eigen::Matrix<T, 3, 1> const & up
)
{
	typedef Eigen::Matrix<T, 4, 4> Matrix4;
	typedef Eigen::Matrix<T, 3, 1> Vector3;

	Vector3 f = (center - eye).normalized();
	Vector3 u = up.normalized();
	Vector3 s = f.cross(u).normalized();
	u = s.cross(f);

	Matrix4 res;
	res << s.x(), s.y(), s.z(), -s.dot(eye),
		u.x(), u.y(), u.z(), -u.dot(eye),
		-f.x(), -f.y(), -f.z(), f.dot(eye),
		0, 0, 0, 1;

	return res;
}

}//name space insight
#endif // opengl_utils_h__
