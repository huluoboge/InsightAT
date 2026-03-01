
#ifndef pca_h__
#define pca_h__

#include "common_global.h"
#include "numeric.h"

namespace insight{


struct  PCA3d
{
	Vec3 meanPt;
	Vec3 eigenValues;
	Mat3 eigenVectors;

	bool operator()(std::vector<Vec3> &pts);
};

}//name space insight
#endif // pca_h__