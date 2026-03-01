#include "pca.h"



bool insight::PCA3d::operator()(std::vector<Vec3> &vec_pts)
{
	Eigen::Matrix3Xd mMeanPts;
	Eigen::Matrix3Xd mMean;
	Eigen::Matrix3Xd pts;
	pts.resize(3, vec_pts.size());
	for (int i = 0; i < vec_pts.size(); ++i){
		pts.col(i) = vec_pts[i];
	}
	mMean = pts.rowwise().mean();
	mMeanPts = pts - mMean.replicate(1, pts.cols());
	Eigen::MatrixXd mCovPts;
	mCovPts = mMeanPts *mMeanPts.transpose();
	mCovPts = mCovPts / (pts.size() - 1);
	Eigen::EigenSolver<Eigen::Matrix3d> es;
	es.compute(mCovPts, true);
	bool bOK = false;
	if (es.info() == Eigen::Success)
	{
		meanPt = mMean.col(0);
		Vec3 eValues = es.eigenvalues().real();

		//sort by desend
		int idx[] = { 0, 1, 2 };
		for (int i = 0; i < 2; ++i)
		{
			for (int j = i + 1; j < 3; ++j)
			{
				if (eValues[idx[i]] < eValues[idx[j]])
				{
					//swap
					int temp = idx[i];
					idx[i] = idx[j];
					idx[j] = temp;
				}
			}
		}
		eigenValues[0] = eValues[idx[0]];
		eigenValues[1] = eValues[idx[1]];
		eigenValues[2] = eValues[idx[2]];

		Eigen::MatrixXd evec = es.eigenvectors().real();
		eigenVectors.col(0) = evec.col(idx[0]);
		eigenVectors.col(1) = evec.col(idx[1]);
		eigenVectors.col(2) = evec.col(idx[2]);
		bOK = true;
	}
	return bOK;
}
