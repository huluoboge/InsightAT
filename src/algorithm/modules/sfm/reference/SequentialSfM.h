#pragma once

#include "sfm_global.h"
#include "SfmOptions.h"

#include "FeatureDetect/ImageFeature.h"
#include "FeatureMatch/match_types.h"
#include "MultiView/Pose.h"
#include "sfm_tracks.h"

#include <mutex>
#include <omp.h>

INSIGHT_NAME_SPACE

/// Export point feature based vector to a matrix [(x,y)'T, (x,y)'T]
/// Use the camera intrinsics in order to get undistorted pixel coordinates
template< typename FeaturesT, typename MatT >
static void PointsToMat(
	const CameraPtr cam,
	const FeaturesT & vec_feats,
	MatT & m)
{
	m.resize(2, vec_feats.size());
	typedef typename FeaturesT::value_type ValueT; // Container type
	typedef typename MatT::Scalar Scalar; // Output matrix type

	size_t i = 0;
	for (typename FeaturesT::const_iterator iter = vec_feats.begin();
		iter != vec_feats.end(); ++iter, ++i)
	{
		Vec2 feat;
		feat << iter->x(), iter->y();
		m.col(i) = cam->get_ud_pixel(feat);
	}
}

#if 0
typedef std::map<Pair, std::pair<size_t, size_t>> PairTriagnlutaions;// imagepair_triangluated_totalmtaches

typedef std::unordered_map<IndexT, Pair_Set> ImagePairs;
#endif


/// Functor to sort a vector of pair given the pair's second value
template<class T1, class T2, class Pred = std::less<T2> >
struct sort_pair_second {
	bool operator()(const std::pair<T1, T2>&left,
		const std::pair<T1, T2>&right)
	{
		Pred p;
		return p(left.second, right.second);
	}
};

/// Allow to select the Keys of a map.
struct RetrieveKey
{
	template <typename T>
	typename T::first_type operator()(const T & keyValuePair) const
	{
		return keyValuePair.first;
	}
};

class SequentialSfM
{
	// Perform retriangulation for under-reconstructed image pairs. Under-
	// reconstruction usually occurs in the case of a drifting reconstruction.
	//
	// Image pairs are under-reconstructed if less than `Options::tri_re_min_ratio
	// > tri_ratio`, where `tri_ratio` is the number of triangulated matches over
	// inlier matches between the image pair.

public:
	SequentialSfM();

	~SequentialSfM();

	void setAutoDeleteFeatureAndMatch(bool del) { _autoDeleteFeatureAndMatch = del; }
#if 0
	struct CorrData
	{
		size_t trackID;
		Vec2 feat;
		SfmTracks::Obv obv;
		Mat34 P;
		PosePtr pose;
		CameraPtr camera;
	};
#endif

	SequentialSfmOptions Options() const { return _options; }
	void Options(SequentialSfmOptions val) { _options = val; }

	void setFeaturesProvider(SIFTImageFeature_PerImage *featuresPerImage);
	void setMatchesProvider(PairWiseMatches *matchesPerPair);

	void setData(SfM_Data *sfm_data);

	void setExternalTracks(const SfmTracks &sfmTracks);

	bool Process();

	void rebuildTracks();
	size_t reconstructViewSize() { return _reconstructed_views.size(); }

	void addResectionCallBack(std::function<void(SfM_Data *)> func){ _resectionCallBack.push_back(func); }

	void setExit(bool bExit) { _bExit = bExit; }
	void setLogDetail(bool val){	_logDetail = val;	}
private:
	void ResectionCallBack();
	bool MakeInitialPair3D(const Pair & current_pair);

	void CheckTracks();

	void InitRemainViewList();
	bool InitLandmarkTracks();

	//void InitImagePairs();

	//void InitPairTriangulations();

	void UpdateTrackVisibilityPyramid();

	

#if 0
	void UpdateReconstructTracks();
	void UpdateReconstructTracksWithNewAddTracks();
#endif
	std::vector<uint32_t> SelectInitialFirst();

	void SelectInitialSecond(IndexT imageId1, std::vector<IndexT> &image_ids);

	bool SelectInitialPair(Pair &initialPair);

	/// List the images that the greatest number of matches to the current 3D reconstruction.
	bool FindImagesWithPossibleResection(std::vector<size_t> & vec_possible_indexes);

	void ResetBACount();

	void InitReconstructedViews();

	bool Resection(size_t imageIndex);

	void Triangulation(const size_t I, const size_t J, std::vector<size_t> &vec_tracks,
		std::vector<FeatureX> &iFeats, std::vector<FeatureX> &jFeats);

#if 0
	size_t Retriangulation();
#endif

	void MarkRegisted(size_t imageIndex);
	void MarkUnRegisted(size_t imageIndex);

	BA_TYPE CheckBAType();

	double BundleAdjustment(BA_TYPE ba_type, const std::vector<size_t> &newRegistedImage);

	double BundleAdjustmentCeres(BA_TYPE ba_type);

	void BundleAdjustmentSub(const std::vector<IndexT> &local_bundle, 
		const std::vector<size_t> &constImageIndex,
		std::set<IndexT> &changedLandMarks);

#if PBA_GPU
	double BundleAdjustmentPBA();
#endif
	std::vector<IndexT> FindLocalBundle(const IndexT viewIndex) const;


	void ClearNewAddedTracksInResection();

	/// Discard track with too large residual error
	size_t badTrackRejector(double dPrecision);

	void GetLandmarksInImages(
		const Structure &inLandmarks,
		const std::set<IndexT> &set_imageIndex,
		Structure &outLandmarks,
		int max_track_length = 15
		);

	void subData(const SfM_Data &sfm_data, SfM_Data &sub_data, const std::set<IndexT> &subView, int max_track_length = 15);

	void mergeData(SfM_Data &target_data,
		const SfM_Data &source_data,
		const std::map<IndexT, bool> &map_bRefineStructures);

#if 0
	size_t Continue(const TriangulationOptions & options, const CorrData& ref_corr_data,
		const std::vector<CorrData>& corrs_data);

	size_t Create(const TriangulationOptions& options, const CorrData &corr_data1, const CorrData &corr_data2);

#endif
	double CalculateAngularError(const Vec2& point2D,
		const Vec3& point3D,
		const Mat34& proj_matrix,
		const CameraPtr cam);

	SequentialSfmOptions _options;
	SIFTImageFeature_PerImage *_featuresPerImage;
	PairWiseMatches *_matchesPerPair;
	SfM_Data *_sfm_data;
	SfmTracks _sfm_tracks;
	
	//ImagePairs _imagePairs;
	std::map<Pair, size_t> _re_num_trials;//per image re_trianglulation times;
	std::unordered_map<uint32_t, double> _map_ACThreshold;
	std::unordered_set<uint32_t> _set_remainingViewId;
	std::vector<size_t> _vec_viewStatus;//0 means not ok,1 means ok
	std::set<Pair>_tried_init_image_pairs;
	std::unordered_set<uint32_t> _reconstructed_views;
	std::map<uint32_t,int> _reconstructed_views_per_intrisic;
	std::unordered_set<uint32_t> _trackChanged;
	//PairTriagnlutaions _pairTrianglulations;
	int _nLastFullBACount;
	int _nLastFullDistortBACount;

	std::vector < std::function<void(SfM_Data *)>> _resectionCallBack;

	bool _bExit = false;
	bool _logDetail = false;

	std::mutex _triMutex;
	bool _autoDeleteFeatureAndMatch = false;
};


INSIGHT_NAME_SPACE_END
