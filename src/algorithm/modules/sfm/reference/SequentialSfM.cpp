#include "SequentialSfM.h"
#include "tracks.hpp"

#include "Common/timer.hpp"
#include "MultiView/sfm_data_triangulation.hpp"
#include "openMVG/multiview/projection.hpp"
#include "openMVG/multiview/sfm_robust_model_estimation.hpp"
#include "openMVG/multiview/triangulation.hpp"

#include "bundle_adjustment_ceres.h"
#include "bundle_adjustment_quaternion_ceres.h"
#include "sfm.h"
#include "sfm_data_filters.hpp"

#include <glog/log_severity.h>
#include <glog/logging.h>
#include <omp.h>

using namespace openMVG::sfm;
#define USE_QUATERNION_CERES
INSIGHT_NAME_SPACE

SequentialSfM::SequentialSfM()
{
    _sfm_data = nullptr;
}

SequentialSfM::~SequentialSfM()
{
}

void SequentialSfM::setData(SfM_Data* sfm_data)
{
    _sfm_data = sfm_data;
}

void SequentialSfM::setExternalTracks(const SfmTracks& sfmTracks)
{
    _sfm_tracks = sfmTracks;
}

bool SequentialSfM::Process()
{
    //-------------------
    //-- Incremental reconstruction
    //-------------------
    LOG(INFO) << "Aerial-triangulation";
    float nImages = static_cast<float>(_sfm_data->views.size());
    // step 1
    InitReconstructedViews();
    InitRemainViewList();

    if (_options.trackOptions.use_external_tracks) {
        LOG(INFO) << "Use input tracks";
        rebuildTracks();
    } else {
        LOG(INFO) << "Init landmark tracks";

        if (!InitLandmarkTracks()) {
            LOG(ERROR) << "Init landmark tracks failed!";
            if (_autoDeleteFeatureAndMatch) {
                delete _featuresPerImage;
                delete _matchesPerPair;
                _featuresPerImage = nullptr;
                _matchesPerPair = nullptr;
            }
            return false;
        }
    }

    if (_reconstructed_views.size() < 2) {
        Pair initPair(_options.initPairOptions.initial_pair_first, _options.initPairOptions.initial_pair_first);
        if (!SelectInitialPair(initPair)) {
            LOG(ERROR) << "Select initial pair failed";
            if (_autoDeleteFeatureAndMatch) {
                delete _featuresPerImage;
                delete _matchesPerPair;
                _featuresPerImage = nullptr;
                _matchesPerPair = nullptr;
            }
            return false;
        }
        LOG(INFO) << "select initial pair(" << initPair.first << "," << initPair.second << ")";
    }

    if (_autoDeleteFeatureAndMatch) {
        delete _featuresPerImage;
        delete _matchesPerPair;
        _featuresPerImage = nullptr;
        _matchesPerPair = nullptr;
    }

    // update visibility pyramid
    UpdateTrackVisibilityPyramid();
    // Compute robust Resection of remaining image
    std::vector<size_t> vec_possible_resection_indexes;

    ResetBACount();
    int nUnfilteredImageCount = 0;
    size_t reject_count = 0;
    std::set<int> failedImages;
    bool find = FindImagesWithPossibleResection(vec_possible_resection_indexes);
    int try_count = 0;
    const bool enableLessBA = _options.sfmOptions.enable_less_ba;
    std::stringstream optionSS;
    _options.sfmOptions.print(optionSS);
    LOG(INFO) << optionSS.str();
    int newResecionImageCount = 0;
    int ba_view_count = 0;

    while (find && !_bExit) {
        Timer timer;
        std::vector<size_t> newRegisted;
        int nResectiond = 0;
        // LOG(INFO) << "resection loop start " << vec_possible_resection_indexes.size();
        for (std::vector<size_t>::const_iterator iter = vec_possible_resection_indexes.begin();
            iter != vec_possible_resection_indexes.end(); ++iter) {
            bool bResect = Resection(*iter);
            if (!bResect) {
                LOG(INFO) << "Failed to register image  " << *iter << std::endl;
                MarkUnRegisted(*iter);
                _set_remainingViewId.erase(*iter);
                failedImages.insert(*iter);
            } else {
                MarkRegisted(*iter);
                // LOG(INFO) << "Register id=" << *iter << ",name=" << _sfm_data->views.at(*iter)->dbImage.image_name << ";[" << _reconstructed_views.size() << "/" << nImages << "]";
                newRegisted.push_back(*iter);
                ++nUnfilteredImageCount;
                _set_remainingViewId.erase(*iter);
                try_count = 0;
                ++newResecionImageCount;
                ++nResectiond;
            }
            // float nRegist = _reconstructed_views.size();
            // prog.percent(nRegist / nImages);
        }

        // LOG(INFO) << "resection loop end";

        bool bBA = (!_options.sfmOptions.enable_less_ba || _reconstructed_views.size() < _options.sfmOptions.less_ba_from || newResecionImageCount > _options.sfmOptions.less_ba_image || vec_possible_resection_indexes.empty());
        bBA = bBA && (nResectiond > 0);
        if (bBA) {
            BA_TYPE ba_type = CheckBAType();
            std::string ba_string;
            if (ba_type == eFullBAAll) {
                ba_string = "FULL";
            } else if (ba_type == ePartialBA) {
                ba_string = "PATIAL";
            } else if (ba_type == eFullBANoDistort) {
                ba_string = "FULL_NO_DISTORT";
            } else if (ba_type == eFullBAK1K2K3) {
                ba_string = "FULL_K1K2K3";
            }
#if PBA_GPU
            else if (ba_type == eFULLPBA) {
                ba_string = "FULL_A_BA";
            }
#endif
            double rmse = -1.0;
            bool continueBA = false;
            do {
                reject_count = 0;
                LOG(INFO) << "BA=" << ba_string << "[views=" << _reconstructed_views.size() << ",camera=" << _sfm_data->cameras.size()
                          << ",structure=" << _sfm_data->structure.size() << "]";
                rmse = BundleAdjustment(ba_type, newRegisted);

                auto itCam = _sfm_data->cameras.begin();
                while (itCam != _sfm_data->cameras.end()) {
                    const DBCamera& dbCam = itCam->second->dbCamera;
                    LOG(INFO) << "f=" << dbCam.focalpx << ",ppx=" << dbCam.ppx << ",ppy=" << dbCam.ppy << ",k1=" << dbCam.k1 << ",k2=" << dbCam.k2 << ",k3=" << dbCam.k3 << ",p1=" << dbCam.p1 << ",p2=" << dbCam.p2 << ",b1=" << dbCam.b1 << ",b2=" << dbCam.b2;
                    ++itCam;
                }
                ba_view_count = _reconstructed_views.size();

                if (_reconstructed_views.size() >= _options.sfmOptions.reject_points_from_image_count) {
                    reject_count = badTrackRejector(_options.sfmOptions.filter_max_reproj_error);
                }
                continueBA = (reject_count > _options.sfmOptions.filter_max_reject_count);
                LOG(INFO) << "RMSE=" << rmse << ",reject " << reject_count << " observations";
            } while (continueBA);
            newResecionImageCount = 0;

            // todo add last sigma filter count
            if (rmse != -1.0 && _options.sfmOptions.enable_sigma_filter) {
                reject_count = badTrackRejector(rmse * 3); // 3 * rmse filter
                LOG(INFO) << "reject " << reject_count << " points by 3 sigma filter";
            }
        }
        ClearNewAddedTracksInResection();
        UpdateTrackVisibilityPyramid();
        find = FindImagesWithPossibleResection(vec_possible_resection_indexes);
        if (!find && !failedImages.empty() && try_count == 0) {
            for (auto fitr = failedImages.begin(); fitr != failedImages.end(); ++fitr) {
                _set_remainingViewId.insert(*fitr);
            }
            failedImages.clear();

            try_count = 1;
            find = FindImagesWithPossibleResection(vec_possible_resection_indexes);
            BA_TYPE ba_type = eFullBAAll;
            LOG(INFO) << "full ba and try more...";
            double rmse = -1.0;
            bool continueBA = false;
            bool retriangle = false;
            do {
                reject_count = 0;
                rmse = BundleAdjustment(ba_type, std::vector<size_t>());
                ba_view_count = _reconstructed_views.size();
                if (_reconstructed_views.size() >= _options.sfmOptions.reject_points_from_image_count) {
                    reject_count = badTrackRejector(_options.sfmOptions.filter_max_reproj_error);
                }
                // 				if (!retriangle) {
                // 					//retri_count = Retriangulation();
                // 					//LOG(INFO) << " re-add " << retri_count;
                // 					retriangle = true;
                // 				}
                continueBA = (reject_count > _options.sfmOptions.filter_max_reject_count);
            } while (continueBA);
            newResecionImageCount = 0;
            // UpdateReconstructTracks();//update
            UpdateTrackVisibilityPyramid();
        }

        ResectionCallBack(); // give user some fuction to do...
    }

    if (_bExit) {
        return true;
    }
    {
        BA_TYPE ba_type = eFullBAAll;
        LOG(INFO) << "Last full ba...";
        double rmse = -1.0;
        bool continueBA = false;
        do {
            reject_count = 0;
            rmse = BundleAdjustment(ba_type, std::vector<size_t>());
            ba_view_count = _reconstructed_views.size();
            if (_reconstructed_views.size() >= _options.sfmOptions.reject_points_from_image_count) {
                reject_count = badTrackRejector(_options.sfmOptions.filter_max_reproj_error);
            }
            continueBA = (reject_count > 0);
        } while (continueBA);
    }
    // UpdateReconstructTracks();
    if (_reconstructed_views.size() == 0) {
        LOG(INFO) << "failed reconstruction , get 0 views.";
        return false;
    }
    if (_sfm_data->structure.size() < 100) {
        LOG(INFO) << "too less structures " << _sfm_data->structure.size();
        return false; // too less point.
    }

    LOG(INFO) << "Success sfm " << _reconstructed_views.size() << " views in " << _sfm_data->views.size() << "views";

#if 0
	//add structure from tracks
	IndexT idx(0);
	Structure & structure = _sfm_data->structure;
	structure.clear();
	STLMAPTracks &totalTracks = _sfm_tracks._map_total_tracks;
	for (STLMAPTracks::const_iterator itTracks = totalTracks.begin();
		itTracks != totalTracks.end();
		++itTracks, ++idx)
	{
		const submapTrack & track = itTracks->second;
		structure[idx] = Landmark();
		Observations & obs = structure.at(idx).obs;
		for (submapTrack::const_iterator it = track.begin(); it != track.end(); ++it)
		{
			const size_t imaIndex = it->first;
			const size_t featIndex = it->second;
			SIFTImageFeature *pt = _featuresPerImage->at(imaIndex).get();
			obs[imaIndex] = Observation(pt->features.at(featIndex).coords().cast<double>(), featIndex);
		}
	}
	//LOG(INFO) << "triangulate structures" ;
	//SfM_Data_Structure_Computation_Robust structure_estimator(true);
	//structure_estimator.triangulate(*_sfm_data);
#endif
    bool colorStructure = _options.sfmOptions.compute_scene_color;
    if (colorStructure) {
        LOG(INFO) << "colorize sfm points";
        colorizeSfmData(*_sfm_data);
    }

    return true;
}

void SequentialSfM::rebuildTracks()
{
    _sfm_tracks.BuildDatas(*_sfm_data);
}

void insight::SequentialSfM::ResectionCallBack()
{
    for (int i = 0; i < _resectionCallBack.size(); ++i) {
        LOG(INFO) << "Resection callback ...";
        _resectionCallBack[i](_sfm_data);
        LOG(INFO) << "Resection callback finished";
    }
}

/// Compute the initial 3D seed (First camera t=0; R=Id, second estimated by 5 point algorithm)
bool SequentialSfM::MakeInitialPair3D(const Pair& current_pair)
{
    // Compute robust Essential matrix for ImageId [I,J]
    // use min max to have I < J
    const size_t I = std::min(current_pair.first, current_pair.second);
    const size_t J = std::max(current_pair.first, current_pair.second);

    // a. Assert we have valid pinhole cameras
    ViewPtr view_I = _sfm_data->views.at(I);
    ViewPtr view_J = _sfm_data->views.at(J);
    CameraPtr cam_I = view_I->camera;
    CameraPtr cam_J = view_J->camera;

    // b. Get common features between the two view
    // use the track to have a more dense match correspondence set
    // STLMAPTracks map_tracksCommon;
    std::vector<FeatureX> iFeatures, jFeatures;
    std::vector<size_t> vecTrackIds;
    _sfm_tracks.GetTracksIn2Images(I, J, SfmTracks::eTrackNotInLandmark, iFeatures, jFeatures, vecTrackIds);
    CHECK(iFeatures.size() == jFeatures.size());
    CHECK(vecTrackIds.size() == iFeatures.size());
    //-- Copy point to arrays
    const size_t n = vecTrackIds.size();
    Mat xI(2, n), xJ(2, n);
    for (size_t i = 0; i < n; ++i) {
        const FeatureX& xi = iFeatures[i];
        const FeatureX& xj = jFeatures[i];
        xI.col(i) = cam_I->get_ud_pixel(xi.x.cast<double>());
        xJ.col(i) = cam_J->get_ud_pixel(xj.x.cast<double>());
    }
    // c. Robust estimation of the relative pose
    RelativePose_Info relativePose_info;
    relativePose_info.initial_residual_tolerance = Square(4.0);
    const std::pair<size_t, size_t> imageSize_I(cam_I->dbCamera.w, cam_I->dbCamera.h);
    const std::pair<size_t, size_t> imageSize_J(cam_J->dbCamera.w, cam_J->dbCamera.h);

    if (!robustRelativePose(
            cam_I->K(), cam_J->K(), xI, xJ, relativePose_info, imageSize_I, imageSize_J, 4096)) {
        LOG(INFO) << " /!\\ Robust estimation failed to compute E for this pair"
                  << std::endl;
        return false;
    }

    // #if 0
    LOG(INFO) << "A-Contrario initial pair residual: "
              << relativePose_info.found_residual_precision << std::endl;
    // #endif
    //  Bound min precision at 1 pix.
    relativePose_info.found_residual_precision = std::max(relativePose_info.found_residual_precision, 1.0);

    bool bRefine_using_BA = true;
    MarkRegisted(current_pair.first);
    MarkRegisted(current_pair.second);
    if (bRefine_using_BA) {
        // Refine the defined scene
        SfM_Data tiny_scene;
        tiny_scene.views[I] = view_I;
        tiny_scene.views[J] = view_J;
        tiny_scene.cameras[cam_I->dbCamera.id] = cam_I;
        tiny_scene.cameras[cam_J->dbCamera.id] = cam_J;

        PosePtr Pose_I = view_I->pose;
        PosePtr Pose_J = view_J->pose;
        Pose_I->R() = Mat3::Identity();
        Pose_I->C() = Vec3::Zero();
        Pose_J->R() = relativePose_info.R;
        Pose_J->C() = relativePose_info.C;

        Mat34 P1, P2;
        view_I->toP(&P1);
        view_J->toP(&P2);

        Structure& landmarks = tiny_scene.structure;

        for (size_t i = 0; i < n; ++i) {
            // Get corresponding points
            Vec3 X;
            openMVG::TriangulateDLT(P1, cam_I->get_ud_pixel(iFeatures[i].x.cast<double>()),
                P2, cam_J->get_ud_pixel(jFeatures[i].x.cast<double>()), &X);
            Landmark lm;
            lm.obs[I] = Observation(iFeatures[i].x.cast<double>(), iFeatures[i].featureId,iFeatures[i].scale);
            lm.obs[J] = Observation(jFeatures[i].x.cast<double>(), jFeatures[i].featureId,jFeatures[i].scale);
            lm.X = X;
            landmarks[vecTrackIds[i]].swap(lm);
        }

        // - refine only Structure and Rotations & translations (keep intrinsic constant)
        // Bundle_Adjustment_Ceres::BA_options options(true, false);
        AdjustOptions tempOptions = _options.globalAdjustOptions;
        tempOptions.solve_minimizer_progress_to_stdout = 0;
        tempOptions.solve_logging_type = 0; // 0:silent 1: ceres::PER_MINIMIZER_ITERATION;

#ifdef USE_QUATERNION_CERES
        Bundle_Adjustment_Quaternion_Ceres bundle_adjustment_obj(tempOptions);
#else
        Bundle_Adjustment_Ceres bundle_adjustment_obj(tempOptions);
#endif
        bundle_adjustment_obj.constantIntrinsics.insert(cam_I->dbCamera.id);
        bundle_adjustment_obj.constantIntrinsics.insert(cam_J->dbCamera.id);

        if (!bundle_adjustment_obj.Adjust(&tiny_scene)) {
            LOG(INFO) << "Init pair adjust failed.\n";
            MarkUnRegisted(current_pair.first);
            MarkUnRegisted(current_pair.second);
            return false;
        } else {
            LOG(INFO) << "Adjust Initial Landmarks " << tiny_scene.structure.size();
        }

        // Save computed data
        // pose already saved because of shared_ptr of pose
        _map_ACThreshold.insert(std::make_pair(I, relativePose_info.found_residual_precision));
        _map_ACThreshold.insert(std::make_pair(J, relativePose_info.found_residual_precision));

        // List inliers and save them
        for (Structure::const_iterator iter = tiny_scene.structure.begin();
            iter != tiny_scene.structure.end(); ++iter) {
            const IndexT trackId = iter->first;
            const Landmark& landmark = iter->second;
            const Observations& obs = landmark.obs;
            CHECK(obs.size() == 2);

            Observations::const_iterator iterObs_xI = obs.begin();
            Observations::const_iterator iterObs_xJ = obs.begin();
            std::advance(iterObs_xJ, 1);
            if (iterObs_xI->first > iterObs_xJ->first) {
                // swap : image(I < J)
                Observations::const_iterator tempIt = iterObs_xI;
                iterObs_xI = iterObs_xJ;
                iterObs_xJ = tempIt;
            }

            const Observation& ob_xI = iterObs_xI->second;
            const IndexT& viewId_xI = iterObs_xI->first;

            const Observation& ob_xJ = iterObs_xJ->second;
            const IndexT& viewId_xJ = iterObs_xJ->first;

            const Vec2 xi = view_I->camera->get_ud_pixel(ob_xI.x);
            const Vec2 xj = view_J->camera->get_ud_pixel(ob_xJ.x);
            const double angle = SfM_Data::AngleBetweenRay(view_I, view_J, xi, xj);
            const Vec2 residual_I = view_I->residualDistort(landmark.X, ob_xI.x);
            const Vec2 residual_J = view_J->residualDistort(landmark.X, ob_xJ.x);

            double depthI = Pose_I->depth(landmark.X);
            double depthJ = Pose_J->depth(landmark.X);
            if (angle > 2.0 && depthI > 0 && depthJ > 0 && residual_I.norm() < relativePose_info.found_residual_precision && residual_J.norm() < relativePose_info.found_residual_precision) {
                _sfm_data->structure[trackId] = landmarks[trackId];
                _sfm_tracks.MarkTrack(trackId, SfmTracks::eTrackInLandmark);
            }
        }
        // Save outlier residual information
        if (_sfm_data->structure.empty()) {
            MarkUnRegisted(current_pair.first);
            MarkUnRegisted(current_pair.second);
            LOG(INFO) << "Init pair structure count is 0 \n";
            return false;
        } else {
            LOG(INFO) << "Initial #3D points: " << _sfm_data->structure.size();
        }
    }

    if (_sfm_data->structure.size() < 50) {
        LOG(INFO) << "Too less structure in initial landmarks : " << _sfm_data->structure.size();
        return false;
    }
    return !_sfm_data->structure.empty();
}

void insight::SequentialSfM::CheckTracks()
{
    for (auto itr = _sfm_data->structure.begin(); itr != _sfm_data->structure.end(); ++itr) {
        CHECK(_sfm_tracks.GetMark(itr->first) == SfmTracks::eTrackInLandmark);
    }
}

void SequentialSfM::InitRemainViewList()
{
    for (Views::const_iterator itV = _sfm_data->views.begin();
        itV != _sfm_data->views.end(); ++itV) {
        ViewPtr view = itV->second;
        if (!view->dbImage.pose_valid) {
            _set_remainingViewId.insert(itV->first);
        }
    }
}

bool SequentialSfM::FindImagesWithPossibleResection(std::vector<size_t>& vec_possible_indexes)
{

    vec_possible_indexes.clear();
    if (_set_remainingViewId.empty() || _sfm_data->structure.empty()) {
        return false;
    }
    // Collect tracksIds
    // std::set<uint32_t> &reconstructed_trackId = _reconstructed_trackId;
    // Estimate the image on which we could compute a resection safely
    // -> We first find the camera with the greatest number of matches
    //     with the current 3D existing 3D point => M
    // -> Then add any camera with at least 0.75M matches.
    // Keep only the best one.
    const int next_resection_method = _options.sfmOptions.next_resection_method;
    const double dPourcent = _options.sfmOptions.relation_match_precent;
    if (next_resection_method == MOST_RELATION_POINTS) {
        LOG(INFO) << "Select images by MOST_RELATION_POINTS " << dPourcent << "\n ";
        Pair_Vec vec_putative; // ImageId, NbPutativeCommonPoint
        vec_putative.reserve(100);
#pragma omp parallel
        for (std::unordered_set<uint32_t>::const_iterator iter = _set_remainingViewId.begin();
            iter != _set_remainingViewId.end(); ++iter) {
#pragma omp single nowait
            {
                const size_t viewId = *iter;
                // Compute 2D - 3D possible content
                std::vector<size_t> vecTrackIds;
                _sfm_tracks.GetTracksInImage(viewId, SfmTracks::eTrackInLandmark, vecTrackIds);

                if (!vecTrackIds.empty()) {
                    // Count the common possible putative point
                    //  with the already 3D reconstructed trackId
#pragma omp critical
                    {
                        vec_putative.push_back(std::make_pair(viewId, vecTrackIds.size()));
                    }
                }
            }
        }

        if (vec_putative.empty()) {
            return false;
        } else {
            sort(vec_putative.begin(), vec_putative.end(), sort_pair_second<size_t, size_t, std::greater<size_t>>());

            const IndexT M = vec_putative[0].second;
            vec_possible_indexes.push_back(vec_putative[0].first);
            const size_t min_points_for_resection = 30; // min points for resection
            size_t threshold = std::max(min_points_for_resection, static_cast<size_t>(dPourcent * M));
            bool tryMoreImageForResection = _reconstructed_views.size() > _options.sfmOptions.min_image_count_per_camera_for_find_more_resection;
            LOG(INFO) << "ori-common points for resection: " << threshold;
            if (tryMoreImageForResection) {
                int th = std::min<int>(dPourcent * M, _options.sfmOptions.relation_common_structure_count);
                threshold = std::max<int>(min_points_for_resection, th);
            }
            LOG(INFO) << "refined-common points for resection: " << threshold;
            for (size_t i = 1; i < vec_putative.size() && vec_putative[i].second > threshold;
                ++i) {
                vec_possible_indexes.push_back(vec_putative[i].first);
            }
            LOG(INFO) << "Total views: " << vec_possible_indexes.size();
            return true;
        }
    } else if (next_resection_method == VISIBILITY_PYRAMID_SCORE) {
        // selet by view graph first
        std::set<size_t> possibleIndex;
        const std::unordered_map<IndexT, std::vector<IndexT>>& viewGraph = _sfm_tracks.ViewGraph();
        int viewGraphAdd = 0;
        for (auto itr = viewGraph.begin(); itr != viewGraph.end(); ++itr) {
            if (_vec_viewStatus[itr->first]) {
                continue;
            }
            int iConnect = 0;
            for (auto itrCon = itr->second.begin(); itrCon != itr->second.end(); ++itrCon) {
                if (_vec_viewStatus[*itrCon]) {
                    ++iConnect;
                }
            }
            if (iConnect == itr->second.size()) {
                vec_possible_indexes.push_back(itr->first);
                possibleIndex.insert(itr->first);
                ++viewGraphAdd;
            }
        }
        Pair_Vec vec_putativeScore, vecputativePtCount; // ImageId, NbPutativeCommonPoint
        LOG(INFO) << "Select images by VISIBILITY_PYRAMID_SCORE " << dPourcent << "\n ";
        for (std::unordered_set<uint32_t>::const_iterator iter = _set_remainingViewId.begin();
            iter != _set_remainingViewId.end(); ++iter) {
            vec_putativeScore.push_back(std::make_pair(*iter, _sfm_tracks.ImageScore(*iter)));
            vecputativePtCount.push_back(std::make_pair(*iter, _sfm_tracks.ImagePtCount(*iter)));
        }
        if (vec_putativeScore.empty()) {
            return (viewGraphAdd != 0);
        } else {
            sort(vec_putativeScore.begin(), vec_putativeScore.end(), sort_pair_second<size_t, size_t, std::greater<size_t>>());
            sort(vecputativePtCount.begin(), vecputativePtCount.end(), sort_pair_second<size_t, size_t, std::greater<size_t>>());

            int add = 0;
            const IndexT M = vec_putativeScore[0].second;
            if (possibleIndex.find(vec_putativeScore[0].first) == possibleIndex.end()) {
                vec_possible_indexes.push_back(vec_putativeScore[0].first);
                possibleIndex.insert(vec_putativeScore[0].first);
                ++add;
            }

            const size_t threshold = static_cast<size_t>(dPourcent * M);
            for (size_t i = 1; i < vec_putativeScore.size() && vec_putativeScore[i].second > threshold;
                ++i) {
                if (possibleIndex.find(vec_putativeScore[0].first) == possibleIndex.end()) {
                    vec_possible_indexes.push_back(vec_putativeScore[i].first);
                    possibleIndex.insert(vec_putativeScore[i].first);
                    ++add;
                }
            }
            int newadd = 0;
            // if(_reconstructed_views.size() > _options.sfmOptions.min_image_count_per_camera_for_find_more_resection)
            // check every image_count_per_camera,

            // important:!!! if the image is oblique, its important to have the focal length to use tryMoreImageForResection option;
            // use tryMoreImageForResection is good policy for accelerate the speed of SfM
            bool tryMoreImageForResection = _reconstructed_views.size() > _options.sfmOptions.min_image_count_per_camera_for_find_more_resection;

            // LOG(INFO)<<"Reconstruct per camera:";
            // for (auto it = _reconstructed_views_per_intrisic.begin(); it != _reconstructed_views_per_intrisic.end(); ++it)
            // {
            // 	LOG(INFO)<<it->first <<"\t : "<<it->second;
            // 	if (it->second < _options.sfmOptions.min_image_count_per_camera_for_find_more_resection)
            // 	{
            // 		tryMoreImageForResection = false;
            // 		//TODO check the total image count of per camera
            // 		//maybe one camera has only few image less than min_image_count_per_camera_for_find_more_resection
            // 		// break;
            // 	}
            // }

            if (tryMoreImageForResection) {
                for (size_t i = 0; i < vecputativePtCount.size(); ++i) {
                    if (vecputativePtCount[i].second < _options.sfmOptions.relation_common_structure_count)
                        break;

                    if (possibleIndex.find(vecputativePtCount[i].first) == possibleIndex.end()) {
                        vec_possible_indexes.push_back(vecputativePtCount[i].first);
                        ++newadd;
                    }
                }
            }
            LOG(INFO) << "Find " << vec_possible_indexes.size() << "[" << viewGraphAdd << "+" << add << "+" << newadd << "] for resection";
            return true;
        }
    } else {
        CHECK(false) << "Error: unknown next resection method " << next_resection_method << std::endl;
        return false;
    }
}

void SequentialSfM::ResetBACount()
{
    _nLastFullBACount = 0;
    _nLastFullDistortBACount = 0;
}

void SequentialSfM::InitReconstructedViews()
{
    size_t imageId = 0;
    for (auto itr = _sfm_data->views.begin();
        itr != _sfm_data->views.end(); ++itr) {
        imageId = std::max<size_t>(itr->first, imageId);
    }
    _vec_viewStatus.resize(imageId + 1, 0);

    _reconstructed_views.clear();
    _reconstructed_views_per_intrisic.clear();

    // don't use sfm_data->intrisic to intialize the _reconstructed_views_per_intrisic
    // there may be no image in some intrisics
    for (Views::const_iterator it = _sfm_data->views.begin();
        it != _sfm_data->views.end(); ++it) {
        _reconstructed_views_per_intrisic[it->second->dbImage.camera_id] = 0;
    }
    for (Views::const_iterator it = _sfm_data->views.begin();
        it != _sfm_data->views.end(); ++it) {
        ViewPtr view = it->second;
        if (view->dbImage.pose_valid) {
            _reconstructed_views.insert(view->dbImage.id);
            ++_reconstructed_views_per_intrisic[view->dbImage.camera_id];
            _vec_viewStatus[view->dbImage.id] = 1;
        }
    }
}

bool SequentialSfM::Resection(size_t imageIndex)
{

    const size_t viewIndex = imageIndex;
    // LOG(INFO) << "Register " << viewIndex;
    // Compute 2D - 3D possible content
    // a. list tracks ids used by the view
    // b. intersects the track list with the reconstructed

    // a. list tracks ids used by the view

    std::vector<FeatureX> vec_featForResection;
    std::vector<size_t> vec_tracksIds;
    if (!_sfm_tracks.GetTracksInImage(viewIndex, SfmTracks::eTrackInLandmark, vec_featForResection, vec_tracksIds)) {
        LOG(INFO) << "No common track for " << imageIndex;
        return false;
    }

    // b. intersects the track list with the reconstructed
    // Get back featId and tracksID that will be used for the resection
    // Create pt2D, and pt3D array
    Mat pt2D(2, vec_featForResection.size());
    Mat pt3D(3, vec_featForResection.size());
    const ViewPtr view_I = _sfm_data->views.at(viewIndex);

    // Look if intrinsic data is known or not
    // bool bKnownIntrinsic = true;

    const CameraPtr cam_I = view_I->camera;
    Mat3 K = cam_I->K();

#pragma omp parallel for
    for (int i = 0; i < (int)vec_featForResection.size(); ++i) {
        const FeatureX& featX = vec_featForResection[i];
        pt3D.col(i) = _sfm_data->structure.at(vec_tracksIds[i]).X;
        const Vec2 feat = featX.x.cast<double>();
        pt2D.col(i) = cam_I->get_ud_pixel(feat);
    }
    //-------------
    std::vector<size_t> vec_inliers;
    Mat34 P;
    double errorMax = std::numeric_limits<double>::max();

    bool bResection = robustResection(
        std::make_pair(view_I->camera->dbCamera.w, view_I->camera->dbCamera.h),
        pt2D, pt3D,
        &vec_inliers,
        &K,
        &P, &errorMax);

    if (!bResection) {
        LOG(INFO) << "Resection " << imageIndex << " failed";
        return false;
    }

    //-- Refine the found pose
    // Decompose P matrix
    Mat3 K_, R_;
    Vec3 t_;
    openMVG::KRt_From_P(P, &K_, &R_, &t_);
    view_I->pose->R() = R_;
    view_I->pose->C() = -R_.transpose() * t_;
    view_I->pose->updateDBPose();
    // Create a SfM_DataScene with one camera and the 3D points
    SfM_Data tiny_scene;
    tiny_scene.views[view_I->dbImage.id] = view_I;
    tiny_scene.cameras[cam_I->dbCamera.id] = cam_I;
    MarkRegisted(imageIndex);
    // Landmarks

#pragma omp parallel for
    for (int i = 0; i < (int)vec_inliers.size(); ++i) {
        const size_t idx = vec_inliers[i];
        Landmark landmark;
        landmark.X = pt3D.col(idx);
        FeatureX& feat = vec_featForResection[idx];
        Observation ob(feat.x.cast<double>(), feat.featureId, feat.scale);
        landmark.obs[view_I->dbImage.id] = ob;
#pragma omp critical
        {
            tiny_scene.structure[i].swap(landmark);
        }
    }

    AdjustOptions tempOptions = _options.globalAdjustOptions;
    // fix intrinsics and structure
    tempOptions.fix_structure = true;
#ifdef USE_QUATERNION_CERES
    Bundle_Adjustment_Quaternion_Ceres bundle_adjustment_obj(tempOptions);
#else
    Bundle_Adjustment_Ceres bundle_adjustment_obj(tempOptions);
#endif
    bundle_adjustment_obj.constantIntrinsics.insert(cam_I->dbCamera.id);
    bool detailLog = false;
    if (!bundle_adjustment_obj.Adjust(&tiny_scene, detailLog)) {
        LOG(INFO) << "Adjust for resection " << imageIndex << " failed\n";
        MarkUnRegisted(imageIndex);
        return false;
    }

    //---
    // Update the global scene
    //--
    // - With the found camera pose
    // shared ptr ,so don't need  update pose
    _map_ACThreshold.insert(std::make_pair(viewIndex, errorMax));
    // LOG(INFO) << "Resection[10] ";

    //--
    // Update global scene structure
    //--

    // Add new entry to reconstructed track and
    //  remove outlier from the tracks
    int n = vec_featForResection.size();
    // int errPix = std::max(4.0, errorMax);
#pragma omp parallel for
    for (int i = 0; i < n; ++i) {
        size_t trackId = vec_tracksIds[i];
        FeatureX& feat = vec_featForResection[i];
        const CameraPtr cam = view_I->camera;
        const PosePtr pose = view_I->pose;
        const Vec3 X = pt3D.col(i);
        const Vec2 x = feat.x.cast<double>();
        const Vec2 residual = view_I->residualDistort(X, x);
#pragma omp critical
        {
            if (residual.norm() < errorMax && pose->depth(X) > 0) {
                // Inlier, add the point to the reconstructed track
                _sfm_data->structure[trackId].obs[viewIndex] = Observation(x, feat.featureId, feat.scale);
                CHECK(_sfm_tracks.GetMark(trackId) == SfmTracks::eTrackInLandmark) << "Logic error";
            }
        }
    }

    // Add new possible tracks (triangulation)
    // Triangulate new possible tracks:
    // For all Union [ CurrentId, [PreviousReconstructedIds] ]
    //   -- If trackId yet registered:
    //      -- add visibility information
    //   -- If trackId not yet registered:
    //      -- Triangulate and add a new track.
    // For all reconstructed image look if common content in the track
    // const std::unordered_set<uint32_t> &valid_views = _reconstructed_views;
    const std::unordered_map<IndexT, std::vector<IndexT>>& viewGraph = _sfm_tracks.ViewGraph();
    auto findConnectItr = viewGraph.find(imageIndex);
    if (findConnectItr == viewGraph.end()) {
        LOG(INFO) << "Can't resection " << imageIndex << " , no relation images find ";
        return false;
    }
    const std::vector<IndexT>& releIndexs = findConnectItr->second;

    // std::unordered_map<IndexT, std::vector<size_t> > map_tracksNew;
    std::vector<IndexT> valid_views;
    for (size_t i = 0; i < releIndexs.size(); ++i) {
        if (_reconstructed_views.find(releIndexs[i]) != _reconstructed_views.end()) {
            valid_views.push_back(releIndexs[i]);
        }
    }

    // LOG(INFO) << "Resection[11] ";

#pragma omp parallel
    for (const IndexT& indexI : valid_views) {
        if (indexI == viewIndex) {
            continue;
        }
#pragma omp single nowait
        {
            const size_t I = std::min<size_t>(viewIndex, indexI);
            const size_t J = std::max<size_t>(viewIndex, indexI);
            // Compute possible content (match between indexI, indexJ)
            std::vector<FeatureX> iFeatsNew, jFeatsNew;
            std::vector<size_t> vec_tracksNew;
            _sfm_tracks.GetTracksIn2Images(I, J, SfmTracks::eTrackNotInLandmark, iFeatsNew, jFeatsNew, vec_tracksNew);
            Triangulation(I, J, vec_tracksNew, iFeatsNew, jFeatsNew);
        }
    }
    // LOG(INFO) << "Resection[12] ";

    // CheckTracks();

    return true;
}

void SequentialSfM::Triangulation(const size_t I, const size_t J,
    std::vector<size_t>& vec_tracks, std::vector<FeatureX>& iFeats,
    std::vector<FeatureX>& jFeats)
{
    CHECK(I < J) << "I < J";
    CHECK(iFeats.size() == jFeats.size());
    CHECK(iFeats.size() == vec_tracks.size());
    if (!vec_tracks.empty()) {
        const ViewPtr view_1 = _sfm_data->views.at(I);
        const ViewPtr view_2 = _sfm_data->views.at(J);
        const CameraPtr cam_1 = view_1->camera;
        const CameraPtr cam_2 = view_2->camera;
        const PosePtr pose_1 = view_1->pose;
        const PosePtr pose_2 = view_2->pose;
        Mat34 P1, P2;
        view_1->toP(&P1);
        view_2->toP(&P2);
        for (int i = 0; i < vec_tracks.size(); ++i) {
            const size_t trackId = vec_tracks[i];
            // Get corresponding points and triangulate it
            const Vec2 x1 = iFeats[i].x.cast<double>();
            const Vec2 x2 = jFeats[i].x.cast<double>();
            const Vec2 x1_ud = cam_1->get_ud_pixel(x1);
            const Vec2 x2_ud = cam_2->get_ud_pixel(x2);
            Vec3 X_euclidean = Vec3::Zero();
            openMVG::TriangulateDLT(P1, x1_ud, P2, x2_ud, &X_euclidean);
#if 0
			else {
				_triMutex.lock();
				X_euclidean = _sfm_data->structure.at(trackId).X;
				_triMutex.unlock();
			}
#endif
            // Check triangulation results
            //  - Check angle (small angle leads imprecise triangulation)
            //  - Check positive depth
            //  - Check residual values
            if (pose_1->depth(X_euclidean) > 0 && pose_2->depth(X_euclidean) > 0) {
                const double angle = _sfm_data->AngleBetweenRay(view_1, view_2, x1_ud, x2_ud);
                if (angle > _options.sfmOptions.filter_min_tri_angle) {
                    const Vec2 residual_1 = view_1->residualDistort(X_euclidean, x1);
                    const Vec2 residual_2 = view_2->residualDistort(X_euclidean, x2);
                    if (
                        residual_1.norm() < std::max(4.0, _map_ACThreshold[I]) && residual_2.norm() < std::max(4.0, _map_ACThreshold[J])) {
                        _triMutex.lock();
                        // newAddTracks.push_back(trackId);
                        _sfm_tracks.MarkTrack(trackId, SfmTracks::eTrackInLandmark);
                        Landmark& pt3d = _sfm_data->structure[trackId];
                        pt3d.X = X_euclidean;
                        pt3d.obs[I] = Observation(x1, iFeats[i].featureId,jFeats[i].scale);
                        pt3d.obs[J] = Observation(x2, jFeats[i].featureId,jFeats[i].scale);
                        _trackChanged.insert(trackId);
                        _triMutex.unlock();
                    }
                }
            }
        }
    }
}

#if 0
size_t SequentialSfM::Retriangulation()
{
	size_t num_tris = 0;

	TriangulationOptions re_options = _options.triangulationOptions;
	//const sfm::Views &views = _sfm_data.GetViews();
	const PairWiseMatches &map_Matches = *_matchesPerPair;
	for (PairWiseMatches::const_iterator mItr = map_Matches.begin();
		mItr != map_Matches.end(); ++mItr)
	{
		Pair image_pair(std::min(mItr->first.first, mItr->first.second), std::max(mItr->first.first, mItr->first.second));

		// Only perform retriangulation for under-reconstructed image pairs.
		const double tri_ratio =
			static_cast<double>(_pairTrianglulations[image_pair].first) / _pairTrianglulations[image_pair].second;

		bool createNew = true;
		if (tri_ratio >= re_options.re_min_ratio) {
			createNew = false;
		}
		IndexT I = image_pair.first;
		IndexT J = image_pair.second;
		// Check if images are registered yet.
        ViewPtr viewI = _sfm_data->views.at(I);
        ViewPtr viewJ = _sfm_data->views.at(J);

		if (!viewI->dbImage.pose_valid
			|| !viewJ->dbImage.pose_valid) {
			continue;
		}

		// Extract camera poses.
		Mat34 P1;
		Mat34 P2;
		viewI->toP(&P1);
		viewJ->toP(&P2);

		size_t& num_re_trials = _re_num_trials[image_pair];

		if (num_re_trials >= re_options.re_max_trials) {
			continue;
		}
		num_re_trials += 1;

		// Find correspondences and perform re-triangulation.
		const IndMatches &indMatches = _matchesPerPair->at(image_pair);
		for (const auto &mm : indMatches)
		{
			SfmTracks::Obv imageId_featureId1(I, mm._i);
			SfmTracks::Obv imageId_featureId2(J, mm._j);

			auto findItr1 = _sfm_tracks._map_obv_track.find(imageId_featureId1);
			auto findItr2 = _sfm_tracks._map_obv_track.find(imageId_featureId2);

			//invalid obv because of track filter
			if (findItr1 == _sfm_tracks._map_obv_track.end() ||
				findItr2 == _sfm_tracks._map_obv_track.end()) {
				continue;
			}

			CorrData corr_data1;
			SIFTImageFeature &imgFeatsI = *_featuresPerImage->at(I);
			SIFTImageFeature &imgFeatsJ = *_featuresPerImage->at(J);

			corr_data1.feat = viewI->camera->get_ud_pixel(imgFeatsI.features[mm._i].coords().cast<double>());
			corr_data1.camera = viewI->camera;
			corr_data1.obv = imageId_featureId1;
			corr_data1.P = P1;

			corr_data1.pose = viewI->pose;
			corr_data1.trackID = findItr1->second.second;

			CorrData corr_data2;
			corr_data2.feat = viewJ->camera->get_ud_pixel(imgFeatsJ.features[mm._j].coords().cast<double>());
			corr_data2.camera = viewJ->camera;
			corr_data2.obv = imageId_featureId2;
			corr_data2.P = P2;
			corr_data2.pose = viewJ->pose;
			corr_data2.trackID = findItr2->second.second;
			CHECK(corr_data2.trackID == corr_data1.trackID);
			// Two cases are possible here: both points belong to the same 3D point
			// or to different 3D points. In the former case, there is nothing
			// to do. In the latter case, we do not attempt retriangulation,
			// as retriangulated correspondences are very likely bogus and
			// would therefore destroy both 3D points if merged.

			//����������ά��
			if (findItr1->second.first != SfmTracks::eObvNonReconstruct &&
				findItr2->second.first != SfmTracks::eObvNonReconstruct) {
				continue;
			}

			if (findItr1->second.first != SfmTracks::eObvNonReconstruct &&
				findItr2->second.first == SfmTracks::eObvNonReconstruct) {
				const std::vector<CorrData> corrs_data1 = { corr_data1 };
				num_tris += Continue(re_options, corr_data2, corrs_data1);
			}
			else if (findItr1->second.first == SfmTracks::eObvNonReconstruct &&
				findItr2->second.first != SfmTracks::eObvNonReconstruct) {
				const std::vector<CorrData> corrs_data2 = { corr_data2 };
				num_tris += Continue(re_options, corr_data1, corrs_data2);
			}
#if 0 // do not create tracks
			else if (findItr1->second.first == SfmTracks::eObvNonReconstruct &&
				findItr2->second.first == SfmTracks::eObvNonReconstruct && createNew) {

				num_tris += Create(re_options, corr_data1, corr_data2);
			}
#endif
		}

	}
	//std::cout << "re-triangle: " << num_tris << " \n";
	return num_tris;
}
#endif

void SequentialSfM::MarkRegisted(size_t imageIndex)
{
    _sfm_data->views.at(imageIndex)->dbImage.pose_valid = true;
    _reconstructed_views.insert(imageIndex);
    ++_reconstructed_views_per_intrisic[_sfm_data->views.at(imageIndex)->dbImage.camera_id];
    _vec_viewStatus[imageIndex] = 1;
}

void insight::SequentialSfM::MarkUnRegisted(size_t imageIndex)
{
    _sfm_data->views.at(imageIndex)->dbImage.pose_valid = false;
    _reconstructed_views.erase(imageIndex);
    int& countPerCam = _reconstructed_views_per_intrisic[_sfm_data->views.at(imageIndex)->dbImage.camera_id];
    --countPerCam;
    if (countPerCam < 0)
        countPerCam = 0;
    _vec_viewStatus[imageIndex] = 0;
}

BA_TYPE SequentialSfM::CheckBAType()
{
    if (_set_remainingViewId.empty())
        return eFullBAAll; // û��ͼ����

    const int nPose = _reconstructed_views.size();
    int addedCount = nPose - _nLastFullBACount;
    double addThre1 = _options.localBAOptions.full_ba_percent * _nLastFullBACount;
    double addThre2 = _options.localBAOptions.max_image_for_full_ba;

    if (_options.sfmOptions.adaptive_self_distortion && nPose < _options.sfmOptions.adaptive_self_distortion_k1k2k3_image_count) {
        LOG(INFO) << "full BA K1K2K3:" << nPose << "[" << _options.sfmOptions.adaptive_self_distortion_k1k2k3_image_count << "]";
        return eFullBAK1K2K3;
    }

    if (_options.localBAOptions.enable_local_ba && (nPose > _options.localBAOptions.local_ba_num_from)) {
        // maybe partial
        if (addedCount < addThre1 && addedCount < addThre2) {
            LOG(INFO) << "Local BA Views: " << addedCount << ", t1=" << addThre1 << ",t2=" << addThre2;
            return ePartialBA;
        }
    }
#if PBA_GPU
    if (nPose > 100) {
        return eFULLPBA;
    }
#endif
    return eFullBAAll;

#if 0
	// 	if (_options.sfmOptions.adaptive_self_distortion && nPose < _options.sfmOptions.adaptive_self_distortion_k1k2k3_image_count)
	// 	{
	// 		return eFullBAFocalPrincipoint;
	// 	}
	//	else
	if (_options.sfmOptions.adaptive_self_distortion && nPose < _options.sfmOptions.adaptive_self_distortion_p1p2_b1_b2_image_count)
	{
		return eFullBAK1K2K3;
	}
	else
	{
		int nNewAddForUndistort = nPose - _nLastFullDistortBACount;
		//if (nNewAddForUndistort < addThre2) return eFULLPBA;
		return eFullBAAll;
	}
#endif
}

double SequentialSfM::BundleAdjustment(BA_TYPE ba_type, const std::vector<size_t>& newRegistedImage)
{
    CHECK(ba_type < eBA_END);
    CHECK(ba_type > eBA_BEGIN);
    // check adaptive self distortion
    double rmse = -1.0; // means don't do 3sigma filter

    if (ba_type == ePartialBA) {

#if 0
		std::vector<size_t> constViews;
		for (size_t i = 0; i < newRegistedImage.size(); ++i) {
			std::vector<IndexT> local_bundle = FindLocalBundle(newRegistedImage[i]);
			local_bundle.push_back(newRegistedImage[i]);
			//totalLocalBundle.insert(totalLocalBundle.end(), local_bundle.begin(), local_bundle.end());
			if (local_bundle.size() > 2) {
				constViews.push_back(local_bundle[0]);
				constViews.push_back(local_bundle[1]);
			}
			else if (local_bundle.size() > 1) {
				constViews.push_back(local_bundle[0]);
			}
			std::set<IndexT> changedLandMarks;
			//perform local BA ,No intrinsic modified.
			BundleAdjustmentSub(local_bundle, constViews, changedLandMarks);
			rmse = computeRMSE(*_sfm_data);
			LOG(INFO) << "Partial BA RMSE=" << rmse ;
		}
		return rmse;
#else
        std::vector<size_t> constViews;
        std::set<IndexT> totalLocalBundle;
        for (size_t i = 0; i < newRegistedImage.size(); ++i) {
            totalLocalBundle.insert(newRegistedImage[i]);
            std::vector<IndexT> local_bundle = FindLocalBundle(newRegistedImage[i]);
            for (size_t j = 0; j < local_bundle.size(); ++j) {
                totalLocalBundle.insert(local_bundle[j]);
            }
        }
        std::vector<IndexT> vecTotalImage(totalLocalBundle.begin(), totalLocalBundle.end());
        if (!vecTotalImage.empty()) {
            constViews.push_back(vecTotalImage[0]);
        }
        if (vecTotalImage.size() > 2) {
            constViews.push_back(vecTotalImage[1]);
        }
        std::set<IndexT> changedLandMarks;
        BundleAdjustmentSub(vecTotalImage, constViews, changedLandMarks);
        rmse = computeRMSE(*_sfm_data);
        LOG(INFO) << "Partial BA RMSE=" << rmse;
#endif
    } else if (ba_type == eFullBAAll) {
        double ceresError = BundleAdjustmentCeres(ba_type);
        rmse = computeRMSE(*_sfm_data);
        LOG(INFO) << "ceresError=" << ceresError << ",rmse=" << rmse;
        // printf("ceresError=%f,rmse=%f\n", ceresError, rmse);
        _nLastFullBACount = _reconstructed_views.size();
    }
#if PBA_GPU
    else if (ba_type == eFULLPBA) {
        rmse = BundleAdjustmentPBA();
        _nLastFullDistortBACount = _reconstructed_views.size();
        _nLastFullBACount = _reconstructed_views.size();
    }
#endif
    return rmse;
}

double SequentialSfM::BundleAdjustmentCeres(BA_TYPE ba_type)
{
    Timer t;
    t.start();
#ifdef USE_QUATERNION_CERES
    Bundle_Adjustment_Quaternion_Ceres bundle_adjustment_obj(_options.globalAdjustOptions);
#else
    Bundle_Adjustment_Ceres bundle_adjustment_obj(_options.globalAdjustOptions);
#endif
    bool bOk = false;
    if (ba_type == eFullBAAll) {
        bOk = bundle_adjustment_obj.Adjust(_sfm_data);
    } else if (ba_type == eFullBANoDistort) {
        // constant intrinsics
        LOG(INFO) << "eFullBANoDistort";
        for (auto intri : _sfm_data->cameras) {
            bundle_adjustment_obj.constantIntrinsics.insert(intri.first);
        }
        bOk = bundle_adjustment_obj.Adjust(_sfm_data);
    } else if (ba_type == eFullBAK1K2K3) {
        AdjustOptions bkOptions = _options.globalAdjustOptions;
        bkOptions.fix_f = false;
        bkOptions.fix_ppxy = false;
        bkOptions.fix_k1k2k3 = false;
        bkOptions.fix_p1p2 = true; // fix p1p2 and b1b2 only adjust f, ppxy,and k1,k2,k3 if set.
        bkOptions.fix_b1b2 = true;
#ifdef USE_QUATERNION_CERES
        bundle_adjustment_obj = Bundle_Adjustment_Quaternion_Ceres(bkOptions);
#else
        bundle_adjustment_obj = Bundle_Adjustment_Ceres(bkOptions);
#endif
        bOk = bundle_adjustment_obj.Adjust(_sfm_data);
    } else if (ba_type == eFullBAIntrinsic) {
        AdjustOptions opt = _options.globalAdjustOptions;
        opt.fix_view = true;
        opt.fix_structure = true;
#ifdef USE_QUATERNION_CERES
        bundle_adjustment_obj = Bundle_Adjustment_Quaternion_Ceres(opt);
#else
        bundle_adjustment_obj = Bundle_Adjustment_Ceres(opt);
#endif
        bOk = bundle_adjustment_obj.Adjust(_sfm_data);
    } else {
        CHECK(false) << "Logic error!\n";
    }
    double rmse = -1;
    if (bOk) {
        rmse = bundle_adjustment_obj.RMSE();
    }

    /*for (Intrinsics::const_iterator itIntrinsic = _sfm_data.intrinsics.begin();
            itIntrinsic != _sfm_data.intrinsics.end(); ++itIntrinsic)
            {
            LOG(INFO) << "INTRINSIC[";
            std::vector<double> params;
            params = itIntrinsic->second->getParams();
            for (int i = 0; i < params.size(); ++i)
            {
            LOG(INFO) << params[i] << " ";
            }
            LOG(INFO) << "]\n";
            }*/

    // log intrinsics
    if (!bOk) {
        LOG(INFO) << "BA Error !\n";
    }
    LOG(INFO) << "BA " << _reconstructed_views.size() << " Spend " << t.elapsed() << "s";
    return rmse;
}

std::vector<IndexT> SequentialSfM::FindLocalBundle(const IndexT viewIndex) const
{
    CHECK(_reconstructed_views.find(viewIndex) != _reconstructed_views.end()) << "logic error";
    // Extract all images that have at least one 3D point with the query image
    // in common, and simultaneously count the number of common 3D points.
    std::unordered_map<IndexT, size_t> num_shared_observations;
    {
        // a. list tracks ids used by the view
        // b. intersects the track list with the reconstructed

        // a. list tracks ids used by the view

        STLMAPTracks map_tracksCommon;
        // std::set<size_t> set_tracksIds;
        std::vector<size_t> vec_tracksIds;
        if (!_sfm_tracks.GetTracksInImage(viewIndex, SfmTracks::eTrackInLandmark, vec_tracksIds)) {
            CHECK(false) << "logic error";
        }

        for (size_t trackId : vec_tracksIds) {
            auto findTrackItr = _sfm_data->structure.find(trackId);
            CHECK(findTrackItr != _sfm_data->structure.end());
            {
                const Observations& obs = findTrackItr->second.obs;
                for (auto ob : obs) {
                    if (ob.first != viewIndex) {
                        if (num_shared_observations.find(ob.first) == num_shared_observations.end()) {
                            num_shared_observations[ob.first] = 1;
                        } else {
                            ++num_shared_observations[ob.first];
                        }
                    }
                }
            }
        }
    }

    std::vector<std::pair<IndexT, size_t>> local_bundle;
    for (const auto elem : num_shared_observations) {
        local_bundle.emplace_back(elem.first, elem.second);
    }

    // The local bundle is composed of the given image and its most connected
    // neighbor images, hence the subtraction of 1.
    const size_t num_images = static_cast<size_t>(_options.localBAOptions.local_ba_num_images - 1);
    const size_t num_eff_images = std::min(num_images, local_bundle.size());

    // Sort according to number of common 3D points.
    std::partial_sort(local_bundle.begin(), local_bundle.begin() + num_eff_images,
        local_bundle.end(),
        [](const std::pair<IndexT, size_t>& image1,
            const std::pair<IndexT, size_t>& image2) {
            return image1.second > image2.second;
        });

    // Extract most connected images.
    std::vector<IndexT> image_ids(num_eff_images);
    for (size_t i = 0; i < num_eff_images; ++i) {
        image_ids[i] = local_bundle[i].first;
    }
    return image_ids;
}

void SequentialSfM::ClearNewAddedTracksInResection()
{
    _trackChanged.clear();
}

size_t SequentialSfM::badTrackRejector(double dPrecision)
{
    // Remove observation/tracks that have:
    // - too large residual error,
    // - too small angular value.
    return RemoveOutliers_PixelResidualErrorMulti(*_sfm_data, _sfm_tracks, dPrecision, 2) + RemoveOutliers_AngleErrorMulti(*_sfm_data, _sfm_tracks, 2.0);
}

void SequentialSfM::GetLandmarksInImages(const Structure& inLandmarks, const std::set<IndexT>& set_imageIndex, Structure& outLandmarks, int max_track_length /*= 15 */)
{
    for (Structure::const_iterator itr = inLandmarks.begin();
        itr != inLandmarks.end(); ++itr) {
        for (std::set<IndexT>::const_iterator imgItr = set_imageIndex.begin();
            imgItr != set_imageIndex.end(); ++imgItr) {
            if (itr->second.obs.size() < max_track_length && itr->second.obs.find(*imgItr) != itr->second.obs.end()) {
                outLandmarks[itr->first] = itr->second;
                break;
            }
        }
    }
}

void SequentialSfM::subData(const SfM_Data& sfm_data, SfM_Data& sub_data, const std::set<IndexT>& subView, int max_track_length /*= 15*/)
{
    sub_data.views.clear();
    sub_data.structure.clear();
    GetLandmarksInImages(sfm_data.structure, subView, sub_data.structure, max_track_length);

    for (Structure::iterator itr = sub_data.structure.begin();
        itr != sub_data.structure.end(); ++itr) {
        Landmark& lm = itr->second;
        for (Observations::const_iterator iob = lm.obs.begin();
            iob != lm.obs.end(); ++iob) {
            sub_data.views[iob->first] = sfm_data.views.at(iob->first);
        }
    }
}

void SequentialSfM::mergeData(SfM_Data& target_data, const SfM_Data& source_data, const std::map<IndexT, bool>& map_bRefineStructures)
{
    for (Structure::const_iterator itr = source_data.structure.begin();
        itr != source_data.structure.end(); ++itr) {
        auto struItr = map_bRefineStructures.find(itr->first);
        if (struItr != map_bRefineStructures.end()) {
            if (struItr->second) // refined
            {
                const Landmark& lm = itr->second;
                target_data.structure[itr->first].X = lm.X;
            }
        }
    }
}

double SequentialSfM::CalculateAngularError(const Vec2& point2D, const Vec3& point3D, const Mat34& proj_matrix, const CameraPtr cam)
{
    const Vec2 imcam = cam->ima2cam(point2D);
    Vec3 im = proj_matrix * point3D.homogeneous();
    const Vec2 point2D2(im[0] / im[2], im[1] / im[2]);
    const Vec2 imcam2 = cam->ima2cam(point2D2);

    const Vec3 ray1 = imcam.homogeneous();
    const Vec3 ray2 = imcam2.homogeneous();
    return std::acos(ray1.normalized().transpose() * ray2.normalized());
    // 	return CalculateAngularError(
    // 		cam->ima2cam(point2D), point3D, proj_matrix);
}

// double SequentialSfM::CalculateAngularError(
// 	const Vec2& point2D,
// 	const Vec3& point3D,
// 	const Mat34& proj_matrix)
// {
// 	const Vec3 ray1 = point2D.homogeneous();
// 	const Vec3 ray2 = proj_matrix * point3D.homogeneous();
// 	return std::acos(ray1.normalized().transpose() * ray2.normalized());
// }

#if 0
size_t SequentialSfM::Create(
	const TriangulationOptions& options,
	const CorrData &corr_data1,
	const CorrData &corr_data2)
{
	// Extract correspondences without an existing triangulated observation.

	if (_sfm_tracks._map_obv_track[corr_data1.obv].first != SfmTracks::eObvNonReconstruct)
	{
		return 0;
	}
	if (_sfm_tracks._map_obv_track[corr_data2.obv].first != SfmTracks::eObvNonReconstruct)
	{
		return 0;
	}

	//Todo ignore two view tracks or corrs data is 2
	// 			if (options.ignore_two_view_tracks && create_corrs_data.size() == 2) {
	// 				const CorrData& corr_data1 = create_corrs_data[0];
	// 				��ʱδʵ��
	//if (scene_graph_->IsTwoViewObservation(corr_data1.image_id,
	// 	corr_data1.point2D_idx)) {
	// 	return 0;
	//}
	// 			}
	Vec3 X;
	openMVG::TriangulateDLT(corr_data1.P, corr_data1.feat, corr_data2.P, corr_data1.feat, &X);
	size_t trackId1 = corr_data1.trackID;//track id is the same
	//size_t trackId2 = corr_data2.trackID;
	Observations obvs;
	obvs[corr_data1.obv.first].id_feat = corr_data1.obv.second;
	obvs[corr_data1.obv.first].x = corr_data1.feat;
	Pair &pair1 = _sfm_tracks._map_obv_track[Pair(corr_data1.obv.first, corr_data1.obv.second)];
	pair1.first = SfmTracks::eObvReconstruct;
	pair1.second = trackId1;

	obvs[corr_data2.obv.first].id_feat = corr_data2.obv.second;
	obvs[corr_data2.obv.first].x = corr_data2.feat;
	Pair &pair2 = _sfm_tracks._map_obv_track[Pair(corr_data2.obv.first, corr_data2.obv.second)];
	pair2.first = SfmTracks::eObvReconstruct;
	pair2.second = trackId1;

	size_t I = std::min(corr_data1.obv.first, corr_data2.obv.first);
	size_t J = std::min(corr_data1.obv.first, corr_data2.obv.first);
	//++_pairTrianglulations[Pair(I, J)].first;
	Landmark lm;
	lm.X = X;
	lm.obs[corr_data1.obv.first].x = corr_data1.feat;
	lm.obs[corr_data1.obv.first].id_feat = corr_data1.obv.second;
	lm.obs[corr_data2.obv.first].x = corr_data2.feat;
	lm.obs[corr_data2.obv.first].id_feat = corr_data2.obv.second;
	_sfm_data->structure[trackId1] = lm;

	return 2;
}


size_t SequentialSfM::Continue(
	const TriangulationOptions & options,
	const CorrData& ref_corr_data,
	const std::vector<CorrData>& corrs_data)
{
	// No need to continue, if the reference observation is triangulated.
	if (_sfm_tracks._map_obv_track.find(ref_corr_data.obv) == _sfm_tracks._map_obv_track.end()) return 0;

	//has point3d
	if (_sfm_tracks._map_obv_track[ref_corr_data.obv].first != SfmTracks::eObvNonReconstruct) return 0;

	double best_angle_error = std::numeric_limits<double>::max();
	size_t best_idx = std::numeric_limits<size_t>::max();
	size_t trackId;
	for (size_t idx = 0; idx < corrs_data.size(); ++idx) {
		const CorrData& corr_data = corrs_data[idx];
		CHECK(_sfm_tracks._map_obv_track[corr_data.obv].first != SfmTracks::eObvNonReconstruct);
		// 		if (_sfm_tracks._map_obv_track[corr_data.obv].first == SfmTracks::eObvNonReconstruct) {
		// 			continue;
		// 		}

		trackId = _sfm_tracks._map_obv_track[corr_data.obv].second;
		if (_sfm_data->structure.find(trackId) == _sfm_data->structure.end()) {
			continue;
		}
		Landmark &lm = _sfm_data->structure.at(trackId);
		const Vec3 &point3D = lm.X;
		if (ref_corr_data.pose->depth(point3D) < 0) { continue; }

		const double angle_error =
			CalculateAngularError(ref_corr_data.feat, point3D,
				ref_corr_data.P, ref_corr_data.camera);

		if (angle_error < best_angle_error) {
			best_angle_error = angle_error;
			best_idx = idx;
		}
	}


	const double max_angle_error = D2R(options.continue_max_angle_error);
	if (best_angle_error <= max_angle_error &&
		best_idx != std::numeric_limits<size_t>::max()) {
		const CorrData& corr_data = corrs_data[best_idx];
		size_t ViewId = corr_data.obv.first;
		size_t ViewId2 = ref_corr_data.obv.first;
		size_t I = std::min(ViewId, ViewId2);
		size_t J = std::max(ViewId, ViewId2);
		//++_pairTrianglulations[Pair(I, J)].first;

		Observation ob;
		ob.id_feat = ref_corr_data.obv.second;
		ob.x = ref_corr_data.feat;

		_sfm_data->structure[trackId].obs[ViewId2] = ob;

		_sfm_tracks._map_obv_track[Pair(ViewId2, ob.id_feat)].first = SfmTracks::eObvReconstruct;
		_sfm_tracks._map_obv_track[Pair(ViewId2, ob.id_feat)].second = trackId;
		return 1;
	}

	return 0;
}

#endif
// void SequentialSfM::updateReconstructTracks()
// {
// 	_reconstructed_trackId.clear();
// 	std::transform(_sfm_data->structure.begin(), _sfm_data->structure.end(),
// 		std::inserter(_reconstructed_trackId, _reconstructed_trackId.begin()),
// 		RetrieveKey());
// }

// void SequentialSfM::InitPairTriangulations()
// {
// 	//todo update by structures
// 	//now ignore this because of does not do re-triangulation
//
// #if 0
// 	_pairTrianglulations.clear();
//
// 	for (PairWiseMatches::const_iterator mItr = _matchesPerPair->begin();
// 		mItr != _matchesPerPair->end(); ++mItr)
// 	{
// 		Pair temp(std::min(mItr->first.first, mItr->first.second), std::max(mItr->first.first, mItr->first.second));
// 		_pairTrianglulations[temp].first = 0;
// 		_pairTrianglulations[temp].second = mItr->second.size();
// 	}
// #endif
// }

void SequentialSfM::UpdateTrackVisibilityPyramid()
{
    if (_options.sfmOptions.next_resection_method == VISIBILITY_PYRAMID_SCORE) {
        for (auto itr = _sfm_data->structure.begin();
            itr != _sfm_data->structure.end(); ++itr) {
            size_t trackId = itr->first;
            _sfm_tracks.AddTrackToVisibilityPyramid(trackId, _vec_viewStatus);
        }
    }
}

#if 0
void SequentialSfM::UpdateReconstructTracks()
{
	_reconstructed_trackId.clear();
	std::transform(_sfm_data->structure.begin(), _sfm_data->structure.end(),
		std::inserter(_reconstructed_trackId, _reconstructed_trackId.begin()),
		RetrieveKey());
}

void insight::SequentialSfM::UpdateReconstructTracksWithNewAddTracks()
{
	for (auto itr = _trackChanged.begin(); itr != _trackChanged.end(); ++itr)
	{
		_reconstructed_trackId.insert(*itr);
	}
}
#endif

std::vector<uint32_t> SequentialSfM::SelectInitialFirst()
{
    // Struct to hold meta-data for ranking images.
    struct ImageInfo {
        ImageInfo()
            : image_id(UndefinedIndexT)
            , num_correspondences(0)
        {
        }
        IndexT image_id;
        int num_correspondences;
    };

    // Collect information of all not yet registered images with
    // correspondences.
    std::vector<ImageInfo> image_infos;
    image_infos.reserve(_sfm_data->views.size());
    const auto& imageCorrCount = _sfm_tracks.ImageCooresponseCountMap();
    for (auto itr = imageCorrCount.begin();
        itr != imageCorrCount.end();
        ++itr) {
        // Only images with correspondences can be registered.
        if (itr->second == 0) {
            continue;
        }

        // Only use images for initialization that are not registered in any
        // of the other reconstructions.
        if (_set_remainingViewId.find(itr->first) == _set_remainingViewId.end()) {
            continue;
        }
        ImageInfo image_info;
        image_info.image_id = itr->first;
        image_info.num_correspondences = itr->second;
        image_infos.push_back(image_info);
    }

    // Sort images such that images with a prior focal length and more
    // correspondences are preferred, i.e. they appear in the front of the list.
    std::sort(
        image_infos.begin(), image_infos.end(),
        [](const ImageInfo& image_info1, const ImageInfo& image_info2) {
            return image_info1.num_correspondences > image_info2.num_correspondences;
        });

    // Extract image identifiers in sorted order.
    std::vector<IndexT> image_ids;
    image_ids.reserve(image_infos.size());
    for (const ImageInfo& image_info : image_infos) {
        image_ids.push_back(image_info.image_id);
    }
    return image_ids;
}

void SequentialSfM::SelectInitialSecond(IndexT imageId1, std::vector<IndexT>& image_ids)
{
    // Collect images that are connected to the first seed image and have
    // not been registered before in other reconstructions.
    std::unordered_map<IndexT, int> num_correspondences;
    std::vector<size_t> trackIds;
    _sfm_tracks.GetTracksInImage(imageId1, SfmTracks::eTrackNotInLandmark, trackIds);

    // const std::unordered_map<size_t, FeatureX> &trackIds = _sfm_tracks._vec_imageTrackIdFeature[imageId1];
    // const std::map<size_t, size_t> &trackIds = _sfm_tracks._vec_imageTrackIdFeature[imageId1];

    for (auto it = trackIds.begin(); it != trackIds.end(); ++it) {
        size_t trackId = *it;

        const Track& t = _sfm_tracks.GetTrack(trackId);
        // submapTrack &subTrack = _sfm_tracks._map_tracks[trackId];

        for (submapTrack::const_iterator itr = t.begin();
            itr != t.end(); ++itr) {
            if (itr->first == imageId1)
                continue;
            if (_vec_viewStatus[itr->first] == 0) { // if (_set_remainingViewId.find(itr->first) != _set_remainingViewId.end())
                if (num_correspondences.find(itr->first) == num_correspondences.end()) {
                    num_correspondences[itr->first] = 0;
                } else {
                    num_correspondences[itr->first] += 1;
                }
            }
        }
    }

    // Struct to hold meta-data for ranking images.
    struct ImageInfo {
        IndexT image_id;
        int num_correspondences;
    };

    const size_t init_min_num_inliers = static_cast<size_t>(_options.initPairOptions.init_min_num_inliers);

    // Compose image information in a compact form for sorting.
    std::vector<ImageInfo> image_infos;
    image_infos.reserve(_sfm_data->views.size());
    for (const auto elem : num_correspondences) {
        if (elem.second >= init_min_num_inliers) {
            ImageInfo image_info;
            image_info.image_id = elem.first;
            image_info.num_correspondences = elem.second;
            image_infos.push_back(image_info);
        }
    }

    // Sort images such that images with a prior focal length and more
    // correspondences are preferred, i.e. they appear in the front of the list.
    std::sort(
        image_infos.begin(), image_infos.end(),
        [](const ImageInfo& image_info1, const ImageInfo& image_info2) {
            return image_info1.num_correspondences > image_info2.num_correspondences;
        });

    // Extract image identifiers in sorted order.
    image_ids.clear();
    image_ids.reserve(image_infos.size());
    for (const ImageInfo& image_info : image_infos) {
        image_ids.push_back(image_info.image_id);
    }
}

bool SequentialSfM::SelectInitialPair(Pair& initialPair)
{
    LOG(INFO) << "select initial pair ";
    IndexT image_id1 = initialPair.first;
    IndexT image_id2 = initialPair.second;
    if (initialPair.first != UndefinedIndexT && initialPair.second != UndefinedIndexT) {
        if (MakeInitialPair3D(Pair(image_id1, image_id2))) {
            _set_remainingViewId.erase(image_id1);
            _set_remainingViewId.erase(image_id2);
            MarkRegisted(image_id1);
            MarkRegisted(image_id2);
            return true;
        }
    }
    // select first
    std::vector<IndexT> image_ids1;
    if (initialPair.first != UndefinedIndexT && initialPair.second == UndefinedIndexT) {
        // Only *image_id1 provided.
        image_ids1.push_back(initialPair.first);
    } else if (initialPair.first == UndefinedIndexT && initialPair.second != UndefinedIndexT) {
        // Only *image_id2 provided.
        image_ids1.push_back(initialPair.second);
    } else {
        // No initial seed image provided.
        image_ids1 = SelectInitialFirst();
    }

    // Try to find good initial pair.

    for (size_t i1 = 0; i1 < image_ids1.size(); ++i1) {
        image_id1 = image_ids1[i1];
        std::vector<IndexT> image_ids2;
        SelectInitialSecond(image_id1, image_ids2);

        for (size_t i2 = 0; i2 < image_ids2.size(); ++i2) {
            image_id2 = image_ids2[i2];

            // Try every pair only once.
            if (_tried_init_image_pairs.find(Pair(image_id1, image_id2)) != _tried_init_image_pairs.end()) {
                continue;
            }

            _tried_init_image_pairs.insert(Pair(image_id1, image_id2));

            if (MakeInitialPair3D(Pair(image_id1, image_id2))) {
                initialPair = Pair(image_id1, image_id2);
                _set_remainingViewId.erase(image_id1);
                _set_remainingViewId.erase(image_id2);
                MarkRegisted(image_id1);
                MarkRegisted(image_id2);
                return true;
            }
        }
    }

    // No suitable pair found in entire dataset.
    initialPair.first = UndefinedIndexT;
    initialPair.second = UndefinedIndexT;

    return false;
}

// void SequentialSfM::InitImagePairs()
// {
// #if 0
// 	_imagePairs.clear();
// 	for (PairWiseMatches::const_iterator mItr = _matchesPerPair->begin();
// 		mItr != _matchesPerPair->end(); ++mItr)
// 	{
// 		Pair temp(std::min(mItr->first.first, mItr->first.second), std::max(mItr->first.first, mItr->first.second));
// 		_imagePairs[mItr->first.first].insert(temp);
// 		_imagePairs[mItr->first.second].insert(temp);
// 	}
// #endif
// }

bool SequentialSfM::InitLandmarkTracks()
{
    // Compute tracks from matches
    _sfm_tracks.Init(*_sfm_data, _options.trackOptions, _featuresPerImage, _matchesPerPair);
    return _sfm_tracks.TrackCount() > 0;
}

void SequentialSfM::setMatchesProvider(PairWiseMatches* matchesPerPair)
{
    _matchesPerPair = matchesPerPair;
}
void SequentialSfM::setFeaturesProvider(SIFTImageFeature_PerImage* featuresPerImage)
{
    _featuresPerImage = featuresPerImage;
}

void SequentialSfM::BundleAdjustmentSub(const std::vector<IndexT>& local_bundle,
    const std::vector<size_t>& constImageIndex,
    std::set<IndexT>& changedLandMarks)
{

    changedLandMarks.clear();
#ifdef USE_QUATERNION_CERES
    Bundle_Adjustment_Quaternion_Ceres bundle_adjustment_obj(_options.localAdjustOptions);
#else
    Bundle_Adjustment_Ceres bundle_adjustment_obj(_options.localAdjustOptions);
#endif
    std::set<IndexT> subViews;
    std::copy(local_bundle.begin(), local_bundle.end(), std::inserter(subViews, subViews.begin()));
    LOG(INFO) << "SUB BA " << subViews.size() << " ...";
    int max_track_length = std::numeric_limits<int>::max(); // ignore length limit

    SfM_Data sub_data;
    subData(*_sfm_data, sub_data, subViews, max_track_length);

    if (local_bundle.size() == 1) {
        bundle_adjustment_obj.constantPose.insert(local_bundle[0]);
    } else if (local_bundle.size() > 1) {
        for (size_t i = 0; i < constImageIndex.size(); ++i) {
            bundle_adjustment_obj.constantPose.insert(constImageIndex[i]);
        }
    }
    std::map<IndexT, bool> map_bRefineStructures;
    for (auto lm : sub_data.structure) {
        if (_trackChanged.find(lm.first) != _trackChanged.end()) {
            // refine new add tracks
            map_bRefineStructures[lm.first] = true;
        } else {
            map_bRefineStructures[lm.first] = false;
            bundle_adjustment_obj.constantStructure.insert(lm.first);
        }
    }
    // constant intrinsics

    for (auto intri : sub_data.cameras) {
        bundle_adjustment_obj.constantIntrinsics.insert(intri.first);
    }
    bool bOk = bundle_adjustment_obj.Adjust(&sub_data);
    if (bOk) {
        mergeData(*_sfm_data, sub_data, map_bRefineStructures);

        for (Structure::const_iterator itr = sub_data.structure.begin();
            itr != sub_data.structure.end(); ++itr) {
            if (map_bRefineStructures[itr->first]) {
                changedLandMarks.insert(itr->first);
            }
        }
    } else {
        LOG(ERROR) << "SUB BA failed";
    }
}

#if PBA_GPU
double SequentialSfM::BundleAdjustmentPBA()
{
    // const int distort_type = ParallelBA::PBA_MEASUREMENT_DISTORTION;
    // delete nan point first
    std::vector<uint32_t> deleteKeys;
    for (Structure::const_iterator itLand = _sfm_data->structure.begin(); itLand != _sfm_data->structure.end();
        ++itLand) {
        if (isnan(itLand->second.X[0]) || isnan(itLand->second.X[1]) || isnan(itLand->second.X[2])) {
            // deleteKeys.push_back(itLand->first);
            CHECK(false) << "Some is null ";
        }
    }
    for (int i = 0; i < deleteKeys.size(); ++i) {
        _sfm_data->structure.erase(deleteKeys[i]);
    }

    std::vector<CameraT> camera_data;
    std::vector<Point3D> point_data;
    std::vector<Point2D> measurements;
    std::vector<int> ptidx;
    std::vector<int> camidx;
    std::map<int, int> map_view_cam;
    toPBA(*_sfm_data, camera_data, point_data, measurements, ptidx, camidx, map_view_cam);

    /////////////////////////////////////////////////////////////////////////////////////////
    ParallelBA::DeviceT device = ParallelBA::PBA_CUDA_DEVICE_DEFAULT;
    if (!_options.pbaOptions.use_cuda_pba) {
        device = ParallelBA::PBA_CPU_DOUBLE;
    }

    // if (strstr(driver_argument, "--float"))          device = ParallelBA::PBA_CPU_FLOAT;
    // else if (strstr(driver_argument, "--double"))    device = ParallelBA::PBA_CPU_DOUBLE;

    /////////////////////////////////////////////////////////////////////
    ParallelBA pba(device); // You should reusing the same object for all new data
    pba.SetNextBundleMode(ParallelBA::BUNDLE_FULL);
    /////////////////////////////////////////////////////////
    // Parameters can be changed before every call of RunBundleAdjustment
    // But do not change them from another thread when it is running BA.
    // pba.ParseParam(argc, argv);      //indirect parameter tuning from commandline
    // pba.SetFixedIntrinsics((int)_options.intrisic_param & (int)cameras::Intrinsic_Parameter_Type::NONE); //if your focal lengths are calibrated.
    pba.SetFixedIntrinsics(true);
    // ConfigBA* pba_config = pba.GetInternalConfig();// __verbose_level = 1;
    // pba_config->__lm_delta_threshold /= 100.0f;
    // pba_config->__lm_gradient_threshold /= 100.0f;
    // pba_config->__lm_mse_threshold = 0.0f;
    // pba_config->__cg_min_iteration = 10;
    // pba_config->__verbose_level = 0;
    // pba_config->__lm_max_iteration = 50;
    //            equivalent to pba.GetInternalConfig()->__fixed_focallength = true;
    // pba.EnableRadialDistortion(*); // if you want to enable radial distortion
    // pba.EnableRadialDistortion(ParallelBA::PBA_PROJECTION_DISTORTION);
    // pba.SetFocalLengthFixed(true);
    //            equivalent to pba.GetInternalConfig()->__use_radial_distortion = *;
    // check src/pba/ConfigBA.h for more details on the parameter system
    ////////////////////////////////////
    // Tweaks before bundle adjustment
    // 1. For each camera, you can call CameraT::SetConstantCamera() to keep it unchanged.
    // 2. pba.SetNextBundleMode(ParallelBA::BUNDLE_ONLY_MOTION) //chose a truncated mode?

    ////////////////////////////////////////////////////////////////
    pba.SetCameraData(camera_data.size(), &camera_data[0]); // set camera parameters
    pba.SetPointData(point_data.size(), &point_data[0]); // set 3D point data
    pba.SetProjection(measurements.size(), &measurements[0], &ptidx[0], &camidx[0]); // set the projections
    // 	vector<int> cmask;
    // 	if (strstr(driver_argument, "--common"))
    // 	{
    // 		cmask.resize(camera_data.size(), 0);
    // 		pba.SetFocalMask(&cmask[0]);
    // 	}
    // WARNING: measumrents must be stored in correct order
    // all measutments (in different images) for a single 3D point must be stored continously,
    // Currently, there is no order verification internally.
    // Basically, ptidx should be non-decreasing

    //////////////////////////////////////////////////////
    // pba.SetTimeBudget(10);      //use at most 10 seconds?
    pba.RunBundleAdjustment(); // run bundle adjustment, and camera_data/point_data will be modified
    // save result
    fromPBA(*_sfm_data, camera_data, point_data, measurements, ptidx, camidx, map_view_cam);
    return pba.GetMeanSquaredError();
}

#endif
INSIGHT_NAME_SPACE_END
