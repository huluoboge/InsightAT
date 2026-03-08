/**
 * @file  incremental_sfm.cpp
 * @brief run_initial_pair_loop: view graph, load pair, BA, reject, filter.
 */

#include "incremental_sfm.h"
#include "../../io/idc_reader.h"
#include "bundle_adjustment_analytic.h"
#include "incremental_sfm_engine.h"
#include "incremental_sfm_helpers.h"
#include "resection.h"
#include "track_builder.h"
#include "view_graph_loader.h"
#include <glog/logging.h>

namespace insight {
namespace sfm {

namespace {

bool load_pair_geo(const std::string& geo_path, Eigen::Matrix3d* R, Eigen::Vector3d* t,
                   std::vector<uint8_t>* inlier_mask) {
  insight::io::IDCReader reader(geo_path);
  if (!reader.is_valid())
    return false;
  const auto& meta = reader.get_metadata();
  if (!meta.contains("twoview"))
    return false;
  auto R_blob = reader.read_blob<float>("R_matrix");
  auto t_blob = reader.read_blob<float>("t_vector");
  *inlier_mask = reader.read_blob<uint8_t>("F_inliers");
  if (R_blob.size() != 9u || t_blob.size() != 3u || inlier_mask->empty())
    return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      (*R)(i, j) = R_blob[static_cast<size_t>(i * 3 + j)];
  (*t)(0) = t_blob[0];
  (*t)(1) = t_blob[1];
  (*t)(2) = t_blob[2];
  return true;
}

bool load_pair_match(const std::string& match_path, std::vector<uint16_t>* indices,
                     std::vector<float>* coords_pixel, std::vector<float>* scales) {
  insight::io::IDCReader reader(match_path);
  if (!reader.is_valid())
    return false;
  *indices = reader.read_blob<uint16_t>("indices");
  *coords_pixel = reader.read_blob<float>("coords_pixel");
  *scales = reader.read_blob<float>("scales");
  return !indices->empty() && coords_pixel->size() >= indices->size() * 2u;
}

} // namespace

bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, double fx, double fy, double cx, double cy,
                           TrackStore* store_out, Eigen::Matrix3d* R1_out, Eigen::Vector3d* C1_out,
                           int min_tracks_after, uint32_t* image1_index_out, uint32_t* image2_index_out,
                           const IdMapping* id_mapping,
                           const std::vector<camera::Intrinsics>* intrinsics_initial_pair) {
  if (!store_out || !R1_out || !C1_out)
    return false;
  const bool use_per_camera =
      intrinsics_initial_pair && intrinsics_initial_pair->size() >= 2;
  const camera::Intrinsics* K0_ptr = use_per_camera ? &(*intrinsics_initial_pair)[0] : nullptr;
  const camera::Intrinsics* K1_ptr = use_per_camera ? &(*intrinsics_initial_pair)[1] : nullptr;
  ViewGraph vg;
  if (!build_view_graph_from_geo(pairs_json_path, geo_dir, &vg, id_mapping))
    return false;
  const auto pair = vg.choose_initial_pair({});
  if (!pair)
    return false;
  const uint32_t idx1 = pair->first, idx2 = pair->second; // indices 0..n-1 from view graph
  if (image1_index_out)
    *image1_index_out = idx1;
  if (image2_index_out)
    *image2_index_out = idx2;
  std::string dir_g = geo_dir;
  if (!dir_g.empty() && dir_g.back() != '/')
    dir_g += '/';
  std::string dir_m = match_dir;
  if (!dir_m.empty() && dir_m.back() != '/')
    dir_m += '/';
  // Boundary: use original_id for geo/match file paths when id_mapping is provided
  const uint32_t path_id1 = id_mapping ? id_mapping->original_image_id(static_cast<int>(idx1)) : idx1;
  const uint32_t path_id2 = id_mapping ? id_mapping->original_image_id(static_cast<int>(idx2)) : idx2;
  const std::string geo_path =
      dir_g + std::to_string(path_id1) + "_" + std::to_string(path_id2) + ".isat_geo";
  const std::string match_path =
      dir_m + std::to_string(path_id1) + "_" + std::to_string(path_id2) + ".isat_match";

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::vector<uint8_t> inlier_mask;
  if (!load_pair_geo(geo_path, &R, &t, &inlier_mask))
    return false;
  std::vector<uint16_t> indices;
  std::vector<float> coords_pixel, scales;
  if (!load_pair_match(match_path, &indices, &coords_pixel, &scales))
    return false;
  if (inlier_mask.size() != indices.size() / 2u)
    return false;

  store_out->set_num_images(2);
  store_out->reserve_tracks(4096);
  store_out->reserve_observations(8192);
  const int n_added = build_tracks_from_pair_inliers(*store_out, 0, 1, inlier_mask, indices,
                                                     coords_pixel, scales, nullptr);
  if (n_added < 8)
    return false;

  Eigen::Vector3d C2 = -R.transpose() * t; // camera centre of cam2 in world (cam1) frame
  if (use_per_camera)
    retriangulate_two_view_tracks(store_out, R, C2, *K0_ptr, *K1_ptr);
  else
    retriangulate_two_view_tracks(store_out, R, C2, fx, fy, cx, cy);

  TwoViewBAInput ba_in;
  ba_in.R = R;
  ba_in.t = t;
  // Use reference frame (cam0) intrinsics for two-view BA; TwoViewBAInput has single K.
  ba_in.fx = use_per_camera ? K0_ptr->fx : fx;
  ba_in.fy = use_per_camera ? K0_ptr->fy : fy;
  ba_in.cx = use_per_camera ? K0_ptr->cx : cx;
  ba_in.cy = use_per_camera ? K0_ptr->cy : cy;
  const size_t n_tracks = store_out->num_tracks();
  std::vector<int> valid_track_ids;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store_out->is_track_valid(tid))
      continue;
    valid_track_ids.push_back(tid);
    float tx, ty, tz;
    store_out->get_track_xyz(tid, &tx, &ty, &tz);
    ba_in.points3d.emplace_back(tx, ty, tz);
    obs_buf.clear();
    store_out->get_track_observations(tid, &obs_buf);
    const int pt_idx = static_cast<int>(ba_in.points3d.size()) - 1;
    for (const auto& o : obs_buf) {
      ba_in.observations.push_back({pt_idx, static_cast<int>(o.image_index),
                                    static_cast<double>(o.u), static_cast<double>(o.v)});
    }
  }
  if (ba_in.points3d.size() >= 8 && !ba_in.observations.empty()) {
    TwoViewBAResult ba_out;
    if (two_view_bundle(ba_in, &ba_out, 50)) {
      R  = ba_out.R;
      t  = ba_out.t;
      C2 = -R.transpose() * t;
      for (size_t i = 0; i < ba_out.points3d.size() && i < valid_track_ids.size(); ++i)
        store_out->set_track_xyz(valid_track_ids[i], static_cast<float>(ba_out.points3d[i](0)),
                                 static_cast<float>(ba_out.points3d[i](1)),
                                 static_cast<float>(ba_out.points3d[i](2)));
    }
  }

  if (use_per_camera)
    reject_outliers_two_view(store_out, R, C2, *K0_ptr, *K1_ptr, 4.0);
  else
    reject_outliers_two_view(store_out, R, C2, fx, fy, cx, cy, 4.0);
  filter_tracks_two_view(store_out, R, C2, 2, 2.0);

  int n_valid = 0;
  for (size_t ti = 0; ti < store_out->num_tracks(); ++ti) {
    if (store_out->is_track_valid(static_cast<int>(ti)))
      ++n_valid;
  }
  if (n_valid < min_tracks_after)
    return false;
  *R1_out = R;
  *C1_out = C2;
  return true;
}

bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, const MultiCameraSetup* cameras,
                           const IdMapping* id_mapping, TrackStore* store_in_out,
                           Eigen::Matrix3d* R1_out, Eigen::Vector3d* C1_out,
                           int min_tracks_after, uint32_t* image1_index_out,
                           uint32_t* image2_index_out) {
  (void)match_dir;  // unused when using pre-built full store
  if (!store_in_out || !R1_out || !C1_out || !cameras || cameras->empty())
    return false;
  ViewGraph vg;
  if (!build_view_graph_from_geo(pairs_json_path, geo_dir, &vg, id_mapping)) {
    LOG(WARNING) << "run_initial_pair_loop: failed to build view graph";
    return false;
  }
  const auto pair = vg.choose_initial_pair({});
  if (!pair) {
    LOG(WARNING) << "run_initial_pair_loop: no suitable initial pair";
    return false;
  }
  const uint32_t idx1 = pair->first, idx2 = pair->second;
  const camera::Intrinsics* K0 = cameras->for_image_index(static_cast<int>(idx1));
  const camera::Intrinsics* K1 = cameras->for_image_index(static_cast<int>(idx2));
  if (!K0 || !K1 || K0->fx <= 0 || K1->fx <= 0) {
    LOG(ERROR) << "run_initial_pair_loop: missing or invalid intrinsics for chosen pair ("
               << idx1 << ", " << idx2 << ")";
    return false;
  }
  if (image1_index_out)
    *image1_index_out = idx1;
  if (image2_index_out)
    *image2_index_out = idx2;

  std::string dir_g = geo_dir;
  if (!dir_g.empty() && dir_g.back() != '/')
    dir_g += '/';
  const uint32_t path_id1 = id_mapping ? id_mapping->original_image_id(static_cast<int>(idx1)) : idx1;
  const uint32_t path_id2 = id_mapping ? id_mapping->original_image_id(static_cast<int>(idx2)) : idx2;
  const std::string geo_path =
      dir_g + std::to_string(path_id1) + "_" + std::to_string(path_id2) + ".isat_geo";

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::vector<uint8_t> inlier_mask;
  if (!load_pair_geo(geo_path, &R, &t, &inlier_mask)) {
    LOG(WARNING) << "run_initial_pair_loop: failed to load geo " << geo_path;
    return false;
  }
  Eigen::Vector3d C2 = -R.transpose() * t;
  LOG(INFO) << "run_initial_pair_loop: chosen pair (" << idx1 << ", " << idx2 << "), loaded geo";

  retriangulate_two_view_tracks(store_in_out, static_cast<int>(idx1), static_cast<int>(idx2),
                                R, C2, *K0, *K1);
  const size_t n_tracks = store_in_out->num_tracks();
  std::vector<int> valid_track_ids;
  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < n_tracks; ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store_in_out->is_track_valid(tid) || !store_in_out->track_has_triangulated_xyz(tid))
      continue;
    obs_buf.clear();
    store_in_out->get_track_observations(tid, &obs_buf);
    bool has_idx1 = false, has_idx2 = false;
    for (const auto& o : obs_buf) {
      if (o.image_index == idx1) has_idx1 = true;
      else if (o.image_index == idx2) has_idx2 = true;
    }
    if (!has_idx1 || !has_idx2)
      continue;
    valid_track_ids.push_back(tid);
  }

  //这里错误， 应该使用各自的内参，且支持畸变
  TwoViewBAInput ba_in;
  ba_in.R = R;
  ba_in.t = t;
  ba_in.fx = K0->fx;
  ba_in.fy = K0->fy;
  ba_in.cx = K0->cx;
  ba_in.cy = K0->cy;
  for (size_t i = 0; i < valid_track_ids.size(); ++i) {
    const int tid = valid_track_ids[i];
    float tx, ty, tz;
    store_in_out->get_track_xyz(tid, &tx, &ty, &tz);
    ba_in.points3d.emplace_back(static_cast<double>(tx), static_cast<double>(ty),
                                static_cast<double>(tz));
    obs_buf.clear();
    store_in_out->get_track_observations(tid, &obs_buf);
    const int pt_idx = static_cast<int>(ba_in.points3d.size()) - 1;
    for (const auto& o : obs_buf) {
      if (o.image_index == idx1)
        ba_in.observations.push_back(
            {pt_idx, 0, static_cast<double>(o.u), static_cast<double>(o.v)});
      else if (o.image_index == idx2)
        ba_in.observations.push_back(
            {pt_idx, 1, static_cast<double>(o.u), static_cast<double>(o.v)});
    }
  }

  if (ba_in.points3d.size() >= 8 && !ba_in.observations.empty()) {
    TwoViewBAResult ba_out;
    if (two_view_bundle(ba_in, &ba_out, 50)) {
      R = ba_out.R;
      t = ba_out.t;
      C2 = -R.transpose() * t;
      for (size_t i = 0; i < ba_out.points3d.size() && i < valid_track_ids.size(); ++i)
        store_in_out->set_track_xyz(
            valid_track_ids[i], static_cast<float>(ba_out.points3d[i](0)),
            static_cast<float>(ba_out.points3d[i](1)), static_cast<float>(ba_out.points3d[i](2)));
      LOG(INFO) << "run_initial_pair_loop: two-view BA done, RMSE=" << ba_out.rmse_px << " px";
    }
  }

  reject_outliers_two_view(store_in_out, R, C2, static_cast<int>(idx1), static_cast<int>(idx2),
                           *K0, *K1, 4.0);
  filter_tracks_two_view(store_in_out, R, C2, static_cast<int>(idx1), static_cast<int>(idx2),
                         2, 2.0);

  int n_valid = 0;
  for (size_t ti = 0; ti < store_in_out->num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store_in_out->is_track_valid(tid))
      continue;
    obs_buf.clear();
    store_in_out->get_track_observations(tid, &obs_buf);
    bool has1 = false, has2 = false;
    for (const auto& o : obs_buf) {
      if (o.image_index == idx1) has1 = true;
      if (o.image_index == idx2) has2 = true;
    }
    if (has1 && has2)
      ++n_valid;
  }
  if (n_valid < min_tracks_after) {
    LOG(WARNING) << "run_initial_pair_loop: too few tracks after filter: " << n_valid
                 << " < " << min_tracks_after;
    return false;
  }
  LOG(INFO) << "run_initial_pair_loop: " << n_valid << " valid tracks after filter";
  *R1_out = R;
  *C1_out = C2;
  return true;
}

int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                       double fx, double fy, double cx, double cy, int min_correspondences) {
  if (!store || !poses_R || !poses_C || !registered)
    return 0;
  const int n_images = store->num_images();
  if (n_images < 3)
    return 0;
  poses_R->resize(static_cast<size_t>(n_images));
  poses_C->resize(static_cast<size_t>(n_images));
  registered->resize(static_cast<size_t>(n_images), false);
  int n_reg = 0;
  for (bool r : *registered)
    if (r) ++n_reg;
  if (n_reg < 2)
    return 0;

  std::vector<bool> skipped(static_cast<size_t>(n_images), false);
  int added = 0;
  for (;;) {
    const int im = choose_next_resection_image(*store, *registered, &skipped);
    if (im < 0)
      break;
    std::vector<int> track_ids;
    store->get_image_track_observations(im, &track_ids, nullptr);
    int n_corr = 0;
    for (int tid : track_ids)
      if (store->track_has_triangulated_xyz(tid))
        ++n_corr;
    if (n_corr < min_correspondences) {
      skipped[static_cast<size_t>(im)] = true;
      continue;
    }
    Eigen::Matrix3d R_im;
    Eigen::Vector3d t_im;
    if (!resection_single_image(*store, im, fx, fy, cx, cy, &R_im, &t_im, min_correspondences, 8.0,
                                nullptr)) {
      skipped[static_cast<size_t>(im)] = true;
      continue;
    }
    (*poses_R)[static_cast<size_t>(im)] = R_im;
    (*poses_C)[static_cast<size_t>(im)] = -R_im.transpose() * t_im;
    (*registered)[static_cast<size_t>(im)] = true;
    ++added;
    triangulate_tracks_for_new_image(store, im, *poses_R, *poses_C, *registered, fx, fy, cx, cy);
  }
  return added;
}

// ─────────────────────────────────────────────────────────────────────────────
// Intrinsics-aware wrappers (algorithm-only type, no database dependency)
// ─────────────────────────────────────────────────────────────────────────────

bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, const camera::Intrinsics& K,
                           TrackStore* store_out, Eigen::Matrix3d* R1_out, Eigen::Vector3d* C1_out,
                           int min_tracks_after, uint32_t* image1_index_out, uint32_t* image2_index_out,
                           const IdMapping* id_mapping) {
  std::vector<camera::Intrinsics> k01 = {K, K}; // same camera for both views
  return run_initial_pair_loop(pairs_json_path, geo_dir, match_dir, K.fx, K.fy, K.cx, K.cy,
                               store_out, R1_out, C1_out, min_tracks_after, image1_index_out,
                               image2_index_out, id_mapping, &k01);
}

int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                       const camera::Intrinsics& K, int min_correspondences) {
  if (!store || !poses_R || !poses_C || !registered)
    return 0;
  const int n_images = store->num_images();
  if (n_images < 3)
    return 0;
  poses_R->resize(static_cast<size_t>(n_images));
  poses_C->resize(static_cast<size_t>(n_images));
  registered->resize(static_cast<size_t>(n_images), false);
  int n_reg = 0;
  for (bool r : *registered)
    if (r) ++n_reg;
  if (n_reg < 2)
    return 0;

  std::vector<bool> skipped(static_cast<size_t>(n_images), false);
  int added = 0;
  for (;;) {
    const int im = choose_next_resection_image(*store, *registered, &skipped);
    if (im < 0)
      break;
    std::vector<int> track_ids;
    store->get_image_track_observations(im, &track_ids, nullptr);
    int n_corr = 0;
    for (int tid : track_ids)
      if (store->track_has_triangulated_xyz(tid))
        ++n_corr;
    if (n_corr < min_correspondences) {
      skipped[static_cast<size_t>(im)] = true;
      continue;
    }
    Eigen::Matrix3d R_im;
    Eigen::Vector3d t_im;
    if (!resection_single_image(K, *store, im, &R_im, &t_im, min_correspondences, 8.0, nullptr)) {
      skipped[static_cast<size_t>(im)] = true;
      continue;
    }
    (*poses_R)[static_cast<size_t>(im)] = R_im;
    (*poses_C)[static_cast<size_t>(im)] = -R_im.transpose() * t_im;
    (*registered)[static_cast<size_t>(im)] = true;
    ++added;
    triangulate_tracks_for_new_image(store, im, *poses_R, *poses_C, *registered,
                                     K.fx, K.fy, K.cx, K.cy);
  }
  return added;
}

int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_C, std::vector<bool>* registered,
                       const std::vector<camera::Intrinsics>* intrinsics_per_image,
                       double fx, double fy, double cx, double cy, int min_correspondences) {
  if (!store || !poses_R || !poses_C || !registered)
    return 0;
  const int n_images = store->num_images();
  if (n_images < 3)
    return 0;
  poses_R->resize(static_cast<size_t>(n_images));
  poses_C->resize(static_cast<size_t>(n_images));
  registered->resize(static_cast<size_t>(n_images), false);
  int n_reg = 0;
  for (bool r : *registered)
    if (r) ++n_reg;
  if (n_reg < 2)
    return 0;

  const bool use_per_image =
      intrinsics_per_image && intrinsics_per_image->size() == static_cast<size_t>(n_images);

  std::vector<bool> skipped(static_cast<size_t>(n_images), false);
  int added = 0;
  for (;;) {
    const int im = choose_next_resection_image(*store, *registered, &skipped);
    if (im < 0)
      break;
    std::vector<int> track_ids;
    store->get_image_track_observations(im, &track_ids, nullptr);
    int n_corr = 0;
    for (int tid : track_ids)
      if (store->track_has_triangulated_xyz(tid))
        ++n_corr;
    if (n_corr < min_correspondences) {
      skipped[static_cast<size_t>(im)] = true;
      continue;
    }
    Eigen::Matrix3d R_im;
    Eigen::Vector3d t_im;
    if (use_per_image) {
      if (!resection_single_image((*intrinsics_per_image)[static_cast<size_t>(im)], *store, im,
                                  &R_im, &t_im, min_correspondences, 8.0, nullptr)) {
        skipped[static_cast<size_t>(im)] = true;
        continue;
      }
    } else {
      if (!resection_single_image(*store, im, fx, fy, cx, cy, &R_im, &t_im, min_correspondences,
                                 8.0, nullptr)) {
        skipped[static_cast<size_t>(im)] = true;
        continue;
      }
    }
    (*poses_R)[static_cast<size_t>(im)] = R_im;
    (*poses_C)[static_cast<size_t>(im)] = -R_im.transpose() * t_im;
    (*registered)[static_cast<size_t>(im)] = true;
    ++added;
    if (use_per_image)
      triangulate_tracks_for_new_image(store, im, *poses_R, *poses_C, *registered,
                                       *intrinsics_per_image);
    else
      triangulate_tracks_for_new_image(store, im, *poses_R, *poses_C, *registered, fx, fy, cx, cy);
  }
  return added;
}

} // namespace sfm
} // namespace insight
