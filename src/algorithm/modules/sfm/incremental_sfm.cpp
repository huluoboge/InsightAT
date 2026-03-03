/**
 * @file  incremental_sfm.cpp
 * @brief run_initial_pair_loop: view graph, load pair, BA, reject, filter.
 */

#include "incremental_sfm.h"
#include "../../io/idc_reader.h"
#include "bundle_adjustment.h"
#include "incremental_sfm_helpers.h"
#include "resection.h"
#include "track_builder.h"
#include "view_graph_loader.h"

namespace insight {
namespace sfm {

namespace {

bool load_pair_geo(const std::string& geo_path, Eigen::Matrix3d* R, Eigen::Vector3d* t,
                   std::vector<uint8_t>* inlier_mask) {
  insight::io::IDCReader reader(geo_path);
  if (!reader.isValid())
    return false;
  const auto& meta = reader.getMetadata();
  if (!meta.contains("twoview"))
    return false;
  auto R_blob = reader.readBlob<float>("R_matrix");
  auto t_blob = reader.readBlob<float>("t_vector");
  *inlier_mask = reader.readBlob<uint8_t>("F_inliers");
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
  if (!reader.isValid())
    return false;
  *indices = reader.readBlob<uint16_t>("indices");
  *coords_pixel = reader.readBlob<float>("coords_pixel");
  *scales = reader.readBlob<float>("scales");
  return !indices->empty() && coords_pixel->size() >= indices->size() * 2u;
}

} // namespace

bool run_initial_pair_loop(const std::string& pairs_json_path, const std::string& geo_dir,
                           const std::string& match_dir, double fx, double fy, double cx, double cy,
                           TrackStore* store_out, Eigen::Matrix3d* R1_out, Eigen::Vector3d* t1_out,
                           int min_tracks_after, uint32_t* image1_id_out, uint32_t* image2_id_out) {
  if (!store_out || !R1_out || !t1_out)
    return false;
  ViewGraph vg;
  if (!build_view_graph_from_geo(pairs_json_path, geo_dir, &vg))
    return false;
  const auto pair = vg.choose_initial_pair({});
  if (!pair)
    return false;
  const uint32_t id1 = pair->first, id2 = pair->second;
  if (image1_id_out)
    *image1_id_out = id1;
  if (image2_id_out)
    *image2_id_out = id2;
  std::string dir_g = geo_dir;
  if (!dir_g.empty() && dir_g.back() != '/')
    dir_g += '/';
  std::string dir_m = match_dir;
  if (!dir_m.empty() && dir_m.back() != '/')
    dir_m += '/';
  const std::string geo_path =
      dir_g + std::to_string(id1) + "_" + std::to_string(id2) + ".isat_geo";
  const std::string match_path =
      dir_m + std::to_string(id1) + "_" + std::to_string(id2) + ".isat_match";

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

  retriangulate_two_view_tracks(store_out, R, t, fx, fy, cx, cy);

#if defined(INSIGHTAT_USE_CERES) && INSIGHTAT_USE_CERES
  TwoViewBAInput ba_in;
  ba_in.R = R;
  ba_in.t = t;
  ba_in.fx = fx;
  ba_in.fy = fy;
  ba_in.cx = cx;
  ba_in.cy = cy;
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
      R = ba_out.R;
      t = ba_out.t;
      for (size_t i = 0; i < ba_out.points3d.size() && i < valid_track_ids.size(); ++i)
        store_out->set_track_xyz(valid_track_ids[i], static_cast<float>(ba_out.points3d[i](0)),
                                 static_cast<float>(ba_out.points3d[i](1)),
                                 static_cast<float>(ba_out.points3d[i](2)));
    }
  }
#endif

  reject_outliers_two_view(store_out, R, t, fx, fy, cx, cy, 4.0);
  filter_tracks_two_view(store_out, R, t, 2, 2.0);

  int n_valid = 0;
  for (size_t ti = 0; ti < store_out->num_tracks(); ++ti) {
    if (store_out->is_track_valid(static_cast<int>(ti)))
      ++n_valid;
  }
  if (n_valid < min_tracks_after)
    return false;
  *R1_out = R;
  *t1_out = t;
  return true;
}

int run_resection_loop(TrackStore* store, std::vector<Eigen::Matrix3d>* poses_R,
                       std::vector<Eigen::Vector3d>* poses_t, std::vector<bool>* registered,
                       double fx, double fy, double cx, double cy, int min_correspondences) {
  if (!store || !poses_R || !poses_t || !registered)
    return 0;
  const int n_images = store->num_images();
  if (n_images < 3)
    return 0;
  poses_R->resize(static_cast<size_t>(n_images));
  poses_t->resize(static_cast<size_t>(n_images));
  registered->resize(static_cast<size_t>(n_images), false);
  if (!(*registered)[0] || !(*registered)[1])
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
    (*poses_t)[static_cast<size_t>(im)] = t_im;
    (*registered)[static_cast<size_t>(im)] = true;
    ++added;
    triangulate_tracks_for_new_image(store, im, *poses_R, *poses_t, *registered, fx, fy, cx, cy);
  }
  return added;
}

} // namespace sfm
} // namespace insight
