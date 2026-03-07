/**
 * isat_incremental_sfm.cpp
 * InsightAT Incremental SfM – initial pair bootstrap and (optionally) resection loop.
 *
 * Phase 1: Build view graph from geo, choose best initial pair, run two-view BA,
 *          reject outliers, filter tracks. Writes initial poses and 2-image tracks.
 *
 * Usage:
 *   isat_incremental_sfm -i pairs.json -g geo_dir/ -m match_dir/ -k K.json -o out_dir/
 *   isat_incremental_sfm -i pairs.json -g geo_dir/ -m match_dir/ -k K.json -o out_dir/ -l
 * images.json
 *
 * Requires: isat_geo run with --twoview so geo files contain R, t, F_inliers.
 */

#include <filesystem>
#include <fstream>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/sfm/incremental_sfm.h"
#include "../modules/sfm/incremental_sfm_engine.h"
#include "../modules/sfm/track_store.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "isat_intrinsics.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

static const char* kEventPrefix = "ISAT_EVENT ";
static void printEvent(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

/// Write Bundler format (bundle.out + list.txt) for viewing in Meshlab/Bundler tools.
static bool save_bundler_output(const insight::sfm::IncrementalSfmResult& result,
                                const insight::sfm::IdMapping& id_mapping,
                                const std::string& image_list_path,
                                const std::string& output_dir) {
  const int n_cams = 2;
  uint32_t id0 = id_mapping.original_image_id(static_cast<int>(result.image1_index));
  uint32_t id1 = id_mapping.original_image_id(static_cast<int>(result.image2_index));
  std::ifstream f(image_list_path);
  if (!f.is_open()) {
    LOG(WARNING) << "Cannot open image list for Bundler list.txt: " << image_list_path;
    return false;
  }
  json j;
  try {
    f >> j;
  } catch (...) {
    return false;
  }
  std::vector<std::string> paths(2);
  if (!j.contains("images") || !j["images"].is_array()) {
    LOG(WARNING) << "Image list has no 'images' array";
    return false;
  }
  for (const auto& img : j["images"]) {
    if (!img.contains("id") || !img.contains("path"))
      continue;
    uint32_t id = img["id"].get<uint32_t>();
    std::string path = img["path"].get<std::string>();
    if (id == id0)
      paths[0] = path;
    else if (id == id1)
      paths[1] = path;
  }
  if (paths[0].empty() || paths[1].empty()) {
    LOG(WARNING) << "Could not resolve image paths for Bundler (id0=" << id0 << " id1=" << id1 << ")";
    return false;
  }

  const std::string list_path = output_dir + "/list.txt";
  std::ofstream list_out(list_path);
  if (!list_out) {
    LOG(WARNING) << "Cannot write " << list_path;
    return false;
  }
  for (const auto& p : paths)
    list_out << p << "\n";
  list_out.close();
  LOG(INFO) << "Wrote " << list_path << " (" << n_cams << " images)";

  const insight::sfm::TrackStore& store = result.store;
  std::vector<int> valid_track_ids;
  for (size_t t = 0; t < store.num_tracks(); ++t) {
    if (store.is_track_valid(static_cast<int>(t)))
      valid_track_ids.push_back(static_cast<int>(t));
  }
  const int n_points = static_cast<int>(valid_track_ids.size());

  const std::string bundle_path = output_dir + "/bundle.out";
  std::ofstream out(bundle_path);
  if (!out) {
    LOG(WARNING) << "Cannot write " << bundle_path;
    return false;
  }
  out << "# Bundle file v0.3\n";
  out << n_cams << " " << n_points << "\n";

  const uint32_t im0 = result.image1_index;
  const uint32_t im1 = result.image2_index;
  for (int c = 0; c < n_cams; ++c) {
    const size_t im = (c == 0) ? static_cast<size_t>(im0) : static_cast<size_t>(im1);
    const int cam_idx = result.camera_index_for_image.empty()
                           ? 0
                           : (im < result.camera_index_for_image.size()
                                  ? result.camera_index_for_image[im]
                                  : 0);
    const insight::camera::Intrinsics& K =
        (static_cast<size_t>(cam_idx) < result.cameras.size())
            ? result.cameras[static_cast<size_t>(cam_idx)]
            : insight::camera::Intrinsics{};
    double f = (K.fx + K.fy) * 0.5;
    if (f <= 0.0)
      f = 1000.0;
    out << f << " " << K.k1 << " " << K.k2 << "\n";
    const Eigen::Matrix3d& R = result.poses_R[im];
    const Eigen::Vector3d& C = result.poses_C[im];
    Eigen::Vector3d t = -R * C;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col)
        out << (col ? " " : "") << R(row, col);
      out << "\n";
    }
    out << t(0) << " " << t(1) << " " << t(2) << "\n";
  }

  for (int pt_idx = 0; pt_idx < n_points; ++pt_idx) {
    int t = valid_track_ids[static_cast<size_t>(pt_idx)];
    float x, y, z;
    store.get_track_xyz(t, &x, &y, &z);
    out << x << " " << y << " " << z << "\n";
    out << "128 128 128\n";  // no per-point color
    std::vector<insight::sfm::Observation> obs_list;
    store.get_track_observations(t, &obs_list);
    out << static_cast<int>(obs_list.size()) << "\n";
    for (const auto& o : obs_list) {
      int cam = (o.image_index == im0) ? 0 : (o.image_index == im1) ? 1 : -1;
      if (cam < 0)
        continue;
      const size_t oim = static_cast<size_t>(o.image_index);
      const int cam_idx = result.camera_index_for_image.empty()
                             ? 0
                             : (oim < result.camera_index_for_image.size()
                                    ? result.camera_index_for_image[oim]
                                    : 0);
      const insight::camera::Intrinsics& K =
          (static_cast<size_t>(cam_idx) < result.cameras.size())
              ? result.cameras[static_cast<size_t>(cam_idx)]
              : insight::camera::Intrinsics{};
      double u = static_cast<double>(o.u), v = static_cast<double>(o.v);
      double bx = u - K.cx;
      double by = (K.cy - v);
      out << cam << " " << static_cast<int>(o.feature_id) << " " << bx << " " << by << "\n";
    }
  }
  out.close();
  LOG(INFO) << "Wrote " << bundle_path << " (" << n_cams << " cameras, " << n_points << " points)";
  return true;
}

// Save 2-image subset of track store (only tracks with obs in both image1 and image2) and poses.
// image1_index/image2_index are the store's image indices; output uses 0 and 1.
static bool save_initial_result(const insight::sfm::TrackStore& store, uint32_t image1_id,
                                uint32_t image2_id, uint32_t image1_index, uint32_t image2_index,
                                const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                                const std::string& output_dir) {
  std::vector<uint32_t> image_ids = {image1_id, image2_id};
  const uint32_t im0 = image1_index;
  const uint32_t im1 = image2_index;
  std::vector<int> two_view_track_ids;
  std::vector<insight::sfm::Observation> obs_buf;
  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid))
      continue;
    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);
    bool has0 = false, has1 = false;
    for (const auto& o : obs_buf) {
      if (o.image_index == im0) has0 = true;
      if (o.image_index == im1) has1 = true;
    }
    if (has0 && has1)
      two_view_track_ids.push_back(tid);
  }
  const size_t n_tracks = two_view_track_ids.size();
  json meta;
  meta["schema_version"] = "1.0";
  meta["task_type"] = "incremental_sfm_initial";
  meta["num_images"] = 2;
  meta["image_ids"] = image_ids;
  meta["num_tracks"] = static_cast<int>(n_tracks);
  meta["cam2_R"] = std::vector<std::vector<double>>(3, std::vector<double>(3));
  meta["cam2_t"] = std::vector<double>(3);
  for (int i = 0; i < 3; ++i) {
    meta["cam2_t"][i] = t(i);
    for (int j = 0; j < 3; ++j)
      meta["cam2_R"][i][j] = R(i, j);
  }
  std::vector<float> track_xyz(n_tracks * 3);
  std::vector<uint8_t> track_flags(n_tracks);
  for (size_t i = 0; i < n_tracks; ++i) {
    float x, y, z;
    store.get_track_xyz(two_view_track_ids[i], &x, &y, &z);
    track_xyz[i * 3] = x;
    track_xyz[i * 3 + 1] = y;
    track_xyz[i * 3 + 2] = z;
    track_flags[i] = 1;
  }
  std::vector<uint32_t> track_obs_offset(n_tracks + 1);
  std::vector<uint32_t> obs_image_id;
  std::vector<uint32_t> obs_feature_id;
  std::vector<float> obs_u, obs_v, obs_scale;
  std::vector<uint8_t> obs_flags;
  size_t offset = 0;
  for (size_t i = 0; i < n_tracks; ++i) {
    track_obs_offset[i] = static_cast<uint32_t>(offset);
    obs_buf.clear();
    store.get_track_observations(two_view_track_ids[i], &obs_buf);
    for (const auto& o : obs_buf) {
      if (o.image_index != im0 && o.image_index != im1)
        continue;
      obs_image_id.push_back(o.image_index == im0 ? 0u : 1u);
      obs_feature_id.push_back(o.feature_id);
      obs_u.push_back(o.u);
      obs_v.push_back(o.v);
      obs_scale.push_back(o.scale);
      obs_flags.push_back(1);
      ++offset;
    }
  }
  track_obs_offset[n_tracks] = static_cast<uint32_t>(offset);
  meta["num_observations"] = static_cast<int>(offset);

  const std::string idc_path = output_dir + "/initial_tracks.isat_tracks";
  insight::io::IDCWriter writer(idc_path);
  writer.set_metadata(meta);
  writer.add_blob("track_xyz", track_xyz.data(), track_xyz.size() * sizeof(float), "float32",
                 {static_cast<int>(n_tracks), 3});
  writer.add_blob("track_flags", track_flags.data(), track_flags.size(), "uint8",
                 {static_cast<int>(n_tracks)});
  writer.add_blob("track_obs_offset", track_obs_offset.data(),
                 track_obs_offset.size() * sizeof(uint32_t), "uint32",
                 {static_cast<int>(n_tracks) + 1});
  writer.add_blob("obs_image_id", obs_image_id.data(), obs_image_id.size() * sizeof(uint32_t),
                 "uint32", {static_cast<int>(obs_image_id.size())});
  writer.add_blob("obs_feature_id", obs_feature_id.data(), obs_feature_id.size() * sizeof(uint32_t),
                 "uint32", {static_cast<int>(obs_feature_id.size())});
  writer.add_blob("obs_u", obs_u.data(), obs_u.size() * sizeof(float), "float32",
                 {static_cast<int>(obs_u.size())});
  writer.add_blob("obs_v", obs_v.data(), obs_v.size() * sizeof(float), "float32",
                 {static_cast<int>(obs_v.size())});
  writer.add_blob("obs_scale", obs_scale.data(), obs_scale.size() * sizeof(float), "float32",
                 {static_cast<int>(obs_scale.size())});
  writer.add_blob("obs_flags", obs_flags.data(), obs_flags.size(), "uint8",
                 {static_cast<int>(obs_flags.size())});
  if (!writer.write()) {
    LOG(ERROR) << "Failed to write " << idc_path;
    return false;
  }
  const std::string poses_path = output_dir + "/initial_poses.json";
  json poses_json;
  poses_json["image_ids"] = image_ids;
  poses_json["cam1_identity"] = true;
  poses_json["cam2_R"] = std::vector<std::vector<double>>(3, std::vector<double>(3));
  poses_json["cam2_t"] = std::vector<double>(3);
  for (int i = 0; i < 3; ++i) {
    poses_json["cam2_t"][i] = t(i);
    for (int j = 0; j < 3; ++j)
      poses_json["cam2_R"][i][j] = R(i, j);
  }
  std::ofstream of(poses_path);
  if (!of) {
    LOG(ERROR) << "Failed to write " << poses_path;
    return false;
  }
  of << poses_json.dump(2) << "\n";
  LOG(INFO) << "Wrote " << idc_path << " and " << poses_path;
  return true;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  CmdLine cmd("InsightAT Incremental SfM – multi-camera incremental reconstruction");
  std::string pairs_json, geo_dir, match_dir, intrinsics_path, output_dir, image_list;
  int min_tracks = 20;
  bool opt_intrinsics = false;

  cmd.add(make_option('i', pairs_json, "input").doc("Pairs JSON path"));
  cmd.add(make_option('g', geo_dir, "geo-dir")
              .doc("Directory of .isat_geo (must have been run with --twoview)"));
  cmd.add(make_option('m', match_dir, "match-dir").doc("Directory of .isat_match files"));
  cmd.add(make_option('k', intrinsics_path, "intrinsics")
              .doc("Camera intrinsics JSON (multi-camera, from isat_project intrinsics --all)"));
  cmd.add(make_option('o', output_dir, "output")
              .doc("Output directory for initial_poses.json and initial_tracks.isat_tracks"));
  cmd.add(make_option('l', image_list, "image-list")
              .doc("Image list JSON (InsightAT Image List Format v2.0, e.g. images_all.json). "
                   "Required: provides image_id → camera_id for per-camera intrinsics."));
  cmd.add(make_switch(0, "optimize-intrinsics")
              .doc("Optimize intrinsics and distortion in Global BA (default: fixed)"));
  cmd.add(make_option(0, min_tracks, "min-tracks")
              .doc("Minimum valid tracks after filtering. Default: 20"));
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show help"));

  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  if (cmd.checkHelp(argv[0]))
    return 0;
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  opt_intrinsics = cmd.used("optimize-intrinsics");

  if (pairs_json.empty() || geo_dir.empty() || match_dir.empty() ||
      intrinsics_path.empty() || image_list.empty()) {
    std::cerr << "Error: -i, -g, -m, -k, -l are all required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  // ── Build ID mapping and ordered camera setup (re-encoded for efficiency) ─
  insight::sfm::IdMapping id_mapping = build_id_mapping_from_image_list(image_list);
  if (id_mapping.empty()) {
    LOG(ERROR) << "No images loaded from image list: " << image_list;
    return 1;
  }

  auto cam_map = load_intrinsics_map(intrinsics_path);
  if (cam_map.empty()) {
    LOG(ERROR) << "No intrinsics loaded from " << intrinsics_path;
    return 1;
  }
  insight::sfm::MultiCameraSetup cam_setup;
  cam_setup.image_to_camera = id_mapping.image_to_camera;
  cam_setup.cameras.resize(id_mapping.num_cameras());
  for (size_t j = 0; j < id_mapping.num_cameras(); ++j) {
    uint32_t orig_cam_id = id_mapping.original_camera_id(static_cast<int>(j));
    auto it = cam_map.find(orig_cam_id);
    if (it != cam_map.end() && it->second.valid())
      cam_setup.cameras[j] = it->second.to_algorithm_intrinsics();
    else {
      LOG(WARNING) << "No intrinsics for camera id " << orig_cam_id << ", using fallback";
      if (!cam_setup.cameras.empty())
        cam_setup.cameras[j] = cam_setup.cameras[0];
    }
  }
  if (cam_setup.cameras.empty()) {
    LOG(ERROR) << "No valid cameras in intrinsics file: " << intrinsics_path;
    return 1;
  }
  if (output_dir.empty())
    output_dir = ".";

  // ── Algorithm parameters (config) ────────────────────────────────────────
  insight::sfm::IncrementalSfmConfig config;
  config.pairs_json_path = pairs_json;
  config.geo_dir = geo_dir;
  config.match_dir = match_dir;
  config.id_mapping = &id_mapping;
  config.optimize_intrinsics = opt_intrinsics;
  config.min_tracks_after_initial = min_tracks;
  config.run_global_ba = true;

  insight::sfm::IncrementalSfmResult result;
  const bool ok = insight::sfm::run_incremental_reconstruction(cam_setup, config, &result);

  if (!ok) {
    LOG(ERROR) << "Initial pair loop failed (no suitable pair or too few tracks)";
    printEvent({{"type", "incremental_sfm.initial"}, {"ok", false}});
    return 1;
  }

  uint32_t image1_id = id_mapping.original_image_id(static_cast<int>(result.image1_index));
  uint32_t image2_id = id_mapping.original_image_id(static_cast<int>(result.image2_index));
  const size_t idx2 = static_cast<size_t>(result.image2_index);
  Eigen::Matrix3d R1 = (idx2 < result.poses_R.size()) ? result.poses_R[idx2] : Eigen::Matrix3d::Identity();
  const Eigen::Vector3d C1 = (idx2 < result.poses_C.size()) ? result.poses_C[idx2] : Eigen::Vector3d::Zero();
  Eigen::Vector3d t1 = -R1 * C1;

  int n_valid = 0;
  for (size_t i = 0; i < result.store.num_tracks(); ++i)
    if (result.store.is_track_valid(static_cast<int>(i)))
      ++n_valid;

  if (!output_dir.empty()) {
    fs::create_directories(output_dir);
    if (!save_initial_result(result.store, image1_id, image2_id,
                            result.image1_index, result.image2_index, R1, t1, output_dir))
      return 1;
    if (save_bundler_output(result, id_mapping, image_list, output_dir)) {
      LOG(INFO) << "Bundler export: " << output_dir << "/bundle.out, " << output_dir << "/list.txt";
    }
  }

  printEvent({{"type", "incremental_sfm.initial"},
              {"ok", true},
              {"data",
               {{"image1_id", image1_id},
                {"image2_id", image2_id},
                {"num_tracks", n_valid},
                {"output_dir", output_dir},
                {"bundler_out", output_dir + "/bundle.out"},
                {"list_txt", output_dir + "/list.txt"}}}});
  return 0;
}

