/**
 * isat_focal_from_geo.cpp
 *
 * Estimate shared (or per-camera) focal length from geo F matrices when EXIF
 * is missing / unreliable.  Writes updated fx/fy into --project JSON
 * (images_all.json style) and optionally prints ISAT_EVENT.
 *
 * Typical use (after isat_geo, before incremental SfM):
 *   isat_focal_from_geo -p images_all.json -g geo/ -o images_all.json -v
 */
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"

#include "algorithm/io/geopack_index.h"
#include "algorithm/io/idc_reader.h"
#include "algorithm/modules/sfm/focal_from_view_graph.h"
#include "project_loader.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace fs = std::filesystem;
using insight::tools::ProjectData;
using insight::sfm::FocalFromViewGraphOptions;
using insight::sfm::FocalPairConstraint;
using insight::sfm::estimate_focals_from_view_graph;

static bool load_F_from_idc(const std::string& path, const std::string& blob_name,
                            Eigen::Matrix3d* F) {
  insight::io::IDCReader reader(path);
  if (!reader.is_valid())
    return false;
  auto raw = reader.read_blob<float>(blob_name);
  if (raw.size() != 9)
    return false;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      (*F)(r, c) = static_cast<double>(raw[static_cast<size_t>(r * 3 + c)]);
  return true;
}

static bool collect_pairs_from_geo(const std::string& geo_dir, const ProjectData& project,
                                   int min_inliers, std::vector<FocalPairConstraint>* out) {
  insight::io::GeoPackIndex index;
  const bool have_index = index.load_from_dir(geo_dir);

  auto push_pair = [&](uint32_t i, uint32_t j, const Eigen::Matrix3d& F, int F_inliers,
                       bool degenerate) {
    if (i >= project.num_images() || j >= project.num_images())
      return;
    FocalPairConstraint c;
    c.F = F;
    const int ci = project.image_to_camera_index[i];
    const int cj = project.image_to_camera_index[j];
    const auto& Ki = project.cameras[static_cast<size_t>(ci)];
    const auto& Kj = project.cameras[static_cast<size_t>(cj)];
    c.cx1 = Ki.cx;
    c.cy1 = Ki.cy;
    c.cx2 = Kj.cx;
    c.cy2 = Kj.cy;
    c.camera_id1 = ci;
    c.camera_id2 = cj;
    c.weight = F_inliers;
    c.is_degenerate = degenerate;
    out->push_back(c);
  };

  if (have_index) {
    // Walk directory of geopack blocks referenced by index via legacy pair files
    // and/or pack blobs.  Prefer legacy .isat_geo if present for simplicity;
    // also try geopack F_matrix blobs listed in JSON index.
    for (const auto& entry : fs::directory_iterator(geo_dir)) {
      if (!entry.is_regular_file())
        continue;
      const std::string name = entry.path().filename().string();
      if (name.size() < 10 || name.find(".isat_geo") == std::string::npos)
        continue;
      // Expect im0_im1.isat_geo
      const auto us = name.find('_');
      const auto dot = name.find('.');
      if (us == std::string::npos || dot == std::string::npos || us > dot)
        continue;
      try {
        const uint32_t i = static_cast<uint32_t>(std::stoul(name.substr(0, us)));
        const uint32_t j = static_cast<uint32_t>(std::stoul(name.substr(us + 1, dot - us - 1)));
        const auto* ge = index.find(i, j);
        if (ge && (!ge->F_ok || ge->F_inliers < min_inliers || ge->is_degenerate))
          continue;
        Eigen::Matrix3d F;
        if (!load_F_from_idc(entry.path().string(), "F_matrix", &F))
          continue;
        const int nin = ge ? ge->F_inliers : min_inliers;
        const bool deg = ge ? ge->is_degenerate : false;
        push_pair(i, j, F, nin, deg);
      } catch (...) {
        continue;
      }
    }
  }

  // Legacy: scan all .isat_geo even without index.
  if (out->empty()) {
    for (const auto& entry : fs::directory_iterator(geo_dir)) {
      if (!entry.is_regular_file())
        continue;
      const std::string name = entry.path().filename().string();
      if (name.find(".isat_geo") == std::string::npos)
        continue;
      const auto us = name.find('_');
      const auto dot = name.find('.');
      if (us == std::string::npos || dot == std::string::npos || us > dot)
        continue;
      try {
        const uint32_t i = static_cast<uint32_t>(std::stoul(name.substr(0, us)));
        const uint32_t j = static_cast<uint32_t>(std::stoul(name.substr(us + 1, dot - us - 1)));
        Eigen::Matrix3d F;
        if (!load_F_from_idc(entry.path().string(), "F_matrix", &F))
          continue;
        push_pair(i, j, F, min_inliers + 1, false);
      } catch (...) {
        continue;
      }
    }
  }

  // Geopack blocks: read F_matrix blobs for indexed pairs.
  if (out->empty() && have_index) {
    LOG(WARNING) << "No legacy .isat_geo found; trying geopack F_matrix blobs";
    for (const auto& entry : fs::directory_iterator(geo_dir)) {
      if (!entry.is_regular_file())
        continue;
      if (entry.path().extension() != ".isat_geopack")
        continue;
      insight::io::IDCReader reader(entry.path().string());
      if (!reader.is_valid())
        continue;
      const json& meta = reader.get_metadata();
      if (!meta.contains("blobs"))
        continue;

      auto handle_blob_name = [&](const std::string& key) {
        // pair/{i}_{j}/F_matrix
        if (key.rfind("pair/", 0) != 0 || key.find("/F_matrix") == std::string::npos)
          return;
        const auto slash = key.find('/', 5);
        if (slash == std::string::npos)
          return;
        const std::string ij = key.substr(5, slash - 5);
        const auto us = ij.find('_');
        if (us == std::string::npos)
          return;
        try {
          const uint32_t i = static_cast<uint32_t>(std::stoul(ij.substr(0, us)));
          const uint32_t j = static_cast<uint32_t>(std::stoul(ij.substr(us + 1)));
          const auto* ge = index.find(i, j);
          if (ge && (!ge->F_ok || ge->F_inliers < min_inliers || ge->is_degenerate))
            return;
          Eigen::Matrix3d F;
          if (!load_F_from_idc(entry.path().string(), key, &F))
            return;
          push_pair(i, j, F, ge ? ge->F_inliers : min_inliers + 1,
                    ge ? ge->is_degenerate : false);
        } catch (...) {
        }
      };

      if (meta["blobs"].is_array()) {
        for (const auto& b : meta["blobs"]) {
          if (b.contains("name") && b["name"].is_string())
            handle_blob_name(b["name"].get<std::string>());
        }
      } else if (meta["blobs"].is_object()) {
        for (auto it = meta["blobs"].begin(); it != meta["blobs"].end(); ++it)
          handle_blob_name(it.key());
      }
    }
  }

  return !out->empty();
}

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);

  CmdLine cmd("Estimate focal from geo F matrices (no-EXIF / unreliable prior path).");
  std::string project_path, geo_dir, output_path;
  int min_inliers = 30;
  int min_pairs = 3;
  int no_ceres = 0;
  std::string log_level = "info";

  cmd.add(make_option('p', project_path, "project").doc("images_all.json / project JSON"));
  cmd.add(make_option('g', geo_dir, "geo").doc("Geo directory (.isat_geo / geopack)"));
  cmd.add(make_option('o', output_path, "output")
              .doc("Output JSON (default: overwrite -p)"));
  cmd.add(make_option(0, min_inliers, "min-inliers").doc("Min F inliers per pair (default 30)"));
  cmd.add(make_option(0, min_pairs, "min-pairs").doc("Min pairs per camera (default 3)"));
  cmd.add(make_option(0, no_ceres, "no-ceres").doc("Skip Ceres refine (1=skip)"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level"));
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
  if (cmd.used('h') || project_path.empty() || geo_dir.empty()) {
    cmd.printHelp(std::cerr, argv[0]);
    return cmd.used('h') ? 0 : 1;
  }
  if (output_path.empty())
    output_path = project_path;

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  ProjectData project;
  if (!insight::tools::load_project_data(project_path, &project)) {
    LOG(ERROR) << "Failed to load project " << project_path;
    return 1;
  }

  std::vector<FocalPairConstraint> pairs;
  if (!collect_pairs_from_geo(geo_dir, project, min_inliers, &pairs)) {
    LOG(ERROR) << "No F constraints loaded from " << geo_dir;
    return 1;
  }
  LOG(INFO) << "Loaded " << pairs.size() << " F constraints from " << geo_dir;

  FocalFromViewGraphOptions opts;
  opts.min_inliers = min_inliers;
  opts.min_pairs_per_camera = min_pairs;
  opts.refine_with_ceres = (no_ceres == 0);
  // Soft prior from current (possibly wrong) fx, else 0.7*max(w,h).
  if (!project.cameras.empty() && project.cameras[0].fx > 0.0)
    opts.soft_prior_focal = project.cameras[0].fx;
  else if (!project.cameras.empty())
    opts.soft_prior_focal =
        0.7 * std::max(project.cameras[0].width, project.cameras[0].height);

  const auto res = estimate_focals_from_view_graph(pairs, opts);
  if (!res.ok) {
    LOG(ERROR) << "Focal estimation failed (pairs_used=" << res.num_pairs_used
               << " skipped=" << res.num_pairs_skipped << ")";
    return 1;
  }

  for (const auto& cam_est : res.cameras) {
    if (!cam_est.ok)
      continue;
    if (cam_est.camera_id < 0 ||
        cam_est.camera_id >= static_cast<int>(project.cameras.size()))
      continue;
    auto& K = project.cameras[static_cast<size_t>(cam_est.camera_id)];
    LOG(INFO) << "Update camera " << cam_est.camera_id << " fx: " << K.fx << " → "
              << cam_est.focal << " (" << res.method << ", pairs=" << cam_est.num_pairs << ")";
    K.fx = cam_est.focal;
    K.fy = cam_est.focal;
  }

  // Rewrite JSON: load original, patch cameras[].fx/fy.
  std::ifstream ifs(project_path);
  json root = json::parse(ifs);
  ifs.close();
  if (root.contains("cameras") && root["cameras"].is_array()) {
    for (const auto& cam_est : res.cameras) {
      if (!cam_est.ok)
        continue;
      if (cam_est.camera_id < 0 ||
          cam_est.camera_id >= static_cast<int>(root["cameras"].size()))
        continue;
      root["cameras"][cam_est.camera_id]["fx"] = cam_est.focal;
      root["cameras"][cam_est.camera_id]["fy"] = cam_est.focal;
    }
  }
  std::ofstream ofs(output_path);
  ofs << root.dump(2) << "\n";
  ofs.close();
  LOG(INFO) << "Wrote " << output_path;

  json ev = {{"type", "focal_from_geo.estimate"},
             {"ok", true},
             {"method", res.method},
             {"num_pairs_used", res.num_pairs_used},
             {"num_pairs_skipped", res.num_pairs_skipped},
             {"shared_focal", res.shared_focal}};
  std::cout << "ISAT_EVENT " << ev.dump() << "\n";
  return 0;
}
