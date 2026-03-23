/**
 * isat_incremental_sfm.cpp
 * Incremental SfM CLI: load tracks (IDC) + project (JSON), run pipeline, write poses by index.
 *
 * Usage:
 *   isat_incremental_sfm -t tracks.isat_tracks -p project.json -m pairs.json -g geo_dir/ -o
 * output_dir/
 *
 * Options:
 *   -t / --tracks    Path to .isat_tracks IDC
 *   -p / --project   Path to project JSON (images[] + cameras[], camera_index per image)
 *   -m / --pairs     Path to pairs JSON (for view graph)
 *   -g / --geo       Directory of .isat_geo files (index-based: im0_im1.isat_geo)
 *   -o / --output    Output directory; writes poses.json, bundle.out, list.txt
 */

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "tools/project_loader.h"

#include "../io/track_store_idc.h"
#include "../modules/sfm/incremental_sfm_pipeline.h"
#include "../modules/sfm/track_store.h"

using json = nlohmann::json;
using namespace insight;
using namespace insight::sfm;
using namespace insight::tools;

static bool write_poses_json(const std::string& path, const std::vector<Eigen::Matrix3d>& poses_R,
                             const std::vector<Eigen::Vector3d>& poses_C,
                             const std::vector<bool>& registered) {
  json j = json::array();
  for (size_t i = 0; i < registered.size(); ++i) {
    if (!registered[i])
      continue;
    json pose;
    pose["image_index"] = static_cast<int>(i);
    pose["R"] = std::vector<double>{poses_R[i](0, 0), poses_R[i](0, 1), poses_R[i](0, 2),
                                    poses_R[i](1, 0), poses_R[i](1, 1), poses_R[i](1, 2),
                                    poses_R[i](2, 0), poses_R[i](2, 1), poses_R[i](2, 2)};
    pose["C"] = std::vector<double>{poses_C[i](0), poses_C[i](1), poses_C[i](2)};
    j.push_back(std::move(pose));
  }
  std::ofstream f(path);
  if (!f.is_open()) {
    LOG(ERROR) << "Cannot write " << path;
    return false;
  }
  f << j.dump(2);
  return true;
}

// ─── Bundler output (bundle.out + list.txt) for MeshLab visualisation ────────
// Bundler convention: t = R * (-C), y-axis flipped relative to OpenCV.
// We apply diag(1,-1,-1) to R so cameras face the right direction in MeshLab.
static bool write_bundler(const std::string& out_dir, const std::vector<std::string>& image_paths,
                          const std::vector<Eigen::Matrix3d>& poses_R,
                          const std::vector<Eigen::Vector3d>& poses_C,
                          const std::vector<bool>& registered,
                          const std::vector<camera::Intrinsics>& cameras,
                          const std::vector<int>& image_to_camera_index, const TrackStore& store) {
  const int n_images = static_cast<int>(registered.size());

  // Build list of registered image indices in order
  std::vector<int> reg_indices;
  for (int i = 0; i < n_images; ++i)
    if (registered[static_cast<size_t>(i)])
      reg_indices.push_back(i);

  // Map global image index → bundler camera index (only registered images)
  std::vector<int> global_to_bundler(static_cast<size_t>(n_images), -1);
  for (int bi = 0; bi < static_cast<int>(reg_indices.size()); ++bi)
    global_to_bundler[static_cast<size_t>(reg_indices[bi])] = bi;

  // Collect valid (triangulated) tracks and their observation lists
  struct BundlerPoint {
    float x, y, z;
    std::vector<std::tuple<int, int, float, float>> views; // (cam_idx, key_idx, bx, by)
  };
  std::vector<BundlerPoint> points;
  points.reserve(store.num_tracks());

  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    float px, py, pz;
    store.get_track_xyz(tid, &px, &py, &pz);

    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);

    BundlerPoint bp;
    bp.x = px;
    bp.y = py;
    bp.z = pz;
    for (const auto& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
        continue;
      const int bi = global_to_bundler[static_cast<size_t>(im)];
      if (bi < 0)
        continue;
      const camera::Intrinsics& K =
          cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(im)])];
      // Bundler image coords: origin at principal point, y-axis up
      const float bx = static_cast<float>(o.u) - static_cast<float>(K.cx);
      const float by = -(static_cast<float>(o.v) - static_cast<float>(K.cy));
      bp.views.emplace_back(bi, tid, bx, by);
    }
    if (bp.views.size() >= 2)
      points.push_back(std::move(bp));
  }

  // Write list.txt
  const std::string list_path = out_dir + "/list.txt";
  {
    std::ofstream lf(list_path);
    if (!lf.is_open()) {
      LOG(ERROR) << "Cannot write " << list_path;
      return false;
    }
    for (int gi : reg_indices) {
      const std::string& p = (gi < static_cast<int>(image_paths.size()))
                                 ? image_paths[static_cast<size_t>(gi)]
                                 : "image_" + std::to_string(gi) + ".jpg";
      lf << p << "\n";
    }
  }
  LOG(INFO) << "Wrote " << list_path;

  // Write bundle.out
  const std::string bundle_path = out_dir + "/bundle.out";
  std::ofstream bf(bundle_path);
  if (!bf.is_open()) {
    LOG(ERROR) << "Cannot write " << bundle_path;
    return false;
  }

  bf << "# Bundle file v0.3\n";
  bf << reg_indices.size() << " " << points.size() << "\n";
  bf << std::fixed;

  // Flip matrix: converts OpenCV → Bundler (flip y and z axes)
  const Eigen::Matrix3d flip = Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);

  for (int gi : reg_indices) {
    const camera::Intrinsics& K =
        cameras[static_cast<size_t>(image_to_camera_index[static_cast<size_t>(gi)])];
    const double f = (K.fx + K.fy) * 0.5;
    const double k1 = K.k1;
    const double k2 = K.k2;
    bf << f << " " << k1 << " " << k2 << "\n";

    // R_bundler = flip * R_opencv
    const Eigen::Matrix3d Rb = flip * poses_R[static_cast<size_t>(gi)];
    for (int r = 0; r < 3; ++r) {
      bf << Rb(r, 0) << " " << Rb(r, 1) << " " << Rb(r, 2) << "\n";
    }
    // t = R_bundler * (-C_opencv) = flip * R * (-C)
    const Eigen::Vector3d t = Rb * (-poses_C[static_cast<size_t>(gi)]);
    bf << t(0) << " " << t(1) << " " << t(2) << "\n";
  }

  for (const auto& p : points) {
    bf << p.x << " " << p.y << " " << p.z << "\n";
    bf << "128 128 128\n"; // dummy colour
    bf << p.views.size();
    for (const auto& [cam_idx, key_idx, bx, by] : p.views)
      bf << " " << cam_idx << " " << key_idx << " " << bx << " " << by;
    bf << "\n";
  }

  LOG(INFO) << "Wrote " << bundle_path << " (" << reg_indices.size() << " cameras, "
            << points.size() << " points)";
  return true;
}

int main(int argc, char* argv[]) {
  // google::InitGoogleLogging(argv[0]);
  std::string tracks_path;
  std::string project_path;
  std::string pairs_path;
  std::string geo_dir;
  std::string output_dir;
  std::string log_level;
  CmdLine cmd("Incremental SfM: tracks IDC + project JSON + pairs + geo → poses");
  cmd.add(make_option('t', tracks_path, "tracks").doc("Path to .isat_tracks IDC"));
  cmd.add(make_option('p', project_path, "project").doc("Path to project JSON"));
  cmd.add(make_option('m', pairs_path, "pairs").doc("Path to pairs JSON (view graph)"));
  cmd.add(make_option('g', geo_dir, "geo").doc("Directory of .isat_geo files"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch(0, "fix-intrinsics")
              .doc("Keep camera intrinsics fixed (do not optimize in BA). "));
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
  if (tracks_path.empty() || project_path.empty() || pairs_path.empty() || geo_dir.empty() ||
      output_dir.empty()) {
    std::cerr << "Error: -t, -p, -m, -g, -o are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }
  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);

  ProjectData project;
  if (!load_project_data(project_path, &project)) {
    LOG(ERROR) << "Failed to load project from " << project_path;
    return 1;
  }

  // Load image paths for Bundler list.txt
  std::vector<std::string> image_paths;
  {
    std::ifstream pf(project_path);
    nlohmann::json pj;
    try {
      pf >> pj;
    } catch (...) {
    }
    if (pj.contains("images") && pj["images"].is_array()) {
      for (const auto& img : pj["images"])
        image_paths.push_back(img.value("path", ""));
    }
  }

  TrackStore store;
  std::vector<Eigen::Matrix3d> poses_R;
  std::vector<Eigen::Vector3d> poses_C;
  std::vector<bool> registered;
  IncrementalSfMOptions opts;
  opts.global_ba.max_iterations = 500;
  opts.intrinsics.focal_prior_weight = 100.f;
  if (cmd.used("fix-intrinsics")) {
    opts.global_ba.optimize_intrinsics = false;
    LOG(INFO) << "--fix-intrinsics: camera intrinsics will be held constant in all BA runs.";
  }
  // if (cmd.used("no-local-ba")) {
  //   opts.local_ba.enable = false;
  //   LOG(INFO) << "--no-local-ba: local_ba.enable=false, global BA only until you enable local
  //   BA.";
  // }
  // if (!resection_backend.empty()) {
  //   if (resection_backend == "gpu") {
  //     opts.resection.backend = ResectionBackend::kGpuRansac;
  //   } else if (resection_backend == "poselib") {
  //     opts.resection.backend = ResectionBackend::kPoseLib;
  //   } else {
  //     LOG(ERROR) << "Unknown --resection-backend='" << resection_backend
  //                << "' (expected gpu or poselib)";
  //     return 1;
  //   }
  // }
  if (!run_incremental_sfm_pipeline(tracks_path, pairs_path, geo_dir, &project.cameras,
                                    project.image_to_camera_index, opts, &store, &poses_R, &poses_C,
                                    &registered)) {
    LOG(ERROR) << "Incremental SfM pipeline failed";
    return 1;
  }

  int n_reg = 0;
  for (bool r : registered)
    if (r)
      ++n_reg;
  LOG(INFO) << "Registered " << n_reg << " / " << project.num_images() << " images";

  std::string out_path = output_dir;
  if (!out_path.empty() && out_path.back() != '/')
    out_path += '/';
  out_path += "poses.json";
  if (!write_poses_json(out_path, poses_R, poses_C, registered)) {
    LOG(ERROR) << "Failed to write poses";
    return 1;
  }
  LOG(INFO) << "Wrote " << out_path;

  write_bundler(output_dir, image_paths, poses_R, poses_C, registered, project.cameras,
                project.image_to_camera_index, store);

  return 0;
}
