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
 *
 *   --ba-threads N   Ceres solver thread count for bundle adjustment (0 = hardware default).
 */

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "tools/project_loader.h"

#include "../io/track_store_idc.h"
#include "../modules/camera/camera_utils.h"
#include "../modules/sfm/incremental_sfm_pipeline.h"
#include "../modules/sfm/track_store.h"

using json = nlohmann::json;
using namespace insight;
using namespace insight::sfm;
using namespace insight::tools;

static json intrinsics_to_json(const camera::Intrinsics& K) {
  json j;
  j["fx"] = K.fx;
  j["fy"] = K.fy;
  j["cx"] = K.cx;
  j["cy"] = K.cy;
  j["width"] = K.width;
  j["height"] = K.height;
  j["k1"] = K.k1;
  j["k2"] = K.k2;
  j["k3"] = K.k3;
  j["p1"] = K.p1;
  j["p2"] = K.p2;
  return j;
}

static bool write_poses_json(const std::string& path, const std::vector<Eigen::Matrix3d>& poses_R,
                             const std::vector<Eigen::Vector3d>& poses_C,
                             const std::vector<bool>& registered,
                             const std::vector<camera::Intrinsics>& cameras,
                             const std::vector<int>& image_to_camera_index) {
  json poses = json::array();
  for (size_t i = 0; i < registered.size(); ++i) {
    if (!registered[i])
      continue;
    json pose;
    pose["image_index"]  = static_cast<int>(i);
    pose["camera_index"] = image_to_camera_index[i];
    pose["R"] = std::vector<double>{poses_R[i](0, 0), poses_R[i](0, 1), poses_R[i](0, 2),
                                    poses_R[i](1, 0), poses_R[i](1, 1), poses_R[i](1, 2),
                                    poses_R[i](2, 0), poses_R[i](2, 1), poses_R[i](2, 2)};
    pose["C"] = std::vector<double>{poses_C[i](0), poses_C[i](1), poses_C[i](2)};
    poses.push_back(std::move(pose));
  }

  json cameras_json = json::array();
  for (const auto& K : cameras)
    cameras_json.push_back(intrinsics_to_json(K));

  json root;
  root["format"] = "isat_incremental_sfm_pose_bundle_v2";
  root["poses"] = std::move(poses);
  root["cameras"] = std::move(cameras_json);
  root["image_to_camera_index"] = image_to_camera_index;

  std::ofstream f(path);
  if (!f.is_open()) {
    LOG(ERROR) << "Cannot write " << path;
    return false;
  }
  f << root.dump(2);
  return true;
}

// ─── Bundler output (bundle.out + list.txt) for MeshLab visualisation ────────
// Bundler convention: t = R * (-C), y-axis flipped relative to OpenCV.
// We apply diag(1,-1,-1) to R so cameras face the right direction in MeshLab.
//
// bundler_max_cameras: if > 0, uniformly subsample registered cameras to at most this many.
//   Points are filtered to only include observations from the kept cameras (≥2 views required).
static bool write_bundler(const std::string& out_dir, const std::vector<std::string>& image_paths,
                          const std::vector<Eigen::Matrix3d>& poses_R,
                          const std::vector<Eigen::Vector3d>& poses_C,
                          const std::vector<bool>& registered,
                          const std::vector<camera::Intrinsics>& cameras,
                          const std::vector<int>& image_to_camera_index, const TrackStore& store,
                          int bundler_max_cameras = -1) {
  const int n_images = static_cast<int>(registered.size());

  // Build list of registered image indices in order
  std::vector<int> all_reg_indices;
  for (int i = 0; i < n_images; ++i)
    if (registered[static_cast<size_t>(i)])
      all_reg_indices.push_back(i);

  // Uniformly subsample if requested
  std::vector<int> reg_indices;
  if (bundler_max_cameras > 0 &&
      static_cast<int>(all_reg_indices.size()) > bundler_max_cameras) {
    reg_indices.reserve(static_cast<size_t>(bundler_max_cameras));
    const double step =
        static_cast<double>(all_reg_indices.size() - 1) / (bundler_max_cameras - 1);
    for (int k = 0; k < bundler_max_cameras; ++k) {
      const int idx = static_cast<int>(std::round(k * step));
      reg_indices.push_back(all_reg_indices[static_cast<size_t>(idx)]);
    }
    LOG(INFO) << "write_bundler: subsampled " << all_reg_indices.size() << " registered cameras → "
              << reg_indices.size() << " for Bundler output";
  } else {
    reg_indices = all_reg_indices;
  }

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

// Mean reprojection error (px) for one track: pipeline-consistent with incremental_sfm_pipeline /
// bundle_adjustment_analytic (distorted pixels, same as collect_reproj_errors).
static double track_mean_reprojection_error_px(
    int tid, const TrackStore& store, const Eigen::Vector3d& X,
    const std::vector<Eigen::Matrix3d>& poses_R, const std::vector<Eigen::Vector3d>& poses_C,
    const std::vector<bool>& registered, const std::vector<camera::Intrinsics>& cameras,
    const std::vector<int>& image_to_camera_index, int n_images) {
  std::vector<Observation> obs_buf;
  store.get_track_observations(tid, &obs_buf);
  double sum = 0.0;
  int n = 0;
  for (const auto& o : obs_buf) {
    const int im = static_cast<int>(o.image_index);
    if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
      continue;
    const int ci = image_to_camera_index[static_cast<size_t>(im)];
    if (ci < 0 || ci >= static_cast<int>(cameras.size()))
      continue;
    const Eigen::Matrix3d& R = poses_R[static_cast<size_t>(im)];
    const Eigen::Vector3d& C = poses_C[static_cast<size_t>(im)];
    Eigen::Vector3d p = R * (X - C);
    if (p(2) <= 1e-12)
      continue;
    const camera::Intrinsics& K = cameras[static_cast<size_t>(ci)];
    const double xn = p(0) / p(2), yn = p(1) / p(2);
    double xd = 0.0, yd = 0.0;
    camera::apply_distortion(xn, yn, K, &xd, &yd);
    const double u_pred = K.fx * xd + K.cx;
    const double v_pred = K.fy * yd + K.cy;
    const double du = static_cast<double>(o.u) - u_pred;
    const double dv = static_cast<double>(o.v) - v_pred;
    sum += std::sqrt(du * du + dv * dv);
    ++n;
  }
  return n > 0 ? sum / static_cast<double>(n) : 0.0;
}

// ─── COLMAP sparse text output ────────────────────────────────────────────────
// Writes three text files to <out_dir>/colmap/sparse/0/:
//   cameras.txt  – OPENCV only (OpenCV tangential order; p1/p2 swapped from internal ContextCapture)
//   images.txt   – registered images: pose as quaternion + translation, with per-image 2D points
//   points3D.txt – triangulated 3D points with full track (IMAGE_ID POINT2D_IDX pairs);
//                  ERROR column = mean reprojection error (px) over track observations
//
// COLMAP conventions:
//   rotation: QW QX QY QZ  (Eigen::Quaterniond(R))
//   translation: t = R * (-C)  (same as Bundler but without y-flip)
//   camera IDs and image IDs are 1-indexed
//   POINT2D_IDX: 0-based index into the image's POINTS2D list in images.txt
static bool write_colmap(const std::string& out_dir, const std::vector<std::string>& image_paths,
                         const std::vector<Eigen::Matrix3d>& poses_R,
                         const std::vector<Eigen::Vector3d>& poses_C,
                         const std::vector<bool>& registered,
                         const std::vector<camera::Intrinsics>& cameras,
                         const std::vector<int>& image_to_camera_index,
                         const TrackStore& store) {
  namespace fs = std::filesystem;

  const std::string sparse_dir = out_dir + "/colmap/sparse/0";
  try {
    fs::create_directories(sparse_dir);
  } catch (const std::exception& e) {
    LOG(ERROR) << "write_colmap: cannot create " << sparse_dir << ": " << e.what();
    return false;
  }

  const int n_images = static_cast<int>(registered.size());

  // ── Map global image index → COLMAP 1-based image ID ─────────────────────
  std::vector<int> global_to_colmap_id(static_cast<size_t>(n_images), 0);
  int next_img_id = 1;
  for (int i = 0; i < n_images; ++i)
    if (registered[static_cast<size_t>(i)])
      global_to_colmap_id[static_cast<size_t>(i)] = next_img_id++;

  // ── Determine unique cameras and assign COLMAP camera IDs ─────────────────
  // We map each camera index in project → COLMAP camera ID (1-based)
  const int n_cams = static_cast<int>(cameras.size());
  std::vector<int> cam_to_colmap_id(static_cast<size_t>(n_cams), 0);
  int next_cam_id = 1;
  // Only emit cameras that are actually used by at least one registered image
  for (int i = 0; i < n_images; ++i) {
    if (!registered[static_cast<size_t>(i)])
      continue;
    const int ci = image_to_camera_index[static_cast<size_t>(i)];
    if (ci >= 0 && ci < n_cams && cam_to_colmap_id[static_cast<size_t>(ci)] == 0)
      cam_to_colmap_id[static_cast<size_t>(ci)] = next_cam_id++;
  }

  // ── cameras.txt ──────────────────────────────────────────────────────────
  const std::string cams_path = sparse_dir + "/cameras.txt";
  {
    std::ofstream f(cams_path);
    if (!f.is_open()) { LOG(ERROR) << "Cannot write " << cams_path; return false; }
    f << "# Camera list with one line of data per camera:\n"
      << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
      << "# Number of cameras: " << (next_cam_id - 1) << "\n";
    f << std::fixed << std::setprecision(6);
    for (int ci = 0; ci < n_cams; ++ci) {
      if (cam_to_colmap_id[static_cast<size_t>(ci)] == 0)
        continue;
      const camera::Intrinsics& K = cameras[static_cast<size_t>(ci)];
      const int cmap_id = cam_to_colmap_id[static_cast<size_t>(ci)];
      // COLMAP OpenCV tangential order differs from internal ContextCapture: OpenCV p1 = K.p2,
      // OpenCV p2 = K.p1.
      // - OPENCV: fx fy cx cy k1 k2 p1 p2 — only two radial coeffs (no k3 in this model).
      // - FULL_OPENCV: fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6 — rational radial; with k4=k5=k6=0
      //   matches polynomial (1 + k1*r² + k2*r⁴ + k3*r⁶) / 1, i.e. standard OpenCV + k3.
      if (std::abs(K.k3) <= 1e-12) {
        f << cmap_id << " OPENCV " << K.width << " " << K.height << " " << K.fx << " " << K.fy
          << " " << K.cx << " " << K.cy << " " << K.k1 << " " << K.k2 << " " << K.p2 << " " << K.p1
          << "\n";
      } else {
        f << cmap_id << " FULL_OPENCV " << K.width << " " << K.height << " " << K.fx << " " << K.fy
          << " " << K.cx << " " << K.cy << " " << K.k1 << " " << K.k2 << " " << K.p2 << " " << K.p1
          << " " << K.k3 << " 0 0 0\n";
      }
    }
  }
  LOG(INFO) << "write_colmap: wrote " << cams_path;

  // ── Build per-image 2D → 3D observation lists (for images.txt + points3D.txt) ────
  // obs2d[global_image_index] = list of (u, v, point3d_id_1based, track_id)
  struct Obs2D { float u, v; int point3d_id; };
  std::vector<std::vector<Obs2D>> img_obs(static_cast<size_t>(n_images));

  // Collect valid tracks and assign 1-based COLMAP point3D IDs
  struct ColmapPoint3D {
    float x, y, z;
    int point3d_id; // 1-based
    double mean_reproj_px; ///< COLMAP points3D.txt ERROR field
    std::vector<std::pair<int,int>> track; // (colmap_image_id, point2d_idx)
  };

  std::vector<ColmapPoint3D> colmap_points;
  colmap_points.reserve(store.num_tracks());
  int next_pt_id = 1;

  std::vector<Observation> obs_buf;
  for (size_t ti = 0; ti < store.num_tracks(); ++ti) {
    const int tid = static_cast<int>(ti);
    if (!store.is_track_valid(tid) || !store.track_has_triangulated_xyz(tid))
      continue;
    float px, py, pz;
    store.get_track_xyz(tid, &px, &py, &pz);

    obs_buf.clear();
    store.get_track_observations(tid, &obs_buf);

    ColmapPoint3D pt;
    pt.x = px; pt.y = py; pt.z = pz;
    pt.point3d_id = next_pt_id;
    const Eigen::Vector3d Xw(static_cast<double>(px), static_cast<double>(py), static_cast<double>(pz));
    pt.mean_reproj_px =
        track_mean_reprojection_error_px(tid, store, Xw, poses_R, poses_C, registered, cameras,
                                         image_to_camera_index, n_images);

    for (const auto& o : obs_buf) {
      const int im = static_cast<int>(o.image_index);
      if (im < 0 || im >= n_images || !registered[static_cast<size_t>(im)])
        continue;
      const int cmap_img_id = global_to_colmap_id[static_cast<size_t>(im)];
      if (cmap_img_id == 0)
        continue;
      // POINT2D_IDX will be the current size of img_obs[im] before we push
      const int pt2d_idx = static_cast<int>(img_obs[static_cast<size_t>(im)].size());
      img_obs[static_cast<size_t>(im)].push_back({o.u, o.v, next_pt_id});
      pt.track.emplace_back(cmap_img_id, pt2d_idx);
    }

    if (pt.track.size() >= 2) {
      colmap_points.push_back(std::move(pt));
      ++next_pt_id;
    }
  }

  // ── images.txt ───────────────────────────────────────────────────────────
  const std::string imgs_path = sparse_dir + "/images.txt";
  {
    std::ofstream f(imgs_path);
    if (!f.is_open()) { LOG(ERROR) << "Cannot write " << imgs_path; return false; }
    f << "# Image list with two lines of data per image:\n"
      << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
      << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n"
      << "# Number of images: " << (next_img_id - 1) << "\n";
    f << std::fixed << std::setprecision(9);
    for (int i = 0; i < n_images; ++i) {
      if (!registered[static_cast<size_t>(i)])
        continue;
      const int cmap_img_id = global_to_colmap_id[static_cast<size_t>(i)];
      const int ci = image_to_camera_index[static_cast<size_t>(i)];
      const int cmap_cam_id = cam_to_colmap_id[static_cast<size_t>(ci)];

      const Eigen::Quaterniond q(poses_R[static_cast<size_t>(i)]);
      const Eigen::Vector3d t = poses_R[static_cast<size_t>(i)] * (-poses_C[static_cast<size_t>(i)]);

      const std::string name = (i < static_cast<int>(image_paths.size()))
          ? fs::path(image_paths[static_cast<size_t>(i)]).filename().string()
          : "image_" + std::to_string(i) + ".jpg";

      // Line 1: pose
      f << cmap_img_id
        << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z()
        << " " << t(0) << " " << t(1) << " " << t(2)
        << " " << cmap_cam_id << " " << name << "\n";

      // Line 2: 2D points (X Y POINT3D_ID, -1 if not triangulated)
      const auto& obs = img_obs[static_cast<size_t>(i)];
      if (obs.empty()) {
        f << "\n";
      } else {
        f << std::setprecision(2);
        for (size_t oi = 0; oi < obs.size(); ++oi) {
          if (oi > 0) f << " ";
          f << obs[oi].u << " " << obs[oi].v << " " << obs[oi].point3d_id;
        }
        f << "\n";
        f << std::setprecision(9);
      }
    }
  }
  LOG(INFO) << "write_colmap: wrote " << imgs_path;

  // ── points3D.txt ─────────────────────────────────────────────────────────
  const std::string pts_path = sparse_dir + "/points3D.txt";
  {
    std::ofstream f(pts_path);
    if (!f.is_open()) { LOG(ERROR) << "Cannot write " << pts_path; return false; }
    f << "# 3D point list with one line of data per point:\n"
      << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]\n"
      << "# Number of points: " << colmap_points.size() << "\n";
    f << std::fixed << std::setprecision(6);
    for (const auto& pt : colmap_points) {
      f << pt.point3d_id
        << " " << pt.x << " " << pt.y << " " << pt.z
        << " 128 128 128 " << pt.mean_reproj_px;
      for (const auto& [img_id, pt2d_idx] : pt.track)
        f << " " << img_id << " " << pt2d_idx;
      f << "\n";
    }
  }
  LOG(INFO) << "write_colmap: wrote " << pts_path << " (" << colmap_points.size() << " points)";
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
  std::string debug_dir;
  int debug_interval = 1;
  int bundler_max_cameras = -1;
  int ba_grid_target = 1000;
  int ba_threads = 0;
  CmdLine cmd("Incremental SfM: tracks IDC + project JSON + pairs + geo → poses");
  cmd.add(make_option('t', tracks_path, "tracks").doc("Path to .isat_tracks IDC"));
  cmd.add(make_option('p', project_path, "project").doc("Path to project JSON"));
  cmd.add(make_option('m', pairs_path, "pairs").doc("Path to pairs JSON (view graph)"));
  cmd.add(make_option('g', geo_dir, "geo").doc("Directory of .isat_geo files"));
  cmd.add(make_option('o', output_dir, "output").doc("Output directory"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_option(0, debug_dir, "debug-dir")
              .doc("Directory for per-iteration Bundler snapshots (debug pose drift)"));
  cmd.add(make_option(0, debug_interval, "debug-interval")
              .doc("Write snapshot every N SfM iterations (default: 1, requires --debug-dir)"));
  cmd.add(make_option(0, bundler_max_cameras, "bundler-max-cameras")
              .doc("Subsample to at most N cameras in Bundler output (default: all)"));
  cmd.add(make_switch(0, "fix-intrinsics")
              .doc("Keep camera intrinsics fixed (do not optimize in BA). "));
  int flag_skip_2deg = 1;
  int flag_grid_subset = 1;
  int flag_fixed_pose = 1;
  cmd.add(make_option(0, flag_skip_2deg, "skip-2degree-tracks")
              .doc("Skip stable 2-view tracks from global BA (1=on [default], 0=off)."));
  cmd.add(make_option(0, flag_grid_subset, "ba-grid-subset")
              .doc("Grid-NMS BA subset selection (1=on [default], 0=off)."));
  cmd.add(make_option(0, ba_grid_target, "ba-grid-target")
              .doc("Target BA tracks per image for grid-NMS (default 1000)."));
  cmd.add(make_option(0, flag_fixed_pose, "ba-fixed-pose-skip")
              .doc("Fixed-pose Ceres solve for skipped tracks after global BA (1=on [default], 0=off)."));
  cmd.add(make_option(0, ba_threads, "ba-threads")
              .doc("Ceres num_threads for BA solves (default: 0 = use hardware concurrency)."));
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
  if (ba_threads < 0) {
    std::cerr << "Error: --ba-threads must be >= 0\n\n";
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
  opts.global_ba.early_phase_max_cameras = 100;
  opts.global_ba.periodic_every_n_images = 50; // global BA every 50 registered cameras (was 100)
  opts.global_ba.every_n_images = 3;
  opts.global_ba.early_phase_global_only_images = 35;
  opts.global_ba.intermediate_loose_after_images = 60; // must be > early_phase to avoid loose BA before intrinsics stabilise
  // ── Default-on optimisations (can be disabled via CLI --xxx 0) ─────────────
  opts.global_ba.skip_2degree_tracks = true;
  opts.global_ba.ba_grid_subset = true;
  opts.global_ba.ba_grid_target_per_image = 1000;
  opts.global_ba.ba_fixed_pose_optimize_skipped = true;
  opts.intrinsics.focal_prior_weight = 1.f;
  // kBatchNeighbor: variable = batch cameras + newly triangulated points;
  // constant = top-K co-visible neighbors. Historical camera observations are NEVER deleted
  // in local BA — only global BA (with full scene context) performs outlier rejection.
  // This prevents the cascading observation-loss that kColmap can cause.
  opts.local_ba.enable = true;
  opts.local_ba.strategy = LocalBAStrategy::kBatchNeighbor;
  opts.local_ba.neighbor_k = 8;            // co-visible anchor neighbors per batch camera
  opts.local_ba.switch_after_n_images = 100;
  opts.resection.backend = ResectionBackend::kPoseLib;
  if (ba_threads > 0) {
    opts.global_ba.solver_overrides.num_threads = ba_threads;
    LOG(INFO) << "Ceres BA num_threads=" << ba_threads;
  }
  if (cmd.used("fix-intrinsics")) {
    opts.global_ba.optimize_intrinsics = false;
    LOG(INFO) << "--fix-intrinsics: camera intrinsics will be held constant in all BA runs.";
  }
  // Apply CLI overrides (default 1; pass 0 to disable).
  opts.global_ba.skip_2degree_tracks       = (flag_skip_2deg  != 0);
  opts.global_ba.ba_grid_subset            = (flag_grid_subset != 0);
  opts.global_ba.ba_fixed_pose_optimize_skipped = (flag_fixed_pose != 0);
  opts.global_ba.ba_grid_target_per_image  = ba_grid_target;
  LOG(INFO) << "[opts] skip_2deg=" << opts.global_ba.skip_2degree_tracks
            << " grid_subset=" << opts.global_ba.ba_grid_subset
            << " grid_target=" << opts.global_ba.ba_grid_target_per_image
            << " fixed_pose_skip=" << opts.global_ba.ba_fixed_pose_optimize_skipped;

  // Per-iteration debug snapshots: write bundle.out + list.txt to debug_dir/iter_NNNN/
  if (!debug_dir.empty()) {
    std::filesystem::create_directories(debug_dir);
    opts.debug.snapshot_every_n_iters = (debug_interval > 0) ? debug_interval : 1;
    // Capture by value the data needed for writing (image_paths, cameras, image_to_camera_index).
    // poses_R/C and store are passed by const-ref from the pipeline.
    const std::vector<std::string> snap_image_paths = image_paths;
    const std::vector<camera::Intrinsics> snap_cameras = project.cameras;
    const std::vector<int> snap_img2cam = project.image_to_camera_index;
    const int snap_max_cams = bundler_max_cameras;
    const std::string snap_base = debug_dir;
    opts.debug.on_snapshot = [snap_image_paths, snap_cameras, snap_img2cam, snap_max_cams,
                              snap_base](int sfm_iter, int num_registered,
                                         const std::vector<Eigen::Matrix3d>& R,
                                         const std::vector<Eigen::Vector3d>& C,
                                         const std::vector<bool>& reg, const TrackStore& store) {
      std::ostringstream ss;
      ss << snap_base << "/iter_" << std::setw(4) << std::setfill('0') << sfm_iter;
      const std::string iter_dir = ss.str();
      std::filesystem::create_directories(iter_dir);
      write_bundler(iter_dir, snap_image_paths, R, C, reg, snap_cameras, snap_img2cam, store,
                    snap_max_cams);
      LOG(INFO) << "[debug] iter=" << sfm_iter << " n_reg=" << num_registered
                << " snapshot → " << iter_dir;
    };
    LOG(INFO) << "--debug-dir=" << debug_dir << "  interval=" << opts.debug.snapshot_every_n_iters
              << "  max_cameras=" << (bundler_max_cameras > 0 ? std::to_string(bundler_max_cameras)
                                                               : "all");
  }

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
  if (!write_poses_json(out_path, poses_R, poses_C, registered, project.cameras,
                        project.image_to_camera_index)) {
    LOG(ERROR) << "Failed to write poses";
    return 1;
  }
  LOG(INFO) << "Wrote " << out_path;

  write_bundler(output_dir, image_paths, poses_R, poses_C, registered, project.cameras,
                project.image_to_camera_index, store);

  write_colmap(output_dir, image_paths, poses_R, poses_C, registered, project.cameras,
               project.image_to_camera_index, store);

  // ── Save TrackStore (3-D points + observation flags) to bundle dir ────────
  // Saved as tracks.isat_tracks in the same output directory so downstream
  // tools (test_sfm_diag2, isat_incremental_sfm re-run, …) can load it.
  // All tracks are written (including dead/outlier), track_flags preserves
  // kHasTriangulated so report tools can distinguish triangulated from raw.
  {
    const int n_imgs = project.num_images();
    std::vector<uint32_t> img_indices(static_cast<size_t>(n_imgs));
    for (int i = 0; i < n_imgs; ++i)
      img_indices[static_cast<size_t>(i)] = static_cast<uint32_t>(i);

    TrackSaveOptions sfm_opts;
    sfm_opts.is_sfm_result         = true;
    sfm_opts.num_registered_images = n_reg;
    // num_triangulated / num_inlier auto-counted in save_track_store_to_idc

    const std::string tracks_out = output_dir + "/tracks.isat_tracks";
    if (save_track_store_to_idc(store, img_indices, tracks_out,
                                /*view_graph=*/nullptr, &sfm_opts)) {
      LOG(INFO) << "Saved TrackStore → " << tracks_out;
    } else {
      LOG(ERROR) << "Failed to save TrackStore to " << tracks_out;
    }
  }

  return 0;
}
