/**
 * @file  isat_undistort.cpp
 * @brief Undistort registered SfM images and write COLMAP sparse for 3DGS training.
 *
 * Pipeline:
 *   [pre]   Pre-generate undistortion maps per camera (shared by all images using that camera)
 *   Stage 1 [multi-thread I/O]    Read images from disk
 *   Stage 2 [multi-thread CPU]    Undistort via cv::remap using pre-generated maps
 *   Stage 3 [multi-thread I/O]    Write as <out>/colmap/sparse/0/images/%08d.jpg
 *   [post]  Write COLMAP sparse/0/ cameras.txt / images.txt / points3D.txt
 *
 * Usage:
 *   isat_undistort -p project.json -j poses.json -o out_dir
 *   isat_undistort -p project.json -t tracks.isat_tracks -o out_dir   (poses from IDC)
 *   isat_undistort -p project.json -j poses.json -o out_dir --jpg-quality 95 -j 4
 *
 * Output:
 *   out_dir/colmap/images/%08d.jpg  — undistorted images
 *   out_dir/colmap/sparse/0/       — COLMAP text files (PINHOLE cameras, %08d names)
 */

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iomanip>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cli_logging.h"
#include "cmdLine/cmdLine.h"
#include "task_queue/task_queue.hpp"
#include "../io/track_store_idc.h"
#include "../modules/camera/camera_utils.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Data types
// ─────────────────────────────────────────────────────────────────────────────

/// Optimised camera intrinsics after BA (with distortion)
struct CameraIntrinsics {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int width = 0, height = 0;
  double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;
};

/// Per-camera pre-computed undistortion maps
struct CameraUndistortMaps {
  cv::Mat map1;
  cv::Mat map2;
};

/// Per-image task, flows through the pipeline
struct UndistortTask {
  // Populated by project.json + poses.json (immutable through pipeline)
  int image_index = -1;
  int colmap_image_id = 0;   // 1-based COLMAP image ID
  int camera_idx = 0;
  std::string src_path;

  // Stage 1: loaded image
  cv::Mat image;

  // Stage 2: undistorted image
  cv::Mat undistorted;

  // Stage 3: success flag (set after successful write)
  bool write_success = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Load project.json  → image paths & image_to_camera_index
// ─────────────────────────────────────────────────────────────────────────────

static bool load_project_info(const std::string& path,
                              std::vector<std::string>* image_paths,
                              std::vector<int>* image_to_camera_index) {
  std::ifstream f(path);
  if (!f.is_open()) { LOG(ERROR) << "Cannot open " << path; return false; }
  json j; try { f >> j; } catch (...) {
    LOG(ERROR) << "Failed to parse " << path; return false;
  }
  if (!j.contains("images") || !j["images"].is_array()) {
    LOG(ERROR) << path << ": missing 'images' array"; return false;
  }
  const size_t n = j["images"].size();
  image_paths->resize(n);
  image_to_camera_index->resize(n);
  for (size_t i = 0; i < n; ++i) {
    (*image_paths)[i] = j["images"][i].value("path", std::string());
    (*image_to_camera_index)[i] = j["images"][i].value("camera_index", 0);
  }
  LOG(INFO) << "Loaded " << n << " image paths from " << path;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Load poses.json  → registered poses & optimised camera intrinsics
// ─────────────────────────────────────────────────────────────────────────────

struct PoseBundle {
  // Registered image ordering (same order as COLMAP image IDs)
  std::vector<int> image_indices;          // original image_index per registered image

  // Pose data (indexed by position in image_indices)
  std::vector<Eigen::Matrix3d> poses_R;    // world→camera rotation
  std::vector<Eigen::Vector3d> poses_C;    // camera centre in world

  // Camera intrinsics (indexed by camera_idx from project.json)
  std::vector<CameraIntrinsics> cameras;
};

static bool load_poses_json(const std::string& path, int total_images, PoseBundle* out) {
  std::ifstream f(path);
  if (!f.is_open()) { LOG(ERROR) << "Cannot open " << path; return false; }
  json j; try { f >> j; } catch (...) {
    LOG(ERROR) << "Failed to parse " << path; return false;
  }

  // Cameras
  if (!j.contains("cameras") || !j["cameras"].is_array()) {
    LOG(ERROR) << path << ": missing 'cameras' array"; return false;
  }
  out->cameras.clear();
  for (const auto& cam : j["cameras"]) {
    CameraIntrinsics K;
    K.fx = cam.value("fx", 0.0);     K.fy = cam.value("fy", 0.0);
    K.cx = cam.value("cx", 0.0);     K.cy = cam.value("cy", 0.0);
    K.width = cam.value("width", 0); K.height = cam.value("height", 0);
    K.k1 = cam.value("k1", 0.0);     K.k2 = cam.value("k2", 0.0);
    K.k3 = cam.value("k3", 0.0);
    K.p1 = cam.value("p1", 0.0);     K.p2 = cam.value("p2", 0.0);
    out->cameras.push_back(K);
  }

  // Poses
  if (!j.contains("poses") || !j["poses"].is_array()) {
    LOG(ERROR) << path << ": missing 'poses' array"; return false;
  }
  out->image_indices.clear();
  out->poses_R.clear();
  out->poses_C.clear();
  for (const auto& pose : j["poses"]) {
    const int img_idx = pose.value("image_index", -1);
    if (img_idx < 0 || img_idx >= total_images) {
      LOG(WARNING) << "Skip pose with invalid image_index=" << img_idx;
      continue;
    }
    out->image_indices.push_back(img_idx);

    // Parse rotation R (row-major 9 floats)
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    const auto& rarr = pose["R"];
    if (rarr.is_array() && rarr.size() >= 9)
      R << rarr[0].get<double>(), rarr[1].get<double>(), rarr[2].get<double>(),
           rarr[3].get<double>(), rarr[4].get<double>(), rarr[5].get<double>(),
           rarr[6].get<double>(), rarr[7].get<double>(), rarr[8].get<double>();
    out->poses_R.push_back(R);

    // Parse camera centre C
    Eigen::Vector3d C = Eigen::Vector3d::Zero();
    const auto& carr = pose["C"];
    if (carr.is_array() && carr.size() >= 3)
      C << carr[0].get<double>(), carr[1].get<double>(), carr[2].get<double>();
    out->poses_C.push_back(C);
  }
  LOG(INFO) << "Loaded " << out->image_indices.size() << " poses, "
            << out->cameras.size() << " cameras from " << path;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Load PoseBundle from embedded SfMResultData (tracks IDC)
// ─────────────────────────────────────────────────────────────────────────────

static bool poses_from_sfm_result(const insight::sfm::SfMResultData& sfm,
                                  int total_images, PoseBundle* out) {
  const int n_imgs = static_cast<int>(sfm.registered.size());
  if (n_imgs != total_images) {
    LOG(ERROR) << "SfMResultData images mismatch: " << n_imgs << " vs " << total_images;
    return false;
  }

  out->cameras.clear();
  for (int ci = 0; ci < sfm.num_cameras; ++ci) {
    const float* k = &sfm.intrinsics[static_cast<size_t>(ci) * 11];
    CameraIntrinsics K;
    K.fx = k[0]; K.fy = k[1]; K.cx = k[2]; K.cy = k[3];
    K.width = static_cast<int>(k[4]); K.height = static_cast<int>(k[5]);
    K.k1 = k[6]; K.k2 = k[7]; K.k3 = k[8]; K.p1 = k[9]; K.p2 = k[10];
    out->cameras.push_back(K);
  }

  out->image_indices.clear();
  out->poses_R.clear();
  out->poses_C.clear();

  for (int i = 0; i < n_imgs; ++i) {
    if (!sfm.registered[static_cast<size_t>(i)])
      continue;
    out->image_indices.push_back(i);

    const float* r = &sfm.pose_R[static_cast<size_t>(i) * 9];
    Eigen::Matrix3d R;
    R << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
    out->poses_R.push_back(R);

    const float* c = &sfm.pose_C[static_cast<size_t>(i) * 3];
    out->poses_C.push_back(Eigen::Vector3d(c[0], c[1], c[2]));
  }

  LOG(INFO) << "Loaded " << out->image_indices.size() << " poses, "
            << out->cameras.size() << " cameras from embedded SfMResultData";
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pre-generate undistortion maps per camera
// ─────────────────────────────────────────────────────────────────────────────

static bool generate_camera_maps(
    const std::vector<CameraIntrinsics>& cameras,
    std::vector<CameraUndistortMaps>* maps) {
  const size_t n = cameras.size();
  maps->resize(n);
  for (size_t ci = 0; ci < n; ++ci) {
    const auto& K = cameras[ci];
    cv::Mat cam_mat = (cv::Mat_<double>(3, 3) << K.fx, 0.0, K.cx,
                       0.0, K.fy, K.cy,
                       0.0, 0.0, 1.0);
    // OpenCV order: [k1, k2, p1, p2, k3]; OpenCV p1 = internal p2, p2 = internal p1
    cv::Mat dist = (cv::Mat_<double>(5, 1) << K.k1, K.k2, K.p2, K.p1, K.k3);
    cv::Mat new_cam = cam_mat.clone();
    const cv::Size size(K.width, K.height);
    cv::initUndistortRectifyMap(cam_mat, dist, cv::Mat(), new_cam,
                                size, CV_32FC1, (*maps)[ci].map1, (*maps)[ci].map2);
  }
  LOG(INFO) << "Pre-generated undistortion maps for " << n << " cameras";
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write COLMAP sparse text (PINHOLE, %08d names, undistorted 2D points)
// ─────────────────────────────────────────────────────────────────────────────

/// Build insight::camera::Intrinsics from our CameraIntrinsics (same field layout)
static insight::camera::Intrinsics to_cam_intr(const CameraIntrinsics& K) {
  insight::camera::Intrinsics ck;
  ck.fx = K.fx; ck.fy = K.fy; ck.cx = K.cx; ck.cy = K.cy;
  ck.width = K.width; ck.height = K.height;
  ck.k1 = K.k1; ck.k2 = K.k2; ck.k3 = K.k3;
  ck.p1 = K.p1; ck.p2 = K.p2;
  return ck;
}

static bool write_colmap_sparse(
    const std::string& sparse_dir,
    const std::vector<UndistortTask>& tasks,
    const std::vector<CameraIntrinsics>& cameras,
    const PoseBundle& poses,
    const insight::sfm::TrackStore* store) {

  // Determine unique cameras actually used
  std::map<int, int> cam_idx_to_colmap_id;
  int next_cam_id = 1;
  for (const auto& t : tasks) {
    if (cam_idx_to_colmap_id.find(t.camera_idx) == cam_idx_to_colmap_id.end())
      cam_idx_to_colmap_id[t.camera_idx] = next_cam_id++;
  }

  // Map: image_index → position in tasks vector
  std::map<int, size_t> img_to_task;  // image_index → tasks[].colmap_image_id
  for (const auto& t : tasks)
    img_to_task[t.image_index] = static_cast<size_t>(t.colmap_image_id - 1);

  // Map: image_index → position in poses.image_indices
  std::map<int, size_t> idx_to_pose;
  for (size_t pi = 0; pi < poses.image_indices.size(); ++pi)
    idx_to_pose[poses.image_indices[pi]] = pi;

  // ── Build per-image 2D → 3D observation lists (undistorted) ────────────────
  // img_obs[task_idx] = list of (u_undist, v_undist, point3d_id_1based)
  struct Obs2D { float u, v; int point3d_id; };
  std::vector<std::vector<Obs2D>> img_obs(tasks.size());

  // Collect points3D data
  struct ColmapPoint3D {
    float x, y, z;
    int point3d_id;
    std::vector<std::pair<int, int>> track; // (colmap_image_id, point2d_idx)
  };
  std::vector<ColmapPoint3D> colmap_points;
  int next_pt_id = 1;

  if (store) {
    // Pre-build per-camera Intrinsics for undistortion
    std::vector<insight::camera::Intrinsics> cam_intr(cameras.size());
    for (size_t ci = 0; ci < cameras.size(); ++ci)
      cam_intr[ci] = to_cam_intr(cameras[ci]);

    std::vector<insight::sfm::Observation> obs_buf;
    const size_t n_tracks = store->num_tracks();
    for (size_t ti = 0; ti < n_tracks; ++ti) {
      const int tid = static_cast<int>(ti);
      if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
        continue;
      float px, py, pz;
      store->get_track_xyz(tid, &px, &py, &pz);

      obs_buf.clear();
      store->get_track_observations(tid, &obs_buf);

      ColmapPoint3D pt;
      pt.x = px; pt.y = py; pt.z = pz;
      pt.point3d_id = next_pt_id;

      for (const auto& o : obs_buf) {
        const int img_idx = static_cast<int>(o.image_index);
        auto tit = img_to_task.find(img_idx);
        if (tit == img_to_task.end())
          continue; // image not registered
        const size_t task_idx = tit->second;

        // Undistort (u, v)
        const int ci = tasks[task_idx].camera_idx;
        double uu = static_cast<double>(o.u);
        double vu = static_cast<double>(o.v);
        if (static_cast<size_t>(ci) < cam_intr.size())
          insight::camera::undistort_point(cam_intr[static_cast<size_t>(ci)], o.u, o.v, &uu, &vu);

        const int pt2d_idx = static_cast<int>(img_obs[task_idx].size());
        img_obs[task_idx].push_back({static_cast<float>(uu), static_cast<float>(vu), next_pt_id});
        pt.track.emplace_back(tasks[task_idx].colmap_image_id, pt2d_idx);
      }

      if (pt.track.size() >= 2) {
        colmap_points.push_back(std::move(pt));
        ++next_pt_id;
      }
    }
    LOG(INFO) << "Collected " << colmap_points.size() << " triangulated points from TrackStore";
  }

  // ── cameras.txt ──
  {
    const std::string p = sparse_dir + "/cameras.txt";
    std::ofstream f(p); if (!f.is_open()) { LOG(ERROR) << "Cannot write " << p; return false; }
    f << "# Camera list with one line of data per camera:\n"
      << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
      << "# Number of cameras: " << cam_idx_to_colmap_id.size() << "\n";
    f << std::fixed << std::setprecision(6);
    for (const auto& kv : cam_idx_to_colmap_id) {
      const auto& K = cameras[static_cast<size_t>(kv.first)];
      f << kv.second << " PINHOLE " << K.width << " " << K.height << " "
        << K.fx << " " << K.fy << " " << K.cx << " " << K.cy << "\n";
    }
    LOG(INFO) << "Wrote " << p;
  }

  // ── images.txt ──
  {
    const std::string p = sparse_dir + "/images.txt";
    std::ofstream f(p); if (!f.is_open()) { LOG(ERROR) << "Cannot write " << p; return false; }
    f << "# Image list with two lines of data per image:\n"
      << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
      << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n"
      << "# Number of images: " << tasks.size() << "\n";
    f << std::fixed << std::setprecision(9);

    for (const auto& t : tasks) {
      auto pit = idx_to_pose.find(t.image_index);
      if (pit == idx_to_pose.end()) continue;
      const size_t pi = pit->second;

      const Eigen::Quaterniond q(poses.poses_R[pi]);
      const Eigen::Vector3d tr = poses.poses_R[pi] * (-poses.poses_C[pi]);
      const int cmap_cam_id = cam_idx_to_colmap_id[t.camera_idx];

      std::ostringstream ss;
      ss << std::setw(8) << std::setfill('0') << t.colmap_image_id << ".jpg";
      const std::string name = ss.str();

      f << t.colmap_image_id << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z()
        << " " << tr(0) << " " << tr(1) << " " << tr(2) << " " << cmap_cam_id << " " << name << "\n";

      // Line 2: undistorted POINTS2D
      const size_t task_idx = static_cast<size_t>(t.colmap_image_id - 1);
      const auto& obs = img_obs[task_idx];
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
    LOG(INFO) << "Wrote " << p << " with " << next_pt_id - 1 << " points referenced";
  }

  // ── points3D.txt ──
  {
    const std::string p = sparse_dir + "/points3D.txt";
    std::ofstream f(p); if (!f.is_open()) { LOG(ERROR) << "Cannot write " << p; return false; }
    f << "# 3D point list with one line of data per point:\n"
      << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]\n"
      << "# Number of points: " << colmap_points.size() << "\n";
    f << std::fixed << std::setprecision(6);
    for (const auto& pt : colmap_points) {
      f << pt.point3d_id << " " << pt.x << " " << pt.y << " " << pt.z
        << " 128 128 128 0.0";
      for (const auto& [img_id, pt2d_idx] : pt.track)
        f << " " << img_id << " " << pt2d_idx;
      f << "\n";
    }
    LOG(INFO) << "Wrote " << p << " (" << colmap_points.size() << " points)";
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write COLMAP binary format (cameras.bin / images.bin / points3D.bin)
// ─────────────────────────────────────────────────────────────────────────────

static bool write_colmap_binary(
    const std::string& sparse_dir,
    const std::vector<UndistortTask>& tasks,
    const std::vector<CameraIntrinsics>& cameras,
    const PoseBundle& poses,
    const insight::sfm::TrackStore* store) {

  // Same camera + pose index maps as write_colmap_sparse
  std::map<int, int> cam_idx_to_colmap_id;
  int next_cam_id = 1;
  for (const auto& t : tasks)
    if (cam_idx_to_colmap_id.find(t.camera_idx) == cam_idx_to_colmap_id.end())
      cam_idx_to_colmap_id[t.camera_idx] = next_cam_id++;

  std::map<int, size_t> img_to_task;
  for (const auto& t : tasks)
    img_to_task[t.image_index] = static_cast<size_t>(t.colmap_image_id - 1);

  std::map<int, size_t> idx_to_pose;
  for (size_t pi = 0; pi < poses.image_indices.size(); ++pi)
    idx_to_pose[poses.image_indices[pi]] = pi;

  // Build per-image 2D observations + 3D points
  struct Pt2D { double x, y; uint64_t pt3d_id; };
  std::vector<std::vector<Pt2D>> img_pts(tasks.size());

  struct Pt3D {
    uint64_t id;
    double x, y, z;
    double error;
    std::vector<std::pair<uint32_t, uint32_t>> track;  // (image_id: uint32_t, point2D_idx: uint32_t)
  };
  std::vector<Pt3D> points3d;
  uint64_t next_pt_id = 1;

  if (store) {
    std::vector<insight::camera::Intrinsics> cam_intr(cameras.size());
    for (size_t ci = 0; ci < cameras.size(); ++ci)
      cam_intr[ci] = to_cam_intr(cameras[ci]);

    std::vector<insight::sfm::Observation> obs_buf;
    for (size_t ti = 0; ti < store->num_tracks(); ++ti) {
      const int tid = static_cast<int>(ti);
      if (!store->is_track_valid(tid) || !store->track_has_triangulated_xyz(tid))
        continue;
      float px, py, pz;
      store->get_track_xyz(tid, &px, &py, &pz);

      obs_buf.clear();
      store->get_track_observations(tid, &obs_buf);

      Pt3D pt;
      pt.id = next_pt_id;
      pt.x = static_cast<double>(px);
      pt.y = static_cast<double>(py);
      pt.z = static_cast<double>(pz);

      double sum_err = 0.0;
      int n_err = 0;
      for (const auto& o : obs_buf) {
        const int img_idx = static_cast<int>(o.image_index);
        auto tit = img_to_task.find(img_idx);
        if (tit == img_to_task.end()) continue;
        const size_t task_idx = tit->second;

        const int ci = tasks[task_idx].camera_idx;
        double uu = o.u, vu = o.v;
        if (static_cast<size_t>(ci) < cam_intr.size())
          insight::camera::undistort_point(cam_intr[static_cast<size_t>(ci)], o.u, o.v, &uu, &vu);

        const uint32_t pt2d_idx = static_cast<uint32_t>(img_pts[task_idx].size());
        img_pts[task_idx].push_back({uu, vu, static_cast<uint64_t>(next_pt_id)});
        pt.track.emplace_back(static_cast<uint32_t>(tasks[task_idx].colmap_image_id), pt2d_idx);

        // Reprojection error (in pixel units, using the PINHOLE projection)
        auto pit = idx_to_pose.find(img_idx);
        if (pit != idx_to_pose.end()) {
          const Eigen::Vector3d Xw(pt.x, pt.y, pt.z);
          const size_t pi = pit->second;
          Eigen::Vector3d pc = poses.poses_R[pi] * (Xw - poses.poses_C[pi]);
          if (pc(2) > 1e-12) {
            const double inv_z = 1.0 / pc(2);
            const int ci_err = tasks[task_idx].camera_idx;
            const auto& Kc = cam_intr[static_cast<size_t>(ci_err)];
            const double proj_u = Kc.fx * pc(0) * inv_z + Kc.cx;
            const double proj_v = Kc.fy * pc(1) * inv_z + Kc.cy;
            double du = uu - proj_u;
            double dv = vu - proj_v;
            sum_err += std::sqrt(du * du + dv * dv);
            ++n_err;
          }
        }
      }
      if (pt.track.size() >= 2) {
        pt.error = (n_err > 0) ? sum_err / n_err : 0.0;
        points3d.push_back(std::move(pt));
        ++next_pt_id;
      }
    }
    LOG(INFO) << "Collected " << points3d.size() << " points for binary output";
  }

  // ── cameras.bin ──
  {
    const std::string p = sparse_dir + "/cameras.bin";
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.is_open()) { LOG(ERROR) << "Cannot write " << p; return false; }
    const uint64_t n_cams = static_cast<uint64_t>(cam_idx_to_colmap_id.size());
    f.write(reinterpret_cast<const char*>(&n_cams), sizeof(uint64_t));
    for (const auto& kv : cam_idx_to_colmap_id) {
      const auto& K = cameras[static_cast<size_t>(kv.first)];
      const uint32_t cam_id = static_cast<uint32_t>(kv.second);
      const int32_t model_id = 1; // PINHOLE
      const uint64_t w = static_cast<uint64_t>(K.width);
      const uint64_t h = static_cast<uint64_t>(K.height);
      const double params[4] = {K.fx, K.fy, K.cx, K.cy};
      f.write(reinterpret_cast<const char*>(&cam_id), sizeof(uint32_t));
      f.write(reinterpret_cast<const char*>(&model_id), sizeof(int32_t));
      f.write(reinterpret_cast<const char*>(&w), sizeof(uint64_t));
      f.write(reinterpret_cast<const char*>(&h), sizeof(uint64_t));
      f.write(reinterpret_cast<const char*>(params), 4 * sizeof(double));
    }
    f.close();
    if (f.fail()) { LOG(ERROR) << "Write failed for " << p; return false; }
    LOG(INFO) << "Wrote " << p;
  }

  // ── images.bin ──
  {
    const std::string p = sparse_dir + "/images.bin";
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.is_open()) { LOG(ERROR) << "Cannot write " << p; return false; }
    const uint64_t n_imgs = static_cast<uint64_t>(tasks.size());
    f.write(reinterpret_cast<const char*>(&n_imgs), sizeof(uint64_t));
    for (const auto& t : tasks) {
      auto pit = idx_to_pose.find(t.image_index);
      if (pit == idx_to_pose.end()) continue;
      const size_t pi = pit->second;

      const Eigen::Quaterniond q(poses.poses_R[pi]);
      const Eigen::Vector3d tr = poses.poses_R[pi] * (-poses.poses_C[pi]);

      const uint32_t img_id = static_cast<uint32_t>(t.colmap_image_id);
      const double qvec[4] = {q.w(), q.x(), q.y(), q.z()};
      const double tvec[3] = {tr(0), tr(1), tr(2)};
      const uint32_t cam_id = static_cast<uint32_t>(cam_idx_to_colmap_id[t.camera_idx]);

      std::ostringstream ss;
      ss << std::setw(8) << std::setfill('0') << t.colmap_image_id << ".jpg";
      const std::string name = ss.str();

      f.write(reinterpret_cast<const char*>(&img_id), sizeof(uint32_t));
      f.write(reinterpret_cast<const char*>(qvec), 4 * sizeof(double));
      f.write(reinterpret_cast<const char*>(tvec), 3 * sizeof(double));
      f.write(reinterpret_cast<const char*>(&cam_id), sizeof(uint32_t));
      f.write(name.c_str(), name.size() + 1);

      const size_t task_idx = static_cast<size_t>(t.colmap_image_id - 1);
      const auto& pts = img_pts[task_idx];
      const uint64_t n_pts = static_cast<uint64_t>(pts.size());
      f.write(reinterpret_cast<const char*>(&n_pts), sizeof(uint64_t));
      for (const auto& p2 : pts) {
        const double xy[2] = {p2.x, p2.y};
        f.write(reinterpret_cast<const char*>(xy), 2 * sizeof(double));
        f.write(reinterpret_cast<const char*>(&p2.pt3d_id), sizeof(uint64_t));
      }
    }
    f.close();
    if (f.fail()) { LOG(ERROR) << "Write failed for " << p; return false; }
    LOG(INFO) << "Wrote " << p;
  }

  // ── points3D.bin ──
  {
    const std::string p = sparse_dir + "/points3D.bin";
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.is_open()) { LOG(ERROR) << "Cannot write " << p; return false; }
    const uint64_t n_pts = static_cast<uint64_t>(points3d.size());
    f.write(reinterpret_cast<const char*>(&n_pts), sizeof(uint64_t));
    for (const auto& pt : points3d) {
      const double xyz[3] = {pt.x, pt.y, pt.z};
      const uint8_t rgb[3] = {128, 128, 128};
      const uint64_t track_len = static_cast<uint64_t>(pt.track.size());
      f.write(reinterpret_cast<const char*>(&pt.id), sizeof(uint64_t));
      f.write(reinterpret_cast<const char*>(xyz), 3 * sizeof(double));
      f.write(reinterpret_cast<const char*>(rgb), 3);
      f.write(reinterpret_cast<const char*>(&pt.error), sizeof(double));
      f.write(reinterpret_cast<const char*>(&track_len), sizeof(uint64_t));
      for (const auto& [img_id, pt2d_idx] : pt.track) {
        f.write(reinterpret_cast<const char*>(&img_id), sizeof(uint32_t));
        f.write(reinterpret_cast<const char*>(&pt2d_idx), sizeof(uint32_t));
      }
    }
    f.close();
    if (f.fail()) { LOG(ERROR) << "Write failed for " << p; return false; }
    LOG(INFO) << "Wrote " << p << " (" << points3d.size() << " points)";
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  std::string project_path, poses_path, tracks_path, output_dir, log_level;
  int io_threads = 4;
  int jpg_quality = 95;
  int queue_size = 10;
  int use_binary = 0;

  CmdLine cmd("Undistort registered images for 3DGS / COLMAP output (Stage pipeline)");
  cmd.add(make_option('p', project_path, "project").doc("project.json (isat_project extract)"));
  cmd.add(make_option('j', poses_path, "poses").doc("poses.json (isat_incremental_sfm output)"));
  cmd.add(make_option('t', tracks_path, "tracks")
              .doc("tracks.isat_tracks (SfM output) — for points3D.txt with undistorted 2D obs"));
  cmd.add(make_option('o', output_dir,   "output").doc("Output directory"));
  cmd.add(make_option(0, io_threads, "threads")
              .doc("CPU I/O / undistort worker threads (default: 4)"));
  cmd.add(make_option(0, jpg_quality, "jpg-quality").doc("JPEG quality 1-100 (default: 95)"));
  cmd.add(make_option(0, queue_size, "queue-size").doc("Bounded queue size per stage (default: 10)"));
  cmd.add(make_switch(0, "binary").doc("Write COLMAP binary format (.bin) instead of text (.txt)"));
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
  cmd.add(make_switch('h', "help").doc("Show help"));

  try { cmd.process(argc, argv); }
  catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n"; cmd.printHelp(std::cerr, argv[0]); return 1;
  }
  if (cmd.checkHelp(argv[0])) return 0;
  if (project_path.empty() || output_dir.empty()) {
    std::cerr << "Error: -p and -o are required\n\n";
    cmd.printHelp(std::cerr, argv[0]); return 1;
  }
  if (io_threads < 1) { std::cerr << "Error: --threads must be >= 1\n\n"; return 1; }
  if (jpg_quality < 1) jpg_quality = 1;
  if (jpg_quality > 100) jpg_quality = 100;
  if (queue_size < 2) queue_size = 2;

  insight::tools::apply_log_level(cmd.used('v'), cmd.used('q'), log_level);
  LOG(INFO) << "isat_undistort: threads=" << io_threads << " jpg_quality=" << jpg_quality;
  const auto t_start = std::chrono::steady_clock::now();

  // ── 1. Load project info (image paths + image→camera mapping) ──────────────
  std::vector<std::string> image_paths;
  std::vector<int> image_to_camera_index;
  if (!load_project_info(project_path, &image_paths, &image_to_camera_index)) return 1;
  const int total_images = static_cast<int>(image_paths.size());

  // ── 2. Load poses (from poses.json OR embedded in tracks IDC) ──────────────
  PoseBundle poses;
  std::unique_ptr<insight::sfm::TrackStore> track_store;
  bool have_poses = false;

  if (!tracks_path.empty()) {
    // Try loading extended IDC with embedded pose data
    track_store = std::make_unique<insight::sfm::TrackStore>();
    std::vector<uint32_t> img_indices;
    insight::sfm::SfMResultData sfm_result;
    if (insight::sfm::load_track_store_from_idc(tracks_path, track_store.get(),
                                                 &img_indices, nullptr, &sfm_result)) {
      LOG(INFO) << "Loaded TrackStore: " << track_store->num_tracks() << " tracks, "
                << track_store->num_observations() << " observations";
      if (!sfm_result.pose_R.empty()) {
        have_poses = poses_from_sfm_result(sfm_result, total_images, &poses);
      } else {
        LOG(INFO) << "No embedded pose data in " << tracks_path
                  << ", falling back to poses.json";
      }
    } else {
      LOG(ERROR) << "Failed to load TrackStore from " << tracks_path;
      return 1;
    }
  }

  // Fallback: load standalone poses.json
  if (!have_poses) {
    if (poses_path.empty()) {
      LOG(ERROR) << "No pose data available: provide -j poses.json or -t tracks.isat_tracks";
      return 1;
    }
    if (!load_poses_json(poses_path, total_images, &poses)) return 1;
    have_poses = true;
  }

  if (poses.image_indices.empty()) {
    LOG(ERROR) << "No registered images to undistort"; return 1;
  }

  // ── 3. Pre-generate undistortion maps per camera ───────────────────────────
  std::vector<CameraUndistortMaps> camera_maps;
  if (!generate_camera_maps(poses.cameras, &camera_maps)) return 1;

  // ── 4. Build task list (only registered images) ────────────────────────────
  // COLMAP image ID = 1-based position in registered order
  std::vector<UndistortTask> tasks(poses.image_indices.size());
  for (size_t pi = 0; pi < poses.image_indices.size(); ++pi) {
    const int img_idx = poses.image_indices[pi];
    auto& t = tasks[pi];
    t.image_index = img_idx;
    t.colmap_image_id = static_cast<int>(pi) + 1; // 1-based
    t.camera_idx = (static_cast<size_t>(img_idx) < image_to_camera_index.size())
                       ? image_to_camera_index[static_cast<size_t>(img_idx)]
                       : 0;
    t.src_path = (static_cast<size_t>(img_idx) < image_paths.size())
                     ? image_paths[static_cast<size_t>(img_idx)]
                     : "";
    if (t.camera_idx < 0 || static_cast<size_t>(t.camera_idx) >= poses.cameras.size()) {
      LOG(WARNING) << "Image " << img_idx << " has invalid camera_idx=" << t.camera_idx
                   << ", clamping to 0";
      t.camera_idx = 0;
    }
  }

  const int n_tasks = static_cast<int>(tasks.size());
  LOG(INFO) << "Prepared " << n_tasks << " undistortion tasks";

  // ── 5. Create output directories ──────────────────────────────────────────
  const fs::path sparse_dir = fs::path(output_dir) / "colmap" / "sparse" / "0";
  const fs::path img_dir = fs::path(output_dir) / "colmap" / "images";
  try {
    fs::create_directories(img_dir);
    fs::create_directories(sparse_dir);
  } catch (const std::exception& e) {
    LOG(ERROR) << "Cannot create output dirs: " << e.what(); return 1;
  }

  // JPEG encode params
  std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpg_quality};

  // ── 6. Pipeline: 3-stage chain ────────────────────────────────────────────
  // Stage 1: multi-thread I/O — read images from disk
  Stage readStage("ReadImage", io_threads, queue_size,
      [&tasks](int idx) {
        auto& t = tasks[static_cast<size_t>(idx)];
        if (t.src_path.empty()) {
          LOG(WARNING) << "Task " << idx << ": empty src_path";
          return;
        }
        t.image = cv::imread(t.src_path, cv::IMREAD_UNCHANGED);
        if (t.image.empty()) {
          LOG(ERROR) << "Failed to read: " << t.src_path;
          return;
        }
        LOG(INFO) << "Loaded [" << idx << "/" << tasks.size() << "]: "
                  << t.src_path << " (" << t.image.cols << "x" << t.image.rows << ")";
      });

  // Stage 2: multi-thread CPU — undistort images using pre-generated maps
  Stage undistortStage("Undistort", io_threads, queue_size,
      [&tasks, &camera_maps](int idx) {
        auto& t = tasks[static_cast<size_t>(idx)];
        if (t.image.empty()) return;
        const auto& maps = camera_maps[static_cast<size_t>(t.camera_idx)];
        cv::remap(t.image, t.undistorted, maps.map1, maps.map2, cv::INTER_LINEAR);
        t.image.release(); // free original
        LOG(INFO) << "Undistorted [" << idx << "]";
      });

  // Stage 3: multi-thread I/O — write undistorted images
  Stage writeStage("WriteImage", io_threads, queue_size,
      [&tasks, &img_dir, &encode_params](int idx) {
        auto& t = tasks[static_cast<size_t>(idx)];
        if (t.undistorted.empty()) return;
        std::ostringstream ss;
        ss << std::setw(8) << std::setfill('0') << t.colmap_image_id << ".jpg";
        const std::string dst = (img_dir / ss.str()).string();
        if (!cv::imwrite(dst, t.undistorted, encode_params)) {
          LOG(ERROR) << "Failed to write: " << dst;
          return;
        }
        t.write_success = true;  // Mark successful write before release
        t.undistorted.release(); // free memory
        LOG(INFO) << "Wrote [" << idx << "]: " << dst;
      });

  // Chain: read → undistort → write
  chain(readStage, undistortStage);
  chain(undistortStage, writeStage);

  // Set task counts and push
  readStage.setTaskCount(n_tasks);
  undistortStage.setTaskCount(n_tasks);
  writeStage.setTaskCount(n_tasks);

  for (int i = 0; i < n_tasks; ++i)
    readStage.push(i);

  // Wait for pipeline completion
  readStage.wait();
  undistortStage.wait();
  writeStage.wait();

  // ── 7. Gather statistics ──────────────────────────────────────────────────
  int wrote = 0, failed_read = 0, failed_undist = 0;
  for (const auto& t : tasks) {
    if (t.src_path.empty())
      ++failed_read;
    else if (!t.write_success)
      ++failed_undist;
    else
      ++wrote;
  }

  const auto t_end = std::chrono::steady_clock::now();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
  LOG(INFO) << "Undistortion pipeline: " << wrote << " written, "
            << failed_read << " read failures, " << failed_undist << " undistort failures, "
            << elapsed_ms << " ms total";

  if (wrote == 0) { LOG(ERROR) << "No images were successfully undistorted"; return 1; }

  // ── 8. Write COLMAP sparse files ──────────────────────────────────────────
  if (cmd.used("binary")) {
    if (!write_colmap_binary(sparse_dir.string(), tasks, poses.cameras, poses,
                             track_store ? track_store.get() : nullptr)) {
      LOG(ERROR) << "Failed to write COLMAP binary files"; return 1;
    }
  } else {
    if (!write_colmap_sparse(sparse_dir.string(), tasks, poses.cameras, poses,
                             track_store ? track_store.get() : nullptr)) {
      LOG(ERROR) << "Failed to write COLMAP text files"; return 1;
    }
  }

  LOG(INFO) << "Done. Output:";
  LOG(INFO) << "  COLMAP sparse:      " << sparse_dir.string() << "/";

  return 0;
}
