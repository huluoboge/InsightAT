/**
 * @file  project_loader.h
 * @brief Load project JSON (images + cameras) for index-only incremental SfM.
 *
 * Single JSON format (isat_project export): images[] with camera_index, cameras[].
 * Output: cameras and image_to_camera_index for intrinsics access as
 *   cameras[image_to_camera_index[image_index]]. No IdMapping.
 */

#pragma once

#include "../modules/camera/camera_types.h"
#include <fstream>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

namespace insight {
namespace tools {

/**
 * Project data for incremental SfM: cameras and per-image camera index.
 * image_index = array position (0..n_images-1); intrinsics = cameras[image_to_camera_index[i]].
 */
struct ProjectData {
  std::vector<camera::Intrinsics> cameras;
  std::vector<int> image_to_camera_index;

  int num_images() const {
    return static_cast<int>(image_to_camera_index.size());
  }
  int num_cameras() const {
    return static_cast<int>(cameras.size());
  }
};

/**
 * Load project from a single JSON (images + cameras).
 * Expected format: { "images": [ { "camera_index": 0, "path": "..." }, ... ],
 *                    "cameras": [ { "fx", "fy", "cx", "cy", "k1", "k2", ... }, ... ] }
 * @return true if loaded successfully and at least one image; false on error.
 */
inline bool load_project_data(const std::string& path, ProjectData* out) {
  if (!out || path.empty()) return false;
  out->cameras.clear();
  out->image_to_camera_index.clear();

  std::ifstream f(path);
  if (!f.is_open()) {
    LOG(ERROR) << "project_loader: cannot open " << path;
    return false;
  }
  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    LOG(ERROR) << "project_loader: failed to parse JSON: " << e.what();
    return false;
  }
  if (!j.contains("images") || !j["images"].is_array()) {
    LOG(ERROR) << "project_loader: missing or invalid 'images' array";
    return false;
  }
  if (!j.contains("cameras") || !j["cameras"].is_array()) {
    LOG(ERROR) << "project_loader: missing or invalid 'cameras' array";
    return false;
  }

  const auto& cameras_arr = j["cameras"];
  for (const auto& cam : cameras_arr) {
    camera::Intrinsics K;
    K.fx = cam.value("fx", 0.0);
    K.fy = cam.value("fy", 0.0);
    K.cx = cam.value("cx", 0.0);
    K.cy = cam.value("cy", 0.0);
    K.k1 = cam.value("k1", 0.0);
    K.k2 = cam.value("k2", 0.0);
    K.k3 = cam.value("k3", 0.0);
    K.p1 = cam.value("p1", 0.0);
    K.p2 = cam.value("p2", 0.0);
    out->cameras.push_back(K);
  }

  const size_t n_images = j["images"].size();
  out->image_to_camera_index.reserve(n_images);
  for (size_t i = 0; i < n_images; ++i) {
    const auto& img = j["images"][i];
    int cidx = img.value("camera_index", 0);
    if (cidx < 0 || static_cast<size_t>(cidx) >= out->cameras.size())
      cidx = 0;
    out->image_to_camera_index.push_back(cidx);
  }
  LOG(INFO) << "project_loader: " << out->num_images() << " images, " << out->num_cameras()
            << " cameras from " << path;
  return !out->image_to_camera_index.empty();
}

} // namespace tools
} // namespace insight
