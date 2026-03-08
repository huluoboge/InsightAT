/**
 * isat_intrinsics.h
 * InsightAT – Shared camera intrinsics types for CLI tools.
 *
 * Supports two JSON formats:
 *
 *   Single-camera (legacy, backward-compatible):
 *     { "fx": 3600, "fy": 3600, "cx": 2000, "cy": 1500 }
 *     → Stored in the map under key 1.
 *
 *   Multi-camera (exported by  isat_project intrinsics --all):
 *     {
 *       "schema": "multi_camera_v1",
 *       "cameras": {
 *         "1": { "fx": 3600, "fy": 3600, "cx": 2000, "cy": 1500, "width": 4000, "height": 3000 },
 *         "2": { "fx": 4200, "fy": 4200, "cx": 2048, "cy": 1536 }
 *       }
 *     }
 *     → Each camera stored under its integer key (== group_id from isat_project).
 *
 * Distortion parameters (optional, Bentley-compatible 5-parameter model):
 *   "k1", "k2", "k3", "p1", "p2" can appear in any camera entry.
 *
 * Export JSON has "cameras" and images[].camera_index. isat_geo and isat_twoview
 * use build_image_index_intrinsics_from_list(-l) and index by task.image1_index /
 * task.image2_index for per-pair K.
 */
#pragma once

#include "../modules/camera/camera_types.h"
#include "../modules/sfm/id_mapping.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Pinhole + optional Brown-Conrady distortion intrinsics for CLI tools
// (5-parameter model, identical to Bentley ContextCapture)
// ─────────────────────────────────────────────────────────────────────────────

struct CameraIntrinsics {
  double fx = 0.0, fy = 0.0; ///< Focal lengths (pixels)
  double cx = 0.0, cy = 0.0; ///< Principal point (pixels)
  uint32_t width = 0;        ///< Image width  (pixels, 0 = unknown)
  uint32_t height = 0;       ///< Image height (pixels, 0 = unknown)

  // Bentley-compatible 5-parameter distortion (all default to 0 = no distortion)
  double k1 = 0.0; ///< Radial K₁
  double k2 = 0.0; ///< Radial K₂
  double k3 = 0.0; ///< Radial K₃
  double p1 = 0.0; ///< Tangential P₁
  double p2 = 0.0; ///< Tangential P₂

  bool valid() const { return fx > 0.0 && fy > 0.0; }

  /// Convert to algorithm intrinsics (for resection, undistortion, etc.). No database dependency.
  insight::camera::Intrinsics to_algorithm_intrinsics() const {
    insight::camera::Intrinsics K;
    K.fx = fx;
    K.fy = fy;
    K.cx = cx;
    K.cy = cy;
    K.k1 = k1;
    K.k2 = k2;
    K.k3 = k3;
    K.p1 = p1;
    K.p2 = p2;
    return K;
  }
};

/// Map: camera_id → CameraIntrinsics
using CameraIntrinsicsMap = std::unordered_map<uint32_t, CameraIntrinsics>;

// ─────────────────────────────────────────────────────────────────────────────
// Load helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Load camera intrinsics from a JSON file.
 *
 * Auto-detects format (single-camera or multi-camera).
 *
 * @param path  File path. Returns empty map if path is empty.
 * @return      Populated map on success; empty on failure.
 */
inline CameraIntrinsicsMap load_intrinsics_map(const std::string& path) {
  CameraIntrinsicsMap result;
  if (path.empty())
    return result;

  std::ifstream f(path);
  if (!f.is_open()) {
    LOG(ERROR) << "Cannot open intrinsics file: " << path;
    return result;
  }

  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to parse intrinsics JSON '" << path << "': " << e.what();
    return result;
  }

  if (j.contains("schema") && j["schema"].get<std::string>() == "multi_camera_v1") {
    // ── Multi-camera format ───────────────────────────────────────────────
    if (!j.contains("cameras") || !j["cameras"].is_object()) {
      LOG(ERROR) << "Multi-camera JSON missing 'cameras' object: " << path;
      return result;
    }
    for (auto& [key, val] : j["cameras"].items()) {
      uint32_t id = 1;
      try {
        id = static_cast<uint32_t>(std::stoul(key));
      } catch (...) {
        LOG(WARNING) << "Non-integer camera key '" << key << "' in " << path << ", skipping";
        continue;
      }

      CameraIntrinsics K;
      K.fx = val.value("fx", 0.0);
      K.fy = val.value("fy", 0.0);
      K.cx = val.value("cx", 0.0);
      K.cy = val.value("cy", 0.0);
      K.width = val.value("width", static_cast<uint32_t>(0));
      K.height = val.value("height", static_cast<uint32_t>(0));
      K.k1 = val.value("k1", 0.0);
      K.k2 = val.value("k2", 0.0);
      K.k3 = val.value("k3", 0.0);
      K.p1 = val.value("p1", 0.0);
      K.p2 = val.value("p2", 0.0);

      if (!K.valid()) {
        LOG(WARNING)
            << "Camera id=" << id
            << " in multi-camera JSON has invalid intrinsics (fx/fy must be > 0), skipping";
        continue;
      }
      result[id] = K;
    }
    LOG(INFO) << "Loaded " << result.size() << " camera(s) from multi-camera JSON: " << path;
  } else {
    // ── Single-camera (legacy) format – stored under key 1 ───────────────
    CameraIntrinsics K;
    K.fx = j.value("fx", 0.0);
    K.fy = j.value("fy", 0.0);
    K.cx = j.value("cx", 0.0);
    K.cy = j.value("cy", 0.0);
    K.width = j.value("width", static_cast<uint32_t>(0));
    K.height = j.value("height", static_cast<uint32_t>(0));
    K.k1 = j.value("k1", 0.0);
    K.k2 = j.value("k2", 0.0);
    K.k3 = j.value("k3", 0.0);
    K.p1 = j.value("p1", 0.0);
    K.p2 = j.value("p2", 0.0);

    if (!K.valid()) {
      LOG(ERROR) << "Intrinsics JSON must contain fx, fy > 0: " << path;
      return result;
    }
    result[1] = K;
    LOG(INFO) << "Loaded single-camera intrinsics from " << path << ": fx=" << K.fx
              << " fy=" << K.fy << " cx=" << K.cx << " cy=" << K.cy;
  }

  return result;
}

/**
 * Look up intrinsics for a given camera_id.
 *
 * Fallback order (for backward compatibility with single-camera workflows):
 *   1. Exact key match               (camera_id)
 *   2. Key 1                         (legacy single-camera)
 *   3. Any remaining entry           (last resort)
 *
 * @return Pointer into the map (stable until map is modified), or nullptr.
 */
inline const CameraIntrinsics* lookup_camera(const CameraIntrinsicsMap& cam_map,
                                             uint32_t camera_id) {
  auto it = cam_map.find(camera_id);
  if (it != cam_map.end())
    return &it->second;
  it = cam_map.find(1); // legacy fallback
  if (it != cam_map.end())
    return &it->second;
  if (!cam_map.empty())
    return &cam_map.begin()->second;
  return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// Image-list camera map  (image_id uint32_t → camera_id == group_id)
// Populated from the JSON produced by  isat_project extract (InsightAT Image List
// Format v2.0). Exported files (e.g. images_all.json) for CLI tools contain id and
// camera_id per image so intrinsics can be resolved per camera.
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Build a map of image_id → camera_id (both uint32_t) from an image-list JSON.
 *
 * Expected format (InsightAT Image List Format v2.0, e.g. isat_project extract):
 *   { "$schema": "InsightAT Image List Format v2.0", "images": [ { "id": 1, "camera_id": 2, "path": "..." }, ... ] }
 *
 * @param path  Path to image list JSON. Returns empty map if path is empty.
 *              Calls LOG(FATAL) on parse failure.
 */
inline std::unordered_map<uint32_t, uint32_t> build_image_camera_map(const std::string& path) {
  std::unordered_map<uint32_t, uint32_t> map;
  if (path.empty())
    return map;

  std::ifstream f(path);
  if (!f.is_open()) {
    LOG(FATAL) << "Cannot open image list: " << path;
  }
  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    LOG(FATAL) << "Failed to parse image list JSON '" << path << "': " << e.what();
  }

  if (!j.contains("images") || !j["images"].is_array()) {
    LOG(FATAL) << "Image list JSON missing 'images' array: " << path;
  }
  for (const auto& img : j["images"]) {
    if (!img.contains("id"))
      continue;
    uint32_t img_id = img["id"].get<uint32_t>();
    uint32_t cam_id = img.value("camera_id", uint32_t(1));
    map[img_id] = cam_id;
  }
  LOG(INFO) << "Loaded camera map for " << map.size() << " images from " << path;
  return map;
}

// ─────────────────────────────────────────────────────────────────────────────
// Index-based intrinsics (image list with "cameras" array and images[].camera_index)
// Used when export is index-only; no camera_id / IdMapping needed.
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Build image_index → CameraIntrinsics from an image-list JSON that contains a "cameras" array
 * and each image has "camera_index" (and optionally "image_index").
 *
 * Expected format (isat_project extract with index-only export):
 *   { "images": [ { "image_index": 0, "camera_index": 0, "path": "..." }, ... ],
 *     "cameras": [ { "fx", "fy", "cx", "cy", "width", "height" }, ... ] }
 *
 * @param path  Path to image list JSON. Returns empty vector if path empty or format not matched.
 */
inline std::vector<CameraIntrinsics> build_image_index_intrinsics_from_list(const std::string& path) {
  std::vector<CameraIntrinsics> by_index;
  if (path.empty())
    return by_index;

  std::ifstream f(path);
  if (!f.is_open()) {
    LOG(ERROR) << "Cannot open image list: " << path;
    return by_index;
  }
  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to parse image list JSON '" << path << "': " << e.what();
    return by_index;
  }

  if (!j.contains("images") || !j["images"].is_array())
    return by_index;
  if (!j.contains("cameras") || !j["cameras"].is_array())
    return by_index;

  const auto& cameras_arr = j["cameras"];
  for (const auto& cam : cameras_arr) {
    CameraIntrinsics K;
    K.fx = cam.value("fx", 0.0);
    K.fy = cam.value("fy", 0.0);
    K.cx = cam.value("cx", 0.0);
    K.cy = cam.value("cy", 0.0);
    K.width = cam.value("width", static_cast<uint32_t>(0));
    K.height = cam.value("height", static_cast<uint32_t>(0));
    K.k1 = cam.value("k1", 0.0);
    K.k2 = cam.value("k2", 0.0);
    K.k3 = cam.value("k3", 0.0);
    K.p1 = cam.value("p1", 0.0);
    K.p2 = cam.value("p2", 0.0);
    by_index.push_back(K);
  }

  std::vector<CameraIntrinsics> result;
  for (size_t i = 0; i < j["images"].size(); ++i) {
    const auto& img = j["images"][i];
    int cidx = img.value("camera_index", 0);
    if (cidx < 0 || static_cast<size_t>(cidx) >= by_index.size())
      result.push_back(CameraIntrinsics{});
    else
      result.push_back(by_index[static_cast<size_t>(cidx)]);
  }
  if (!result.empty())
    LOG(INFO) << "Loaded intrinsics for " << result.size() << " images (index-based) from " << path;
  return result;
}

/**
 * Build IdMapping from image-list JSON (same format as build_image_camera_map).
 * Preserves image order from the "images" array; camera order = first appearance.
 * Used for SfM ID re-encoding: internal indices 0..n_images-1, 0..n_cameras-1.
 *
 * @param path  Path to image list JSON. Returns empty IdMapping if path is empty.
 */
inline insight::sfm::IdMapping build_id_mapping_from_image_list(const std::string& path) {
  insight::sfm::IdMapping out;
  if (path.empty())
    return out;

  std::ifstream f(path);
  if (!f.is_open()) {
    LOG(FATAL) << "Cannot open image list: " << path;
  }
  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    LOG(FATAL) << "Failed to parse image list JSON '" << path << "': " << e.what();
  }

  if (!j.contains("images") || !j["images"].is_array()) {
    LOG(FATAL) << "Image list JSON missing 'images' array: " << path;
  }

  std::unordered_set<uint32_t> seen_camera;
  for (const auto& img : j["images"]) {
    if (!img.contains("id"))
      continue;
    uint32_t img_id = img["id"].get<uint32_t>();
    uint32_t cam_id = img.value("camera_id", uint32_t(1));
    int img_idx = static_cast<int>(out.original_image_ids.size());
    out.original_image_ids.push_back(img_id);
    out.original_to_internal_image[img_id] = img_idx;
    int cam_idx = 0;
    if (seen_camera.insert(cam_id).second) {
      cam_idx = static_cast<int>(out.original_camera_ids.size());
      out.original_camera_ids.push_back(cam_id);
      out.original_to_internal_camera[cam_id] = cam_idx;
    } else {
      cam_idx = out.original_to_internal_camera[cam_id];
    }
    out.image_to_camera.push_back(cam_idx);
  }
  LOG(INFO) << "IdMapping: " << out.num_images() << " images, " << out.num_cameras()
            << " cameras from " << path;
  return out;
}
