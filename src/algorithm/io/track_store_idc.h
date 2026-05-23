/**
 * @file  track_store_idc.h
 * @brief Load/save TrackStore from .isat_tracks IDC (shared by isat_tracks and incremental SfM).
 *
 * Schema 1.0: tracks + observations only.
 * Schema 1.1: adds optional "view_graph_pairs" JSON array.
 * Schema 1.2: adds SfM-result metadata + preserves kHasTriangulated bit.
 * Schema 1.3: adds optional pose + intrinsics + registered blobs (SfMResultData).
 *              Backward compatible: older loaders ignore unknown blobs.
 */

#pragma once

#include "../modules/sfm/track_store.h"
#include <cstdint>
#include <string>
#include <vector>

namespace insight {
namespace sfm {

class ViewGraph;

// ─────────────────────────────────────────────────────────────────────────────
// SfM pose / intrinsics data (optional, embedded in schema >= 1.3)
// ─────────────────────────────────────────────────────────────────────────────

struct SfMResultData {
  // Per-image pose data (size = num_images)
  std::vector<float> pose_R;        // 9 * num_images
  std::vector<float> pose_C;        // 3 * num_images
  std::vector<uint8_t> registered;  // num_images, 1=registered
  std::vector<int32_t> cam_idx;     // num_images, camera_index per image

  // Camera intrinsics (size = num_cameras)
  std::vector<float> intrinsics;   // 11 * num_cameras [fx,fy,cx,cy,w,h,k1,k2,k3,p1,p2]
  int num_cameras = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Save options (with optional sfm_pose extension)
// ─────────────────────────────────────────────────────────────────────────────

struct TrackSaveOptions {
    bool is_sfm_result          = false;
    int  num_registered_images  = 0;
    int  num_triangulated       = -1;
    int  num_inlier             = -1;

    // ── Optional embedded pose/intrinsics (schema 1.3) ──────────────────────
    const SfMResultData* sfm_pose = nullptr;
};

// ─────────────────────────────────────────────────────────────────────────────
// Load / Save API (extended with optional SfMResultData)
// ─────────────────────────────────────────────────────────────────────────────

bool load_track_store_from_idc(const std::string& path, TrackStore* store_out,
                               std::vector<uint32_t>* image_indices_out = nullptr,
                               ViewGraph* view_graph_out = nullptr,
                               SfMResultData* sfm_pose_out = nullptr);

bool save_track_store_to_idc(const TrackStore& store, const std::vector<uint32_t>& image_indices,
                             const std::string& path,
                             const ViewGraph* view_graph        = nullptr,
                             const TrackSaveOptions* opts       = nullptr);

} // namespace sfm
} // namespace insight
