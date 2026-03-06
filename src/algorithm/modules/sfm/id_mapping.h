/**
 * @file  id_mapping.h
 * @brief Boundary-only mapping: original image/camera IDs ↔ dense indices (0..n-1).
 *
 * The SfM pipeline uses only index (0..num_images-1). This struct is used at the
 * boundary when: (1) loading pair JSON and geo/match file paths (original_id → index,
 * and index → original_id for paths); (2) writing results back (index → original_id).
 * Export produces an image list (array order = index) and this mapping for write-back.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace insight {
namespace sfm {

/**
 * Boundary mapping: index i (0..n-1) ↔ original image/camera id.
 * Built from the exported image list (array position = index). Not used inside
 * algorithm layer; algorithm sees only indices.
 */
struct IdMapping {
  /// index i → original image id (length = num_images; array order = index)
  std::vector<uint32_t> original_image_ids;
  /// index j → original camera id (length = num_cameras)
  std::vector<uint32_t> original_camera_ids;
  /// image index i → camera index (length = num_images)
  std::vector<int> image_to_camera;
  /// original image id → index (for loading pair JSON / file paths at boundary)
  std::unordered_map<uint32_t, int> original_to_internal_image;
  /// original camera id → index
  std::unordered_map<uint32_t, int> original_to_internal_camera;

  size_t num_images() const { return original_image_ids.size(); }
  size_t num_cameras() const { return original_camera_ids.size(); }

  uint32_t original_image_id(int internal_index) const {
    if (internal_index < 0 || static_cast<size_t>(internal_index) >= original_image_ids.size())
      return static_cast<uint32_t>(-1);
    return original_image_ids[static_cast<size_t>(internal_index)];
  }

  uint32_t original_camera_id(int internal_index) const {
    if (internal_index < 0 || static_cast<size_t>(internal_index) >= original_camera_ids.size())
      return static_cast<uint32_t>(-1);
    return original_camera_ids[static_cast<size_t>(internal_index)];
  }

  /// -1 if not found
  int internal_image_index(uint32_t original_id) const {
    auto it = original_to_internal_image.find(original_id);
    return it != original_to_internal_image.end() ? it->second : -1;
  }

  int internal_camera_index(uint32_t original_id) const {
    auto it = original_to_internal_camera.find(original_id);
    return it != original_to_internal_camera.end() ? it->second : -1;
  }

  bool empty() const { return original_image_ids.empty(); }
};

} // namespace sfm
} // namespace insight
