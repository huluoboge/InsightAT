/**
 * @file  point_color_utils.h
 * @brief Load optional per-feature RGB colors from .isat_feat and average
 *        track colors for COLMAP / Bundler export.
 *
 * Colors are never sampled from images here (export-time decode is too slow).
 * Missing colors → caller should write placeholder gray (128,128,128).
 */

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "algorithm/modules/sfm/track_store.h"

namespace insight {
namespace export_util {

/**
 * Lazily load and cache optional `colors` blobs from `{features_dir}/{image_index}.isat_feat`.
 * Empty vector for an image means no colors blob (or invalid / missing file).
 */
class FeatureColorCache {
public:
  explicit FeatureColorCache(std::string features_dir);

  /// Returns nullptr if features_dir empty; otherwise pointer to per-image colors
  /// (may be empty if that image has no colors blob).
  const std::vector<uint8_t>* get(uint32_t image_index);

  bool enabled() const { return !features_dir_.empty(); }

private:
  std::string features_dir_;
  std::unordered_map<uint32_t, std::vector<uint8_t>> cache_;
};

/**
 * Load optional colors blob from an .isat_feat path.
 * Returns empty vector if file invalid or blob absent (no ERROR spam for missing blob).
 */
std::vector<uint8_t> load_feature_colors(const std::string& feat_path);

/**
 * Average RGB over track observations that have a valid feature color.
 * Returns nullopt if no observation contributed a color.
 */
std::optional<std::array<uint8_t, 3>>
average_track_rgb(const std::vector<sfm::Observation>& observations, FeatureColorCache& cache);

} // namespace export_util
} // namespace insight
