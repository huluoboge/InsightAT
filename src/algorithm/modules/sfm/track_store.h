/**
 * @file  track_store.h
 * @brief Efficient track storage for incremental SfM: SoA arrays, state flags
 *        for logical delete, reverse index image→observations.
 *
 * Design
 * ──────
 * - Image identity is index only: 0..num_images()-1. No "image id" in the algorithm;
 *   export/write-back uses a mapping (index → original id) at the boundary.
 * - Tracks: xyz[3*cap], flags[cap]; observations in flat SoA with obs_track_id.
 * - Delete: set flag bits only (no array shift). track bit0=alive, bit1=needs_retriangulation;
 *           obs bit0=alive.
 * - Reverse index: image_index → list of observation indices (image_index is 0..n-1).
 *
 * Usage
 * ─────
 *   TrackStore store;
 *   store.set_num_images(n_images);  // image order = index 0..n_images-1
 *   store.reserve_tracks(1000);
 *   store.reserve_observations(10000);
 *   int t = store.add_track(x, y, z);
 *   store.add_observation(t, image_index, feat_id, u, v, scale);  // image_index in [0, n_images)
 *   store.get_image_track_observations(image_index, ...);
 */

#pragma once

#ifndef TRACK_STORE_H
#define TRACK_STORE_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace insight {
namespace sfm {

// ─────────────────────────────────────────────────────────────────────────────
// Flags (bit masks)
// ─────────────────────────────────────────────────────────────────────────────

namespace track_flags {
constexpr uint8_t kAlive = 1u << 0;
constexpr uint8_t kNeedsRetriangulation = 1u << 1;
constexpr uint8_t kHasTriangulated =
    1u << 2; ///< Track has valid 3D (set by set_track_xyz / triangulation).
/// Set by select_ba_subset(): track is excluded from the BA point/obs set for this round.
/// Persists across BA calls; cleared and recomputed every ba_grid_reselect_every_n calls.
/// Does NOT affect triangulation, outlier rejection, or re-triangulation paths.
constexpr uint8_t kSkipFromBA = 1u << 3;
} // namespace track_flags

namespace obs_flags {
constexpr uint8_t kAlive = 1u << 0;
/// Set when an observation is deleted because its reprojection error exceeded the
/// MAD-based outlier threshold in reject_outliers_multiview.  Such observations may
/// be restored by restore_observations_from_cameras when camera intrinsics change
/// significantly (e.g. focal length shift during early-phase BA).
/// Observations deleted for geometric reasons (depth ≤ 0, angle, PnP outlier) do NOT
/// carry this flag and are never automatically restored.
constexpr uint8_t kRestorable = 1u << 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// Observation (single 2D view of a 3D point)
// ─────────────────────────────────────────────────────────────────────────────

struct Observation {
  uint32_t image_index = 0; ///< Image index 0..num_images()-1 (array position; no external id here)
  uint32_t feature_id = 0;
  float u = 0.f;
  float v = 0.f;
  float scale = 1.f;
};

// ─────────────────────────────────────────────────────────────────────────────
// TrackStore
// ─────────────────────────────────────────────────────────────────────────────

class TrackStore {
public:
  TrackStore() = default;

  /// Number of images. Indices 0..n-1; set before add_observation. Order = export image list order.
  void set_num_images(int n);
  int num_images() const { return num_images_; }

  /// Reserve capacity to avoid realloc during batch add.
  void reserve_tracks(size_t cap);
  void reserve_observations(size_t cap);

  /// Add a new track with 3D position; returns track index. Track is alive by default.
  int add_track(float x, float y, float z);

  /// Add observation to an existing track; updates reverse index. Returns global observation index.
  int add_observation(int track_id, uint32_t image_index, uint32_t feature_id, float u, float v,
                      float scale = 1.f);

  /// Query track
  bool is_track_valid(int track_id) const;
  bool track_has_triangulated_xyz(int track_id) const;
  void get_track_xyz(int track_id, float* x, float* y, float* z) const;
  void set_track_xyz(int track_id, float x, float y, float z);
  /// Clear the triangulated-XYZ flag (e.g. after all supporting observations
  /// are rejected).  Decrements num_triangulated_; the stale XYZ value is left
  /// in-place and will be overwritten on the next successful triangulation.
  /// Also enqueues this track for run_retriangulation (same as set_track_retriangulation_flag).
  void clear_track_xyz(int track_id);
  void set_track_retriangulation_flag(int track_id, bool value);
  bool track_needs_retriangulation(int track_id) const;

  // ── BA subset selection flag (kSkipFromBA) ───────────────────────────────
  bool is_track_skip_ba(int track_id) const;
  void set_track_skip_ba(int track_id, bool skip);
  /// Clear kSkipFromBA on ALL tracks (called before each select_ba_subset pass).
  void clear_all_skip_ba_flags();

  /// Consume and return all track ids that had their retriangulation flag set.
  /// The caller should re-check is_track_valid() and track_needs_retriangulation()
  /// on each element to handle duplicates and already-cleared entries.
  void drain_retriangulation_pending(std::vector<int>* out);

  /// Iterate valid observations of a track (writes to obs_out; returns count).
  int get_track_observations(int track_id, std::vector<Observation>* obs_out) const;

  /// Get the global observation-ids belonging to a track (only alive obs).
  /// Faster than get_track_observations when only obs_ids are needed (no struct copy).
  int get_track_obs_ids(int track_id, std::vector<int>* obs_ids_out) const;

  /// Get ALL global observation-ids for a track, including logically-deleted ones.
  /// Does not require is_track_valid: only checks track_id in range. Callers that must ignore
  /// mark_track_deleted tracks should filter with is_track_valid() first.
  /// Used by run_retriangulation / restore paths.
  int get_track_all_obs_ids(int track_id, std::vector<int>* obs_ids_out) const;

  /// Iterate valid observations for an image (obs indices and optionally Observation structs).
  int get_image_observation_indices(int image_index, std::vector<int>* obs_indices_out) const;
  int get_image_track_observations(int image_index, std::vector<int>* track_ids_out,
                                   std::vector<Observation>* obs_out) const;

  /// Like get_image_observation_indices but returns ALL observation ids for the image,
  /// including logically-deleted ones.  Used by restore_observations_from_cameras to
  /// find kRestorable deleted observations without a full store-wide scan.
  int get_image_all_obs_ids(int image_index, std::vector<int>* obs_ids_out) const;

  /// Observation by global index
  bool is_obs_valid(int obs_id) const;
  int obs_track_id(int obs_id) const;
  void get_obs(int obs_id, Observation* out) const;

  /// Logical delete (set flag only)
  void mark_track_deleted(int track_id);
  /// Logically delete one observation; also enqueues the parent track for run_retriangulation
  /// (restore deleted views when XYZ is still valid, or re-triangulate after clear_track_xyz).
  void mark_observation_deleted(int obs_id);
  /// Like mark_observation_deleted but also sets obs_flags::kRestorable so that
  /// restore_observations_from_cameras can later re-evaluate this observation when
  /// camera intrinsics change significantly.  Use only for reproj-error outlier deletions.
  /// Also enqueues the parent track for run_retriangulation (same as mark_observation_deleted).
  void mark_observation_deleted_restorable(int obs_id);
  void mark_observation_restored(int obs_id);

  /// True if the observation has been logically deleted AND carries obs_flags::kRestorable.
  bool is_obs_restorable(int obs_id) const;

  size_t num_tracks() const { return track_xyz_.size() / 3u; }
  size_t num_observations() const { return obs_track_id_.size(); }
  /// O(1): number of alive tracks that have been triangulated.
  int num_triangulated_tracks() const { return num_triangulated_; }

private:
  int num_images_ = 0;
  std::vector<float> track_xyz_;
  std::vector<uint8_t> track_flags_;
  std::vector<std::vector<int>>
      track_obs_ids_; // per-track list of global obs indices (for iteration)

  std::vector<int> obs_track_id_;
  std::vector<uint32_t> obs_image_id_;
  std::vector<uint32_t> obs_feature_id_;
  std::vector<float> obs_u_, obs_v_, obs_scale_;
  std::vector<uint8_t> obs_flags_;

  std::vector<std::vector<int>> image_obs_ids_; // image_index -> list of global obs indices

  int num_triangulated_ = 0; ///< Maintained by set_track_xyz (+1 on first XYZ) and mark_track_deleted (-1 if triangulated).
  std::vector<int> retri_pending_ids_; ///< Accumulates track ids whenever kNeedsRetriangulation is set; drained by drain_retriangulation_pending().
};

} // namespace sfm
} // namespace insight

#endif /* TRACK_STORE_H */
