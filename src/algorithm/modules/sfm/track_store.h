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

  // ── Incremental state cache helpers (for query engines / schedulers) ─────
  /// Monotonic counters for observation/XYZ/topology changes.
  uint64_t obs_epoch() const { return obs_epoch_; }
  uint64_t xyz_epoch() const { return xyz_epoch_; }
  uint64_t registration_epoch() const { return registration_epoch_; }
  /// Bumps only when kHasTriangulated flag transitions on any track (not on BA xyz-value updates).
  /// Use this instead of xyz_epoch() for resection/pyramid score cache invalidation.
  uint64_t tri_status_epoch() const { return tri_status_epoch_; }

  /// O(1) count of alive observations pointing to a triangulated track for a given image.
  /// Maintained incrementally — use instead of iterating image observations to count.
  int image_tri_count(int image_index) const;

  /// Recompute image_n_tri_[] from scratch (O(N_obs)). Call after loading a TrackStore
  /// that already has some triangulated XYZ (e.g., resume from checkpoint).
  void rebuild_image_n_tri();

  // ── Pose epoch (for epoch-gate on full-scan retriangulation) ─────────────
  /// Monotonically increasing; bumped by the pipeline after BA updates poses significantly.
  /// When the epoch does not change between two full-scan calls, already-stable tracks
  /// (whose last_tri_epoch == global_pose_epoch) are skipped in O(1).
  uint64_t global_pose_epoch() const { return global_pose_epoch_; }
  /// Increment the pose epoch (call after global/local BA if poses changed significantly).
  void bump_global_pose_epoch() { ++global_pose_epoch_; }
  /// Returns true if the track may need re-triangulation because poses have changed
  /// since it was last successfully triangulated (or it has never been triangulated).
  bool is_track_tri_stale(int track_id) const;

  /// Consume and clear dirty image/track ids accumulated since last consume.
  int consume_dirty_images(std::vector<int>* out);
  int consume_dirty_tracks(std::vector<int>* out);

  /// Clear all pending dirty ids without consuming.
  void clear_dirty_sets();

  /// Zero-copy read-only views over structural observation lists.
  /// These return all observation ids for the owner (alive + deleted). Callers that only want
  /// alive observations must filter with is_obs_valid(). They are intended for query engines
  /// that want to avoid repeatedly materializing temporary vectors.
  const std::vector<int>& track_all_obs_ids_view(int track_id) const;
  const std::vector<int>& image_all_obs_ids_view(int image_index) const;

  /// Zero-copy scalar accessors for observation SoA fields.
  uint32_t obs_image_index(int obs_id) const;
  uint32_t obs_feature_id(int obs_id) const;
  float obs_u(int obs_id) const;
  float obs_v(int obs_id) const;
  float obs_scale(int obs_id) const;

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
  /// O(1): number of alive (valid) observations.
  int num_valid_observations() const { return n_valid_obs_; }
  /// O(1): number of alive tracks that have been triangulated.
  int num_triangulated_tracks() const { return num_triangulated_; }

private:
  void mark_dirty_image(int image_index);
  void mark_dirty_track(int track_id);
  void mark_track_observation_images_dirty(int track_id);

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

  int num_triangulated_ = 0; ///< Maintained by set_track_xyz (+1 on first XYZ) and mark_track_deleted (-1 if triangulated)
  int n_valid_obs_ = 0;      ///< Maintained incrementally: +1 in add_observation, -1 in mark_observation_deleted/mark_observation_deleted_restorable..
  std::vector<int> retri_pending_ids_; ///< Accumulates track ids whenever kNeedsRetriangulation is set; drained by drain_retriangulation_pending().
  std::vector<uint8_t> retri_pending_mark_; ///< 0/1 marker to avoid duplicate enqueue in retri_pending_ids_.

  uint64_t obs_epoch_ = 0;
  uint64_t xyz_epoch_ = 0;
  uint64_t registration_epoch_ = 0;
  uint64_t tri_status_epoch_ = 0; ///< Bumps only on kHasTriangulated flag transitions (not on xyz value changes).

  std::vector<int> dirty_images_;
  std::vector<int> dirty_tracks_;
  std::vector<uint8_t> dirty_image_mark_; ///< 0/1 marker to avoid duplicate dirty_images_ entries.
  std::vector<uint8_t> dirty_track_mark_; ///< 0/1 marker to avoid duplicate dirty_tracks_ entries.

  std::vector<int> image_n_tri_; ///< Per-image count of alive obs whose track has kHasTriangulated. Maintained incrementally.

  uint64_t global_pose_epoch_ = 1; ///< Starts at 1; track_last_tri_epoch_ starts at 0 so all tracks are stale initially.
  std::vector<uint64_t> track_last_tri_epoch_; ///< Per-track epoch at which XYZ was last written. 0 = never triangulated.
};

} // namespace sfm
} // namespace insight

#endif /* TRACK_STORE_H */
