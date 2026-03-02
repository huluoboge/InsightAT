/**
 * @file  track_store.h
 * @brief Efficient track storage for incremental SfM: SoA arrays, state flags
 *        for logical delete, reverse index image→observations.
 *
 * Design
 * ──────
 * - Tracks: xyz[3*cap], flags[cap]; observations in flat SoA with obs_track_id.
 * - Delete: set flag bits only (no array shift). track bit0=alive, bit1=needs_retriangulation;
 *           obs bit0=alive.
 * - Reverse index: per-image list of observation indices for resection / next-image selection.
 *
 * Usage
 * ─────
 *   TrackStore store;
 *   store.set_num_images(n_images);
 *   store.reserve_tracks(1000);
 *   store.reserve_observations(10000);
 *   int t = store.add_track(x, y, z);
 *   store.add_observation(t, image_index, feat_id, u, v, scale);
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
constexpr uint8_t kHasTriangulated = 1u << 2;  ///< Track has valid 3D (set by set_track_xyz / triangulation).
}

namespace obs_flags {
constexpr uint8_t kAlive = 1u << 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Observation (single 2D view of a 3D point)
// ─────────────────────────────────────────────────────────────────────────────

struct Observation {
    uint32_t image_index = 0;
    uint32_t feature_id  = 0;
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

    /// Number of images (for reverse index). Must be set before add_observation with that image_index.
    void set_num_images(int n);
    int num_images() const { return num_images_; }

    /// Reserve capacity to avoid realloc during batch add.
    void reserve_tracks(size_t cap);
    void reserve_observations(size_t cap);

    /// Add a new track with 3D position; returns track index. Track is alive by default.
    int add_track(float x, float y, float z);

    /// Add observation to an existing track; updates reverse index. Returns global observation index.
    int add_observation(int track_id, uint32_t image_index, uint32_t feature_id,
                       float u, float v, float scale = 1.f);

    /// Query track
    bool is_track_valid(int track_id) const;
    bool track_has_triangulated_xyz(int track_id) const;
    void get_track_xyz(int track_id, float* x, float* y, float* z) const;
    void set_track_xyz(int track_id, float x, float y, float z);
    void set_track_retriangulation_flag(int track_id, bool value);
    bool track_needs_retriangulation(int track_id) const;

    /// Iterate valid observations of a track (writes to obs_out; returns count).
    int get_track_observations(int track_id, std::vector<Observation>* obs_out) const;

    /// Iterate valid observations for an image (obs indices and optionally Observation structs).
    int get_image_observation_indices(int image_index, std::vector<int>* obs_indices_out) const;
    int get_image_track_observations(int image_index,
                                     std::vector<int>* track_ids_out,
                                     std::vector<Observation>* obs_out) const;

    /// Observation by global index
    bool is_obs_valid(int obs_id) const;
    int obs_track_id(int obs_id) const;
    void get_obs(int obs_id, Observation* out) const;

    /// Logical delete (set flag only)
    void mark_track_deleted(int track_id);
    void mark_observation_deleted(int obs_id);

    size_t num_tracks() const { return track_xyz_.size() / 3u; }
    size_t num_observations() const { return obs_track_id_.size(); }

private:
    int num_images_ = 0;
    std::vector<float>   track_xyz_;
    std::vector<uint8_t> track_flags_;
    std::vector<std::vector<int>> track_obs_ids_;   // per-track list of global obs indices (for iteration)

    std::vector<int>     obs_track_id_;
    std::vector<uint32_t> obs_image_id_;
    std::vector<uint32_t> obs_feature_id_;
    std::vector<float>   obs_u_, obs_v_, obs_scale_;
    std::vector<uint8_t> obs_flags_;

    std::vector<std::vector<int>> image_obs_ids_;  // image_index -> list of global obs indices
};

}  // namespace sfm
}  // namespace insight

#endif /* TRACK_STORE_H */
