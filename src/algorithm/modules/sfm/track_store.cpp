/**
 * @file  track_store.cpp
 * @brief Implementation of TrackStore for incremental SfM.
 */

#include "track_store.h"
#include <algorithm>
#include <cassert>

namespace insight {
namespace sfm {

void TrackStore::set_num_images(int n) {
    assert(n >= 0);
    num_images_ = n;
    image_obs_ids_.resize(static_cast<size_t>(n));
}

void TrackStore::reserve_tracks(size_t cap) {
    track_xyz_.reserve(cap * 3);
    track_flags_.reserve(cap);
    track_obs_ids_.reserve(cap);
}

void TrackStore::reserve_observations(size_t cap) {
    obs_track_id_.reserve(cap);
    obs_image_id_.reserve(cap);
    obs_feature_id_.reserve(cap);
    obs_u_.reserve(cap);
    obs_v_.reserve(cap);
    obs_scale_.reserve(cap);
    obs_flags_.reserve(cap);
}

int TrackStore::add_track(float x, float y, float z) {
    const int track_id = static_cast<int>(num_tracks());
    track_xyz_.push_back(x);
    track_xyz_.push_back(y);
    track_xyz_.push_back(z);
    track_flags_.push_back(track_flags::kAlive);
    track_obs_ids_.emplace_back();
    return track_id;
}

int TrackStore::add_observation(int track_id, uint32_t image_index, uint32_t feature_id,
                                float u, float v, float scale) {
    assert(track_id >= 0 && static_cast<size_t>(track_id) < num_tracks());
    assert(static_cast<int>(image_index) < num_images_);

    const int obs_id = static_cast<int>(num_observations());
    obs_track_id_.push_back(track_id);
    obs_image_id_.push_back(image_index);
    obs_feature_id_.push_back(feature_id);
    obs_u_.push_back(u);
    obs_v_.push_back(v);
    obs_scale_.push_back(scale);
    obs_flags_.push_back(obs_flags::kAlive);

    track_obs_ids_[static_cast<size_t>(track_id)].push_back(obs_id);
    if (static_cast<size_t>(image_index) < image_obs_ids_.size())
        image_obs_ids_[image_index].push_back(obs_id);

    return obs_id;
}

bool TrackStore::is_track_valid(int track_id) const {
    if (track_id < 0 || static_cast<size_t>(track_id) >= track_flags_.size())
        return false;
    return (track_flags_[static_cast<size_t>(track_id)] & track_flags::kAlive) != 0;
}

void TrackStore::get_track_xyz(int track_id, float* x, float* y, float* z) const {
    assert(track_id >= 0 && static_cast<size_t>(track_id) < num_tracks());
    const size_t i = static_cast<size_t>(track_id) * 3u;
    *x = track_xyz_[i];
    *y = track_xyz_[i + 1];
    *z = track_xyz_[i + 2];
}

void TrackStore::set_track_xyz(int track_id, float x, float y, float z) {
    assert(track_id >= 0 && static_cast<size_t>(track_id) < num_tracks());
    const size_t i = static_cast<size_t>(track_id) * 3u;
    track_xyz_[i]     = x;
    track_xyz_[i + 1] = y;
    track_xyz_[i + 2] = z;
}

void TrackStore::set_track_retriangulation_flag(int track_id, bool value) {
    if (track_id < 0 || static_cast<size_t>(track_id) >= track_flags_.size()) return;
    auto& f = track_flags_[static_cast<size_t>(track_id)];
    if (value)
        f |= track_flags::kNeedsRetriangulation;
    else
        f &= ~track_flags::kNeedsRetriangulation;
}

bool TrackStore::track_needs_retriangulation(int track_id) const {
    if (track_id < 0 || static_cast<size_t>(track_id) >= track_flags_.size())
        return false;
    return (track_flags_[static_cast<size_t>(track_id)] & track_flags::kNeedsRetriangulation) != 0;
}

int TrackStore::get_track_observations(int track_id, std::vector<Observation>* obs_out) const {
    assert(obs_out);
    obs_out->clear();
    if (track_id < 0 || static_cast<size_t>(track_id) >= track_obs_ids_.size())
        return 0;
    if (!is_track_valid(track_id))
        return 0;
    const std::vector<int>& ids = track_obs_ids_[static_cast<size_t>(track_id)];
    for (int obs_id : ids) {
        if (!is_obs_valid(obs_id)) continue;
        Observation o;
        get_obs(obs_id, &o);
        obs_out->push_back(o);
    }
    return static_cast<int>(obs_out->size());
}

int TrackStore::get_image_observation_indices(int image_index, std::vector<int>* obs_indices_out) const {
    assert(obs_indices_out);
    obs_indices_out->clear();
    if (image_index < 0 || static_cast<size_t>(image_index) >= image_obs_ids_.size())
        return 0;
    const std::vector<int>& ids = image_obs_ids_[static_cast<size_t>(image_index)];
    for (int obs_id : ids) {
        if (is_obs_valid(obs_id))
            obs_indices_out->push_back(obs_id);
    }
    return static_cast<int>(obs_indices_out->size());
}

int TrackStore::get_image_track_observations(int image_index,
                                             std::vector<int>* track_ids_out,
                                             std::vector<Observation>* obs_out) const {
    if (track_ids_out) track_ids_out->clear();
    if (obs_out) obs_out->clear();
    std::vector<int> obs_ids;
    const int n = get_image_observation_indices(image_index, &obs_ids);
    if (n == 0) return 0;
    if (track_ids_out) track_ids_out->reserve(static_cast<size_t>(n));
    if (obs_out) obs_out->reserve(static_cast<size_t>(n));
    for (int obs_id : obs_ids) {
        if (track_ids_out) track_ids_out->push_back(obs_track_id_[static_cast<size_t>(obs_id)]);
        if (obs_out) {
            Observation o;
            get_obs(obs_id, &o);
            obs_out->push_back(o);
        }
    }
    return n;
}

bool TrackStore::is_obs_valid(int obs_id) const {
    if (obs_id < 0 || static_cast<size_t>(obs_id) >= obs_flags_.size())
        return false;
    return (obs_flags_[static_cast<size_t>(obs_id)] & obs_flags::kAlive) != 0;
}

int TrackStore::obs_track_id(int obs_id) const {
    assert(obs_id >= 0 && static_cast<size_t>(obs_id) < obs_track_id_.size());
    return obs_track_id_[static_cast<size_t>(obs_id)];
}

void TrackStore::get_obs(int obs_id, Observation* out) const {
    assert(out && obs_id >= 0 && static_cast<size_t>(obs_id) < obs_track_id_.size());
    const size_t i = static_cast<size_t>(obs_id);
    out->image_index = obs_image_id_[i];
    out->feature_id  = obs_feature_id_[i];
    out->u           = obs_u_[i];
    out->v           = obs_v_[i];
    out->scale       = obs_scale_[i];
}

void TrackStore::mark_track_deleted(int track_id) {
    if (track_id < 0 || static_cast<size_t>(track_id) >= track_flags_.size()) return;
    track_flags_[static_cast<size_t>(track_id)] &= ~track_flags::kAlive;
}

void TrackStore::mark_observation_deleted(int obs_id) {
    if (obs_id < 0 || static_cast<size_t>(obs_id) >= obs_flags_.size()) return;
    obs_flags_[static_cast<size_t>(obs_id)] &= ~obs_flags::kAlive;
}

}  // namespace sfm
}  // namespace insight
