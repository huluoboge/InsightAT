/**
 * @file  track_store_idc.cpp
 * @brief Load TrackStore from .isat_tracks IDC.
 */

#include "track_store_idc.h"
#include "idc_reader.h"
#include "../modules/sfm/track_store.h"
#include <glog/logging.h>
#include <nlohmann/json.hpp>

namespace insight {
namespace sfm {

bool load_track_store_from_idc(const std::string& path, TrackStore* store_out,
                               std::vector<uint32_t>* image_indices_out) {
  if (!store_out) return false;
  io::IDCReader reader(path);
  if (!reader.is_valid()) {
    LOG(ERROR) << "load_track_store_from_idc: invalid IDC " << path;
    return false;
  }
  const nlohmann::json& meta = reader.get_metadata();
  const int num_images = meta["num_images"].get<int>();
  const int num_tracks = meta["num_tracks"].get<int>();
  const int num_observations = meta["num_observations"].get<int>();

  std::vector<uint32_t> image_indices;
  if (image_indices_out || meta.contains("image_indices") || meta.contains("image_ids")) {
    const auto& ids_arr = meta.contains("image_indices") ? meta["image_indices"] : meta["image_ids"];
    for (const auto& v : ids_arr) {
      if (v.is_number_unsigned())
        image_indices.push_back(v.get<uint32_t>());
      else if (v.is_string())
        image_indices.push_back(static_cast<uint32_t>(std::stoul(v.get<std::string>())));
      else
        image_indices.push_back(static_cast<uint32_t>(v.get<int64_t>()));
    }
    if (image_indices_out)
      *image_indices_out = image_indices;
  }

  auto track_xyz = reader.read_blob<float>("track_xyz");
  auto track_flags = reader.read_blob<uint8_t>("track_flags");
  auto track_obs_offset = reader.read_blob<uint32_t>("track_obs_offset");
  auto obs_image_slot = reader.read_blob<uint32_t>("obs_image_index");
  if (obs_image_slot.empty())
    obs_image_slot = reader.read_blob<uint32_t>("obs_image_id");
  auto obs_feature_id = reader.read_blob<uint32_t>("obs_feature_id");
  auto obs_u = reader.read_blob<float>("obs_u");
  auto obs_v = reader.read_blob<float>("obs_v");
  auto obs_scale = reader.read_blob<float>("obs_scale");
  auto obs_flags = reader.read_blob<uint8_t>("obs_flags");

  if (track_xyz.size() != static_cast<size_t>(num_tracks) * 3u ||
      track_flags.size() != static_cast<size_t>(num_tracks) ||
      track_obs_offset.size() != static_cast<size_t>(num_tracks) + 1u ||
      obs_image_slot.size() != static_cast<size_t>(num_observations)) {
    LOG(ERROR) << "load_track_store_from_idc: IDC blob size mismatch";
    return false;
  }

  store_out->set_num_images(num_images);
  for (int t = 0; t < num_tracks; ++t) {
    float x = track_xyz[static_cast<size_t>(t) * 3];
    float y = track_xyz[static_cast<size_t>(t) * 3 + 1];
    float z = track_xyz[static_cast<size_t>(t) * 3 + 2];
    store_out->add_track(x, y, z);
    if ((track_flags[static_cast<size_t>(t)] & track_flags::kAlive) == 0)
      store_out->mark_track_deleted(t);
  }
  for (int t = 0; t < num_tracks; ++t) {
    const size_t beg = track_obs_offset[static_cast<size_t>(t)];
    const size_t end = track_obs_offset[static_cast<size_t>(t) + 1];
    for (size_t g = beg; g < end; ++g) {
      float s = (g < obs_scale.size()) ? obs_scale[g] : 1.f;
      store_out->add_observation(t, obs_image_slot[g], obs_feature_id[g], obs_u[g], obs_v[g], s);
    }
  }
  for (int g = 0; g < num_observations; ++g) {
    if (static_cast<size_t>(g) < obs_flags.size() &&
        (obs_flags[static_cast<size_t>(g)] & obs_flags::kAlive) == 0)
      store_out->mark_observation_deleted(g);
  }
  return true;
}

} // namespace sfm
} // namespace insight
