/**
 * @file  track_store_idc.cpp
 * @brief Load/save TrackStore (+ optional ViewGraph) from .isat_tracks IDC.
 */

#include "track_store_idc.h"
#include "idc_reader.h"
#include "idc_writer.h"
#include "../modules/sfm/view_graph.h"
#include "../modules/sfm/view_graph_loader.h"
#include <glog/logging.h>
#include <nlohmann/json.hpp>

namespace insight {
namespace sfm {

bool load_track_store_from_idc(const std::string& path, TrackStore* store_out,
                               std::vector<uint32_t>* image_indices_out, ViewGraph* view_graph_out) {
  if (!store_out)
    return false;
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

  if (view_graph_out && meta.contains("view_graph_pairs") && meta["view_graph_pairs"].is_array()) {
    if (!view_graph_from_json_array(meta["view_graph_pairs"], view_graph_out)) {
      LOG(WARNING) << "load_track_store_from_idc: failed to parse view_graph_pairs; leaving view graph empty";
      view_graph_out->clear();
    }
  } else if (view_graph_out) {
    view_graph_out->clear();
  }
  return true;
}

bool save_track_store_to_idc(const TrackStore& store, const std::vector<uint32_t>& image_indices,
                             const std::string& path, const ViewGraph* view_graph,
                             const TrackSaveOptions* opts) {
  const size_t n_tracks = store.num_tracks();
  const bool embed_vg  = view_graph != nullptr && view_graph->num_pairs() > 0;
  const bool is_sfm    = opts != nullptr && opts->is_sfm_result;

  // ── Determine schema_version ─────────────────────────────────────────────
  std::string schema_ver = "1.0";
  if (is_sfm)           schema_ver = "1.2";
  else if (embed_vg)    schema_ver = "1.1";

  nlohmann::json meta;
  meta["schema_version"] = schema_ver;
  meta["task_type"]      = "tracks";
  meta["num_images"]     = static_cast<int>(image_indices.size());
  meta["image_indices"]  = image_indices;
  meta["num_tracks"]     = static_cast<int>(n_tracks);

  if (embed_vg)
    meta["view_graph_pairs"] = view_graph_pairs_to_json_array(*view_graph);

  // ── Serialize tracks (always all tracks, never filter) ───────────────────
  std::vector<float>   track_xyz(static_cast<size_t>(n_tracks) * 3);
  std::vector<uint8_t> track_flag_bytes(static_cast<size_t>(n_tracks));

  int auto_num_triangulated = 0;
  int auto_num_inlier       = 0;

  for (size_t t = 0; t < n_tracks; ++t) {
    float x, y, z;
    store.get_track_xyz(static_cast<int>(t), &x, &y, &z);
    track_xyz[t * 3]     = x;
    track_xyz[t * 3 + 1] = y;
    track_xyz[t * 3 + 2] = z;

    uint8_t flags = 0;
    const bool alive = store.is_track_valid(static_cast<int>(t));
    const bool tri   = store.track_has_triangulated_xyz(static_cast<int>(t));
    if (alive) flags |= track_flags::kAlive;
    if (tri)   flags |= track_flags::kHasTriangulated;
    if (is_sfm && store.is_track_skip_ba(static_cast<int>(t)))
      flags |= track_flags::kSkipFromBA;
    track_flag_bytes[t] = flags;

    if (tri) {
      ++auto_num_triangulated;
      if (alive) ++auto_num_inlier;
    }
  }

  // ── Embed SfM-result metadata ─────────────────────────────────────────────
  if (is_sfm) {
    const int num_tri   = (opts->num_triangulated >= 0) ? opts->num_triangulated
                                                        : auto_num_triangulated;
    const int num_inlier = (opts->num_inlier >= 0) ? opts->num_inlier
                                                   : auto_num_inlier;
    const int num_outlier        = num_tri - num_inlier;
    const int num_not_triangulated = static_cast<int>(n_tracks) - num_tri;
    meta["is_sfm_result"]           = true;
    meta["num_registered_images"]   = opts->num_registered_images;
    meta["num_triangulated"]        = num_tri;
    meta["num_inlier"]              = num_inlier;
    meta["num_outlier"]             = num_outlier;
    meta["num_not_triangulated"]    = num_not_triangulated;
  }

  std::vector<uint32_t> track_obs_offset(static_cast<size_t>(n_tracks) + 1);
  std::vector<uint32_t> obs_image_slot, obs_feature_id;
  std::vector<float> obs_u, obs_v, obs_scale;
  std::vector<uint8_t> obs_flag_bytes;
  obs_image_slot.reserve(store.num_observations());
  obs_feature_id.reserve(store.num_observations());
  obs_u.reserve(store.num_observations());
  obs_v.reserve(store.num_observations());
  obs_scale.reserve(store.num_observations());
  obs_flag_bytes.reserve(store.num_observations());

  std::vector<Observation> obs_buf;
  size_t offset = 0;
  for (size_t t = 0; t < n_tracks; ++t) {
    track_obs_offset[t] = static_cast<uint32_t>(offset);
    obs_buf.clear();
    store.get_track_observations(static_cast<int>(t), &obs_buf);
    for (const auto& o : obs_buf) {
      obs_image_slot.push_back(o.image_index);
      obs_feature_id.push_back(o.feature_id);
      obs_u.push_back(o.u);
      obs_v.push_back(o.v);
      obs_scale.push_back(o.scale);
      obs_flag_bytes.push_back(obs_flags::kAlive);
    }
    offset += obs_buf.size();
  }
  track_obs_offset[n_tracks] = static_cast<uint32_t>(offset);
  const size_t n_obs = obs_image_slot.size();
  meta["num_observations"] = static_cast<int>(n_obs);

  io::IDCWriter writer(path);
  writer.set_metadata(meta);
  writer.add_blob("track_xyz", track_xyz.data(), track_xyz.size() * sizeof(float), "float32",
                  {static_cast<int>(n_tracks), 3});
  writer.add_blob("track_flags", track_flag_bytes.data(), track_flag_bytes.size() * sizeof(uint8_t),
                  "uint8", {static_cast<int>(n_tracks)});
  writer.add_blob("track_obs_offset", track_obs_offset.data(),
                  track_obs_offset.size() * sizeof(uint32_t), "uint32",
                  {static_cast<int>(n_tracks) + 1});
  writer.add_blob("obs_image_index", obs_image_slot.data(), obs_image_slot.size() * sizeof(uint32_t),
                  "uint32", {static_cast<int>(obs_image_slot.size())});
  writer.add_blob("obs_feature_id", obs_feature_id.data(), obs_feature_id.size() * sizeof(uint32_t),
                  "uint32", {static_cast<int>(obs_feature_id.size())});
  writer.add_blob("obs_u", obs_u.data(), obs_u.size() * sizeof(float), "float32",
                  {static_cast<int>(obs_u.size())});
  writer.add_blob("obs_v", obs_v.data(), obs_v.size() * sizeof(float), "float32",
                  {static_cast<int>(obs_v.size())});
  writer.add_blob("obs_scale", obs_scale.data(), obs_scale.size() * sizeof(float), "float32",
                  {static_cast<int>(obs_scale.size())});
  writer.add_blob("obs_flags", obs_flag_bytes.data(), obs_flag_bytes.size() * sizeof(uint8_t),
                  "uint8", {static_cast<int>(obs_flag_bytes.size())});
  if (!writer.write()) {
    LOG(ERROR) << "save_track_store_to_idc: failed to write " << path;
    return false;
  }
  VLOG(1) << "save_track_store_to_idc: wrote " << path << " (" << n_tracks << " tracks, " << n_obs
          << " observations"
          << (embed_vg ? ", view_graph embedded" : "")
          << (is_sfm   ? ", sfm_result meta"     : "") << ")";
  return true;
}

} // namespace sfm
} // namespace insight
