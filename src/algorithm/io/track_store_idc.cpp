/**
 * @file  track_store_idc.cpp
 * @brief Load/save TrackStore (+ optional ViewGraph) from .isat_tracks IDC.
 */

#include "track_store_idc.h"
#include "idc_reader.h"
#include "idc_writer.h"
#include "../modules/sfm/view_graph.h"
#include "../modules/sfm/view_graph_loader.h"
#include "track_graph_idc.h"
#include <glog/logging.h>
#include <nlohmann/json.hpp>

namespace insight {
namespace sfm {

bool load_track_store_from_idc(const std::string& path, TrackStore* store_out,
                               std::vector<uint32_t>* image_indices_out,
                               ViewGraph* view_graph_out,
                               SfMResultData* sfm_pose_out,
                               TrackGraphStore* track_graph_out) {
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
  const bool has_graph_lineage = meta.value("graph_lineage", false) ||
                                 meta.value("schema_version", std::string()) == "1.4";

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
  std::vector<uint32_t> track_graph_id;
  std::vector<int32_t> track_parent_id;
  if (has_graph_lineage) {
    track_graph_id = reader.read_blob<uint32_t>("track_graph_id");
    track_parent_id = reader.read_blob<int32_t>("track_parent_id");
  }
  auto track_obs_offset = reader.read_blob<uint32_t>("track_obs_offset");
  auto obs_image_slot = reader.read_blob<uint32_t>("obs_image_index");
  if (obs_image_slot.empty())
    obs_image_slot = reader.read_blob<uint32_t>("obs_image_id");
  auto obs_feature_id = reader.read_blob<uint32_t>("obs_feature_id");
  auto obs_u = reader.read_blob<float>("obs_u");
  auto obs_v = reader.read_blob<float>("obs_v");
  auto obs_scale = reader.read_blob<float>("obs_scale");
  auto obs_flags = reader.read_blob<uint8_t>("obs_flags");
  std::vector<uint32_t> obs_source_id;
  std::vector<int32_t> serialized_obs_track_id;
  if (has_graph_lineage) {
    obs_source_id = reader.read_blob<uint32_t>("obs_source_id");
    serialized_obs_track_id = reader.read_blob<int32_t>("obs_track_id");
    if (track_graph_id.size() != static_cast<size_t>(num_tracks) ||
        track_parent_id.size() != static_cast<size_t>(num_tracks) ||
        obs_source_id.size() != static_cast<size_t>(num_observations) ||
        serialized_obs_track_id.size() != static_cast<size_t>(num_observations) ||
        obs_flags.size() != static_cast<size_t>(num_observations)) {
      LOG(ERROR) << "load_track_store_from_idc: graph lineage blob size mismatch";
      return false;
    }
    for (size_t i = 0; i < obs_source_id.size(); ++i) {
      if (obs_source_id[i] != i) {
        LOG(ERROR) << "load_track_store_from_idc: obs_source_id is not canonical";
        return false;
      }
    }
  }

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
    if (t < static_cast<int>(track_graph_id.size()))
      store_out->set_track_graph_id(t, track_graph_id[static_cast<size_t>(t)]);
    if (t < static_cast<int>(track_parent_id.size()))
      store_out->set_track_parent_id(t, track_parent_id[static_cast<size_t>(t)]);
    const uint8_t flags = track_flags[static_cast<size_t>(t)];
    if ((flags & track_flags::kAlive) == 0)
      store_out->mark_track_deleted(t);
    if (flags & track_flags::kSplitParent)
      store_out->set_track_split_parent(t, true);
    // Restore kHasTriangulated so that track_has_triangulated_xyz() works after reload.
    // add_track sets the flag only on camera-ready triangulations; on-disk the bit may
    // have been set by the SfM pipeline (schema >= 1.2).
    if (flags & track_flags::kHasTriangulated)
      store_out->set_track_xyz(t, x, y, z);  // also increments num_triangulated_
  }
  std::vector<int> loaded_obs_ids;
  loaded_obs_ids.reserve(static_cast<size_t>(num_observations));
  if (serialized_obs_track_id.size() == static_cast<size_t>(num_observations)) {
    for (int g = 0; g < num_observations; ++g) {
      const int owner = serialized_obs_track_id[static_cast<size_t>(g)];
      if (owner < 0 || owner >= num_tracks) return false;
      float s = (static_cast<size_t>(g) < obs_scale.size()) ? obs_scale[static_cast<size_t>(g)] : 1.f;
      loaded_obs_ids.push_back(store_out->add_observation(owner, obs_image_slot[static_cast<size_t>(g)],
                                                            obs_feature_id[static_cast<size_t>(g)],
                                                            obs_u[static_cast<size_t>(g)],
                                                            obs_v[static_cast<size_t>(g)], s));
    }
  } else {
    for (int t = 0; t < num_tracks; ++t) {
      const size_t beg = track_obs_offset[static_cast<size_t>(t)];
      const size_t end = track_obs_offset[static_cast<size_t>(t) + 1];
      for (size_t g = beg; g < end; ++g) {
        float s = (g < obs_scale.size()) ? obs_scale[g] : 1.f;
        loaded_obs_ids.push_back(store_out->add_observation(t, obs_image_slot[g], obs_feature_id[g],
                                                              obs_u[g], obs_v[g], s));
      }
    }
  }
  for (int g = 0; g < num_observations; ++g) {
    if (static_cast<size_t>(g) < obs_flags.size() &&
        (obs_flags[static_cast<size_t>(g)] & obs_flags::kAlive) == 0)
      if (static_cast<size_t>(g) < loaded_obs_ids.size())
        store_out->mark_observation_deleted(loaded_obs_ids[static_cast<size_t>(g)]);
  }

  // New graph files carry a structural track-local observation index. The normal
  // track_obs_offset blob remains the active observation stream for old readers.
  auto struct_offset = reader.read_blob<uint32_t>("track_struct_obs_offset");
  auto struct_index = reader.read_blob<uint32_t>("track_struct_obs_index");
  if (!struct_offset.empty() || !struct_index.empty()) {
    if (struct_offset.size() != static_cast<size_t>(num_tracks) + 1u) {
      LOG(ERROR) << "load_track_store_from_idc: structural observation offset mismatch";
      return false;
    }
    for (int t = 0; t < num_tracks; ++t) {
      const size_t beg = struct_offset[static_cast<size_t>(t)];
      const size_t end = struct_offset[static_cast<size_t>(t) + 1];
      if (end > struct_index.size()) return false;
      for (size_t i = beg; i < end; ++i) {
        const uint32_t canonical = struct_index[i];
        if (canonical >= static_cast<uint32_t>(loaded_obs_ids.size())) return false;
        // The canonical observation may belong to another active child. Historical
        // parents retain the fixed local-node list without changing active ownership.
        if (!store_out->is_track_valid(t) && store_out->is_track_split_parent(t))
          store_out->retain_observation_in_track_history(t, loaded_obs_ids[canonical]);
      }
    }
  }

  if (view_graph_out && meta.contains("view_graph_pairs") && meta["view_graph_pairs"].is_array()) {
    if (!view_graph_from_json_array(meta["view_graph_pairs"], view_graph_out)) {
      LOG(WARNING) << "load_track_store_from_idc: failed to parse view_graph_pairs; leaving view graph empty";
      view_graph_out->clear();
    }
  } else if (view_graph_out) {
    view_graph_out->clear();
  }
  // Rebuild the per-image triangulation counters (image_n_tri_) from scratch.
  store_out->rebuild_image_n_tri();

  // ── Optional: load embedded pose + intrinsics blobs (schema 1.3) ──────────
  if (sfm_pose_out && meta.value("has_pose_data", false)) {
    const int n_imgs = num_images;
    sfm_pose_out->pose_R = reader.read_blob<float>("pose_R");
    sfm_pose_out->pose_C = reader.read_blob<float>("pose_C");
    sfm_pose_out->registered = reader.read_blob<uint8_t>("registered");
    sfm_pose_out->cam_idx = reader.read_blob<int32_t>("cam_idx");
    sfm_pose_out->intrinsics = reader.read_blob<float>("intrinsics");
    sfm_pose_out->num_cameras = meta.value("num_cameras", 0);

    // Validate sizes
    if (sfm_pose_out->pose_R.size() != static_cast<size_t>(n_imgs) * 9) {
      LOG(WARNING) << "load_track_store_from_idc: pose_R size mismatch, clearing pose data";
      *sfm_pose_out = SfMResultData();
    } else if (sfm_pose_out->intrinsics.size() != static_cast<size_t>(sfm_pose_out->num_cameras) * 11) {
      LOG(WARNING) << "load_track_store_from_idc: intrinsics size mismatch, clearing pose data";
      *sfm_pose_out = SfMResultData();
    } else {
      LOG(INFO) << "Loaded " << n_imgs << " poses, " << sfm_pose_out->num_cameras
                << " cameras from embedded pose data";
    }
  }

  if (track_graph_out) {
    std::string graph_path = path;
    constexpr const char* suffix = ".isat_tracks";
    if (graph_path.size() >= std::char_traits<char>::length(suffix) &&
        graph_path.compare(graph_path.size() - std::char_traits<char>::length(suffix),
                           std::char_traits<char>::length(suffix), suffix) == 0)
      graph_path.replace(graph_path.size() - std::char_traits<char>::length(suffix),
                         std::char_traits<char>::length(suffix), ".isat_graph");
    else
      graph_path += ".isat_graph";
    if (!load_track_graph_from_idc(graph_path, track_graph_out)) {
      LOG(WARNING) << "load_track_store_from_idc: graph sidecar unavailable or invalid: " << graph_path;
      track_graph_out->clear();
    }
  }

  return true;
}

bool save_track_store_to_idc(const TrackStore& store, const std::vector<uint32_t>& image_indices,
                             const std::string& path, const ViewGraph* view_graph,
                             const TrackSaveOptions* opts) {
  const size_t n_tracks = store.num_tracks();
  const bool embed_vg  = view_graph != nullptr && view_graph->num_pairs() > 0;
  const bool is_sfm    = opts != nullptr && opts->is_sfm_result;

  // Determine schema_version
  const bool has_sfm_pose = (opts != nullptr && opts->sfm_pose != nullptr);
  const bool include_graph = opts != nullptr && opts->include_graph_lineage;
  std::string schema_ver = "1.0";
  if (include_graph)     schema_ver = "1.4";
  else if (has_sfm_pose) schema_ver = "1.3";
  else if (is_sfm)       schema_ver = "1.2";
  else if (embed_vg)     schema_ver = "1.1";

  nlohmann::json meta;
  meta["schema_version"] = schema_ver;
  meta["task_type"]      = "tracks";
  meta["num_images"]     = static_cast<int>(image_indices.size());
  meta["image_indices"]  = image_indices;
  meta["num_tracks"]     = static_cast<int>(n_tracks);

  std::vector<uint32_t> track_graph_id;
  std::vector<int32_t> track_parent_id;
  if (include_graph) {
    meta["graph_lineage"] = true;
    meta["observation_stream"] = "global_source_id_order";
    track_graph_id.resize(n_tracks);
    track_parent_id.resize(n_tracks);
    for (size_t t = 0; t < n_tracks; ++t) {
      track_graph_id[t] = store.track_graph_id(static_cast<int>(t));
      track_parent_id[t] = store.track_parent_id(static_cast<int>(t));
    }
  }

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
    if (include_graph && store.is_track_split_parent(static_cast<int>(t)))
      flags |= track_flags::kSplitParent;
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
  std::vector<uint32_t> obs_source_ids;
  std::vector<uint32_t> struct_obs_offset;
  std::vector<uint32_t> struct_obs_index;
  std::vector<int32_t> serialized_obs_track_id;
  obs_image_slot.reserve(store.num_observations());
  obs_feature_id.reserve(store.num_observations());
  obs_u.reserve(store.num_observations());
  obs_v.reserve(store.num_observations());
  obs_scale.reserve(store.num_observations());
  obs_flag_bytes.reserve(store.num_observations());

  size_t offset = 0;
  if (include_graph) {
    // Graph schema keeps every observation exactly once, including deleted ones,
    // in global source-id order. Current ownership is serialized separately.
    serialized_obs_track_id.resize(store.num_observations(), -1);
    for (size_t g = 0; g < store.num_observations(); ++g) {
      const int obs_id = static_cast<int>(g);
      const int owner = store.obs_track_id(obs_id);
      if (owner < 0 || static_cast<size_t>(owner) >= n_tracks) return false;
      Observation o;
      store.get_obs(obs_id, &o);
      obs_image_slot.push_back(o.image_index);
      obs_feature_id.push_back(o.feature_id);
      obs_u.push_back(o.u);
      obs_v.push_back(o.v);
      obs_scale.push_back(o.scale);
      uint8_t flags = 0;
      if (store.is_obs_valid(obs_id)) flags |= obs_flags::kAlive;
      if (store.is_obs_restorable(obs_id)) flags |= obs_flags::kRestorable;
      obs_flag_bytes.push_back(flags);
      obs_source_ids.push_back(static_cast<uint32_t>(obs_id));
      serialized_obs_track_id[g] = owner;
    }
    offset = obs_image_slot.size();
    std::vector<uint8_t> track_has_serialized_obs(n_tracks, 0u);
    for (size_t t = 0; t < n_tracks; ++t) {
      track_obs_offset[t] = 0;
      for (size_t g = 0; g < serialized_obs_track_id.size(); ++g) {
        if (serialized_obs_track_id[g] == static_cast<int32_t>(t) &&
            !track_has_serialized_obs[t]) {
          track_obs_offset[t] = static_cast<uint32_t>(g);
          track_has_serialized_obs[t] = 1u;
        }
      }
    }
    track_obs_offset[n_tracks] = static_cast<uint32_t>(offset);
  } else {
    for (size_t t = 0; t < n_tracks; ++t) {
      track_obs_offset[t] = static_cast<uint32_t>(offset);
      if (store.is_track_valid(static_cast<int>(t))) {
        for (int obs_id : store.track_all_obs_ids_view(static_cast<int>(t))) {
          if (!store.is_obs_valid(obs_id)) continue;
          Observation o;
          store.get_obs(obs_id, &o);
          obs_image_slot.push_back(o.image_index);
          obs_feature_id.push_back(o.feature_id);
          obs_u.push_back(o.u);
          obs_v.push_back(o.v);
          obs_scale.push_back(o.scale);
          obs_flag_bytes.push_back(obs_flags::kAlive);
          ++offset;
        }
      }
    }
    track_obs_offset[n_tracks] = static_cast<uint32_t>(offset);
  }
  track_obs_offset[n_tracks] = static_cast<uint32_t>(offset);
  const size_t n_obs = obs_image_slot.size();
  meta["num_observations"] = static_cast<int>(n_obs);

  // For graph lineage, store the original observation id alongside the active
  // serialized stream. This preserves stable parent-local node mapping without
  // relying on image/feature lookup.
  if (include_graph) {
    std::vector<uint32_t> original_to_serialized(store.num_observations(), 0xffffffffu);
    for (size_t g = 0; g < obs_source_ids.size(); ++g)
      original_to_serialized[obs_source_ids[g]] = static_cast<uint32_t>(g);
    struct_obs_offset.resize(n_tracks + 1u, 0u);
    for (size_t t = 0; t < n_tracks; ++t) {
      struct_obs_offset[t] = static_cast<uint32_t>(struct_obs_index.size());
      for (int obs_id : store.track_all_obs_ids_view(static_cast<int>(t))) {
        if (obs_id >= 0 && static_cast<size_t>(obs_id) < original_to_serialized.size() &&
            original_to_serialized[static_cast<size_t>(obs_id)] != 0xffffffffu)
          struct_obs_index.push_back(original_to_serialized[static_cast<size_t>(obs_id)]);
      }
    }
    struct_obs_offset[n_tracks] = static_cast<uint32_t>(struct_obs_index.size());
  }

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
  if (include_graph) {
    writer.add_blob("obs_source_id", obs_source_ids.data(), obs_source_ids.size() * sizeof(uint32_t),
                    "uint32", {static_cast<int>(obs_source_ids.size())});
    writer.add_blob("obs_track_id", serialized_obs_track_id.data(),
                    serialized_obs_track_id.size() * sizeof(int32_t), "int32",
                    {static_cast<int>(serialized_obs_track_id.size())});
    writer.add_blob("track_graph_id", track_graph_id.data(), track_graph_id.size() * sizeof(uint32_t),
                    "uint32", {static_cast<int>(track_graph_id.size())});
    writer.add_blob("track_parent_id", track_parent_id.data(), track_parent_id.size() * sizeof(int32_t),
                    "int32", {static_cast<int>(track_parent_id.size())});
    writer.add_blob("track_struct_obs_offset", struct_obs_offset.data(),
                    struct_obs_offset.size() * sizeof(uint32_t), "uint32",
                    {static_cast<int>(struct_obs_offset.size())});
    writer.add_blob("track_struct_obs_index", struct_obs_index.data(),
                    struct_obs_index.size() * sizeof(uint32_t), "uint32",
                    {static_cast<int>(struct_obs_index.size())});
  }

  // ── Optional: embed pose + intrinsics blobs (schema 1.3) ──────────────────
  if (has_sfm_pose) {
    const auto* sp = opts->sfm_pose;
    meta["has_pose_data"] = true;
    meta["num_cameras"]   = sp->num_cameras;

    writer.add_blob("pose_R", sp->pose_R.data(), sp->pose_R.size() * sizeof(float),
                    "float32", {static_cast<int>(sp->pose_R.size() / 9), 9});
    writer.add_blob("pose_C", sp->pose_C.data(), sp->pose_C.size() * sizeof(float),
                    "float32", {static_cast<int>(sp->pose_C.size() / 3), 3});
    writer.add_blob("registered", sp->registered.data(), sp->registered.size() * sizeof(uint8_t),
                    "uint8", {static_cast<int>(sp->registered.size())});
    writer.add_blob("cam_idx", sp->cam_idx.data(), sp->cam_idx.size() * sizeof(int32_t),
                    "int32", {static_cast<int>(sp->cam_idx.size())});
    writer.add_blob("intrinsics", sp->intrinsics.data(), sp->intrinsics.size() * sizeof(float),
                    "float32", {static_cast<int>(sp->num_cameras), 11});
    VLOG(1) << "save_track_store_to_idc: embedded " << sp->pose_R.size() / 9 << " poses, "
            << sp->num_cameras << " cameras";
  }

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
