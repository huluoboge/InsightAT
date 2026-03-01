/**
 * isat_tracks.cpp
 * InsightAT Track Building CLI – load match + geo, build tracks, write IDC.
 *
 * Pipeline:
 *   Phase 1  Union-Find over match indices → track equivalence classes
 *   Phase 2  Fill observations (coords + scales) from match + geo inlier masks; set xyz from geo points3d when available
 *   Write    Single .isat_tracks IDC for downstream SfM
 *
 * Usage:
 *   isat_tracks -i pairs.json -m match_dir/ -g geo_dir/ -o tracks.isat_tracks
 *   isat_tracks -i pairs.json -m match_dir/ -g geo_dir/ -l image_list.json -o tracks.isat_tracks
 *   isat_tracks --stats -o tracks.isat_tracks
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "pair_json_utils.h"

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/sfm/track_store.h"
#include "../modules/sfm/track_builder.h"
#include "cmdLine/cmdLine.h"
#include "cli_logging.h"
#include "isat_intrinsics.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::io;
using namespace insight::sfm;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void printEvent(const json& j) {
    std::cout << kEventPrefix << j.dump() << "\n";
    std::cout.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Pairs and image list
// ─────────────────────────────────────────────────────────────────────────────

struct PairDesc {
    uint32_t image1_id = 0;
    uint32_t image2_id = 0;
    std::string match_file;
    std::string geo_file;
};

static std::vector<PairDesc> loadPairs(const std::string& json_path,
                                       const std::string& match_dir,
                                       const std::string& geo_dir) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        LOG(FATAL) << "Cannot open pairs file: " << json_path;
    }
    json j;
    file >> j;
    std::vector<PairDesc> pairs;
    for (const auto& p : j["pairs"]) {
        PairDesc d;
        d.image1_id = insight::tools::getImageIdFromPair(p, "image1_id");
        d.image2_id = insight::tools::getImageIdFromPair(p, "image2_id");
        d.match_file = match_dir + "/" + std::to_string(d.image1_id) + "_" + std::to_string(d.image2_id) + ".isat_match";
        d.geo_file = geo_dir + "/" + std::to_string(d.image1_id) + "_" + std::to_string(d.image2_id) + ".isat_geo";
        pairs.push_back(std::move(d));
    }
    LOG(INFO) << "Loaded " << pairs.size() << " pairs from " << json_path;
    return pairs;
}

/** Return ordered image_ids (image_index = array index). From image list JSON if path non-empty, else from pairs. */
static std::vector<uint32_t> getImageIds(const std::string& image_list_path,
                                         const std::vector<PairDesc>& pairs) {
    if (!image_list_path.empty()) {
        std::ifstream f(image_list_path);
        if (!f.is_open()) LOG(FATAL) << "Cannot open image list: " << image_list_path;
        json j;
        f >> j;
        if (!j.contains("images") || !j["images"].is_array())
            LOG(FATAL) << "Image list JSON missing 'images' array";
        std::set<uint32_t> ids;
        for (const auto& img : j["images"]) {
            if (img.contains("id")) ids.insert(img["id"].get<uint32_t>());
        }
        std::vector<uint32_t> out(ids.begin(), ids.end());
        LOG(INFO) << "Image ids from list: " << out.size();
        return out;
    }
    std::set<uint32_t> seen;
    for (const auto& p : pairs) {
        seen.insert(p.image1_id);
        seen.insert(p.image2_id);
    }
    std::vector<uint32_t> out(seen.begin(), seen.end());
    std::sort(out.begin(), out.end());
    LOG(INFO) << "Image ids from pairs: " << out.size();
    return out;
}

static std::unordered_map<uint32_t, int> buildImageIdToIndex(const std::vector<uint32_t>& image_ids) {
    std::unordered_map<uint32_t, int> m;
    for (size_t i = 0; i < image_ids.size(); ++i)
        m[image_ids[static_cast<size_t>(i)]] = static_cast<int>(i);
    return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// Union-Find for (image_id, feature_id) → track (key = uint64_t: high=image_id, low=feature_id)
// ─────────────────────────────────────────────────────────────────────────────

static uint64_t nodeKey(uint32_t image_id, uint32_t feature_id) {
    return (static_cast<uint64_t>(image_id) << 32) | feature_id;
}

struct UnionFind {
    std::unordered_map<uint64_t, int> node_id_;
    std::vector<int> parent_;

    int getOrCreate(uint64_t key) {
        auto it = node_id_.find(key);
        if (it != node_id_.end()) return it->second;
        int id = static_cast<int>(parent_.size());
        node_id_[key] = id;
        parent_.push_back(id);
        return id;
    }

    int find(int i) {
        if (parent_[static_cast<size_t>(i)] == i) return i;
        return parent_[static_cast<size_t>(i)] = find(parent_[static_cast<size_t>(i)]);
    }

    int findKey(uint64_t key) {
        return find(getOrCreate(key));
    }

    void merge(int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) parent_[static_cast<size_t>(a)] = b;
    }

    void mergeKeys(uint64_t k1, uint64_t k2) {
        merge(getOrCreate(k1), getOrCreate(k2));
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Track IDC write / read
// ─────────────────────────────────────────────────────────────────────────────

static bool save_track_store_to_idc(const TrackStore& store,
                                    const std::vector<uint32_t>& image_ids,
                                    const std::string& path) {
    const size_t n_tracks = store.num_tracks();
    json meta;
    meta["schema_version"] = "1.0";
    meta["task_type"] = "tracks";
    meta["num_images"] = static_cast<int>(image_ids.size());
    meta["image_ids"] = image_ids;  // JSON array of numbers
    meta["num_tracks"] = static_cast<int>(n_tracks);

    std::vector<float> track_xyz(static_cast<size_t>(n_tracks) * 3);
    std::vector<uint8_t> track_flags(static_cast<size_t>(n_tracks));
    for (size_t t = 0; t < n_tracks; ++t) {
        float x, y, z;
        store.get_track_xyz(static_cast<int>(t), &x, &y, &z);
        track_xyz[t * 3] = x;
        track_xyz[t * 3 + 1] = y;
        track_xyz[t * 3 + 2] = z;
        track_flags[static_cast<size_t>(t)] = store.is_track_valid(static_cast<int>(t)) ? track_flags::kAlive : 0;
    }

    std::vector<uint32_t> track_obs_offset(static_cast<size_t>(n_tracks) + 1);
    std::vector<uint32_t> obs_image_id, obs_feature_id;
    std::vector<float> obs_u, obs_v, obs_scale;
    std::vector<uint8_t> obs_flags;
    obs_image_id.reserve(store.num_observations());
    obs_feature_id.reserve(store.num_observations());
    obs_u.reserve(store.num_observations());
    obs_v.reserve(store.num_observations());
    obs_scale.reserve(store.num_observations());
    obs_flags.reserve(store.num_observations());

    std::vector<Observation> obs_buf;
    size_t offset = 0;
    for (size_t t = 0; t < n_tracks; ++t) {
        track_obs_offset[t] = static_cast<uint32_t>(offset);
        obs_buf.clear();
        store.get_track_observations(static_cast<int>(t), &obs_buf);
        for (const auto& o : obs_buf) {
            obs_image_id.push_back(o.image_index);
            obs_feature_id.push_back(o.feature_id);
            obs_u.push_back(o.u);
            obs_v.push_back(o.v);
            obs_scale.push_back(o.scale);
            obs_flags.push_back(obs_flags::kAlive);
        }
        offset += obs_buf.size();
    }
    track_obs_offset[n_tracks] = static_cast<uint32_t>(offset);
    const size_t n_obs = obs_image_id.size();
    meta["num_observations"] = static_cast<int>(n_obs);

    IDCWriter writer(path);
    writer.setMetadata(meta);
    writer.addBlob("track_xyz", track_xyz.data(), track_xyz.size() * sizeof(float),
                   "float32", {static_cast<int>(n_tracks), 3});
    writer.addBlob("track_flags", track_flags.data(), track_flags.size(),
                   "uint8", {static_cast<int>(n_tracks)});
    writer.addBlob("track_obs_offset", track_obs_offset.data(), track_obs_offset.size() * sizeof(uint32_t),
                   "uint32", {static_cast<int>(n_tracks) + 1});
    writer.addBlob("obs_image_id", obs_image_id.data(), obs_image_id.size() * sizeof(uint32_t),
                   "uint32", {static_cast<int>(obs_image_id.size())});
    writer.addBlob("obs_feature_id", obs_feature_id.data(), obs_feature_id.size() * sizeof(uint32_t),
                   "uint32", {static_cast<int>(obs_feature_id.size())});
    writer.addBlob("obs_u", obs_u.data(), obs_u.size() * sizeof(float),
                   "float32", {static_cast<int>(obs_u.size())});
    writer.addBlob("obs_v", obs_v.data(), obs_v.size() * sizeof(float),
                   "float32", {static_cast<int>(obs_v.size())});
    writer.addBlob("obs_scale", obs_scale.data(), obs_scale.size() * sizeof(float),
                   "float32", {static_cast<int>(obs_scale.size())});
    writer.addBlob("obs_flags", obs_flags.data(), obs_flags.size(),
                   "uint8", {static_cast<int>(obs_flags.size())});
    if (!writer.write()) {
        LOG(ERROR) << "Failed to write " << path;
        return false;
    }
    VLOG(1) << "Wrote " << path << " (" << n_tracks << " tracks, " << n_obs << " observations)";
    return true;
}

static bool load_track_store_from_idc(const std::string& path,
                                      TrackStore* store,
                                      std::vector<uint32_t>* image_ids_out) {
    IDCReader reader(path);
    if (!reader.isValid()) {
        LOG(ERROR) << "Invalid IDC: " << path;
        return false;
    }
    const json& meta = reader.getMetadata();
    const int num_images = meta["num_images"].get<int>();
    const int num_tracks = meta["num_tracks"].get<int>();
    const int num_observations = meta["num_observations"].get<int>();
    std::vector<uint32_t> image_ids;
    for (const auto& v : meta["image_ids"]) {
        if (v.is_number_unsigned())
            image_ids.push_back(v.get<uint32_t>());
        else if (v.is_string())
            image_ids.push_back(static_cast<uint32_t>(std::stoul(v.get<std::string>())));
        else
            image_ids.push_back(static_cast<uint32_t>(v.get<int64_t>()));
    }

    auto track_xyz = reader.readBlob<float>("track_xyz");
    auto track_flags = reader.readBlob<uint8_t>("track_flags");
    auto track_obs_offset = reader.readBlob<uint32_t>("track_obs_offset");
    auto obs_image_id = reader.readBlob<uint32_t>("obs_image_id");
    auto obs_feature_id = reader.readBlob<uint32_t>("obs_feature_id");
    auto obs_u = reader.readBlob<float>("obs_u");
    auto obs_v = reader.readBlob<float>("obs_v");
    auto obs_scale = reader.readBlob<float>("obs_scale");
    auto obs_flags = reader.readBlob<uint8_t>("obs_flags");

    if (track_xyz.size() != static_cast<size_t>(num_tracks) * 3u ||
        track_flags.size() != static_cast<size_t>(num_tracks) ||
        track_obs_offset.size() != static_cast<size_t>(num_tracks) + 1u ||
        obs_image_id.size() != static_cast<size_t>(num_observations)) {
        LOG(ERROR) << "IDC blob size mismatch";
        return false;
    }

    store->set_num_images(num_images);
    for (int t = 0; t < num_tracks; ++t) {
        float x = track_xyz[static_cast<size_t>(t) * 3];
        float y = track_xyz[static_cast<size_t>(t) * 3 + 1];
        float z = track_xyz[static_cast<size_t>(t) * 3 + 2];
        store->add_track(x, y, z);
        if ((track_flags[static_cast<size_t>(t)] & track_flags::kAlive) == 0)
            store->mark_track_deleted(t);
    }
    for (int t = 0; t < num_tracks; ++t) {
        const size_t beg = track_obs_offset[static_cast<size_t>(t)];
        const size_t end = track_obs_offset[static_cast<size_t>(t) + 1];
        for (size_t g = beg; g < end; ++g) {
            float s = (g < obs_scale.size()) ? obs_scale[g] : 1.f;
            store->add_observation(t, obs_image_id[g], obs_feature_id[g],
                                   obs_u[g], obs_v[g], s);
        }
    }
    for (int g = 0; g < num_observations; ++g) {
        if (static_cast<size_t>(g) < obs_flags.size() &&
            (obs_flags[static_cast<size_t>(g)] & obs_flags::kAlive) == 0)
            store->mark_observation_deleted(g);
    }
    if (image_ids_out) *image_ids_out = image_ids;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1: Union-Find from match indices
// ─────────────────────────────────────────────────────────────────────────────

static void phase1UnionFind(UnionFind* uf,
                            const std::vector<PairDesc>& pairs) {
    for (const auto& p : pairs) {
        IDCReader reader(p.match_file);
        if (!reader.isValid()) continue;
        auto indices = reader.readBlob<uint16_t>("indices");
        if (indices.size() < 2) continue;
        const int n = static_cast<int>(indices.size() / 2);
        for (int i = 0; i < n; ++i) {
            uint16_t idx1 = indices[static_cast<size_t>(i) * 2];
            uint16_t idx2 = indices[static_cast<size_t>(i) * 2 + 1];
            uf->mergeKeys(nodeKey(p.image1_id, idx1), nodeKey(p.image2_id, idx2));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 2: Fill observations and xyz from match + geo
// ─────────────────────────────────────────────────────────────────────────────

static void phase2FillObservations(TrackStore* store,
                                   const std::vector<PairDesc>& pairs,
                                   const std::unordered_map<uint32_t, int>& image_id_to_index,
                                   const std::unordered_map<int, int>& root_to_track_id,
                                   UnionFind& uf,
                                   std::set<int>* tracks_with_xyz) {
    std::set<std::tuple<int, int, uint32_t>> added_obs;  // (track_id, image_index, feature_id)
    for (const auto& p : pairs) {
        IDCReader geo_rd(p.geo_file);
        IDCReader match_rd(p.match_file);
        if (!geo_rd.isValid() || !match_rd.isValid()) continue;
        auto inlier_mask = geo_rd.readBlob<uint8_t>("F_inliers");
        if (inlier_mask.empty())
            inlier_mask = geo_rd.readBlob<uint8_t>("E_inliers");
        if (inlier_mask.empty()) continue;
        auto indices = match_rd.readBlob<uint16_t>("indices");
        auto coords = match_rd.readBlob<float>("coords_pixel");
        auto scales = match_rd.readBlob<float>("scales");
        if (indices.size() < 2 || coords.size() < 4) continue;
        const int num_matches = static_cast<int>(inlier_mask.size());
        const bool have_scales = (scales.size() >= static_cast<size_t>(num_matches) * 2u);
        auto it1 = image_id_to_index.find(p.image1_id);
        auto it2 = image_id_to_index.find(p.image2_id);
        if (it1 == image_id_to_index.end() || it2 == image_id_to_index.end()) continue;
        const int img1_idx = it1->second;
        const int img2_idx = it2->second;

        std::vector<float> points3d;
        bool have_points3d = false;
        if (geo_rd.getMetadata().contains("twoview")) {
            points3d = geo_rd.readBlob<float>("points3d");
            have_points3d = (geo_rd.getMetadata()["twoview"].contains("num_valid_points") &&
                             !points3d.empty());
        }
        int inlier_count = 0;
        for (int m = 0; m < num_matches; ++m) {
            if (!inlier_mask[static_cast<size_t>(m)]) continue;
            const size_t mi = static_cast<size_t>(m);
            uint16_t idx1 = indices[mi * 2];
            uint16_t idx2 = indices[mi * 2 + 1];
            uint64_t k1 = nodeKey(p.image1_id, idx1);
            int root = uf.findKey(k1);
            auto rit = root_to_track_id.find(root);
            if (rit == root_to_track_id.end()) continue;
            int track_id = rit->second;
            float x1 = coords[mi * 4];
            float y1 = coords[mi * 4 + 1];
            float x2 = coords[mi * 4 + 2];
            float y2 = coords[mi * 4 + 3];
            float s1 = 1.f, s2 = 1.f;
            if (have_scales) {
                s1 = scales[mi * 2];
                s2 = scales[mi * 2 + 1];
            }
            auto key1 = std::make_tuple(track_id, img1_idx, static_cast<uint32_t>(idx1));
            if (added_obs.find(key1) == added_obs.end()) {
                store->add_observation(track_id, static_cast<uint32_t>(img1_idx), idx1, x1, y1, s1);
                added_obs.insert(key1);
            }
            auto key2 = std::make_tuple(track_id, img2_idx, static_cast<uint32_t>(idx2));
            if (added_obs.find(key2) == added_obs.end()) {
                store->add_observation(track_id, static_cast<uint32_t>(img2_idx), idx2, x2, y2, s2);
                added_obs.insert(key2);
            }
            if (have_points3d && tracks_with_xyz->count(track_id) == 0) {
                const size_t base = static_cast<size_t>(inlier_count) * 3u;
                if (base + 2 < points3d.size()) {
                    store->set_track_xyz(track_id, points3d[base], points3d[base + 1], points3d[base + 2]);
                    tracks_with_xyz->insert(track_id);
                }
            }
            ++inlier_count;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;

    CmdLine cmd("InsightAT Track Building – match + geo → tracks IDC");
    std::string pairs_json, match_dir, geo_dir, image_list, output_path;
    bool stats_only = false;

    cmd.add(make_option('i', pairs_json, "input").doc("Pairs JSON (e.g. from isat_geo output)"));
    cmd.add(make_option('m', match_dir, "match-dir").doc("Directory of .isat_match files"));
    cmd.add(make_option('g', geo_dir, "geo-dir").doc("Directory of .isat_geo files"));
    cmd.add(make_option('l', image_list, "image-list").doc("Image list JSON for stable image ordering (optional)"));
    cmd.add(make_option('o', output_path, "output").doc("Output .isat_tracks IDC path"));
    cmd.add(make_switch(0, "stats").doc("Only load existing IDC and print stats to stderr"));
    std::string log_level;
    cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
    cmd.add(make_switch('v', "verbose").doc("Verbose (INFO)"));
    cmd.add(make_switch('q', "quiet").doc("Quiet (ERROR only)"));
    cmd.add(make_switch('h', "help").doc("Show help"));

    try {
        cmd.process(argc, argv);
    } catch (const std::string& s) {
        std::cerr << "Error: " << s << "\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }
    if (cmd.checkHelp(argv[0])) return 0;
    insight::tools::ApplyLogLevel(cmd.used('v'), cmd.used('q'), log_level);
    stats_only = cmd.used("stats");

    if (output_path.empty()) {
        std::cerr << "Error: -o/--output is required\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }

    if (stats_only) {
        TrackStore store;
        std::vector<uint32_t> image_ids;
        if (!load_track_store_from_idc(output_path, &store, &image_ids)) return 1;
        LOG(INFO) << "Tracks: " << store.num_tracks()
                  << "  Observations: " << store.num_observations()
                  << "  Images: " << image_ids.size();
        printEvent({{"type", "tracks.stats"},
                   {"ok", true},
                   {"data", {{"num_tracks", static_cast<int>(store.num_tracks())},
                            {"num_observations", static_cast<int>(store.num_observations())},
                            {"num_images", static_cast<int>(image_ids.size())}}}});
        return 0;
    }

    if (pairs_json.empty() || match_dir.empty() || geo_dir.empty()) {
        std::cerr << "Error: -i, -m, -g are required for building tracks\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }

    if (!fs::is_directory(match_dir)) {
        LOG(ERROR) << "Match directory not found: " << match_dir;
        return 1;
    }
    if (!fs::is_directory(geo_dir)) {
        LOG(ERROR) << "Geo directory not found: " << geo_dir;
        return 1;
    }

    std::vector<PairDesc> pairs = loadPairs(pairs_json, match_dir, geo_dir);
    if (pairs.empty()) {
        LOG(ERROR) << "No pairs to process";
        return 1;
    }

    std::vector<uint32_t> image_ids = getImageIds(image_list, pairs);
    std::unordered_map<uint32_t, int> image_id_to_index = buildImageIdToIndex(image_ids);
    const int n_images = static_cast<int>(image_ids.size());

    UnionFind uf;
    phase1UnionFind(&uf, pairs);

    std::unordered_map<int, int> root_to_track_id;
    int next_track = 0;
    for (const auto& kv : uf.node_id_) {
        int root = uf.find(kv.second);
        if (root_to_track_id.find(root) == root_to_track_id.end())
            root_to_track_id[root] = next_track++;
    }
    const int num_tracks = next_track;
    LOG(INFO) << "Phase 1: " << num_tracks << " tracks from Union-Find";

    TrackStore store;
    store.set_num_images(n_images);
    store.reserve_tracks(static_cast<size_t>(num_tracks));
    store.reserve_observations(static_cast<size_t>(num_tracks) * 4u);
    for (int t = 0; t < num_tracks; ++t)
        store.add_track(0.f, 0.f, 0.f);

    std::set<int> tracks_with_xyz;
    phase2FillObservations(&store, pairs, image_id_to_index, root_to_track_id, uf, &tracks_with_xyz);
    LOG(INFO) << "Phase 2: " << store.num_observations() << " observations, " << tracks_with_xyz.size() << " tracks with xyz";

    if (!save_track_store_to_idc(store, image_ids, output_path)) return 1;
    printEvent({{"type", "tracks.build"},
                {"ok", true},
                {"data", {{"output", output_path},
                         {"num_tracks", static_cast<int>(store.num_tracks())},
                         {"num_observations", static_cast<int>(store.num_observations())}}}});
    return 0;
}
