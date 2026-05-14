#include "geopack_index.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <glog/logging.h>

#include "idc_reader.h"
#include "idc_writer.h"

namespace fs = std::filesystem;

namespace insight {
namespace io {

namespace {

constexpr uint8_t kFlagFOk = 1u << 0;
constexpr uint8_t kFlagHOk = 1u << 1;
constexpr uint8_t kFlagDegenerate = 1u << 2;
constexpr uint8_t kFlagEOk = 1u << 3;
constexpr uint8_t kFlagTwoviewOk = 1u << 4;
constexpr uint8_t kFlagStable = 1u << 5;

void fill_entry_from_record(const std::string& geo_dir, const GeoPackIndexRecordV1& r,
                            GeoPackPairEntry* e) {
  if (!e)
    return;
  e->image1_index = std::min(r.image1_index, r.image2_index);
  e->image2_index = std::max(r.image1_index, r.image2_index);
  e->block_index = r.block_index;

  fs::path pack_path = fs::path(geo_dir) / GeoPackIndex::geopack_file_name_from_block_index(r.block_index);
  e->pack_path = pack_path.string();

  const std::string prefix = "pair/" + std::to_string(e->image1_index) + "_" +
                             std::to_string(e->image2_index);
  e->f_inliers_blob = prefix + "/F_inliers";
  e->e_inliers_blob = prefix + "/E_inliers";

  e->F_ok = (r.flags & kFlagFOk) != 0;
  e->H_ok = (r.flags & kFlagHOk) != 0;
  e->is_degenerate = (r.flags & kFlagDegenerate) != 0;
  e->E_ok = (r.flags & kFlagEOk) != 0;
  e->twoview_ok = (r.flags & kFlagTwoviewOk) != 0;
  e->stable = (r.flags & kFlagStable) != 0;

  e->F_inliers = r.F_inliers;
  e->F_inlier_ratio = r.F_inlier_ratio;
  e->H_inliers = r.H_inliers;
  e->E_inliers = r.E_inliers;
  e->num_valid_points = r.num_valid_points;
  e->median_pixel_disp = static_cast<double>(r.median_pixel_disp);
  e->score_prelim = static_cast<double>(r.score_prelim);
  e->median_parallax_deg = static_cast<double>(r.median_parallax_deg);
  e->median_depth_baseline = static_cast<double>(r.median_depth_baseline);
}

} // namespace

uint64_t GeoPackIndex::pair_key(uint32_t image1_index, uint32_t image2_index) {
  uint32_t lo = image1_index;
  uint32_t hi = image2_index;
  if (lo > hi)
    std::swap(lo, hi);
  return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

std::string GeoPackIndex::geopack_file_name_from_block_index(uint32_t block_index) {
  std::ostringstream oss;
  oss << "geo_block_" << std::setfill('0') << std::setw(6) << block_index << ".isat_geopack";
  return oss.str();
}

bool GeoPackIndex::write_binary_index(const std::string& geo_dir,
                                      const std::vector<GeoPackIndexRecordV1>& records,
                                      int block_size) {
  const std::string out_path = (fs::path(geo_dir) / kBinaryIndexFileName).string();

  nlohmann::json meta;
  meta["schema_version"] = "1.0";
  meta["task_type"] = "two_view_geometry_pack_binary_index";
  meta["pack_suffix"] = ".isat_geopack";
  meta["record_blob"] = kBinaryIndexBlobName;
  meta["record_layout"] = "GeoPackIndexRecordV1";
  meta["record_size_bytes"] = static_cast<int>(sizeof(GeoPackIndexRecordV1));
  meta["num_pairs"] = static_cast<int>(records.size());
  meta["block_size"] = std::max(1, block_size);

  IDCWriter writer(out_path);
  writer.set_metadata(meta);
  if (!records.empty()) {
    writer.add_blob(kBinaryIndexBlobName, records.data(),
                    records.size() * sizeof(GeoPackIndexRecordV1), "uint8",
                    {static_cast<int>(records.size()),
                     static_cast<int>(sizeof(GeoPackIndexRecordV1))});
  }

  if (!writer.write()) {
    LOG(ERROR) << "Failed to write binary geopack index: " << out_path;
    return false;
  }
  return true;
}

const GeoPackPairEntry* GeoPackIndex::find(uint32_t image1_index, uint32_t image2_index) const {
  const auto it = entries_.find(pair_key(image1_index, image2_index));
  if (it == entries_.end())
    return nullptr;
  return &it->second;
}

bool GeoPackIndex::load_from_dir(const std::string& geo_dir) {
  valid_ = false;
  entries_.clear();

  if (load_binary_index(geo_dir)) {
    valid_ = true;
    return true;
  }
  if (load_json_index(geo_dir)) {
    valid_ = true;
    return true;
  }
  return false;
}

bool GeoPackIndex::load_binary_index(const std::string& geo_dir) {
  const std::string index_path = (fs::path(geo_dir) / kBinaryIndexFileName).string();
  IDCReader reader(index_path);
  if (!reader.is_valid()) {
    VLOG(1) << "GeoPack binary index not found: " << index_path;
    return false;
  }

  std::vector<GeoPackIndexRecordV1> records = reader.read_blob<GeoPackIndexRecordV1>(kBinaryIndexBlobName);
  if (records.empty()) {
    LOG(WARNING) << "GeoPack binary index has no records: " << index_path;
    return false;
  }

  entries_.reserve(records.size());
  for (const auto& r : records) {
    GeoPackPairEntry e;
    fill_entry_from_record(geo_dir, r, &e);
    entries_[pair_key(e.image1_index, e.image2_index)] = std::move(e);
  }

  LOG(INFO) << "GeoPack binary index loaded: " << index_path
            << " entries=" << entries_.size();
  return !entries_.empty();
}

bool GeoPackIndex::load_json_index(const std::string& geo_dir) {
  fs::path index_path = fs::path(geo_dir) / kJsonIndexFileName;
  std::ifstream f(index_path);
  if (!f.is_open()) {
    VLOG(1) << "GeoPack JSON index not found: " << index_path.string();
    return false;
  }

  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    LOG(WARNING) << "Failed to parse geopack index: " << index_path.string() << " error="
                 << e.what();
    return false;
  }

  if (!j.is_object() || !j.contains("pairs") || !j["pairs"].is_array()) {
    LOG(WARNING) << "Invalid geopack index format: " << index_path.string();
    return false;
  }

  entries_.reserve(j["pairs"].size());
  for (const auto& p : j["pairs"]) {
    if (!p.is_object())
      continue;

    GeoPackPairEntry e;
    e.image1_index = p.value("image1_index", 0u);
    e.image2_index = p.value("image2_index", 0u);
    uint32_t lo = e.image1_index;
    uint32_t hi = e.image2_index;
    if (lo > hi)
      std::swap(lo, hi);
    e.image1_index = lo;
    e.image2_index = hi;

    std::string pack_file = p.value("pack_file", std::string());
    if (pack_file.empty())
      continue;
    fs::path pack_path(pack_file);
    if (pack_path.is_relative())
      pack_path = fs::path(geo_dir) / pack_path;
    e.pack_path = pack_path.string();
    if (p.contains("block_index"))
      e.block_index = p.value("block_index", 0u);

    if (p.contains("blobs") && p["blobs"].is_object()) {
      const auto& blobs = p["blobs"];
      e.f_inliers_blob = blobs.value("F_inliers", std::string());
      e.e_inliers_blob = blobs.value("E_inliers", std::string());
    }
    if (e.f_inliers_blob.empty() || e.e_inliers_blob.empty()) {
      const std::string prefix = "pair/" + std::to_string(e.image1_index) + "_" +
                                 std::to_string(e.image2_index);
      if (e.f_inliers_blob.empty())
        e.f_inliers_blob = prefix + "/F_inliers";
      if (e.e_inliers_blob.empty())
        e.e_inliers_blob = prefix + "/E_inliers";
    }

    if (p.contains("summary") && p["summary"].is_object()) {
      const auto& s = p["summary"];
      e.F_ok = s.value("F_ok", false);
      e.F_inliers = s.value("F_inliers", 0);
      e.F_inlier_ratio = s.value("F_inlier_ratio", 0.0f);
      e.H_ok = s.value("H_ok", false);
      e.H_inliers = s.value("H_inliers", 0);
      e.is_degenerate = s.value("is_degenerate", false);
      e.E_ok = s.value("E_ok", false);
      e.E_inliers = s.value("E_inliers", 0);
      e.twoview_ok = s.value("twoview_ok", false);
      e.stable = s.value("stable", false);
      e.num_valid_points = s.value("num_valid_points", 0);
      e.median_pixel_disp = s.value("median_pixel_disp", 0.0);
      e.score_prelim = s.value("score_prelim", 0.0);
      e.median_parallax_deg = s.value("median_parallax_deg", 0.0);
      e.median_depth_baseline = s.value("median_depth_baseline", 0.0);
    }

    entries_[pair_key(e.image1_index, e.image2_index)] = std::move(e);
  }

  LOG(INFO) << "GeoPack JSON index loaded: " << index_path.string() << " entries=" << entries_.size();
  return !entries_.empty();
}

} // namespace io
} // namespace insight
