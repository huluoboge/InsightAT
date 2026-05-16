#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace insight {
namespace io {

struct GeoPackPairEntry {
  uint32_t image1_index = 0;
  uint32_t image2_index = 0;
  uint32_t block_index = 0;
  std::string pack_path;
  std::string f_inliers_blob;
  std::string e_inliers_blob;

  bool F_ok = false;
  int F_inliers = 0;
  float F_inlier_ratio = 0.0f;
  bool H_ok = false;
  int H_inliers = 0;
  bool is_degenerate = false;
  bool E_ok = false;
  int E_inliers = 0;
  bool twoview_ok = false;
  bool stable = false;
  int num_valid_points = 0;
  double median_pixel_disp = 0.0;
  double score_prelim = 0.0;
  double median_parallax_deg = 0.0;
  double median_depth_baseline = 0.0;
};

#pragma pack(push, 1)
struct GeoPackIndexRecordV1 {
  uint32_t image1_index;
  uint32_t image2_index;
  uint32_t block_index;
  uint8_t flags; // bit0 F_ok, bit1 H_ok, bit2 is_degenerate, bit3 E_ok, bit4 twoview_ok, bit5 stable
  int32_t F_inliers;
  float F_inlier_ratio;
  int32_t H_inliers;
  int32_t E_inliers;
  int32_t num_valid_points;
  float median_pixel_disp;
  float score_prelim;
  float median_parallax_deg;
  float median_depth_baseline;
};
#pragma pack(pop)

static_assert(sizeof(GeoPackIndexRecordV1) == 49, "GeoPackIndexRecordV1 layout mismatch");

class GeoPackIndex {
public:
  bool load_from_dir(const std::string& geo_dir);
  bool is_valid() const { return valid_; }
  const GeoPackPairEntry* find(uint32_t image1_index, uint32_t image2_index) const;

  static constexpr const char* kJsonIndexFileName = "geopack_index.json";
  static constexpr const char* kBinaryIndexFileName = "geopack_index.isat_gpkx";
  static constexpr const char* kBinaryIndexBlobName = "entries_v1";

  static std::string geopack_file_name_from_block_index(uint32_t block_index);
  static bool write_binary_index(const std::string& geo_dir,
                                 const std::vector<GeoPackIndexRecordV1>& records,
                                 int block_size);

  static uint64_t pair_key(uint32_t image1_index, uint32_t image2_index);

private:
  bool load_binary_index(const std::string& geo_dir);
  bool load_json_index(const std::string& geo_dir);

  bool valid_ = false;
  std::unordered_map<uint64_t, GeoPackPairEntry> entries_;
};

} // namespace io
} // namespace insight
