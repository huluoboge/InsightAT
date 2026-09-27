#include "point_color_utils.h"

#include <filesystem>

#include <glog/logging.h>

#include "algorithm/io/idc_reader.h"

namespace insight {
namespace export_util {

namespace fs = std::filesystem;

namespace {

std::string feat_path_for(const std::string& features_dir, uint32_t image_index) {
  return (fs::path(features_dir) / (std::to_string(image_index) + ".isat_feat")).string();
}

fs::path parent_dir_of_hint(const std::string& hint) {
  if (hint.empty())
    return {};
  fs::path p(hint);
  std::error_code ec;
  if (fs::is_directory(p, ec))
    return p;
  return p.parent_path();
}

} // namespace

FeatureColorCache::FeatureColorCache(std::string features_dir)
    : features_dir_(std::move(features_dir)) {}

const std::vector<uint8_t>* FeatureColorCache::get(uint32_t image_index) {
  if (features_dir_.empty())
    return nullptr;

  auto it = cache_.find(image_index);
  if (it != cache_.end())
    return &it->second;

  std::vector<uint8_t> colors = load_feature_colors(feat_path_for(features_dir_, image_index));
  auto [ins, _] = cache_.emplace(image_index, std::move(colors));
  return &ins->second;
}

std::vector<uint8_t> load_feature_colors(const std::string& feat_path) {
  if (!fs::exists(feat_path))
    return {};

  io::IDCReader reader(feat_path);
  if (!reader.is_valid() || !reader.has_blob("colors"))
    return {};

  auto desc = reader.get_blob_descriptor("colors");
  if (desc.is_null() || !desc.contains("dtype") || desc["dtype"].get<std::string>() != "uint8") {
    LOG(WARNING) << "Invalid colors blob dtype in " << feat_path;
    return {};
  }

  std::vector<uint8_t> colors = reader.read_blob<uint8_t>("colors");
  if (colors.empty() || (colors.size() % 3) != 0) {
    LOG(WARNING) << "Invalid colors blob size " << colors.size() << " in " << feat_path;
    return {};
  }
  return colors;
}

std::optional<std::array<uint8_t, 3>>
average_track_rgb(const std::vector<sfm::Observation>& observations, FeatureColorCache& cache) {
  if (!cache.enabled() || observations.empty())
    return std::nullopt;

  double sum_r = 0.0, sum_g = 0.0, sum_b = 0.0;
  int count = 0;

  for (const auto& o : observations) {
    const std::vector<uint8_t>* colors = cache.get(o.image_index);
    if (!colors || colors->empty())
      continue;
    const size_t n = colors->size() / 3;
    if (static_cast<size_t>(o.feature_id) >= n)
      continue;
    const size_t base = static_cast<size_t>(o.feature_id) * 3;
    sum_r += (*colors)[base + 0];
    sum_g += (*colors)[base + 1];
    sum_b += (*colors)[base + 2];
    ++count;
  }

  if (count == 0)
    return std::nullopt;

  auto clamp_u8 = [](double v) -> uint8_t {
    if (v < 0.0)
      return 0;
    if (v > 255.0)
      return 255;
    return static_cast<uint8_t>(v + 0.5);
  };

  return std::array<uint8_t, 3>{clamp_u8(sum_r / count), clamp_u8(sum_g / count),
                                clamp_u8(sum_b / count)};
}

bool is_features_dir(const std::string& dir) {
  if (dir.empty())
    return false;
  std::error_code ec;
  if (!fs::is_directory(dir, ec))
    return false;
  for (const auto& entry : fs::directory_iterator(dir, ec)) {
    if (ec)
      break;
    if (entry.path().extension() == ".isat_feat")
      return true;
  }
  return false;
}

bool features_dir_has_colors(const std::string& features_dir, int max_probe) {
  if (!is_features_dir(features_dir) || max_probe <= 0)
    return false;

  int probed = 0;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(features_dir, ec)) {
    if (ec)
      break;
    if (entry.path().extension() != ".isat_feat")
      continue;
    if (!load_feature_colors(entry.path().string()).empty())
      return true;
    if (++probed >= max_probe)
      break;
  }
  return false;
}

std::string resolve_features_dir(const std::string& explicit_dir,
                                 const std::vector<std::string>& hint_paths) {
  if (!explicit_dir.empty()) {
    if (is_features_dir(explicit_dir))
      return explicit_dir;
    LOG(WARNING) << "features dir does not contain .isat_feat: " << explicit_dir;
    return explicit_dir; // keep caller choice; cache will just yield empty colors
  }

  static const char* kCandidates[] = {"feat", "features", "features_matching"};
  for (const auto& hint : hint_paths) {
    const fs::path parent = parent_dir_of_hint(hint);
    if (parent.empty())
      continue;
    for (const char* name : kCandidates) {
      const fs::path cand = parent / name;
      if (is_features_dir(cand.string())) {
        LOG(INFO) << "Auto-detected feature directory for point colors: " << cand.string();
        return cand.string();
      }
    }
  }
  return {};
}

} // namespace export_util
} // namespace insight
