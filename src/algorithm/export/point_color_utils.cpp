#include "point_color_utils.h"

#include <filesystem>

#include <glog/logging.h>

#include "algorithm/io/idc_reader.h"

namespace insight {
namespace export_util {

namespace {

std::string feat_path_for(const std::string& features_dir, uint32_t image_index) {
  return (std::filesystem::path(features_dir) / (std::to_string(image_index) + ".isat_feat"))
      .string();
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
  if (!std::filesystem::exists(feat_path))
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

} // namespace export_util
} // namespace insight
