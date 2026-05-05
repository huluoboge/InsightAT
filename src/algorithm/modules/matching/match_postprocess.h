#pragma once

#include "match_types.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <unordered_set>
#include <vector>

namespace insight {
namespace algorithm {
namespace matching {

inline std::vector<int> select_top_n_spatial_indices(const FeatureData& f, int max_n,
                                                     int grid_rows, int grid_cols) {
  const int n = static_cast<int>(f.num_features);
  if (n == 0) {
    return {};
  }
  if (max_n <= 0 || n <= max_n) {
    std::vector<int> idx(static_cast<size_t>(n));
    std::iota(idx.begin(), idx.end(), 0);
    return idx;
  }

  const int rows = grid_rows > 0 ? grid_rows : 4;
  const int cols = grid_cols > 0 ? grid_cols : 4;
  const int n_cells = rows * cols;

  float xmin = f.keypoints[0][0], xmax = xmin;
  float ymin = f.keypoints[0][1], ymax = ymin;
  for (int i = 1; i < n; ++i) {
    xmin = std::min(xmin, f.keypoints[i][0]);
    xmax = std::max(xmax, f.keypoints[i][0]);
    ymin = std::min(ymin, f.keypoints[i][1]);
    ymax = std::max(ymax, f.keypoints[i][1]);
  }
  const float cell_w = (xmax - xmin + 1.0f) / static_cast<float>(cols);
  const float cell_h = (ymax - ymin + 1.0f) / static_cast<float>(rows);

  std::vector<std::vector<int>> cells(static_cast<size_t>(n_cells));
  for (int i = 0; i < n; ++i) {
    int cx = static_cast<int>((f.keypoints[i][0] - xmin) / cell_w);
    int cy = static_cast<int>((f.keypoints[i][1] - ymin) / cell_h);
    cx = std::min(cx, cols - 1);
    cy = std::min(cy, rows - 1);
    cells[static_cast<size_t>(cy * cols + cx)].push_back(i);
  }
  for (auto& cell : cells) {
    std::sort(cell.begin(), cell.end(),
              [&f](int a, int b) { return f.keypoints[a][2] > f.keypoints[b][2]; });
  }

  std::vector<int> result;
  result.reserve(static_cast<size_t>(max_n));
  std::vector<int> cell_pos(static_cast<size_t>(n_cells), 0);
  while (static_cast<int>(result.size()) < max_n) {
    bool any = false;
    for (int c = 0; c < n_cells && static_cast<int>(result.size()) < max_n; ++c) {
      auto& pos = cell_pos[static_cast<size_t>(c)];
      const auto& cell = cells[static_cast<size_t>(c)];
      if (pos < static_cast<int>(cell.size())) {
        result.push_back(cell[static_cast<size_t>(pos++)]);
        any = true;
      }
    }
    if (!any) {
      break;
    }
  }
  return result;
}

inline float descriptor_distance_l2(const FeatureData& features1, const FeatureData& features2,
                                    size_t idx1, size_t idx2) {
  if (features1.descriptor_type != features2.descriptor_type) {
    return 1e6f;
  }
  float sum = 0.0f;
  if (features1.descriptor_type == DescriptorType::kUInt8) {
    const uint8_t* desc1 = &features1.descriptors_uint8[idx1 * 128];
    const uint8_t* desc2 = &features2.descriptors_uint8[idx2 * 128];
    for (int i = 0; i < 128; ++i) {
      const float diff = static_cast<float>(desc1[i]) - static_cast<float>(desc2[i]);
      sum += diff * diff;
    }
  } else {
    const float* desc1 = &features1.descriptors_float[idx1 * 128];
    const float* desc2 = &features2.descriptors_float[idx2 * 128];
    for (int i = 0; i < 128; ++i) {
      const float diff = desc1[i] - desc2[i];
      sum += diff * diff;
    }
  }
  return std::sqrt(sum);
}

inline void deduplicate_match_indices(MatchResult* result) {
  if (result == nullptr || result->indices.empty()) {
    return;
  }
  struct Entry {
    std::pair<uint16_t, uint16_t> idx;
    Eigen::Vector4f coord;
    float dist;
  };
  std::vector<Entry> entries;
  entries.reserve(result->indices.size());
  for (size_t i = 0; i < result->indices.size(); ++i) {
    const float dist = i < result->distances.size() ? result->distances[i] : 0.0f;
    entries.push_back({result->indices[i], result->coords_pixel[i], dist});
  }
  std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
    if (a.idx.first != b.idx.first) return a.idx.first < b.idx.first;
    return a.idx.second < b.idx.second;
  });
  std::vector<Entry> unique_entries;
  unique_entries.reserve(entries.size());
  for (const auto& e : entries) {
    if (!unique_entries.empty() && unique_entries.back().idx == e.idx) {
      continue;
    }
    unique_entries.push_back(e);
  }
  result->clear();
  result->reserve(unique_entries.size());
  for (const auto& e : unique_entries) {
    result->indices.push_back(e.idx);
    result->coords_pixel.push_back(e.coord);
    result->distances.push_back(e.dist);
  }
  result->num_matches = result->indices.size();
}

inline void append_match(MatchResult* result, const FeatureData& features1, const FeatureData& features2,
                         uint16_t idx1, uint16_t idx2, bool with_distance) {
  result->indices.emplace_back(idx1, idx2);
  const auto& kp1 = features1.keypoints[static_cast<size_t>(idx1)];
  const auto& kp2 = features2.keypoints[static_cast<size_t>(idx2)];
  result->coords_pixel.emplace_back(kp1[0], kp1[1], kp2[0], kp2[1]);
  if (with_distance) {
    result->distances.push_back(
        descriptor_distance_l2(features1, features2, static_cast<size_t>(idx1), static_cast<size_t>(idx2)));
  }
}

}  // namespace matching
}  // namespace algorithm
}  // namespace insight

