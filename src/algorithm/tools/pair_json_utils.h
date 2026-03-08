/**
 * pair_json_utils.h
 * Helpers for reading pair JSON. Index-only: pairs use image1_index / image2_index only.
 */
#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

namespace insight {
namespace tools {

/** Read a numeric field from pair JSON (number or string). */
inline uint32_t get_image_index_from_pair(const nlohmann::json& pair, const char* key) {
  const auto& v = pair[key];
  if (v.is_number_unsigned())
    return v.get<uint32_t>();
  if (v.is_number_integer())
    return static_cast<uint32_t>(v.get<int64_t>());
  return static_cast<uint32_t>(std::stoul(v.get<std::string>()));
}

} // namespace tools
} // namespace insight
