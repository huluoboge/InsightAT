/**
 * pair_json_utils.h
 * Helpers for reading pair JSON with image_id as uint32_t (accepts number or string).
 */
#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

namespace insight {
namespace tools {

/** Read image1_id/image2_id from pair JSON as uint32_t. Accepts both number and string (backward
 * compat). */
inline uint32_t get_image_id_from_pair(const nlohmann::json& pair, const char* key) {
  const auto& v = pair[key];
  if (v.is_number_unsigned())
    return v.get<uint32_t>();
  if (v.is_number_integer())
    return static_cast<uint32_t>(v.get<int64_t>());
  return static_cast<uint32_t>(std::stoul(v.get<std::string>()));
}

} // namespace tools
} // namespace insight
