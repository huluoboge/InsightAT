#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <glog/logging.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace insight {
namespace io {

// Forward declaration
struct DescriptorSchema;

/**
 * IDC (Insight Data Container) Reader
 *
 * Reads binary files in IDC format:
 * - Magic Header: "ISAT" (4 bytes)
 * - Format Version: uint32_t (4 bytes)
 * - JSON Size: uint64_t (8 bytes)
 * - JSON Descriptor: UTF-8 string
 * - Padding: 0-7 bytes (8-byte alignment)
 * - Binary Payload: raw data blobs
 */
class IDCReader {
public:
  explicit IDCReader(const std::string& filepath);
  ~IDCReader() = default;

  const nlohmann::json& get_metadata() const { return metadata_; }
  nlohmann::json get_blob_descriptor(const std::string& blob_name) const;
  std::vector<uint8_t> read_blob_raw(const std::string& blob_name);
  template <typename T> std::vector<T> read_blob(const std::string& blob_name);
  size_t get_payload_offset() const { return payload_offset_; }
  bool is_valid() const { return is_valid_; }
  std::optional<DescriptorSchema> get_descriptor_schema() const;

private:
  std::string filepath_;
  nlohmann::json metadata_;
  size_t payload_offset_ = 0;
  bool is_valid_ = false;

  static constexpr uint32_t MAGIC_NUMBER = 0x54415349; // "ISAT"
  static constexpr uint32_t FORMAT_VERSION = 1;
  static constexpr size_t ALIGNMENT = 8;

  // Parse header and load metadata
  bool parse_header();
};

// Template implementation
template <typename T> std::vector<T> IDCReader::read_blob(const std::string& blob_name) {
  auto blob_desc = get_blob_descriptor(blob_name);

  if (blob_desc.is_null()) {
    LOG(ERROR) << "Blob '" << blob_name << "' not found in " << filepath_;
    return {};
  }

  size_t offset = blob_desc["offset"].get<size_t>();
  size_t size = blob_desc["size"].get<size_t>();

  // Validate size
  size_t expected_size = size / sizeof(T);
  if (size % sizeof(T) != 0) {
    LOG(ERROR) << "Blob size " << size << " not divisible by element size " << sizeof(T);
    return {};
  }

  // Read data
  std::ifstream file(filepath_, std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to open file: " << filepath_;
    return {};
  }

  file.seekg(payload_offset_ + offset);
  std::vector<T> data(expected_size);
  file.read(reinterpret_cast<char*>(data.data()), size);

  if (!file) {
    LOG(ERROR) << "Failed to read blob '" << blob_name << "' from " << filepath_;
    return {};
  }

  return data;
}

} // namespace io
} // namespace insight
