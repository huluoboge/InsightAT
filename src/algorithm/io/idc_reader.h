#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <glog/logging.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_map>
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

  // Block-load API: read entire payload in one sequential fread, then access
  // individual blobs via pointer arithmetic (zero seeks, zero copies).
  std::vector<uint8_t> read_full_payload() const;
  // In-place variant: reuses caller's buffer (resize, no realloc if capacity fits).
  // Preferred in hot OMP loops to avoid repeated heap allocations.
  void read_full_payload_into(std::vector<uint8_t>& buf) const;
  // Returns pointer into pre-loaded payload buffer, or nullptr if not found.
  // out_size receives the byte length of the blob.
  const uint8_t* get_blob_from_payload(const std::string& blob_name,
                                       const std::vector<uint8_t>& payload,
                                       size_t* out_size) const;

private:
  std::string filepath_;
  nlohmann::json metadata_;
  size_t payload_offset_ = 0;
  bool is_valid_ = false;

  // O(1) blob lookup index: name → {offset, size}
  struct BlobInfo { size_t offset; size_t size; };
  std::unordered_map<std::string, BlobInfo> blob_index_;

  static constexpr uint32_t MAGIC_NUMBER = 0x54415349; // "ISAT"
  static constexpr uint32_t FORMAT_VERSION = 1;
  static constexpr size_t ALIGNMENT = 8;

  bool parse_header();
};

// Template implementation
template <typename T> std::vector<T> IDCReader::read_blob(const std::string& blob_name) {
  auto it = blob_index_.find(blob_name);
  if (it == blob_index_.end()) {
    LOG(ERROR) << "Blob '" << blob_name << "' not found in " << filepath_;
    return {};
  }

  const size_t offset = it->second.offset;
  const size_t size   = it->second.size;

  if (size % sizeof(T) != 0) {
    LOG(ERROR) << "Blob size " << size << " not divisible by element size " << sizeof(T);
    return {};
  }
  const size_t n_elems = size / sizeof(T);

  std::ifstream file(filepath_, std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to open file: " << filepath_;
    return {};
  }
  file.seekg(static_cast<std::streamoff>(payload_offset_ + offset));
  std::vector<T> data(n_elems);
  file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(size));
  if (!file) {
    LOG(ERROR) << "Failed to read blob '" << blob_name << "' from " << filepath_;
    return {};
  }
  return data;
}

} // namespace io
} // namespace insight
