#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <Eigen/Core>
#include <glog/logging.h>

namespace insight {
namespace io {

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

    // Get JSON metadata
    const nlohmann::json& getMetadata() const { return metadata_; }
    
    // Get blob descriptor by name
    nlohmann::json getBlobDescriptor(const std::string& blob_name) const;
    
    // Read raw blob data
    std::vector<uint8_t> readBlobRaw(const std::string& blob_name);
    
    // Read typed blob data
    template <typename T>
    std::vector<T> readBlob(const std::string& blob_name);
    
    // Get payload offset (start of binary data)
    size_t getPayloadOffset() const { return payload_offset_; }
    
    // Check if file is valid IDC format
    bool isValid() const { return is_valid_; }

private:
    std::string filepath_;
    nlohmann::json metadata_;
    size_t payload_offset_ = 0;
    bool is_valid_ = false;
    
    static constexpr uint32_t MAGIC_NUMBER = 0x54415349; // "ISAT"
    static constexpr uint32_t FORMAT_VERSION = 1;
    static constexpr size_t ALIGNMENT = 8;
    
    // Parse header and load metadata
    bool parseHeader();
};

// Template implementation
template <typename T>
std::vector<T> IDCReader::readBlob(const std::string& blob_name) {
    auto blob_desc = getBlobDescriptor(blob_name);
    
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

}  // namespace io
}  // namespace insight
