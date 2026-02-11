#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <nlohmann/json.hpp>

namespace insight {
namespace io {

/**
 * IDC (Insight Data Container) Writer
 * 
 * Binary format:
 * - Magic Header: "ISAT" (4 bytes)
 * - Format Version: uint32_t (4 bytes)
 * - JSON Size: uint64_t (8 bytes)
 * - JSON Descriptor: UTF-8 string (variable length)
 * - Padding: 0-7 bytes to align next section to 8-byte boundary
 * - Binary Payload: raw data blobs (8-byte aligned)
 * 
 * Alignment: Binary payload starts at 8-byte boundary for optimal
 * performance with SIMD, GPU upload, and cross-platform compatibility.
 */
class IDCWriter {
public:
    IDCWriter(const std::string& filepath);
    ~IDCWriter();

    // Set JSON metadata descriptor
    void setMetadata(const nlohmann::json& metadata);

    // Add binary blob
    void addBlob(const std::string& name, 
                 const void* data, 
                 size_t size,
                 const std::string& dtype,
                 const std::vector<int>& shape);

    // Finalize and write to disk
    bool write();

private:
    std::string filepath_;
    nlohmann::json metadata_;
    std::vector<uint8_t> payload_;
    std::vector<nlohmann::json> blob_descriptors_;
    
    static constexpr uint32_t MAGIC_NUMBER = 0x54415349; // "ISAT" in little-endian
    static constexpr uint32_t FORMAT_VERSION = 1;
    static constexpr size_t ALIGNMENT = 8; // 8-byte alignment for Binary Payload
    
    // Calculate padding needed to align to ALIGNMENT bytes
    static size_t calculatePadding(size_t current_offset) {
        return (ALIGNMENT - (current_offset % ALIGNMENT)) % ALIGNMENT;
    }
};

// Helper function to create feature extraction metadata
inline nlohmann::json createFeatureMetadata(
    const std::string& image_path,
    const std::string& algorithm_name,
    const std::string& algorithm_version,
    const nlohmann::json& parameters,
    int execution_time_ms = 0)
{
    nlohmann::json meta;
    meta["schema_version"] = "1.0";
    meta["task_type"] = "feature_extraction";
    meta["algorithm"]["name"] = algorithm_name;
    meta["algorithm"]["version"] = algorithm_version;
    meta["algorithm"]["parameters"] = parameters;
    meta["metadata"]["image_path"] = image_path;
    meta["metadata"]["execution_time_ms"] = execution_time_ms;
    
    // Add timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&time_t));
    meta["metadata"]["timestamp"] = buffer;
    
    return meta;
}

} // namespace io
} // namespace insight
