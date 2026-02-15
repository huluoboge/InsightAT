#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <optional>
#include <nlohmann/json.hpp>

namespace insight {
namespace io {

/**
 * Descriptor Schema for Feature Extraction
 * Schema version 1.1: Adds explicit descriptor metadata
 */
struct DescriptorSchema {
    std::string feature_type;        // "sift", "superpoint", etc.
    int descriptor_dim;               // 128, 256, etc.
    std::string descriptor_dtype;     // "uint8", "float32"
    std::string normalization;        // "l2", "none"
    float quantization_scale;         // 512.0 for SIFT uint8, 1.0 for float
    std::string schema_version = "1.1";  // Schema version
    
    // Convert to JSON
    nlohmann::json toJson() const {
        nlohmann::json j;
        j["feature_type"] = feature_type;
        j["descriptor_dim"] = descriptor_dim;
        j["descriptor_dtype"] = descriptor_dtype;
        j["normalization"] = normalization;
        j["quantization_scale"] = quantization_scale;
        return j;
    }
    
    // Create from JSON
    static std::optional<DescriptorSchema> fromJson(const nlohmann::json& j) {
        if (!j.contains("descriptor_dim") || !j.contains("descriptor_dtype")) {
            return std::nullopt;
        }
        
        DescriptorSchema schema;
        schema.feature_type = j.value("feature_type", "unknown");
        schema.descriptor_dim = j["descriptor_dim"].get<int>();
        schema.descriptor_dtype = j["descriptor_dtype"].get<std::string>();
        schema.normalization = j.value("normalization", "none");
        schema.quantization_scale = j.value("quantization_scale", 1.0f);
        return schema;
    }
};

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

// Helper function to create feature extraction metadata (v1.1 with descriptor_schema)
inline nlohmann::json createFeatureMetadata(
    const std::string& image_path,
    const std::string& algorithm_name,
    const std::string& algorithm_version,
    const nlohmann::json& parameters,
    const std::optional<DescriptorSchema>& descriptor_schema = std::nullopt,
    int execution_time_ms = 0)
{
    nlohmann::json meta;
    
    // Use schema_version 1.1 if descriptor_schema provided, otherwise 1.0 for backward compatibility
    if (descriptor_schema.has_value()) {
        meta["schema_version"] = "1.1";
        meta["descriptor_schema"] = descriptor_schema->toJson();
    } else {
        meta["schema_version"] = "1.0";
    }
    
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
