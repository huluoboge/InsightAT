#include "idc_writer.h"
#include <glog/logging.h>
#include <cstring>

namespace insight {
namespace io {

IDCWriter::IDCWriter(const std::string& filepath)
    : filepath_(filepath)
{
    metadata_["blobs"] = nlohmann::json::array();
}

IDCWriter::~IDCWriter() = default;

void IDCWriter::setMetadata(const nlohmann::json& metadata)
{
    metadata_ = metadata;
    if (!metadata_.contains("blobs")) {
        metadata_["blobs"] = nlohmann::json::array();
    }
}

void IDCWriter::addBlob(const std::string& name,
                        const void* data,
                        size_t size,
                        const std::string& dtype,
                        const std::vector<int>& shape)
{
    // Record blob descriptor
    nlohmann::json blob_desc;
    blob_desc["name"] = name;
    blob_desc["dtype"] = dtype;
    blob_desc["shape"] = shape;
    blob_desc["offset"] = payload_.size();
    blob_desc["size"] = size;
    
    metadata_["blobs"].push_back(blob_desc);
    
    // Append data to payload
    const uint8_t* byte_data = static_cast<const uint8_t*>(data);
    payload_.insert(payload_.end(), byte_data, byte_data + size);
}

bool IDCWriter::write()
{
    std::ofstream file(filepath_, std::ios::binary);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file for writing: " << filepath_;
        return false;
    }
    
    // Write magic header
    file.write(reinterpret_cast<const char*>(&MAGIC_NUMBER), sizeof(MAGIC_NUMBER));
    
    // Write format version
    file.write(reinterpret_cast<const char*>(&FORMAT_VERSION), sizeof(FORMAT_VERSION));
    
    // Serialize JSON
    std::string json_str = metadata_.dump();
    uint64_t json_size = json_str.size();
    
    // Write JSON size
    file.write(reinterpret_cast<const char*>(&json_size), sizeof(json_size));
    
    // Write JSON descriptor
    file.write(json_str.data(), json_size);
    
    // Calculate current offset: 4 (magic) + 4 (version) + 8 (json_size) + json_size
    size_t current_offset = sizeof(MAGIC_NUMBER) + sizeof(FORMAT_VERSION) + sizeof(json_size) + json_size;
    
    // Add padding to align Binary Payload to 8-byte boundary
    size_t padding = calculatePadding(current_offset);
    if (padding > 0) {
        std::vector<uint8_t> padding_bytes(padding, 0);
        file.write(reinterpret_cast<const char*>(padding_bytes.data()), padding);
        LOG(INFO) << "Added " << padding << " bytes padding for alignment";
    }
    
    // Write binary payload (now 8-byte aligned)
    if (!payload_.empty()) {
        file.write(reinterpret_cast<const char*>(payload_.data()), payload_.size());
    }
    
    file.close();
    
    LOG(INFO) << "IDC file written: " << filepath_ 
              << " (JSON: " << json_size << " bytes, Padding: " << padding 
              << " bytes, Payload: " << payload_.size() << " bytes)";
    
    return true;
}

} // namespace io
} // namespace insight
