#include "idc_reader.h"
#include <fstream>
#include <cstring>

namespace insight {
namespace io {

IDCReader::IDCReader(const std::string& filepath) 
    : filepath_(filepath) {
    is_valid_ = parseHeader();
    
    if (!is_valid_) {
        LOG(ERROR) << "Failed to parse IDC file: " << filepath;
    }
}

bool IDCReader::parseHeader() {
    std::ifstream file(filepath_, std::ios::binary);
    if (!file.is_open()) {
        LOG(ERROR) << "Cannot open file: " << filepath_;
        return false;
    }
    
    // 1. Read magic number
    uint32_t magic;
    file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    if (magic != MAGIC_NUMBER) {
        LOG(ERROR) << "Invalid magic number: " << std::hex << magic 
                   << " (expected " << MAGIC_NUMBER << ")";
        return false;
    }
    
    // 2. Read format version
    uint32_t version;
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    if (version != FORMAT_VERSION) {
        LOG(WARNING) << "Format version mismatch: " << version 
                     << " (expected " << FORMAT_VERSION << ")";
        // Continue anyway, might be compatible
    }
    
    // 3. Read JSON size
    uint64_t json_size;
    file.read(reinterpret_cast<char*>(&json_size), sizeof(json_size));
    
    // 4. Read JSON descriptor
    std::string json_str(json_size, '\0');
    file.read(&json_str[0], json_size);
    
    if (!file) {
        LOG(ERROR) << "Failed to read JSON descriptor";
        return false;
    }
    
    // 5. Parse JSON
    try {
        metadata_ = nlohmann::json::parse(json_str);
    } catch (const nlohmann::json::exception& e) {
        LOG(ERROR) << "Failed to parse JSON: " << e.what();
        return false;
    }
    
    // 6. Calculate payload offset (with alignment)
    size_t header_size = 4 + 4 + 8 + json_size;  // magic + version + json_size + json
    size_t padding = (ALIGNMENT - (header_size % ALIGNMENT)) % ALIGNMENT;
    payload_offset_ = header_size + padding;
    
    VLOG(1) << "IDC file parsed: " << filepath_
            << ", payload_offset=" << payload_offset_
            << ", blobs=" << metadata_["blobs"].size();
    
    return true;
}

nlohmann::json IDCReader::getBlobDescriptor(const std::string& blob_name) const {
    if (!metadata_.contains("blobs")) {
        return nlohmann::json();
    }
    
    for (const auto& blob : metadata_["blobs"]) {
        if (blob["name"] == blob_name) {
            return blob;
        }
    }
    
    return nlohmann::json();
}

std::vector<uint8_t> IDCReader::readBlobRaw(const std::string& blob_name) {
    return readBlob<uint8_t>(blob_name);
}

}  // namespace io
}  // namespace insight
