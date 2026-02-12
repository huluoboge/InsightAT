#include "vocab_tree_retrieval.h"

#include <algorithm>
#include <filesystem>
#include <glog/logging.h>
#include <opencv2/core.hpp>

// DBoW3 headers
#include <DBoW3.h>

// InsightAT I/O
#include "../../io/idc_reader.h"

namespace fs = std::filesystem;

namespace insight::algorithm::retrieval {

// ============================================================================
// VocabTreeRetriever Implementation
// ============================================================================

VocabTreeRetriever::VocabTreeRetriever()
    : vocab_(std::make_unique<DBoW3::Vocabulary>())
    , database_(std::make_unique<DBoW3::Database>())
{
}

VocabTreeRetriever::~VocabTreeRetriever() = default;

bool VocabTreeRetriever::loadVocabulary(const std::string& vocab_file) {
    try {
        LOG(INFO) << "Loading vocabulary from " << vocab_file;
        vocab_->load(vocab_file);
        
        if (vocab_->empty()) {
            LOG(ERROR) << "Loaded vocabulary is empty";
            return false;
        }
        
        // Create database with loaded vocabulary
        database_ = std::make_unique<DBoW3::Database>(*vocab_);
        
        LOG(INFO) << "Vocabulary loaded: " 
                  << vocab_->size() << " words, "
                  << "branching factor " << vocab_->getBranchingFactor() << ", "
                  << "depth " << vocab_->getDepthLevels();
        
        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to load vocabulary: " << e.what();
        return false;
    }
}

bool VocabTreeRetriever::isVocabularyLoaded() const {
    return !vocab_->empty();
}

int VocabTreeRetriever::addImage(int image_idx, const std::vector<float>& descriptors) {
    if (descriptors.empty()) {
        LOG(WARNING) << "Empty descriptors for image " << image_idx;
        return -1;
    }
    
    int num_features = descriptors.size() / 128;
    
    // Convert to cv::Mat for DBoW3
    cv::Mat desc_mat(num_features, 128, CV_32F);
    std::memcpy(desc_mat.data, descriptors.data(), descriptors.size() * sizeof(float));
    
    // Add to database
    int entry_id = database_->add(desc_mat);
    
    // Store mapping
    if (static_cast<size_t>(entry_id) >= image_indices_.size()) {
        image_indices_.resize(entry_id + 1, -1);
    }
    image_indices_[entry_id] = image_idx;
    
    return entry_id;
}

std::vector<std::pair<int, float>> VocabTreeRetriever::query(
    const std::vector<float>& descriptors,
    int max_results
) {
    if (descriptors.empty()) {
        return {};
    }
    
    int num_features = descriptors.size() / 128;
    
    // Convert to cv::Mat
    cv::Mat desc_mat(num_features, 128, CV_32F);
    std::memcpy(desc_mat.data, descriptors.data(), descriptors.size() * sizeof(float));
    
    // Query database
    DBoW3::QueryResults results;
    database_->query(desc_mat, results, max_results);
    
    // Convert results to (image_idx, score) pairs
    std::vector<std::pair<int, float>> output;
    output.reserve(results.size());
    
    for (const auto& result : results) {
        int entry_id = result.Id;
        float score = result.Score;
        
        if (entry_id >= 0 && entry_id < static_cast<int>(image_indices_.size())) {
            int image_idx = image_indices_[entry_id];
            if (image_idx >= 0) {
                output.push_back({image_idx, score});
            }
        }
    }
    
    return output;
}

void VocabTreeRetriever::clearDatabase() {
    database_->clear();
    image_indices_.clear();
}

int VocabTreeRetriever::getDatabaseSize() const {
    return database_->size();
}

// ============================================================================
// Retrieval Strategy
// ============================================================================

std::vector<ImagePair> retrieveByVocabTree(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options,
    const std::string& vocab_file,
    const std::string& cache_dir
) {
    if (images.empty()) {
        LOG(WARNING) << "No images provided for vocabulary tree retrieval";
        return {};
    }
    
    if (vocab_file.empty()) {
        LOG(ERROR) << "Vocabulary file not specified";
        return {};
    }
    
    // Create retriever and load vocabulary
    VocabTreeRetriever retriever;
    if (!retriever.loadVocabulary(vocab_file)) {
        LOG(ERROR) << "Failed to load vocabulary from " << vocab_file;
        return {};
    }
    
    LOG(INFO) << "Vocabulary tree retrieval: " << images.size() << " images, "
              << "top-k=" << options.top_k;
    
    // Load descriptors for all images
    std::vector<std::vector<float>> all_descriptors;
    all_descriptors.reserve(images.size());
    
    for (size_t i = 0; i < images.size(); ++i) {
        const auto& img = images[i];
        
        // Load descriptors from feature file using IDC reader
        io::IDCReader reader(img.feature_file);
        if (!reader.isValid()) {
            LOG(WARNING) << "Failed to open feature file: " << img.feature_file;
            continue;
        }
        
        auto blob = reader.getBlobDescriptor("descriptors");
        std::string dtype = blob["dtype"];
        
        std::vector<float> descriptors;
        if (dtype == "float32") {
            descriptors = reader.readBlob<float>("descriptors");
        } else if (dtype == "uint8") {
            auto desc_uint8 = reader.readBlob<uint8_t>("descriptors");
            descriptors.resize(desc_uint8.size());
            for (size_t j = 0; j < desc_uint8.size(); ++j) {
                descriptors[j] = static_cast<float>(desc_uint8[j]);
            }
        } else {
            LOG(WARNING) << "Unsupported descriptor type: " << dtype;
            continue;
        }
        
        if (descriptors.empty()) {
            LOG(WARNING) << "Empty descriptors for " << img.image_id;
            continue;
        }
        
        all_descriptors.push_back(std::move(descriptors));
    }
    
    // Build database
    LOG(INFO) << "Building vocabulary tree database...";
    for (size_t i = 0; i < images.size(); ++i) {
        if (i < all_descriptors.size()) {
            retriever.addImage(i, all_descriptors[i]);
        }
    }
    
    LOG(INFO) << "Database built with " << retriever.getDatabaseSize() << " images";
    
    // Query for each image
    std::vector<ImagePair> pairs;
    pairs.reserve(images.size() * options.top_k);
    
    for (size_t i = 0; i < images.size(); ++i) {
        auto similar_images = retriever.query(all_descriptors[i], options.top_k + 1);
        
        for (const auto& [j, score] : similar_images) {
            // Skip self-matches
            if (static_cast<size_t>(j) == i) {
                continue;
            }
            
            // Only create pairs where i < j to avoid duplicates
            if (static_cast<size_t>(j) > i) {
                ImagePair pair;
                pair.image1_idx = i;
                pair.image2_idx = j;
                pair.score = score;
                pair.method = "vocab_tree";
                pair.visual_similarity = score;
                
                if (pair.isValid()) {
                    pairs.push_back(pair);
                }
            }
        }
    }
    
    // Sort by score descending
    std::sort(pairs.begin(), pairs.end(), 
              [](const ImagePair& a, const ImagePair& b) {
                  return a.score > b.score;
              });
    
    LOG(INFO) << "Vocabulary tree retrieval: generated " << pairs.size() 
              << " pairs from " << images.size() << " images";
    
    return pairs;
}

}  // namespace insight::algorithm::retrieval
