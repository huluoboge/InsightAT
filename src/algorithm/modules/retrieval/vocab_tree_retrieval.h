#pragma once

#include "retrieval_types.h"
#include <string>
#include <vector>
#include <memory>

// Forward declarations for DBoW3
namespace DBoW3 {
    class Vocabulary;
    class Database;
}

namespace insight::algorithm::retrieval {

/**
 * @brief Vocabulary tree retrieval using DBoW3
 * 
 * Implements bag-of-words based image retrieval with hierarchical k-means
 * vocabulary tree and TF-IDF weighting.
 */
class VocabTreeRetriever {
public:
    VocabTreeRetriever();
    ~VocabTreeRetriever();
    
    /**
     * @brief Load vocabulary tree from file
     * @param vocab_file Path to .dbow3 vocabulary file
     * @return true on success
     */
    bool loadVocabulary(const std::string& vocab_file);
    
    /**
     * @brief Check if vocabulary is loaded
     */
    bool isVocabularyLoaded() const;
    
    /**
     * @brief Add image to database
     * @param image_idx Image index
     * @param descriptors Image descriptors [N x 128] as flat vector
     * @return Entry ID in database
     */
    int addImage(int image_idx, const std::vector<float>& descriptors);
    
    /**
     * @brief Query similar images from database
     * @param descriptors Query image descriptors [N x 128]
     * @param max_results Maximum number of results to return
     * @return Vector of (image_idx, score) pairs, sorted by score descending
     */
    std::vector<std::pair<int, float>> query(
        const std::vector<float>& descriptors,
        int max_results = 20
    );
    
    /**
     * @brief Clear database (keep vocabulary loaded)
     */
    void clearDatabase();
    
    /**
     * @brief Get database size
     */
    int getDatabaseSize() const;

private:
    std::unique_ptr<DBoW3::Vocabulary> vocab_;
    std::unique_ptr<DBoW3::Database> database_;
    std::vector<int> image_indices_;  // Map DB entry ID → image index
};

/**
 * @brief Retrieve image pairs using DBoW3 vocabulary tree
 * 
 * Pure function: Builds temporary database, queries for each image,
 * and returns top-k similar pairs.
 * 
 * @param images Image list with feature files
 * @param options Retrieval configuration (top_k)
 * @param vocab_file Path to pre-trained .dbow3 vocabulary file
 * @param cache_dir Optional cache directory for BoW vectors
 * @return Image pairs sorted by visual similarity
 */
std::vector<ImagePair> retrieveByVocabTree(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options,
    const std::string& vocab_file,
    const std::string& cache_dir = ""
);

}  // namespace insight::algorithm::retrieval
