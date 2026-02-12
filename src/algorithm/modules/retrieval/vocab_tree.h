#pragma once

#include <vector>
#include <string>
#include <map>
#include <unordered_map>

namespace insight::algorithm::retrieval {

// ============================================================================
// Vocabulary Tree Data Structures
// ============================================================================

/**
 * @brief Visual word node in hierarchical vocabulary tree
 */
struct VocabNode {
    int id = -1;                              ///< Node ID (leaf nodes are visual words)
    std::vector<float> descriptor;            ///< Cluster center [128]
    std::vector<int> children;                ///< Child node IDs
    int level = 0;                            ///< Tree level (0 = root)
    bool is_leaf = false;                     ///< True if visual word (leaf node)
    
    // TF-IDF weights (for leaf nodes)
    float idf_weight = 0.0f;                  ///< Inverse document frequency
    int num_images_with_word = 0;             ///< Number of images containing this word
};

/**
 * @brief Vocabulary tree structure
 */
struct VocabularyTree {
    std::vector<VocabNode> nodes;             ///< All tree nodes (BFS order)
    int branching_factor = 10;                ///< k in k-means at each level
    int depth = 6;                            ///< Tree depth
    int num_words = 0;                        ///< Number of visual words (leaf nodes)
    
    std::vector<int> leaf_node_ids;           ///< IDs of leaf nodes (visual words)
    std::unordered_map<int, int> node_id_to_index; ///< Node ID → index in nodes vector
};

/**
 * @brief Bag-of-words vector for an image
 */
struct BagOfWords {
    std::map<int, float> word_weights;        ///< Word ID → TF-IDF weight
    float norm = 0.0f;                        ///< L2 norm for normalization
    
    void normalize();
};

// ============================================================================
// Vocabulary Tree Training
// ============================================================================

/**
 * @brief Train hierarchical vocabulary tree using k-means
 * 
 * @param descriptors Training descriptors [N x 128]
 * @param branching_factor k in k-means (typical: 10)
 * @param depth Tree depth (typical: 6, gives 10^6 words)
 * @param max_iterations k-means iterations per level
 * @return Trained vocabulary tree
 */
VocabularyTree trainVocabularyTree(
    const std::vector<float>& descriptors,
    int branching_factor = 10,
    int depth = 6,
    int max_iterations = 25
);

/**
 * @brief Compute IDF weights for vocabulary tree
 * 
 * Updates idf_weight for each leaf node based on document frequency
 * 
 * @param tree Vocabulary tree (modified in-place)
 * @param image_descriptors Descriptors for each image in training set
 */
void computeIDFWeights(
    VocabularyTree& tree,
    const std::vector<std::vector<float>>& image_descriptors
);

// ============================================================================  
// Bag-of-Words Encoding
// ============================================================================

/**
 * @brief Transform image descriptors to bag-of-words vector
 * 
 * @param descriptors Image descriptors [N x 128]
 * @param tree Trained vocabulary tree
 * @return Bag-of-words representation with TF-IDF weights
 */
BagOfWords transformToBagOfWords(
    const std::vector<float>& descriptors,
    const VocabularyTree& tree
);

/**
 * @brief Traverse tree to find best matching visual word
 * 
 * @param descriptor Single descriptor [128]
 * @param tree Vocabulary tree
 * @return Visual word ID (leaf node ID)
 */
int findVisualWord(
    const std::vector<float>& descriptor,
    const VocabularyTree& tree
);

// ============================================================================
// Similarity Scoring
// ============================================================================

/**
 * @brief Compute L1-normalized score between two bag-of-words vectors
 * 
 * Uses L1-norm scoring as in DBoW2/DBoW3:
 * score = 1 - 0.5 * sum(|w1_i - w2_i|)
 * 
 * @param bow1 First bag-of-words
 * @param bow2 Second bag-of-words
 * @return Similarity score in [0, 1]
 */
float computeBagOfWordsScore(
    const BagOfWords& bow1,
    const BagOfWords& bow2
);

// ============================================================================
// File I/O
// ============================================================================

/**
 * @brief Save vocabulary tree to binary file
 * 
 * @param filepath Output file path (.voctree format)
 * @param tree Vocabulary tree
 * @return true on success
 */
bool saveVocabularyTree(
    const std::string& filepath,
    const VocabularyTree& tree
);

/**
 * @brief Load vocabulary tree from binary file
 * 
 * @param filepath Input file path (.voctree format)
 * @return Loaded vocabulary tree, empty if failed
 */
VocabularyTree loadVocabularyTree(const std::string& filepath);

/**
 * @brief Save bag-of-words to cache file
 * 
 * @param filepath Cache file path (.bow format)
 * @param bow Bag-of-words vector
 * @return true on success
 */
bool saveBagOfWordsCache(
    const std::string& filepath,
    const BagOfWords& bow
);

/**
 * @brief Load bag-of-words from cache file
 * 
 * @param filepath Cache file path (.bow format)
 * @return Loaded bag-of-words, empty if failed
 */
BagOfWords loadBagOfWordsCache(const std::string& filepath);

}  // namespace insight::algorithm::retrieval
