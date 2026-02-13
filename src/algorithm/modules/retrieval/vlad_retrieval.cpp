#include "vlad_retrieval.h"
#include "vlad_encoding.h"
#include "pca_whitening.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <glog/logging.h>

namespace fs = std::filesystem;

namespace insight::algorithm::retrieval {

// ============================================================================
// VLAD Similarity
// ============================================================================

double computeVLADScore(double distance, double sigma) {
    // Exponential decay: closer vectors have higher scores
    return std::exp(-distance / sigma);
}

std::vector<std::tuple<int, int, float>> findTopKSimilar(
    const std::vector<std::vector<float>>& vlad_vectors,
    int top_k
) {
    int num_images = vlad_vectors.size();
    std::vector<std::tuple<int, int, float>> results;
    results.reserve(num_images * top_k);
    
    // For each query image
    for (int i = 0; i < num_images; ++i) {
        // Compute distances to all other images
        std::vector<std::pair<float, int>> distances;
        distances.reserve(num_images - 1);
        
        for (int j = 0; j < num_images; ++j) {
            if (i == j) continue;
            
            float dist = computeL2Distance(vlad_vectors[i], vlad_vectors[j]);
            distances.push_back({dist, j});
        }
        
        // Partial sort to get top-k smallest distances
        int k = std::min(top_k, static_cast<int>(distances.size()));
        std::partial_sort(
            distances.begin(),
            distances.begin() + k,
            distances.end()
        );
        
        // Add top-k pairs
        for (int t = 0; t < k; ++t) {
            results.push_back({i, distances[t].second, distances[t].first});
        }
    }
    
    return results;
}

// ============================================================================
// VLAD Retrieval Strategy
// ============================================================================

std::vector<ImagePair> retrieveByVLAD(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options,
    const std::vector<float>& centroids,
    const std::string& cache_dir,
    const PCAModel* pca_model,
    bool scale_weighted,
    float target_scale,
    float scale_sigma
) {
    if (images.empty()) {
        LOG(WARNING) << "No images provided for VLAD retrieval";
        return {};
    }
    
    if (centroids.empty()) {
        LOG(ERROR) << "VLAD centroids not provided";
        return {};
    }
    
    int num_clusters = options.vlad_clusters;
    int descriptor_dim = 128;
    
    // Validate centroids size
    if (centroids.size() != static_cast<size_t>(num_clusters * descriptor_dim)) {
        LOG(ERROR) << "Invalid centroids size: expected " 
                   << num_clusters * descriptor_dim << ", got " << centroids.size();
        return {};
    }
    
    bool use_pca = (pca_model != nullptr && pca_model->isValid());
    int final_dim = use_pca ? pca_model->n_components : (num_clusters * descriptor_dim);
    
    LOG(INFO) << "VLAD retrieval: " << images.size() << " images, "
              << num_clusters << " clusters, top-k=" << options.top_k;
    if (use_pca) {
        LOG(INFO) << "PCA enabled: " << (num_clusters * descriptor_dim) 
                  << " -> " << final_dim << " dimensions";
    }
    if (scale_weighted) {
        LOG(INFO) << "Scale weighting enabled: target=" << target_scale 
                  << ", sigma=" << scale_sigma;
    }
    
    // Load or compute VLAD vectors for all images
    std::vector<std::vector<float>> vlad_vectors;
    vlad_vectors.reserve(images.size());
    
    for (size_t i = 0; i < images.size(); ++i) {
        const auto& img = images[i];
        
        // Determine cache file path
        std::string cache_file;
        if (!cache_dir.empty()) {
            fs::path cache_path = fs::path(cache_dir) / (img.image_id + ".isat_vlad");
            cache_file = cache_path.string();
        }
        
        // Load or compute VLAD
        auto vlad = loadOrComputeVLAD(
            img.feature_file,
            cache_file,
            centroids,
            num_clusters,
            false,  // use cache if available
            scale_weighted,
            target_scale,
            scale_sigma
        );
        
        if (vlad.empty()) {
            LOG(WARNING) << "Failed to compute VLAD for " << img.image_id;
            vlad.resize(num_clusters * descriptor_dim, 0.0f);
        }
        
        // Apply PCA if provided
        if (use_pca) {
            vlad = applyPCA(vlad, *pca_model);
            if (vlad.empty() || vlad.size() != static_cast<size_t>(final_dim)) {
                LOG(WARNING) << "PCA transformation failed for " << img.image_id;
                vlad.resize(final_dim, 0.0f);
            }
        }
        
        vlad_vectors.push_back(std::move(vlad));
    }
    
    LOG(INFO) << "VLAD encoding complete for " << vlad_vectors.size() << " images";
    
    // Find top-k similar pairs
    auto similar_pairs = findTopKSimilar(vlad_vectors, options.top_k);
    
    // Convert to ImagePair format
    std::vector<ImagePair> pairs;
    pairs.reserve(similar_pairs.size());
    
    for (const auto& [idx1, idx2, distance] : similar_pairs) {
        ImagePair pair;
        pair.image1_idx = idx1;
        pair.image2_idx = idx2;
        pair.method = "vlad";
        
        // Convert distance to similarity score
        double similarity = computeVLADScore(distance, 1.0);
        pair.score = similarity;
        pair.visual_similarity = similarity;
        
        if (pair.isValid()) {
            pairs.push_back(pair);
        }
    }
    
    LOG(INFO) << "VLAD retrieval: generated " << pairs.size() << " pairs from "
              << images.size() << " images";
    
    return pairs;
}

}  // namespace insight::algorithm::retrieval
