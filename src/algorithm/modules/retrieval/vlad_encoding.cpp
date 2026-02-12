#include "vlad_encoding.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "../matching/match_types.h"
#include "../../io/idc_reader.h"

namespace insight {
namespace algorithm {
namespace retrieval {

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

using namespace insight::io;

/**
 * Load descriptors from .isat_feat file
 */
std::vector<float> loadDescriptorsFromFile(const std::string& feature_file) {
    
    IDCReader reader(feature_file);
    if (!reader.isValid()) {
        LOG(ERROR) << "Invalid feature file: " << feature_file;
        return {};
    }
    
    auto desc_blob = reader.getBlobDescriptor("descriptors");
    std::string dtype = desc_blob["dtype"];
    
    if (dtype == "float32") {
        return reader.readBlob<float>("descriptors");
    } else if (dtype == "uint8") {
        // Convert uint8 to float32
        auto desc_uint8 = reader.readBlob<uint8_t>("descriptors");
        std::vector<float> desc_float(desc_uint8.size());
        for (size_t i = 0; i < desc_uint8.size(); ++i) {
            desc_float[i] = static_cast<float>(desc_uint8[i]);
        }
        return desc_float;
    } else {
        LOG(ERROR) << "Unsupported descriptor dtype: " << dtype;
        return {};
    }
}

}  // anonymous namespace

// ============================================================================
// k-means Clustering
// ============================================================================

std::vector<int> assignToClusters(
    const std::vector<float>& descriptors,
    const std::vector<float>& centroids,
    int descriptor_dim
) {
    if (descriptors.empty() || centroids.empty()) {
        return {};
    }
    
    int num_descriptors = descriptors.size() / descriptor_dim;
    int num_clusters = centroids.size() / descriptor_dim;
    
    std::vector<int> assignments(num_descriptors);
    
    // For each descriptor, find nearest centroid
    #pragma omp parallel for
    for (int i = 0; i < num_descriptors; ++i) {
        float min_dist = std::numeric_limits<float>::max();
        int best_cluster = 0;
        
        const float* desc = &descriptors[i * descriptor_dim];
        
        for (int k = 0; k < num_clusters; ++k) {
            const float* centroid = &centroids[k * descriptor_dim];
            
            // Compute L2 distance
            float dist = 0.0f;
            for (int d = 0; d < descriptor_dim; ++d) {
                float diff = desc[d] - centroid[d];
                dist += diff * diff;
            }
            
            if (dist < min_dist) {
                min_dist = dist;
                best_cluster = k;
            }
        }
        
        assignments[i] = best_cluster;
    }
    
    return assignments;
}

std::vector<float> trainKMeans(
    const std::vector<float>& descriptors,
    int num_clusters,
    int max_iterations,
    float convergence_threshold
) {
    if (descriptors.empty() || num_clusters <= 0) {
        LOG(ERROR) << "Invalid input for k-means training";
        return {};
    }
    
    const int descriptor_dim = 128;
    int num_descriptors = descriptors.size() / descriptor_dim;
    
    if (num_descriptors < num_clusters) {
        LOG(WARNING) << "Too few descriptors (" << num_descriptors 
                     << ") for " << num_clusters << " clusters";
        num_clusters = num_descriptors;
    }
    
    LOG(INFO) << "Training k-means: " << num_descriptors << " descriptors, "
              << num_clusters << " clusters, max_iter=" << max_iterations;
    
    // Convert to OpenCV Mat
    cv::Mat samples(num_descriptors, descriptor_dim, CV_32F);
    std::memcpy(samples.data, descriptors.data(), descriptors.size() * sizeof(float));
    
    // Run k-means
    cv::Mat labels;
    cv::Mat centers;
    
    cv::TermCriteria criteria(
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
        max_iterations,
        convergence_threshold
    );
    
    cv::kmeans(
        samples,
        num_clusters,
        labels,
        criteria,
        3,  // attempts
        cv::KMEANS_PP_CENTERS  // Use k-means++ initialization
    );
    
    // Convert centers back to std::vector
    std::vector<float> centroids(num_clusters * descriptor_dim);
    std::memcpy(centroids.data(), centers.data, centroids.size() * sizeof(float));
    
    LOG(INFO) << "k-means training complete";
    
    return centroids;
}

// ============================================================================
// VLAD Encoding
// ============================================================================

std::vector<float> encodeVLAD(
    const std::vector<float>& descriptors,
    const std::vector<float>& centroids,
    int num_clusters
) {
    if (descriptors.empty() || centroids.empty()) {
        return {};
    }
    
    const int descriptor_dim = 128;
    int num_descriptors = descriptors.size() / descriptor_dim;
    
    // Initialize VLAD vector (K x D)
    std::vector<float> vlad(num_clusters * descriptor_dim, 0.0f);
    
    // Assign descriptors to clusters
    auto assignments = assignToClusters(descriptors, centroids, descriptor_dim);
    
    // Accumulate residuals
    for (int i = 0; i < num_descriptors; ++i) {
        int cluster_id = assignments[i];
        const float* desc = &descriptors[i * descriptor_dim];
        const float* centroid = &centroids[cluster_id * descriptor_dim];
        float* vlad_cluster = &vlad[cluster_id * descriptor_dim];
        
        // Add residual (descriptor - centroid)
        for (int d = 0; d < descriptor_dim; ++d) {
            vlad_cluster[d] += desc[d] - centroid[d];
        }
    }
    
    // L2 normalization
    normalizeL2(vlad);
    
    return vlad;
}

void normalizeL2(std::vector<float>& vec) {
    float norm = 0.0f;
    for (float val : vec) {
        norm += val * val;
    }
    norm = std::sqrt(norm);
    
    if (norm > 1e-12f) {
        for (float& val : vec) {
            val /= norm;
        }
    }
}

float computeL2Distance(
    const std::vector<float>& vec1,
    const std::vector<float>& vec2
) {
    if (vec1.size() != vec2.size()) {
        LOG(ERROR) << "Vector size mismatch: " << vec1.size() << " vs " << vec2.size();
        return std::numeric_limits<float>::max();
    }
    
    float dist = 0.0f;
    for (size_t i = 0; i < vec1.size(); ++i) {
        float diff = vec1[i] - vec2[i];
        dist += diff * diff;
    }
    
    return std::sqrt(dist);
}

// ============================================================================
// Caching
// ============================================================================

std::vector<float> loadVLADCache(const std::string& cache_path) {
    std::ifstream ifs(cache_path, std::ios::binary);
    if (!ifs.is_open()) {
        return {};
    }
    
    // Read header: magic number + version + size
    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t vlad_size = 0;
    
    ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    ifs.read(reinterpret_cast<char*>(&vlad_size), sizeof(vlad_size));
    
    // Validate magic number (ASCII "VLAD")
    if (magic != 0x44414C56) {
        LOG(WARNING) << "Invalid VLAD cache file: " << cache_path;
        return {};
    }
    
    // Read VLAD vector
    std::vector<float> vlad(vlad_size);
    ifs.read(reinterpret_cast<char*>(vlad.data()), vlad_size * sizeof(float));
    
    if (!ifs.good()) {
        LOG(WARNING) << "Failed to read VLAD cache: " << cache_path;
        return {};
    }
    
    return vlad;
}

bool saveVLADCache(
    const std::string& cache_path,
    const std::vector<float>& vlad_vector
) {
    std::ofstream ofs(cache_path, std::ios::binary);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open cache file for writing: " << cache_path;
        return false;
    }
    
    // Write header
    uint32_t magic = 0x44414C56;  // ASCII "VLAD"
    uint32_t version = 1;
    uint32_t vlad_size = vlad_vector.size();
    
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&vlad_size), sizeof(vlad_size));
    
    // Write VLAD vector
    ofs.write(reinterpret_cast<const char*>(vlad_vector.data()), 
              vlad_size * sizeof(float));
    
    return ofs.good();
}

std::vector<float> loadOrComputeVLAD(
    const std::string& feature_file,
    const std::string& cache_file,
    const std::vector<float>& centroids,
    int num_clusters,
    bool force_recompute
) {
    // Try to load from cache first
    if (!force_recompute) {
        auto vlad = loadVLADCache(cache_file);
        if (!vlad.empty()) {
            return vlad;
        }
    }
    
    // Load descriptors from feature file
    auto descriptors = loadDescriptorsFromFile(feature_file);
    if (descriptors.empty()) {
        LOG(ERROR) << "Failed to load descriptors from " << feature_file;
        return {};
    }
    
    // Encode VLAD
    auto vlad = encodeVLAD(descriptors, centroids, num_clusters);
    
    // Save to cache
    if (!vlad.empty()) {
        if (!saveVLADCache(cache_file, vlad)) {
            LOG(WARNING) << "Failed to save VLAD cache: " << cache_file;
        }
    }
    
    return vlad;
}

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
