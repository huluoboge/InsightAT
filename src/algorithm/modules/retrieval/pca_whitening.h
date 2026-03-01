#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace insight {
namespace algorithm {
namespace retrieval {

/**
 * @brief PCA+Whitening model for dimensionality reduction
 */
struct PCAModel {
    Eigen::VectorXf mean;               // Mean vector [input_dim]
    Eigen::MatrixXf components;         // PCA components [n_components x input_dim]
    Eigen::VectorXf explained_variance; // Explained variance [n_components]
    int n_components;                   // Output dimensions
    int input_dim;                      // Input dimensions
    bool whiten;                        // Whether whitening is enabled
    
    PCAModel() : n_components(0), input_dim(0), whiten(false) {}
    
    /**
     * @brief Load PCA model from binary file
     * @param filepath Path to .pca file
     * @return PCAModel instance
     */
    static PCAModel load(const std::string& filepath);
    
    /**
     * @brief Save PCA model to binary file
     * @param filepath Path to .pca file
     * @return True if successful
     */
    bool save(const std::string& filepath) const;
    
    /**
     * @brief Check if model is valid
     */
    bool isValid() const {
        return n_components > 0 && input_dim > 0 && 
               mean.size() == input_dim &&
               components.rows() == n_components &&
               components.cols() == input_dim;
    }
};

/**
 * @brief Train PCA model from VLAD vectors
 * @param vlad_vectors Input VLAD vectors [N x D] as flat vector
 * @param num_samples Number of samples (N)
 * @param input_dim Input dimensionality (D)
 * @param n_components Target dimensionality
 * @param whiten Whether to apply whitening (variance normalization)
 * @return Trained PCA model
 */
PCAModel trainPCA(
    const std::vector<float>& vlad_vectors,
    int num_samples,
    int input_dim,
    int n_components,
    bool whiten = false
);

/**
 * @brief Apply PCA transformation to a VLAD vector
 * @param vlad Input VLAD vector [input_dim]
 * @param model PCA model
 * @return Transformed vector [n_components], L2-normalized
 */
std::vector<float> applyPCA(
    const std::vector<float>& vlad,
    const PCAModel& model
);

/**
 * @brief Batch apply PCA transformation
 * @param vlad_vectors Input VLAD vectors [N x input_dim] as flat vector
 * @param num_samples Number of samples (N)
 * @param model PCA model
 * @return Transformed vectors [N x n_components] as flat vector
 */
std::vector<float> applyPCABatch(
    const std::vector<float>& vlad_vectors,
    int num_samples,
    const PCAModel& model
);

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
