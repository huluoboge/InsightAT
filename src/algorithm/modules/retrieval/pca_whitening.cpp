#include "pca_whitening.h"

#include <fstream>
#include <glog/logging.h>
#include <cstring>
#include <Eigen/Eigenvalues>

namespace insight {
namespace algorithm {
namespace retrieval {

// ============================================================================
// PCA Model Save/Load
// ============================================================================

PCAModel PCAModel::load(const std::string& filepath) {
    PCAModel model;
    
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs.is_open()) {
        LOG(ERROR) << "Failed to open PCA model file: " << filepath;
        return model;
    }
    
    // Read header
    uint32_t magic_number;
    uint32_t version;
    uint32_t n_comp;
    uint32_t in_dim;
    uint8_t whiten_flag;
    
    ifs.read(reinterpret_cast<char*>(&magic_number), sizeof(uint32_t));
    ifs.read(reinterpret_cast<char*>(&version), sizeof(uint32_t));
    ifs.read(reinterpret_cast<char*>(&n_comp), sizeof(uint32_t));
    ifs.read(reinterpret_cast<char*>(&in_dim), sizeof(uint32_t));
    ifs.read(reinterpret_cast<char*>(&whiten_flag), sizeof(uint8_t));
    
    // Skip reserved bytes (23 bytes to make header 32 bytes total)
    ifs.seekg(23, std::ios::cur);
    
    if (magic_number != 0x50434100) {  // "PCA\0"
        LOG(ERROR) << "Invalid PCA model file (bad magic number)";
        return model;
    }
    
    model.n_components = n_comp;
    model.input_dim = in_dim;
    model.whiten = (whiten_flag != 0);
    
    // Read mean vector
    model.mean.resize(in_dim);
    ifs.read(reinterpret_cast<char*>(model.mean.data()), in_dim * sizeof(float));
    
    // Read components matrix (row-major)
    model.components.resize(n_comp, in_dim);
    ifs.read(reinterpret_cast<char*>(model.components.data()), 
             n_comp * in_dim * sizeof(float));
    
    // Read explained variance
    model.explained_variance.resize(n_comp);
    ifs.read(reinterpret_cast<char*>(model.explained_variance.data()), 
             n_comp * sizeof(float));
    
    if (!ifs.good()) {
        LOG(ERROR) << "Failed to read PCA model data";
        return PCAModel();
    }
    
    LOG(INFO) << "Loaded PCA model: " << n_comp << " components, "
              << "input_dim=" << in_dim << ", whiten=" << model.whiten;
    
    return model;
}

bool PCAModel::save(const std::string& filepath) const {
    if (!isValid()) {
        LOG(ERROR) << "Cannot save invalid PCA model";
        return false;
    }
    
    std::ofstream ofs(filepath, std::ios::binary);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open file for writing: " << filepath;
        return false;
    }
    
    // Write header (32 bytes total)
    uint32_t magic_number = 0x50434100;  // "PCA\0"
    uint32_t version = 1;
    uint32_t n_comp = n_components;
    uint32_t in_dim = input_dim;
    uint8_t whiten_flag = whiten ? 1 : 0;
    
    ofs.write(reinterpret_cast<const char*>(&magic_number), sizeof(uint32_t));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(uint32_t));
    ofs.write(reinterpret_cast<const char*>(&n_comp), sizeof(uint32_t));
    ofs.write(reinterpret_cast<const char*>(&in_dim), sizeof(uint32_t));
    ofs.write(reinterpret_cast<const char*>(&whiten_flag), sizeof(uint8_t));
    
    // Write reserved bytes (23 bytes)
    char reserved[23] = {0};
    ofs.write(reserved, 23);
    
    // Write mean vector
    ofs.write(reinterpret_cast<const char*>(mean.data()), 
              input_dim * sizeof(float));
    
    // Write components matrix (row-major)
    ofs.write(reinterpret_cast<const char*>(components.data()), 
              n_components * input_dim * sizeof(float));
    
    // Write explained variance
    ofs.write(reinterpret_cast<const char*>(explained_variance.data()), 
              n_components * sizeof(float));
    
    if (!ofs.good()) {
        LOG(ERROR) << "Failed to write PCA model data";
        return false;
    }
    
    LOG(INFO) << "Saved PCA model to " << filepath 
              << " (" << (ofs.tellp() / 1024.0 / 1024.0) << " MB)";
    
    return true;
}

// ============================================================================
// PCA Training
// ============================================================================

PCAModel trainPCA(
    const std::vector<float>& vlad_vectors,
    int num_samples,
    int input_dim,
    int n_components,
    bool whiten
) {
    PCAModel model;
    
    if (num_samples == 0 || input_dim == 0 || n_components <= 0) {
        LOG(ERROR) << "Invalid PCA training parameters";
        return model;
    }
    
    if (n_components > input_dim) {
        LOG(WARNING) << "n_components (" << n_components 
                     << ") > input_dim (" << input_dim 
                     << "), clamping to input_dim";
        n_components = input_dim;
    }
    
    if (vlad_vectors.size() != static_cast<size_t>(num_samples * input_dim)) {
        LOG(ERROR) << "VLAD vectors size mismatch: expected " 
                   << (num_samples * input_dim) << ", got " 
                   << vlad_vectors.size();
        return model;
    }
    
    LOG(INFO) << "Training PCA: " << num_samples << " samples, "
              << input_dim << " dims -> " << n_components << " dims, "
              << "whiten=" << whiten;
    
    // Convert to Eigen matrix [num_samples x input_dim]
    Eigen::MatrixXf data = Eigen::Map<const Eigen::MatrixXf>(
        vlad_vectors.data(), input_dim, num_samples).transpose();
    
    // Step 1: Compute mean
    model.mean = data.colwise().mean();
    
    // Step 2: Center data
    Eigen::MatrixXf centered = data.rowwise() - model.mean.transpose();
    
    // Step 3: Compute covariance matrix [input_dim x input_dim]
    Eigen::MatrixXf cov = (centered.transpose() * centered) / (num_samples - 1);
    
    // Step 4: Eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> solver(cov);
    if (solver.info() != Eigen::Success) {
        LOG(ERROR) << "PCA eigen decomposition failed";
        return model;
    }
    
    // Eigenvalues and eigenvectors (sorted in ascending order by default)
    Eigen::VectorXf eigenvalues = solver.eigenvalues();
    Eigen::MatrixXf eigenvectors = solver.eigenvectors();
    
    // Step 5: Select top n_components (reverse to get descending order)
    model.components.resize(n_components, input_dim);
    model.explained_variance.resize(n_components);
    
    for (int i = 0; i < n_components; ++i) {
        int idx = input_dim - 1 - i;  // Reverse index (largest eigenvalues first)
        model.components.row(i) = eigenvectors.col(idx);
        model.explained_variance(i) = eigenvalues(idx);
    }
    
    model.n_components = n_components;
    model.input_dim = input_dim;
    model.whiten = whiten;
    
    // Compute variance retention
    float total_variance = eigenvalues.sum();
    float retained_variance = model.explained_variance.sum();
    float retention_ratio = retained_variance / total_variance;
    
    LOG(INFO) << "PCA training complete: variance retained = " 
              << (retention_ratio * 100.0f) << "%";
    
    return model;
}

// ============================================================================
// PCA Application
// ============================================================================

std::vector<float> applyPCA(
    const std::vector<float>& vlad,
    const PCAModel& model
) {
    if (!model.isValid()) {
        LOG(ERROR) << "Invalid PCA model";
        return {};
    }
    
    if (vlad.size() != static_cast<size_t>(model.input_dim)) {
        LOG(ERROR) << "VLAD dimension mismatch: expected " 
                   << model.input_dim << ", got " << vlad.size();
        return {};
    }
    
    // Convert to Eigen vector
    Eigen::Map<const Eigen::VectorXf> vlad_vec(vlad.data(), model.input_dim);
    
    // Step 1: Center
    Eigen::VectorXf centered = vlad_vec - model.mean;
    
    // Step 2: Project to principal components
    Eigen::VectorXf projected = model.components * centered;
    
    // Step 3: Whitening (if enabled)
    if (model.whiten) {
        for (int i = 0; i < model.n_components; ++i) {
            projected(i) /= std::sqrt(model.explained_variance(i) + 1e-10f);
        }
    }
    
    // Step 4: L2 normalize (standard for retrieval)
    float norm = projected.norm();
    if (norm > 1e-10f) {
        projected /= norm;
    }
    
    // Convert back to std::vector
    std::vector<float> result(model.n_components);
    Eigen::Map<Eigen::VectorXf>(result.data(), model.n_components) = projected;
    
    return result;
}

std::vector<float> applyPCABatch(
    const std::vector<float>& vlad_vectors,
    int num_samples,
    const PCAModel& model
) {
    if (!model.isValid()) {
        LOG(ERROR) << "Invalid PCA model";
        return {};
    }
    
    if (vlad_vectors.size() != static_cast<size_t>(num_samples * model.input_dim)) {
        LOG(ERROR) << "VLAD vectors size mismatch";
        return {};
    }
    
    std::vector<float> result;
    result.reserve(num_samples * model.n_components);
    
    for (int i = 0; i < num_samples; ++i) {
        std::vector<float> vlad(
            vlad_vectors.begin() + i * model.input_dim,
            vlad_vectors.begin() + (i + 1) * model.input_dim
        );
        
        auto projected = applyPCA(vlad, model);
        result.insert(result.end(), projected.begin(), projected.end());
    }
    
    return result;
}

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
