/**
 * @file  pca_whitening.h
 * @brief PCA+白化模型与训练/应用接口；可选 CUDA 训练见 pca_whitening_cuda.h。
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace insight {
namespace algorithm {
namespace retrieval {

struct PCAModel {
  Eigen::VectorXf mean;
  Eigen::MatrixXf components;
  Eigen::VectorXf explained_variance;
  int n_components = 0;
  int input_dim = 0;
  bool whiten = false;

  static PCAModel load(const std::string& filepath);
  bool save(const std::string& filepath) const;
  bool is_valid() const {
    return n_components > 0 && input_dim > 0 && mean.size() == input_dim &&
           components.rows() == n_components && components.cols() == input_dim;
  }
};

PCAModel train_pca(const std::vector<float>& vlad_vectors, int num_samples, int input_dim,
                  int n_components, bool whiten = false);
std::vector<float> apply_pca(const std::vector<float>& vlad, const PCAModel& model);
std::vector<float> apply_pca_batch(const std::vector<float>& vlad_vectors, int num_samples,
                                  const PCAModel& model);

}  // namespace retrieval
}  // namespace algorithm
}  // namespace insight
