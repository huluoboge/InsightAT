/**
 * isat_train_vlad.cpp
 * InsightAT VLAD Codebook Training Tool
 *
 * Samples descriptors from .isat_feat files in a directory, runs k-means
 * to obtain VLAD centroids, and optionally trains a PCA model on VLAD
 * vectors for dimensionality reduction. Outputs a binary codebook and
 * optional PCA model for use with isat_retrieve --strategy vlad.
 *
 * Usage:
 *   isat_train_vlad -f feat_dir/ -o codebook.bin -k 64
 *   isat_train_vlad -f feat_dir/ -o codebook.bin -k 64 -P pca.bin --pca-dims 128
 */

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>

#include "../io/idc_reader.h"
#include "../modules/matching/match_types.h"
#include "../modules/retrieval/pca_whitening.h"
#include "../modules/retrieval/vlad_encoding.h"
#include "cli_logging.h"
#include "cmdLine/cmdLine.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::algorithm::retrieval;
using namespace insight::io;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void printEvent(const json& j) {
  std::cout << kEventPrefix << j.dump() << "\n";
  std::cout.flush();
}

/**
 * Sample descriptors from multiple feature files
 */
std::vector<float> sampleDescriptorsMultiFile(const std::vector<std::string>& feature_files,
                                              int max_descriptors_per_file, int& total_sampled) {
  std::vector<float> all_descriptors;
  total_sampled = 0;

  std::random_device rd;
  std::mt19937 gen(rd());

  for (const auto& file : feature_files) {
    IDCReader reader(file);
    if (!reader.isValid()) {
      LOG(WARNING) << "Skipping invalid file: " << file;
      continue;
    }

    // Read descriptors
    auto desc_blob = reader.getBlobDescriptor("descriptors");
    std::string dtype = desc_blob["dtype"];

    std::vector<float> descriptors;
    if (dtype == "float32") {
      descriptors = reader.readBlob<float>("descriptors");
    } else if (dtype == "uint8") {
      // uint8 descriptors were scaled by 512 during storage, need to reverse
      auto desc_uint8 = reader.readBlob<uint8_t>("descriptors");
      descriptors.resize(desc_uint8.size());
      for (size_t i = 0; i < desc_uint8.size(); ++i) {
        descriptors[i] = static_cast<float>(desc_uint8[i]) / 512.0f;
      }
    } else {
      LOG(WARNING) << "Unsupported descriptor type in " << file;
      continue;
    }

    int num_features = descriptors.size() / 128;

    // Sample or take all
    if (num_features <= max_descriptors_per_file) {
      all_descriptors.insert(all_descriptors.end(), descriptors.begin(), descriptors.end());
      total_sampled += num_features;
    } else {
      // Random sampling
      std::vector<int> indices(num_features);
      std::iota(indices.begin(), indices.end(), 0);
      std::shuffle(indices.begin(), indices.end(), gen);

      for (int i = 0; i < max_descriptors_per_file; ++i) {
        int idx = indices[i];
        all_descriptors.insert(all_descriptors.end(), descriptors.begin() + idx * 128,
                               descriptors.begin() + (idx + 1) * 128);
      }
      total_sampled += max_descriptors_per_file;
    }
  }

  return all_descriptors;
}

/**
 * Save centroids to binary file
 */
bool saveCentroids(const std::string& filepath, const std::vector<float>& centroids,
                   int num_clusters) {
  std::ofstream ofs(filepath, std::ios::binary);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Failed to open file for writing: " << filepath;
    return false;
  }

  // Write header: magic number + version + num_clusters + descriptor_dim
  uint32_t magic = 0x56434254; // ASCII "VCBT" (Visual Codebook)
  uint32_t version = 1;
  uint32_t num_clusters_u32 = num_clusters;
  uint32_t descriptor_dim = 128;

  ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
  ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
  ofs.write(reinterpret_cast<const char*>(&num_clusters_u32), sizeof(num_clusters_u32));
  ofs.write(reinterpret_cast<const char*>(&descriptor_dim), sizeof(descriptor_dim));

  // Write centroids
  ofs.write(reinterpret_cast<const char*>(centroids.data()), centroids.size() * sizeof(float));

  return ofs.good();
}

/**
 * Load centroids from binary file
 */
std::vector<float> loadCentroids(const std::string& filepath, int& num_clusters) {
  std::ifstream ifs(filepath, std::ios::binary);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Failed to open file: " << filepath;
    return {};
  }

  // Read header
  uint32_t magic = 0;
  uint32_t version = 0;
  uint32_t num_clusters_u32 = 0;
  uint32_t descriptor_dim = 0;

  ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic));
  ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
  ifs.read(reinterpret_cast<char*>(&num_clusters_u32), sizeof(num_clusters_u32));
  ifs.read(reinterpret_cast<char*>(&descriptor_dim), sizeof(descriptor_dim));

  if (magic != 0x56434254) {
    LOG(ERROR) << "Invalid codebook file (wrong magic number)";
    return {};
  }

  if (descriptor_dim != 128) {
    LOG(ERROR) << "Unsupported descriptor dimension: " << descriptor_dim;
    return {};
  }

  num_clusters = num_clusters_u32;

  // Read centroids
  std::vector<float> centroids(num_clusters * descriptor_dim);
  ifs.read(reinterpret_cast<char*>(centroids.data()), centroids.size() * sizeof(float));

  if (!ifs.good()) {
    LOG(ERROR) << "Failed to read centroids from file";
    return {};
  }

  return centroids;
}

int main(int argc, char* argv[]) {
  // Initialize logging
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  // Command-line options
  CmdLine cmd("InsightAT VLAD Codebook Training - Train k-means visual vocabulary");

  std::string feature_dir;
  std::string output_file;
  std::string pca_output_file;
  int num_clusters = 64;
  int max_descriptors = 1000000;
  int max_per_image = 500;
  int max_iterations = 100;
  int pca_dims = 256;
  float target_scale = 4.0f;
  float scale_sigma = 2.0f;

  // Required arguments
  cmd.add(make_option('f', feature_dir, "features")
              .doc("Feature directory containing .isat_feat files"));
  cmd.add(make_option('o', output_file, "output").doc("Output codebook file (.vcbt format)"));

  // Optional parameters
  cmd.add(
      make_option('k', num_clusters, "clusters").doc("Number of k-means clusters (default: 64)"));
  cmd.add(make_option('n', max_descriptors, "max-descriptors")
              .doc("Maximum total descriptors for training (default: 1M)"));
  cmd.add(make_option('p', max_per_image, "max-per-image")
              .doc("Maximum descriptors per image (default: 500)"));
  cmd.add(
      make_option('i', max_iterations, "iterations").doc("k-means max iterations (default: 100)"));

  // PCA parameters
  cmd.add(make_option('P', pca_output_file, "pca-output")
              .doc("Output PCA model file (.pca format, optional)"));
  cmd.add(make_option('d', pca_dims, "pca-dims").doc("PCA output dimensions (default: 256)"));

  // Scale weighting parameters
  cmd.add(make_option('t', target_scale, "target-scale")
              .doc("Target scale for weighting (default: 4.0)"));
  cmd.add(make_option('s', scale_sigma, "scale-sigma")
              .doc("Gaussian sigma for scale weighting (default: 2.0)"));

  // Switches
  cmd.add(make_switch('w', "whiten").doc("Enable PCA whitening (variance normalization)"));
  cmd.add(make_switch('S', "scale-weighted").doc("Enable scale-weighted VLAD encoding"));

  // Logging options
  std::string log_level;
  cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
  cmd.add(make_switch('v', "verbose").doc("Verbose logging (INFO level)"));
  cmd.add(make_switch('q', "quiet").doc("Quiet mode (ERROR level only)"));
  cmd.add(make_switch('h', "help").doc("Show this help message"));

  // Parse command line
  try {
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  if (cmd.checkHelp(argv[0]))
    return 0;

  // Validate required arguments
  if (feature_dir.empty() || output_file.empty()) {
    std::cerr << "Error: -f/--features and -o/--output are required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 1;
  }

  // Set logging level
  insight::tools::ApplyLogLevel(cmd.used('v'), cmd.used('q'), log_level);

  LOG(INFO) << "=== VLAD Codebook Training ===";
  LOG(INFO) << "Feature directory: " << feature_dir;
  LOG(INFO) << "Output file: " << output_file;
  LOG(INFO) << "Clusters: " << num_clusters;
  LOG(INFO) << "Max descriptors: " << max_descriptors;
  LOG(INFO) << "Max per image: " << max_per_image;

  // Collect all feature files
  std::vector<std::string> feature_files;
  for (const auto& entry : fs::directory_iterator(feature_dir)) {
    if (entry.path().extension() == ".isat_feat") {
      feature_files.push_back(entry.path().string());
    }
  }

  if (feature_files.empty()) {
    LOG(ERROR) << "No .isat_feat files found in " << feature_dir;
    return 1;
  }

  LOG(INFO) << "Found " << feature_files.size() << " feature files";

  // Sample descriptors
  auto start = std::chrono::high_resolution_clock::now();

  int total_sampled = 0;
  auto descriptors = sampleDescriptorsMultiFile(feature_files, max_per_image, total_sampled);

  auto end_sample = std::chrono::high_resolution_clock::now();
  auto sample_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_sample - start).count();

  LOG(INFO) << "Sampled " << total_sampled << " descriptors from " << feature_files.size()
            << " files in " << sample_ms << "ms";

  if (descriptors.empty()) {
    LOG(ERROR) << "Failed to sample descriptors";
    return 1;
  }

  // Limit total descriptors if needed
  if (total_sampled > max_descriptors) {
    LOG(INFO) << "Downsampling from " << total_sampled << " to " << max_descriptors;
    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<int> indices(total_sampled);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), gen);

    std::vector<float> downsampled(max_descriptors * 128);
    for (int i = 0; i < max_descriptors; ++i) {
      int idx = indices[i];
      std::copy(descriptors.begin() + idx * 128, descriptors.begin() + (idx + 1) * 128,
                downsampled.begin() + i * 128);
    }
    descriptors = std::move(downsampled);
    total_sampled = max_descriptors;
  }

  // Train k-means
  LOG(INFO) << "Training k-means with " << total_sampled << " descriptors...";
  auto centroids = train_k_means(descriptors, num_clusters, max_iterations);

  auto end_train = std::chrono::high_resolution_clock::now();
  auto train_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_train - end_sample).count();

  if (centroids.empty()) {
    LOG(ERROR) << "k-means training failed";
    return 1;
  }

  LOG(INFO) << "k-means training complete in " << train_ms << "ms";

  // Save codebook
  if (!saveCentroids(output_file, centroids, num_clusters)) {
    LOG(ERROR) << "Failed to save codebook to " << output_file;
    return 1;
  }

  LOG(INFO) << "Saved codebook to " << output_file;

  // ========================================================================
  // PCA Training (Optional)
  // ========================================================================

  bool enable_pca = !pca_output_file.empty();
  bool enable_whiten = cmd.used('w');
  int vlad_dim = num_clusters * 128; // VLAD dimension

  if (enable_pca) {
    LOG(INFO) << "=== PCA Training ===";
    LOG(INFO) << "PCA output: " << pca_output_file;
    LOG(INFO) << "PCA dimensions: " << pca_dims;
    LOG(INFO) << "Whitening: " << (enable_whiten ? "enabled" : "disabled");

    // Encode all feature files as VLAD vectors
    LOG(INFO) << "Encoding VLAD vectors from " << feature_files.size() << " files...";

    std::vector<float> all_vlad_vectors;
    int num_encoded = 0;

    auto start_encode = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < feature_files.size(); ++i) {
      const auto& file = feature_files[i];

      // Read descriptors
      IDCReader reader(file);
      if (!reader.isValid()) {
        LOG(WARNING) << "Skipping invalid file: " << file;
        continue;
      }

      auto desc_blob = reader.getBlobDescriptor("descriptors");
      std::string dtype = desc_blob["dtype"];

      std::vector<float> desc;
      if (dtype == "float32") {
        desc = reader.readBlob<float>("descriptors");
      } else if (dtype == "uint8") {
        auto desc_uint8 = reader.readBlob<uint8_t>("descriptors");
        desc.resize(desc_uint8.size());
        for (size_t j = 0; j < desc_uint8.size(); ++j) {
          desc[j] = static_cast<float>(desc_uint8[j]) / 512.0f;
        }
      } else {
        LOG(WARNING) << "Unsupported descriptor type in " << file;
        continue;
      }

      if (desc.empty()) {
        LOG(WARNING) << "No descriptors in " << file;
        continue;
      }

      // Encode VLAD (with or without scale weighting)
      std::vector<float> vlad;
      if (cmd.used('S')) {
        // Load keypoints for scale weighting
        auto keypoints = reader.readBlob<float>("keypoints");
        if (keypoints.empty()) {
          LOG(WARNING) << "No keypoints for scale weighting in " << file;
          continue;
        }

        auto scales = extract_scales(keypoints);
        if (scales.empty()) {
          LOG(WARNING) << "Failed to extract scales from " << file;
          continue;
        }

        vlad = encode_vlad_scale_weighted(desc, scales, centroids, num_clusters, target_scale,
                                       scale_sigma);
      } else {
        vlad = encode_vlad(desc, centroids, num_clusters);
      }

      if (vlad.size() != static_cast<size_t>(vlad_dim)) {
        LOG(WARNING) << "VLAD encoding failed for " << file;
        continue;
      }

      all_vlad_vectors.insert(all_vlad_vectors.end(), vlad.begin(), vlad.end());
      num_encoded++;

      if ((i + 1) % 100 == 0 || i == feature_files.size() - 1) {
        LOG(INFO) << "Encoded " << (i + 1) << "/" << feature_files.size() << " files";
      }
    }

    auto end_encode = std::chrono::high_resolution_clock::now();
    auto encode_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_encode - start_encode).count();

    LOG(INFO) << "Encoded " << num_encoded << " VLAD vectors in " << encode_ms << "ms";

    if (num_encoded == 0) {
      LOG(ERROR) << "No VLAD vectors encoded, PCA training aborted";
      return 1;
    }

    // Train PCA
    LOG(INFO) << "Training PCA model...";
    auto pca_model = train_pca(all_vlad_vectors, num_encoded, vlad_dim, pca_dims, enable_whiten);

    auto end_pca = std::chrono::high_resolution_clock::now();
    auto pca_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_pca - end_encode).count();

    if (!pca_model.is_valid()) {
      LOG(ERROR) << "PCA training failed";
      return 1;
    }

    LOG(INFO) << "PCA training complete in " << pca_ms << "ms";

    // Save PCA model
    if (!pca_model.save(pca_output_file)) {
      LOG(ERROR) << "Failed to save PCA model to " << pca_output_file;
      return 1;
    }

    LOG(INFO) << "Saved PCA model to " << pca_output_file;
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_total - start).count();

  LOG(INFO) << "=== Training Complete ===";
  LOG(INFO) << "Total time: " << total_ms << "ms";
  LOG(INFO) << "Clusters: " << num_clusters;
  LOG(INFO) << "Training samples: " << total_sampled;
  if (enable_pca) {
    LOG(INFO) << "PCA dimensions: " << vlad_dim << " -> " << pca_dims;
    LOG(INFO) << "Compression ratio: " << (static_cast<float>(vlad_dim) / pca_dims) << "x";
  }

  json data = {{"output_file", output_file},
               {"num_clusters", num_clusters},
               {"total_sampled", total_sampled},
               {"total_ms", total_ms}};
  if (enable_pca) {
    data["pca_output_file"] = pca_output_file;
    data["pca_dims"] = pca_dims;
    data["vlad_dim"] = vlad_dim;
  }
  printEvent({{"type", "train_vlad.complete"}, {"ok", true}, {"data", data}});

  return 0;
}
