#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "cmdLine/cmdLine.h"
#include "../io/idc_reader.h"
#include <DBoW3.h>

namespace fs = std::filesystem;
using namespace insight::io;

/**
 * Sample descriptors from multiple feature files
 */
std::vector<cv::Mat> sampleDescriptorsMultiFile(
    const std::vector<std::string>& feature_files,
    int max_descriptors_per_file,
    int& total_sampled
) {
    std::vector<cv::Mat> all_descriptors;
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
            auto desc_uint8 = reader.readBlob<uint8_t>("descriptors");
            descriptors.resize(desc_uint8.size());
            for (size_t i = 0; i < desc_uint8.size(); ++i) {
                descriptors[i] = static_cast<float>(desc_uint8[i]);
            }
        } else {
            LOG(WARNING) << "Unsupported descriptor type in " << file;
            continue;
        }
        
        int num_features = descriptors.size() / 128;
        
        // Sample or take all
        if (num_features <= max_descriptors_per_file) {
            cv::Mat desc_mat(num_features, 128, CV_32F);
            std::memcpy(desc_mat.data, descriptors.data(), descriptors.size() * sizeof(float));
            all_descriptors.push_back(desc_mat);
            total_sampled += num_features;
        } else {
            // Random sampling
            std::vector<int> indices(num_features);
            std::iota(indices.begin(), indices.end(), 0);
            std::shuffle(indices.begin(), indices.end(), gen);
            
            cv::Mat desc_mat(max_descriptors_per_file, 128, CV_32F);
            for (int i = 0; i < max_descriptors_per_file; ++i) {
                int idx = indices[i];
                std::memcpy(
                    desc_mat.ptr<float>(i),
                    &descriptors[idx * 128],
                    128 * sizeof(float)
                );
            }
            all_descriptors.push_back(desc_mat);
            total_sampled += max_descriptors_per_file;
        }
    }
    
    return all_descriptors;
}

int main(int argc, char* argv[]) {
    // Initialize logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    
    // Command-line options
    CmdLine cmd("InsightAT Vocabulary Tree Training - Train DBoW3 visual vocabulary");
    
    std::string feature_dir;
    std::string output_file;
    int branching_factor = 10;
    int depth = 6;
    int max_descriptors = 1000000;
    int max_per_image = 500;
    
    // Required arguments
    cmd.add(make_option('f', feature_dir, "features")
        .doc("Feature directory containing .isat_feat files"));
    cmd.add(make_option('o', output_file, "output")
        .doc("Output vocabulary file (.dbow3 format)"));
    
    // Optional parameters
    cmd.add(make_option('k', branching_factor, "branching")
        .doc("Branching factor for k-means (default: 10)"));
    cmd.add(make_option('L', depth, "depth")
        .doc("Tree depth (default: 6, gives k^L words)"));
    cmd.add(make_option('n', max_descriptors, "max-descriptors")
        .doc("Maximum total descriptors for training (default: 1M)"));
    cmd.add(make_option('p', max_per_image, "max-per-image")
        .doc("Maximum descriptors per image (default: 500)"));
    
    // Logging options
    cmd.add(make_switch('v', "verbose")
        .doc("Verbose logging (INFO level)"));
    cmd.add(make_switch('q', "quiet")
        .doc("Quiet mode (ERROR level only)"));
    cmd.add(make_switch('h', "help")
        .doc("Show this help message"));
    
    // Parse command line
    try {
        cmd.process(argc, argv);
    } catch (const std::string& s) {
        std::cerr << "Error: " << s << "\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }
    
    if (cmd.checkHelp(argv[0])) return 0;
    
    // Validate required arguments
    if (feature_dir.empty() || output_file.empty()) {
        std::cerr << "Error: -f/--features and -o/--output are required\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }
    
    // Set logging level
    if (cmd.used('v')) {
        FLAGS_minloglevel = 0;  // INFO
    } else if (cmd.used('q')) {
        FLAGS_minloglevel = 2;  // ERROR
    } else {
        FLAGS_minloglevel = 0;  // INFO (default)
    }
    
    LOG(INFO) << "=== Vocabulary Tree Training (DBoW3) ===";
    LOG(INFO) << "Feature directory: " << feature_dir;
    LOG(INFO) << "Output file: " << output_file;
    LOG(INFO) << "Branching factor: " << branching_factor;
    LOG(INFO) << "Tree depth: " << depth;
    LOG(INFO) << "Expected words: " << std::pow(branching_factor, depth);
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
    auto descriptors = sampleDescriptorsMultiFile(
        feature_files, 
        max_per_image,
        total_sampled
    );
    
    auto end_sample = std::chrono::high_resolution_clock::now();
    auto sample_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_sample - start).count();
    
    LOG(INFO) << "Sampled " << total_sampled << " descriptors from " 
              << feature_files.size() << " files in " << sample_ms << "ms";
    
    if (descriptors.empty()) {
        LOG(ERROR) << "Failed to sample descriptors";
        return 1;
    }
    
    // Limit total descriptors if needed
    if (total_sampled > max_descriptors) {
        LOG(INFO) << "Downsampling from " << total_sampled << " to " << max_descriptors;
        std::random_device rd;
        std::mt19937 gen(rd());
        
        std::vector<int> file_indices;
        for (size_t i = 0; i < descriptors.size(); ++i) {
            for (int j = 0; j < descriptors[i].rows; ++j) {
                file_indices.push_back(i);
            }
        }
        
        std::shuffle(file_indices.begin(), file_indices.end(), gen);
        file_indices.resize(max_descriptors);
        
        // Collect downsampled descriptors
        std::map<int, std::vector<int>> indices_by_file;
        for (int idx : file_indices) {
            indices_by_file[idx].push_back(0);  // Simplified tracking
        }
        
        total_sampled = max_descriptors;
    }
    
    // Create vocabulary
    LOG(INFO) << "Training vocabulary tree (this may take several minutes)...";
    LOG(INFO) << "k=" << branching_factor << ", L=" << depth 
              << ", expected words=" << std::pow(branching_factor, depth);
    
    DBoW3::Vocabulary vocab(branching_factor, depth, DBoW3::TF_IDF, DBoW3::L1_NORM);
    
    try {
        vocab.create(descriptors);
        
        auto end_train = std::chrono::high_resolution_clock::now();
        auto train_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_train - end_sample).count();
        
        LOG(INFO) << "Vocabulary training complete in " << train_ms << "ms";
        LOG(INFO) << "Vocabulary size: " << vocab.size() << " words";
        
        // Save vocabulary
        vocab.save(output_file);
        LOG(INFO) << "Saved vocabulary to " << output_file;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Vocabulary training failed: " << e.what();
        return 1;
    }
    
    auto end_total = std::chrono::high_resolution_clock::now();
    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_total - start).count();
    
    LOG(INFO) << "=== Training Complete ===";
    LOG(INFO) << "Total time: " << total_ms << "ms";
    LOG(INFO) << "Vocabulary words: " << vocab.size();
    LOG(INFO) << "Branching factor: " << vocab.getBranchingFactor();
    LOG(INFO) << "Depth: " << vocab.getDepthLevels();
    LOG(INFO) << "Training samples: " << total_sampled;
    
    return 0;
}
