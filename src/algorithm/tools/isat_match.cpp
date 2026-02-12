#include <chrono>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <algorithm>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/matching/sift_matcher.h"
#include "../modules/matching/match_types.h"
#include "cmdLine/cmdLine.h"
#include "task_queue/task_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

using namespace insight::algorithm::matching;
using namespace insight::io;

/**
 * Image pair task
 */
struct PairTask {
    std::string image1_id;
    std::string image2_id;
    std::string feature1_file;
    std::string feature2_file;
    float priority = 1.0f;
    
    // Loaded features (stage 1)
    FeatureData features1;
    FeatureData features2;
    
    // Match result (stage 2)
    MatchResult matches;
    
    int index;
};

/**
 * Load pairs from JSON file
 */
std::vector<PairTask> loadPairsJSON(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        LOG(FATAL) << "Failed to open pairs file: " << json_path;
    }
    
    json j;
    file >> j;
    
    std::vector<PairTask> pairs;
    int index = 0;
    
    for (const auto& pair : j["pairs"]) {
        PairTask task;
        task.image1_id = pair["image1_id"];
        task.image2_id = pair["image2_id"];
        task.feature1_file = pair["feature1_file"];
        task.feature2_file = pair["feature2_file"];
        task.priority = pair.value("priority", 1.0f);
        task.index = index++;
        pairs.push_back(task);
    }
    
    LOG(INFO) << "Loaded " << pairs.size() << " pairs from " << json_path;
    return pairs;
}

/**
 * Load features from IDC file
 */
FeatureData loadFeaturesIDC(const std::string& idc_path) {
    IDCReader reader(idc_path);
    
    if (!reader.isValid()) {
        LOG(ERROR) << "Invalid IDC file: " << idc_path;
        return FeatureData();
    }
    
    // Read keypoints blob
    auto keypoints_raw = reader.readBlob<float>("keypoints");
    if (keypoints_raw.empty()) {
        LOG(ERROR) << "Failed to read keypoints from " << idc_path;
        return FeatureData();
    }
    
    // Read descriptors blob - auto-detect dtype
    auto desc_blob = reader.getBlobDescriptor("descriptors");
    std::string dtype = desc_blob["dtype"];
    
    size_t num_features = keypoints_raw.size() / 4;
    DescriptorType descriptor_type;
    
    if (dtype == "uint8") {
        descriptor_type = DescriptorType::kUInt8;
    } else if (dtype == "float32") {
        descriptor_type = DescriptorType::kFloat32;
    } else {
        LOG(ERROR) << "Unsupported descriptor dtype: " << dtype << " in " << idc_path;
        return FeatureData();
    }
    
    // Create FeatureData with appropriate descriptor type
    FeatureData features(num_features, descriptor_type);
    
    // Fill keypoints
    for (size_t i = 0; i < num_features; ++i) {
        features.keypoints[i] << 
            keypoints_raw[i * 4 + 0],
            keypoints_raw[i * 4 + 1],
            keypoints_raw[i * 4 + 2],
            keypoints_raw[i * 4 + 3];
    }
    
    // Read descriptors based on type
    if (descriptor_type == DescriptorType::kUInt8) {
        features.descriptors_uint8 = reader.readBlob<uint8_t>("descriptors");
        if (features.descriptors_uint8.empty()) {
            LOG(ERROR) << "Failed to read uint8 descriptors from " << idc_path;
            return FeatureData();
        }
    } else {
        features.descriptors_float = reader.readBlob<float>("descriptors");
        if (features.descriptors_float.empty()) {
            LOG(ERROR) << "Failed to read float32 descriptors from " << idc_path;
            return FeatureData();
        }
    }
    
    VLOG(1) << "Loaded " << num_features << " features (" << dtype << ") from " << idc_path;
    
    return features;
}

/**
 * Write match result to IDC file
 */
bool writeMatchIDC(const MatchResult& matches,
                   const PairTask& pair,
                   const std::string& output_dir) {
    
    if (matches.num_matches == 0) {
        LOG(WARNING) << "No matches for pair " << pair.image1_id 
                    << " - " << pair.image2_id;
        return false;
    }
    
    // Create output filename
    std::string output_file = output_dir + "/" + 
                             pair.image1_id + "_" + pair.image2_id + ".isat_match";
    
    // Prepare metadata
    json metadata;
    metadata["schema_version"] = "1.0";
    metadata["task_type"] = "feature_matching";
    metadata["algorithm"]["name"] = "SiftGPU";
    metadata["algorithm"]["version"] = "1.1";
    
    metadata["image_pair"]["image1_id"] = pair.image1_id;
    metadata["image_pair"]["image2_id"] = pair.image2_id;
    
    metadata["metadata"]["num_matches"] = matches.num_matches;
    
    // Prepare binary data
    std::vector<uint16_t> indices_flat;
    indices_flat.reserve(matches.num_matches * 2);
    for (const auto& idx : matches.indices) {
        indices_flat.push_back(idx.first);
        indices_flat.push_back(idx.second);
    }
    
    std::vector<float> coords_flat;
    coords_flat.reserve(matches.num_matches * 4);
    for (const auto& coord : matches.coords_pixel) {
        coords_flat.push_back(coord(0));
        coords_flat.push_back(coord(1));
        coords_flat.push_back(coord(2));
        coords_flat.push_back(coord(3));
    }
    
    // Write IDC file
    IDCWriter writer(output_file);
    writer.setMetadata(metadata);
    
    writer.addBlob("indices", indices_flat.data(), 
                  indices_flat.size() * sizeof(uint16_t),
                  "uint16", {static_cast<int>(matches.num_matches), 2});
    
    writer.addBlob("coords_pixel", coords_flat.data(),
                  coords_flat.size() * sizeof(float),
                  "float32", {static_cast<int>(matches.num_matches), 4});
    
    writer.addBlob("distances", matches.distances.data(),
                  matches.distances.size() * sizeof(float),
                  "float32", {static_cast<int>(matches.num_matches)});
    
    if (!writer.write()) {
        LOG(ERROR) << "Failed to write match file: " << output_file;
        return false;
    }
    
    VLOG(1) << "Wrote " << matches.num_matches << " matches to " << output_file;
    return true;
}

int main(int argc, char* argv[]) {
    // Initialize glog
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    
    // Define command line options
    CmdLine cmd("InsightAT Feature Matching Tool - GPU-accelerated SIFT feature matching");
    
    std::string pairs_json;
    std::string output_dir;
    float ratio_test = 0.8f;
    int max_matches = -1;
    int num_threads = 4;
    
    // Required arguments
    cmd.add(make_option('i', pairs_json, "input")
        .doc("Input pairs list (JSON format, from isat_retrieve)"));
    cmd.add(make_option('o', output_dir, "output")
        .doc("Output directory for .isat_match files"));
    
    // Matching parameters
    cmd.add(make_option('r', ratio_test, "ratio")
        .doc("Ratio test threshold (default: 0.8)"));
    cmd.add(make_option(0, max_matches, "max-matches")
        .doc("Max matches per pair, -1=unlimited (default: -1)"));
    
    // Performance options
    cmd.add(make_option('j', num_threads, "threads")
        .doc("Number of CPU threads for I/O (default: 4)"));
    
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
    
    // Check for help
    if (cmd.checkHelp(argv[0])) {
        return 0;
    }
    
    // Validate required arguments
    if (pairs_json.empty() || output_dir.empty()) {
        std::cerr << "Error: -i/--input and -o/--output are required\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }
    
    // Set logging level
    int log_level = google::WARNING;
    if (cmd.used('v')) log_level = google::INFO;
    if (cmd.used('q')) log_level = google::ERROR;
    FLAGS_minloglevel = log_level;
    
    // Log configuration
    LOG(INFO) << "Feature matching configuration:";
    LOG(INFO) << "  Ratio test: " << ratio_test;
    LOG(INFO) << "  Max matches: " << (max_matches > 0 ? std::to_string(max_matches) : "unlimited");
    LOG(INFO) << "  CPU threads: " << num_threads;
    
    // Create output directory
    fs::create_directories(output_dir);
    
    // Load pairs
    std::vector<PairTask> pair_tasks = loadPairsJSON(pairs_json);
    int total_pairs = pair_tasks.size();
    
    if (total_pairs == 0) {
        LOG(ERROR) << "No pairs to process";
        return 1;
    }
    
    // Create matching options
    MatchOptions match_options;
    match_options.ratio_test = ratio_test;
    match_options.max_matches = max_matches;
    match_options.mutual_best_match = true;
    
    // Create pipeline stages
    const int IO_QUEUE_SIZE = 10;
    const int GPU_QUEUE_SIZE = 3;
    
    // Stage 1: Load features (multi-threaded I/O)
    Stage loadStage("LoadFeatures", num_threads, IO_QUEUE_SIZE,
        [&pair_tasks](int index) {
            auto& task = pair_tasks[index];
            
            auto start = std::chrono::high_resolution_clock::now();
            
            task.features1 = loadFeaturesIDC(task.feature1_file);
            task.features2 = loadFeaturesIDC(task.feature2_file);
            
            auto end = std::chrono::high_resolution_clock::now();
            int load_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            
            LOG(INFO) << "Loaded pair [" << index << "/" << pair_tasks.size() << "]: "
                     << task.image1_id << " (" << task.features1.num_features << ") vs "
                     << task.image2_id << " (" << task.features2.num_features << ") "
                     << "in " << load_time << "ms";
        });
    
    // Stage 2: GPU matching (single thread, GPU context requirement)
    SiftMatcher matcher(10000);  // Max 10000 features per image
    
    if (!matcher.verifyContext()) {
        LOG(FATAL) << "Failed to initialize SiftMatchGPU - OpenGL context error";
    }
    
    StageCurrent matchStage("GPUMatch", 1, GPU_QUEUE_SIZE,
        [&pair_tasks, &matcher, &match_options](int index) {
            auto& task = pair_tasks[index];
            
            if (task.features1.num_features == 0 || task.features2.num_features == 0) {
                LOG(WARNING) << "Skipping pair [" << index << "] - empty features";
                return;
            }
            
            auto start = std::chrono::high_resolution_clock::now();
            
            task.matches = matcher.match(task.features1, task.features2, match_options);
            
            auto end = std::chrono::high_resolution_clock::now();
            int match_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            
            LOG(INFO) << "Matched pair [" << index << "/" << pair_tasks.size() << "]: "
                     << task.matches.num_matches << " matches in " << match_time << "ms";
            
            // Free feature memory
            task.features1.clear();
            task.features2.clear();
        });
    
    // Stage 3: Write results (multi-threaded I/O)
    Stage writeStage("WriteResults", num_threads, IO_QUEUE_SIZE,
        [&pair_tasks, &output_dir](int index) {
            auto& task = pair_tasks[index];
            
            if (task.matches.num_matches == 0) {
                return;
            }
            
            auto start = std::chrono::high_resolution_clock::now();
            
            writeMatchIDC(task.matches, task, output_dir);
            
            auto end = std::chrono::high_resolution_clock::now();
            int write_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            
            VLOG(1) << "Wrote pair [" << index << "] in " << write_time << "ms";
            
            // Progress reporting
            float progress = static_cast<float>(index + 1) / pair_tasks.size();
            std::cerr << "PROGRESS: " << progress << "\n";
        });
    
    // Chain stages
    chain(loadStage, matchStage);
    chain(matchStage, writeStage);
    
    // Set task counts
    loadStage.setTaskCount(total_pairs);
    matchStage.setTaskCount(total_pairs);
    writeStage.setTaskCount(total_pairs);
    
    // Process all pairs
    auto pipeline_start = std::chrono::high_resolution_clock::now();
    
    // Push tasks in background thread, process GPU in main thread
    std::thread push_thread([&]() {
        for (int i = 0; i < total_pairs; ++i) {
            loadStage.push(i);
        }
    });
    
    // Run GPU stage in main thread (OpenGL context requirement)
    matchStage.run();
    
    // Wait for all stages to complete
    push_thread.join();
    loadStage.wait();
    writeStage.wait();
    
    auto pipeline_end = std::chrono::high_resolution_clock::now();
    int total_time = std::chrono::duration_cast<std::chrono::seconds>(pipeline_end - pipeline_start).count();
    
    // Statistics
    int total_matches = 0;
    int pairs_with_matches = 0;
    
    for (const auto& task : pair_tasks) {
        if (task.matches.num_matches > 0) {
            total_matches += task.matches.num_matches;
            pairs_with_matches++;
        }
    }
    
    LOG(INFO) << "=== Matching Complete ===";
    LOG(INFO) << "Total pairs: " << total_pairs;
    LOG(INFO) << "Pairs with matches: " << pairs_with_matches;
    LOG(INFO) << "Total matches: " << total_matches;
    LOG(INFO) << "Average matches/pair: " << (pairs_with_matches > 0 ? total_matches / pairs_with_matches : 0);
    LOG(INFO) << "Total time: " << total_time << "s";
    LOG(INFO) << "Average time/pair: " << (total_pairs > 0 ? total_time / total_pairs : 0) << "s";
    
    return 0;
}
