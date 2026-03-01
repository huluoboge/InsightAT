/**
 * isat_match.cpp
 * InsightAT Feature Matching Tool – GPU-accelerated SIFT matching
 *
 * Reads a pairs JSON and .isat_feat files from a feature directory, runs
 * GPU-accelerated matching (SiftGPU or SuperPoint matcher), and writes
 * .isat_match files (pixel correspondences in IDC format).
 *
 * Pipeline (Stage/chain):
 *   Stage 1  [multi-thread I/O]   Load .isat_feat for each pair
 *   Stage 2  [main thread, EGL]   GPU matching (ratio test, optional cross-check)
 *   Stage 3  [multi-thread I/O]  Write .isat_match
 *
 * Output .isat_match (IDC): coords_pixel blob [x1,y1,x2,y2,...], metadata.
 *
 * Usage:
 *   isat_match -i pairs.json -f feat_dir/ -o match_dir/
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "../io/idc_reader.h"
#include "../io/idc_writer.h"
#include "../modules/matching/sift_matcher.h"
#include "../modules/matching/match_types.h"
#include "cmdLine/cmdLine.h"
#include "cli_logging.h"
#include "pair_json_utils.h"
#include "task_queue/task_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

using namespace insight::algorithm::matching;

static constexpr const char* kEventPrefix = "ISAT_EVENT ";

static void printEvent(const json& j) {
    std::cout << kEventPrefix << j.dump() << "\n";
    std::cout.flush();
}
using namespace insight::io;

/**
 * Image pair task
 */
struct PairTask {
    uint32_t image1_id = 0;
    uint32_t image2_id = 0;
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
 * Load pairs from JSON file.
 * 
 * If feature_dir is non-empty, feature paths are rebuilt from image IDs
 * as {feature_dir}/{image_id}.isat_feat (matching features, not retrieval features).
 * Otherwise, feature1_file/feature2_file from the JSON are used as-is.
 */
std::vector<PairTask> loadPairsJSON(const std::string& json_path,
                                    const std::string& feature_dir = "") {
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
        task.image1_id = insight::tools::getImageIdFromPair(pair, "image1_id");
        task.image2_id = insight::tools::getImageIdFromPair(pair, "image2_id");
        task.priority = pair.value("priority", 1.0f);
        task.index = index++;
        
        if (!feature_dir.empty()) {
            // Rebuild paths from image IDs to point at matching features directory
            task.feature1_file = feature_dir + "/" + std::to_string(task.image1_id) + ".isat_feat";
            task.feature2_file = feature_dir + "/" + std::to_string(task.image2_id) + ".isat_feat";
        } else {
            // Fall back to paths recorded in the pairs JSON (retrieval feature paths)
            task.feature1_file = pair["feature1_file"];
            task.feature2_file = pair["feature2_file"];
        }
        
        pairs.push_back(task);
    }
    
    LOG(INFO) << "Loaded " << pairs.size() << " pairs from " << json_path;
    if (!feature_dir.empty()) {
        LOG(INFO) << "  Using matching features from: " << feature_dir;
    }
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
                             std::to_string(pair.image1_id) + "_" + std::to_string(pair.image2_id) + ".isat_match";
    
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

    // Scales per match [s1, s2] for weighted BA; from keypoints (index 2 = scale)
    std::vector<float> scales_flat;
    scales_flat.reserve(matches.num_matches * 2);
    const auto& kpts1 = pair.features1.keypoints;
    const auto& kpts2 = pair.features2.keypoints;
    for (size_t m = 0; m < matches.num_matches; ++m) {
        float s1 = 1.0f;
        float s2 = 1.0f;
        if (matches.indices[m].first < kpts1.size())
            s1 = kpts1[matches.indices[m].first](2);
        if (matches.indices[m].second < kpts2.size())
            s2 = kpts2[matches.indices[m].second](2);
        scales_flat.push_back(s1);
        scales_flat.push_back(s2);
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
    
    writer.addBlob("scales", scales_flat.data(),
                  scales_flat.size() * sizeof(float),
                  "float32", {static_cast<int>(matches.num_matches), 2});
    
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
    std::string feature_dir;
    float ratio_test = 0.8f;
    int max_matches = -1;
    int max_features = 4096;  // cap per image before GPU upload
    int num_threads = 4;
    
    // Required arguments
    cmd.add(make_option('i', pairs_json, "input")
        .doc("Input pairs list (JSON format, from isat_retrieve)"));
    cmd.add(make_option('o', output_dir, "output")
        .doc("Output directory for .isat_match files"));
    cmd.add(make_option('f', feature_dir, "feature-dir")
        .doc("Matching feature directory (.isat_feat files from isat_extract -o). "
             "Paths are rebuilt as {dir}/{image_id}.isat_feat, overriding the "
             "retrieval feature paths stored in the pairs JSON."));
    
    // Matching parameters
    cmd.add(make_option('r', ratio_test, "ratio")
        .doc("Ratio test threshold (default: 0.8)"));
    cmd.add(make_option(0, max_matches, "max-matches")
        .doc("Max matches per pair, -1=unlimited (default: -1)"));
    cmd.add(make_option(0, max_features, "max-features")
        .doc("Max features per image uploaded to GPU; top-N selected by scale. "
             "-1=all (default: 4096). Reducing this cuts GPU time quadratically."));
    
    // Performance options
    cmd.add(make_option('j', num_threads, "threads")
        .doc("Number of CPU threads for I/O (default: 4)"));
    std::string match_backend = "cuda";
    cmd.add(make_option(0, match_backend, "match-backend")
        .doc("GPU backend for matching: cuda or glsl (default: cuda when built with CUDA)"));
    
    // Logging options
    std::string log_level;
    cmd.add(make_option(0, log_level, "log-level").doc("Log level: error|warn|info|debug"));
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
    insight::tools::ApplyLogLevel(cmd.used('v'), cmd.used('q'), log_level);

    // Log configuration
    LOG(INFO) << "Feature matching configuration:";
    LOG(INFO) << "  Pairs JSON: " << pairs_json;
    LOG(INFO) << "  Feature dir: " << (feature_dir.empty() ? "(from pairs JSON)" : feature_dir);
    LOG(INFO) << "  Ratio test: " << ratio_test;
    LOG(INFO) << "  Max features/image: " << (max_features > 0 ? std::to_string(max_features) : "unlimited");
    LOG(INFO) << "  Max matches: " << (max_matches > 0 ? std::to_string(max_matches) : "unlimited");
    LOG(INFO) << "  CPU threads: " << num_threads;
    bool use_cuda_match = (match_backend == "cuda");
    LOG(INFO) << "  Match backend: " << (use_cuda_match ? "cuda" : "glsl");
    
    // Validate feature directory if provided
    if (!feature_dir.empty() && !fs::is_directory(feature_dir)) {
        LOG(ERROR) << "Feature directory not found: " << feature_dir;
        return 1;
    }
    
    // Create output directory
    fs::create_directories(output_dir);
    
    // Load pairs
    std::vector<PairTask> pair_tasks = loadPairsJSON(pairs_json, feature_dir);
    int total_pairs = pair_tasks.size();
    
    if (total_pairs == 0) {
        LOG(ERROR) << "No pairs to process";
        printEvent({{"type", "match.complete"}, {"ok", false}, {"error", "no pairs to process"}});
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
    SiftMatcher matcher(10000, use_cuda_match);  // Max 10000 features per image
    
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
    
    int failed_pairs = total_pairs - pairs_with_matches;
    double avg_matches = (pairs_with_matches > 0) ? static_cast<double>(total_matches) / pairs_with_matches : 0.0;
    double avg_time_per_pair = (total_pairs > 0) ? static_cast<double>(total_time) / total_pairs : 0.0;

    // Machine-readable result (CLI_IO_CONVENTIONS: stdout = ISAT_EVENT only)
    printEvent({
        {"type", "match.complete"},
        {"ok", true},
        {"data", {
            {"total_pairs", total_pairs},
            {"pairs_with_matches", pairs_with_matches},
            {"total_matches", total_matches},
            {"failed_pairs", failed_pairs},
            {"total_time_s", total_time},
            {"avg_matches_per_pair", std::round(avg_matches * 100) / 100.0},
            {"avg_time_per_pair_s", std::round(avg_time_per_pair * 100) / 100.0},
            {"output_dir", output_dir}
        }}
    });

    LOG(INFO) << "=== Matching Complete ===";
    LOG(INFO) << "Total pairs: " << total_pairs;
    LOG(INFO) << "Pairs with matches: " << pairs_with_matches;
    LOG(INFO) << "Total matches: " << total_matches;
    LOG(INFO) << "Average matches/pair: " << avg_matches;
    LOG(INFO) << "Total time: " << total_time << "s";
    LOG(INFO) << "Average time/pair: " << avg_time_per_pair << "s";

    return 0;
}
