#include <chrono>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <algorithm>

#include "cmdLine/cmdLine.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

/**
 * Image info for pairing
 */
struct ImageInfo {
    std::string image_id;
    std::string image_path;
    std::string feature_file;
    int camera_id = 1;
};

/**
 * Load images from feature directory
 */
std::vector<ImageInfo> loadImagesFromFeatures(const std::string& feature_dir) {
    std::vector<ImageInfo> images;
    
    for (const auto& entry : fs::directory_iterator(feature_dir)) {
        if (entry.path().extension() != ".isat_feat") {
            continue;
        }
        
        ImageInfo info;
        info.feature_file = entry.path().string();
        info.image_id = entry.path().stem().string();
        info.image_path = "";  // Not needed for matching
        info.camera_id = 1;
        
        images.push_back(info);
    }
    
    std::sort(images.begin(), images.end(), 
              [](const ImageInfo& a, const ImageInfo& b) {
                  return a.image_id < b.image_id;
              });
    
    LOG(INFO) << "Found " << images.size() << " feature files in " << feature_dir;
    return images;
}

/**
 * Load images from JSON list
 */
std::vector<ImageInfo> loadImagesFromJSON(const std::string& json_path, 
                                         const std::string& feature_dir) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        LOG(FATAL) << "Failed to open image list: " << json_path;
    }
    
    json j;
    file >> j;
    
    std::vector<ImageInfo> images;
    
    for (const auto& img : j["images"]) {
        ImageInfo info;
        info.image_path = img["path"];
        info.camera_id = img.value("camera_id", 1);
        
        // Extract image ID from path
        fs::path img_path(info.image_path);
        info.image_id = img_path.stem().string();
        
        // Construct feature file path
        info.feature_file = feature_dir + "/" + info.image_id + ".isat_feat";
        
        // Check if feature file exists
        if (!fs::exists(info.feature_file)) {
            LOG(WARNING) << "Feature file not found: " << info.feature_file;
            continue;
        }
        
        images.push_back(info);
    }
    
    LOG(INFO) << "Loaded " << images.size() << " images with features from " << json_path;
    return images;
}

/**
 * Generate exhaustive pairs
 */
std::vector<std::pair<int, int>> generateExhaustivePairs(int num_images, int max_pairs = -1) {
    std::vector<std::pair<int, int>> pairs;
    
    // Generate all combinations (i, j) where i < j
    for (int i = 0; i < num_images; ++i) {
        for (int j = i + 1; j < num_images; ++j) {
            pairs.push_back({i, j});
            
            if (max_pairs > 0 && pairs.size() >= static_cast<size_t>(max_pairs)) {
                LOG(WARNING) << "Reached max_pairs limit: " << max_pairs;
                return pairs;
            }
        }
    }
    
    return pairs;
}

/**
 * Generate sequential pairs (for video sequences)
 */
std::vector<std::pair<int, int>> generateSequentialPairs(int num_images, int window_size = 10) {
    std::vector<std::pair<int, int>> pairs;
    
    for (int i = 0; i < num_images; ++i) {
        for (int j = i + 1; j < std::min(i + window_size + 1, num_images); ++j) {
            pairs.push_back({i, j});
        }
    }
    
    return pairs;
}

/**
 * Write pairs to JSON file
 */
bool writePairsJSON(const std::vector<ImageInfo>& images,
                   const std::vector<std::pair<int, int>>& pairs,
                   const std::string& output_path,
                   const std::string& retrieval_method) {
    
    json output;
    output["schema_version"] = "1.0";
    output["retrieval_method"] = retrieval_method;
    output["retrieval_params"] = json::object();
    output["pairs"] = json::array();
    
    for (const auto& [i, j] : pairs) {
        json pair_obj;
        pair_obj["image1_id"] = images[i].image_id;
        pair_obj["image2_id"] = images[j].image_id;
        pair_obj["feature1_file"] = images[i].feature_file;
        pair_obj["feature2_file"] = images[j].feature_file;
        pair_obj["priority"] = 1.0;
        
        output["pairs"].push_back(pair_obj);
    }
    
    // Write to file
    std::ofstream out_file(output_path);
    if (!out_file.is_open()) {
        LOG(ERROR) << "Failed to open output file: " << output_path;
        return false;
    }
    
    out_file << output.dump(2);  // Pretty print with 2-space indent
    out_file.close();
    
    LOG(INFO) << "Wrote " << pairs.size() << " pairs to " << output_path;
    return true;
}

int main(int argc, char* argv[]) {
    // Initialize glog
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    
    // Define command line options
    CmdLine cmd("InsightAT Image Pair Retrieval Tool - Generate image pair candidates for matching");
    
    std::string feature_dir;
    std::string output_file;
    std::string image_list;
    std::string strategy = "exhaustive";
    int max_pairs = -1;
    int window_size = 10;
    
    // Required arguments
    cmd.add(make_option('f', feature_dir, "features")
        .doc("Feature directory containing .isat_feat files"));
    cmd.add(make_option('o', output_file, "output")
        .doc("Output pairs file (JSON format)"));
    
    // Optional arguments
    cmd.add(make_option('i', image_list, "input")
        .doc("Input image list (JSON format, optional)"));
    cmd.add(make_option('s', strategy, "strategy")
        .doc("Pairing strategy: exhaustive|sequential (default: exhaustive)"));
    cmd.add(make_option('m', max_pairs, "max-pairs")
        .doc("Maximum number of pairs, -1=unlimited (default: -1)"));
    cmd.add(make_option('w', window_size, "window")
        .doc("Window size for sequential strategy (default: 10)"));
    
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
    if (feature_dir.empty() || output_file.empty()) {
        std::cerr << "Error: -f/--features and -o/--output are required\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }
    
    // Set logging level
    int log_level = google::WARNING;
    if (cmd.used('v')) log_level = google::INFO;
    if (cmd.used('q')) log_level = google::ERROR;
    FLAGS_minloglevel = log_level;
    
    // Log configuration
    LOG(INFO) << "Image pair retrieval configuration:";
    LOG(INFO) << "  Feature directory: " << feature_dir;
    LOG(INFO) << "  Strategy: " << strategy;
    LOG(INFO) << "  Max pairs: " << (max_pairs > 0 ? std::to_string(max_pairs) : "unlimited");
    if (strategy == "sequential") {
        LOG(INFO) << "  Window size: " << window_size;
    }
    
    // Load images
    std::vector<ImageInfo> images;
    if (!image_list.empty()) {
        images = loadImagesFromJSON(image_list, feature_dir);
    } else {
        images = loadImagesFromFeatures(feature_dir);
    }
    
    if (images.empty()) {
        LOG(ERROR) << "No images found";
        return 1;
    }
    
    LOG(INFO) << "Processing " << images.size() << " images";
    
    // Generate pairs
    std::vector<std::pair<int, int>> pairs;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (strategy == "exhaustive") {
        pairs = generateExhaustivePairs(images.size(), max_pairs);
    } else if (strategy == "sequential") {
        pairs = generateSequentialPairs(images.size(), window_size);
        if (max_pairs > 0 && pairs.size() > static_cast<size_t>(max_pairs)) {
            pairs.resize(max_pairs);
        }
    } else {
        LOG(ERROR) << "Unknown strategy: " << strategy;
        return 1;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    int gen_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    LOG(INFO) << "Generated " << pairs.size() << " pairs in " << gen_time << "ms";
    
    // Calculate statistics
    int total_possible = (images.size() * (images.size() - 1)) / 2;
    float coverage = 100.0f * pairs.size() / total_possible;
    LOG(INFO) << "Coverage: " << pairs.size() << "/" << total_possible 
              << " (" << coverage << "%)";
    
    // Write output
    if (!writePairsJSON(images, pairs, output_file, strategy)) {
        return 1;
    }
    
    LOG(INFO) << "=== Retrieval Complete ===";
    LOG(INFO) << "Images: " << images.size();
    LOG(INFO) << "Pairs: " << pairs.size();
    LOG(INFO) << "Output: " << output_file;
    
    return 0;
}
