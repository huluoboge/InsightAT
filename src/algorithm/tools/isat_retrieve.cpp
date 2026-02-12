#include <chrono>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <map>

#include "cmdLine/cmdLine.h"
#include "../modules/retrieval/retrieval_types.h"
#include "../modules/retrieval/spatial_retrieval.h"
#include "../modules/retrieval/vlad_retrieval.h"
#include "../modules/retrieval/vocab_tree_retrieval.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::algorithm::retrieval;

/**
 * Load VLAD centroids from codebook file
 */
std::vector<float> loadVLADCentroids(const std::string& filepath, int& num_clusters) {
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs.is_open()) {
        LOG(ERROR) << "Failed to open codebook file: " << filepath;
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
    ifs.read(reinterpret_cast<char*>(centroids.data()), 
             centroids.size() * sizeof(float));
    
    if (!ifs.good()) {
        LOG(ERROR) << "Failed to read centroids from file";
        return {};
    }
    
    return centroids;
}

/**
 * Legacy ImageInfo for backward compatibility
 * Will be merged with retrieval::ImageInfo
 */
struct LegacyImageInfo {
    std::string image_id;
    std::string image_path;
    std::string feature_file;
    int camera_id = 1;
};

/**
 * Convert legacy format to new retrieval format
 */
ImageInfo convertToRetrievalFormat(const LegacyImageInfo& legacy, const json& metadata = {}) {
    ImageInfo info;
    info.image_id = legacy.image_id;
    info.image_path = legacy.image_path;
    info.feature_file = legacy.feature_file;
    info.camera_id = legacy.camera_id;
    
    // Try to load GNSS/IMU from metadata if present
    if (metadata.contains("gnss")) {
        GNSSData gnss;
        gnss.x = metadata["gnss"].value("x", 0.0);
        gnss.y = metadata["gnss"].value("y", 0.0);
        gnss.z = metadata["gnss"].value("z", 0.0);
        gnss.cov_xx = metadata["gnss"].value("cov_xx", 1.0);
        gnss.cov_yy = metadata["gnss"].value("cov_yy", 1.0);
        gnss.cov_zz = metadata["gnss"].value("cov_zz", 1.0);
        gnss.num_satellites = metadata["gnss"].value("num_satellites", 0);
        gnss.hdop = metadata["gnss"].value("hdop", 0.0);
        gnss.vdop = metadata["gnss"].value("vdop", 0.0);
        info.gnss = gnss;
    }
    
    if (metadata.contains("imu")) {
        IMUData imu;
        imu.roll = metadata["imu"].value("roll", 0.0);
        imu.pitch = metadata["imu"].value("pitch", 0.0);
        imu.yaw = metadata["imu"].value("yaw", 0.0);
        info.imu = imu;
    }
    
    return info;
}

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
 * Load images from JSON list (with optional GNSS/IMU data)
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
        
        // Load optional GNSS data
        if (img.contains("gnss")) {
            GNSSData gnss;
            gnss.x = img["gnss"].value("x", 0.0);
            gnss.y = img["gnss"].value("y", 0.0);
            gnss.z = img["gnss"].value("z", 0.0);
            gnss.cov_xx = img["gnss"].value("cov_xx", 1.0);
            gnss.cov_yy = img["gnss"].value("cov_yy", 1.0);
            gnss.cov_zz = img["gnss"].value("cov_zz", 1.0);
            gnss.cov_xy = img["gnss"].value("cov_xy", 0.0);
            gnss.cov_xz = img["gnss"].value("cov_xz", 0.0);
            gnss.cov_yz = img["gnss"].value("cov_yz", 0.0);
            gnss.num_satellites = img["gnss"].value("num_satellites", 0);
            gnss.hdop = img["gnss"].value("hdop", 0.0);
            gnss.vdop = img["gnss"].value("vdop", 0.0);
            info.gnss = gnss;
        }
        
        // Load optional IMU data
        if (img.contains("imu")) {
            IMUData imu;
            imu.roll = img["imu"].value("roll", 0.0) *  M_PI / 180.0;  // Convert to radians
            imu.pitch = img["imu"].value("pitch", 0.0) * M_PI / 180.0;
            imu.yaw = img["imu"].value("yaw", 0.0) * M_PI / 180.0;
            imu.cov_att_xx = img["imu"].value("cov_att_xx", 0.1);
            imu.cov_att_yy = img["imu"].value("cov_att_yy", 0.1);
            imu.cov_att_zz = img["imu"].value("cov_att_zz", 0.1);
            info.imu = imu;
        }
        
        images.push_back(info);
    }
    
    LOG(INFO) << "Loaded " << images.size() << " images with features from " << json_path;
    
    int gnss_count = std::count_if(images.begin(), images.end(), 
                                    [](const ImageInfo& i) { return i.hasGNSS(); });
    int imu_count = std::count_if(images.begin(), images.end(), 
                                   [](const ImageInfo& i) { return i.hasIMU(); });
    LOG(INFO) << "  GNSS data: " << gnss_count << "/" << images.size();
    LOG(INFO) << "  IMU data: " << imu_count << "/" << images.size();
    
    return images;
}

/**
 * Generate exhaustive pairs (pure function)
 */
std::vector<ImagePair> retrieveExhaustive(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
) {
    std::vector<ImagePair> pairs;
    int num_images = images.size();
    
    // Generate all combinations (i, j) where i < j
    for (int i = 0; i < num_images; ++i) {
        for (int j = i + 1; j < num_images; ++j) {
            ImagePair pair;
            pair.image1_idx = i;
            pair.image2_idx = j;
            pair.score = 1.0;  // All pairs equally likely
            pair.method = "exhaustive";
            pairs.push_back(pair);
            
            if (options.max_pairs > 0 && pairs.size() >= static_cast<size_t>(options.max_pairs)) {
                LOG(WARNING) << "Reached max_pairs limit: " << options.max_pairs;
                return pairs;
            }
        }
    }
   
    return pairs;
}

/**
 * Generate sequential pairs (pure function - for video sequences)
 */
std::vector<ImagePair> retrieveSequential(
    const std::vector<ImageInfo>& images,
    const RetrievalOptions& options
) {
    std::vector<ImagePair> pairs;
    int num_images = images.size();
    int window_size = options.window_size;
    
    for (int i = 0; i < num_images; ++i) {
        for (int j = i + 1; j < std::min(i + window_size + 1, num_images); ++j) {
            ImagePair pair;
            pair.image1_idx = i;
            pair.image2_idx = j;
            // Score decreases with distance in sequence
            pair.score = 1.0 - static_cast<double>(j - i) / window_size;
            pair.method = "sequential";
            pairs.push_back(pair);
        }
    }
    
    return pairs;
}

/**
 * Strategy function registry (compile-time map)
 */
/**
 * NOTE: VLAD strategy requires runtime binding due to centroids parameter.
 * Use std::function wrapper in main() instead of static registration.
 */
const std::map<std::string, RetrievalFunction> STRATEGIES = {
    {"exhaustive", retrieveExhaustive},
    {"sequential", retrieveSequential},
    {"gps", retrieveByGPS}
    // "vlad" registered dynamically in main() with centroids
};

/**
 * Parse strategy string (e.g., "gps+sequential" → ["gps", "sequential"])
 */
std::vector<std::string> parseStrategyString(const std::string& strategy_str) {
    std::vector<std::string> strategies;
    std::istringstream ss(strategy_str);
    std::string token;
    while (std::getline(ss, token, '+')) {
        strategies.push_back(token);
    }
    return strategies;
}

/**
 * Write pairs to JSON file (updated for ImagePair format)
 */
bool writePairsJSON(const std::vector<ImageInfo>& images,
                   const std::vector<ImagePair>& pairs,
                   const std::string& output_path,
                   const std::string& retrieval_method) {
    
    json output;
    output["schema_version"] = "1.0";
    output["retrieval_method"] = retrieval_method;
    output["pairs"] = json::array();
    
    for (const auto& p : pairs) {
        if (!p.isValid() || p.image1_idx >= static_cast<int>(images.size()) || 
            p.image2_idx >= static_cast<int>(images.size())) {
            continue;
        }
        
        json pair_obj;
        pair_obj["image1_id"] = images[p.image1_idx].image_id;
        pair_obj["image2_id"] = images[p.image2_idx].image_id;
        pair_obj["feature1_file"] = images[p.image1_idx].feature_file;
        pair_obj["feature2_file"] = images[p.image2_idx].feature_file;
        pair_obj["score"] = p.score;
        pair_obj["method"] = p.method;
        pair_obj["priority"] = 1.0 + p.score;  // Higher score = higher priority
        
        // Add optional metadata
        if (p.spatial_distance) {
            pair_obj["spatial_distance"] = *p.spatial_distance;
        }
        if (p.visual_similarity) {
            pair_obj["visual_similarity"] = *p.visual_similarity;
        }
        if (p.angle_difference) {
            pair_obj["angle_difference"] = *p.angle_difference;
        }
        
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
    
    // GPS spatial retrieval options
    double distance_threshold = 200.0;
    double angle_threshold = 0.0;  // 0 = disabled
    int max_neighbors = 50;
    
    // VLAD visual retrieval options
    std::string vlad_codebook;
    std::string vlad_cache_dir;
    int vlad_top_k = 20;
    
    // Vocabulary tree retrieval options
    std::string vocab_file;
    std::string vocab_cache_dir;
    int vocab_top_k = 20;
    
    // Required arguments
    cmd.add(make_option('f', feature_dir, "features")
        .doc("Feature directory containing .isat_feat files"));
    cmd.add(make_option('o', output_file, "output")
        .doc("Output pairs file (JSON format)"));
    
    // Optional arguments
    cmd.add(make_option('i', image_list, "input")
        .doc("Input image list (JSON format with optional GNSS/IMU)"));
    cmd.add(make_option('s', strategy, "strategy")
        .doc("Strategy: exhaustive|sequential|gps|vlad|gps+sequential|gps+vlad (default: exhaustive)"));
    cmd.add(make_option('m', max_pairs, "max-pairs")
        .doc("Maximum number of pairs, -1=unlimited (default: -1)"));
    cmd.add(make_option('w', window_size, "window")
        .doc("Window size for sequential strategy (default: 10)"));
    
    // GPS retrieval options
    cmd.add(make_option('d', distance_threshold, "distance-threshold")
        .doc("GPS distance threshold in meters (default: 200)"));
    cmd.add(make_option('a', angle_threshold, "angle-threshold")
        .doc("IMU angle threshold in degrees, 0=disabled (default: 0)"));
    cmd.add(make_option('n', max_neighbors, "max-neighbors")
        .doc("Max neighbors per image for GPS retrieval (default: 50)"));
    
    // VLAD retrieval options
    cmd.add(make_option(0, vlad_codebook, "vlad-codebook")
        .doc("VLAD codebook file (.vcbt format) for visual retrieval"));
    cmd.add(make_option(0, vlad_cache_dir, "vlad-cache")
        .doc("Directory for VLAD vector cache (.isat_vlad files)"));
    cmd.add(make_option(0, vlad_top_k, "vlad-top-k")
        .doc("Top-k most similar images per query for VLAD (default: 20)"));
    
    // Vocabulary tree retrieval options
    cmd.add(make_option(0, vocab_file, "vocab-file")
        .doc("DBoW3 vocabulary file (.dbow3 format) for visual retrieval"));
    cmd.add(make_option(0, vocab_cache_dir, "vocab-cache")
        .doc("Directory for vocabulary tree query cache"));
    cmd.add(make_option(0, vocab_top_k, "vocab-top-k")
        .doc("Top-k most similar images per query for vocab tree (default: 20)"));
    
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
    LOG(INFO) << "=== Image Pair Retrieval Configuration ===";
    LOG(INFO) << "Feature directory: " << feature_dir;
    LOG(INFO) << "Strategy: " << strategy;
    LOG(INFO) << "Max pairs: " << (max_pairs > 0 ? std::to_string(max_pairs) : "unlimited");
    if (strategy.find("sequential") != std::string::npos) {
        LOG(INFO) << "Window size: " << window_size;
    }
    if (strategy.find("gps") != std::string::npos) {
        LOG(INFO) << "GPS distance threshold: " << distance_threshold << "m";
        LOG(INFO) << "GPS max neighbors: " << max_neighbors;
        if (angle_threshold > 0) {
            LOG(INFO) << "IMU angle threshold: " << angle_threshold << "°";
        }
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
    
    // Build retrieval options
    RetrievalOptions options;
    options.distance_threshold = distance_threshold;
    options.angle_threshold = angle_threshold;
    options.max_neighbors = max_neighbors;
    options.use_imu_filter = (angle_threshold > 0);
    options.window_size = window_size;
    options.max_pairs = max_pairs;
    options.verbose = cmd.used('v');
    
    // Check if VLAD or vocab tree strategies are used
    bool vlad_enabled = (strategy.find("vlad") != std::string::npos);
    bool vocab_enabled = (strategy.find("vocab") != std::string::npos);
    
    // Load VLAD centroids if needed
    std::vector<float> vlad_centroids;
    
    if (vlad_enabled) {
        if (vlad_codebook.empty()) {
            LOG(ERROR) << "VLAD strategy requires --vlad-codebook parameter";
            return 1;
        }
        
        int num_clusters = 0;
        vlad_centroids = loadVLADCentroids(vlad_codebook, num_clusters);
        
        if (vlad_centroids.empty()) {
            LOG(ERROR) << "Failed to load VLAD codebook from " << vlad_codebook;
            return 1;
        }
        
        LOG(INFO) << "Loaded VLAD codebook: " << num_clusters << " clusters";
        options.vlad_clusters = num_clusters;
        options.top_k = vlad_top_k;
    }
    
    // Validate vocabulary tree file if needed
    if (vocab_enabled) {
        if (vocab_file.empty()) {
            LOG(ERROR) << "Vocab tree strategy requires --vocab-file parameter";
            return 1;
        }
        
        if (!fs::exists(vocab_file)) {
            LOG(ERROR) << "Vocabulary file not found: " << vocab_file;
            return 1;
        }
        
        LOG(INFO) << "Using DBoW3 vocabulary: " << vocab_file;
        options.top_k = vocab_top_k;
    }
    
    // Create dynamic strategy registry (extends static STRATEGIES with VLAD/vocab tree)
    auto strategies = STRATEGIES;  // Copy static registry
    if (vlad_enabled) {
        strategies["vlad"] = [&vlad_centroids, &vlad_cache_dir](
            const std::vector<ImageInfo>& imgs,
            const RetrievalOptions& opts
        ) -> std::vector<ImagePair> {
            return retrieveByVLAD(imgs, opts, vlad_centroids, vlad_cache_dir);
        };
    }
    if (vocab_enabled) {
        strategies["vocab"] = [&vocab_file, &vocab_cache_dir](
            const std::vector<ImageInfo>& imgs,
            const RetrievalOptions& opts
        ) -> std::vector<ImagePair> {
            return retrieveByVocabTree(imgs, opts, vocab_file, vocab_cache_dir);
        };
    }
    if (vocab_enabled) {
        strategies["vocab"] = [&vocab_file, &vocab_cache_dir](
            const std::vector<ImageInfo>& imgs,
            const RetrievalOptions& opts
        ) -> std::vector<ImagePair> {
            return retrieveByVocabTree(imgs, opts, vocab_file, vocab_cache_dir);
        };
    }
    
    // Execute retrieval strategy
    std::vector<ImagePair> pairs;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto strategy_names = parseStrategyString(strategy);
    
    if (strategy_names.size() == 1) {
        // Single strategy
        auto it = strategies.find(strategy_names[0]);
        if (it == strategies.end()) {
            LOG(ERROR) << "Unknown strategy: " << strategy_names[0];
            LOG(ERROR) << "Available strategies: exhaustive, sequential, gps, vlad, vocab";
            return 1;
        }
        pairs = it->second(images, options);
    } else {
        // Combined strategies
        std::vector<std::vector<ImagePair>> all_pairs;
        for (const auto& name : strategy_names) {
            auto it = strategies.find(name);
            if (it == strategies.end()) {
                LOG(ERROR) << "Unknown strategy: " << name;
                continue;
            }
            LOG(INFO) << "Executing strategy: " << name;
            auto strategy_pairs = it->second(images, options);
            LOG(INFO) << "  Generated " << strategy_pairs.size() << " pairs";
            all_pairs.push_back(strategy_pairs);
        }
        
        // Combine and deduplicate
        pairs = combinePairs(all_pairs, true);
        LOG(INFO) << "Combined " << strategy_names.size() << " strategies -> " 
                  << pairs.size() << " unique pairs";
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    int gen_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    LOG(INFO) << "Generated " << pairs.size() << " pairs in " << gen_time << "ms";
    
    // Post-processing: filter by score and limit count
    pairs = filterPairs(pairs, [](const ImagePair& p) { return p.score > 0.01; });
    pairs = sortByScore(pairs);
    if (options.max_pairs > 0 && pairs.size() > static_cast<size_t>(options.max_pairs)) {
        pairs.resize(options.max_pairs);
        LOG(INFO) << "Limited to " << options.max_pairs << " pairs";
    }
    
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
