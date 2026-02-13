#include <chrono>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "../io/idc_writer.h"
#include "../modules/extraction/sift_gpu_extractor.h"
#include "cmdLine/cmdLine.h"
#include "task_queue/task_queue.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

struct ImageTask {
    std::string image_id;       // Unique image ID (for output filename)
    std::string image_path;
    cv::Mat image;              // Original image
    cv::Mat image_retrieval;    // Resized image for retrieval features
    int image_cols;             // cache original image size
    int image_rows;
    int image_retrieval_cols;   // cache resized image size
    int image_retrieval_rows;
    int camera_id;
    int index;

    // Matching features (full resolution)
    std::vector<SiftGPU::SiftKeypoint> keypoints;
    std::vector<float> descriptors;
    std::vector<unsigned char> descriptors_uchar;
    
    // Retrieval features (resized resolution)
    std::vector<SiftGPU::SiftKeypoint> keypoints_retrieval;
    std::vector<float> descriptors_retrieval;
    std::vector<unsigned char> descriptors_uchar_retrieval;
};

std::vector<ImageTask> loadImageList(const std::string& json_path)
{
    std::ifstream file(json_path);
    if (!file.is_open()) {
        LOG(FATAL) << "Failed to open image list: " << json_path;
    }

    json j;
    file >> j;

    std::vector<ImageTask> tasks;
    int index = 0;
    for (const auto& img : j["images"]) {
        ImageTask task;
        
        // Read image ID (required, unique identifier)
        if (!img.contains("id")) {
            LOG(ERROR) << "Image entry missing required 'id' field, skipping";
            continue;
        }
        task.image_id = std::to_string(img["id"].get<int>());
        
        // Read image path (required)
        if (!img.contains("path")) {
            LOG(ERROR) << "Image entry missing 'path' field for ID " << task.image_id;
            continue;
        }
        task.image_path = img["path"];
        task.camera_id = img.value("camera_id", 1);
        task.index = index++;
        tasks.push_back(task);
    }

    LOG(INFO) << "Loaded " << tasks.size() << " images from " << json_path;
    return tasks;
}

int main(int argc, char* argv[])
{
    // Initialize glog
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;

    // Define command line options with documentation
    CmdLine cmd("InsightAT SIFT Feature Extractor - GPU-accelerated SIFT feature extraction with RootSIFT normalization and NMS");

    std::string input_file;
    std::string output_dir;
    std::string output_retrieval_dir;  // Dual-output: retrieval features directory
    int nfeatures = 8000;
    int nfeatures_retrieval = 1500;     // Dual-output: retrieval features count
    int resize_retrieval = 1024;        // Dual-output: retrieval image resize dimension
    float threshold = 0.02f;
    int octaves = -1;
    int levels = 3;
    std::string normalization = "l1root";
    float nms_radius = 3.0f;

    // Required arguments
    cmd.add(make_option('i', input_file, "input")
            .doc("Input image list (JSON format)"));
    cmd.add(make_option('o', output_dir, "output")
            .doc("Output directory for matching .isat_feat files"));
    
    // Dual-output options
    cmd.add(make_option(0, output_retrieval_dir, "output-retrieval")
            .doc("Output directory for retrieval features (enables dual-output)"));
    cmd.add(make_option(0, nfeatures_retrieval, "nfeatures-retrieval")
            .doc("Maximum retrieval features (default: 1500)"));
    cmd.add(make_option(0, resize_retrieval, "resize-retrieval")
            .doc("Resize long edge for retrieval features (default: 1024)"));
    cmd.add(make_switch(0, "only-retrieval")
            .doc("Only output retrieval features (skip matching features)"));

    // Feature extraction parameters
    cmd.add(make_option('n', nfeatures, "nfeatures")
            .doc("Maximum features per image (default: 8000)"));
    cmd.add(make_option('t', threshold, "threshold")
            .doc("Peak threshold (default: 0.02)"));
    cmd.add(make_option(0, octaves, "octaves")
            .doc("Number of octaves, -1=auto (default: -1)"));
    cmd.add(make_option(0, levels, "levels")
            .doc("Levels per octave (default: 3)"));
    cmd.add(make_switch(0, "no-adapt")
            .doc("Disable dark image adaptation"));

    // Descriptor options
    cmd.add(make_option(0, normalization, "norm")
            .doc("Normalization: l1root (RootSIFT) or l2 (default: l1root)"));
    cmd.add(make_switch(0, "uint8")
            .doc("Convert descriptors to uint8 (saves memory)"));

    // NMS options
    cmd.add(make_switch(0, "nms")
            .doc("Enable non-maximum suppression"));
    cmd.add(make_option(0, nms_radius, "nms-radius")
            .doc("NMS radius in pixels (default: 3.0)"));
    cmd.add(make_switch(0, "nms-no-orient")
            .doc("NMS ignores orientation (removes multi-orientation)"));

    // Logging options
    cmd.add(make_switch('v', "verbose")
            .doc("Verbose logging (INFO level)"));
    cmd.add(make_switch('q', "quiet")
            .doc("Quiet mode (ERROR level only)"));

    // Help option
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
    if (input_file.empty() || output_dir.empty()) {
        std::cerr << "Error: -i/--input and -o/--output are required\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }
    
    // Process dual-output options
    bool only_retrieval = cmd.used("only-retrieval");
    bool enable_dual_output = !output_retrieval_dir.empty();
    bool process_matching = !only_retrieval;
    bool process_retrieval = enable_dual_output || only_retrieval;
    
    // Validation
    if (only_retrieval && output_retrieval_dir.empty()) {
        output_retrieval_dir = output_dir;  // Use main output dir for retrieval-only mode
        process_matching = false;
    }
    if (only_retrieval && enable_dual_output) {
        std::cerr << "Warning: --only-retrieval ignores --output-retrieval, using -o for retrieval features\n";
        output_retrieval_dir = output_dir;
    }

    // Process switches
    bool adapt_darkness = !cmd.used("no-adapt");
    bool use_uint8 = cmd.used("uint8");
    bool enable_nms = cmd.used("nms");
    bool nms_keep_orientation = !cmd.used("nms-no-orient");

    // Set logging level
    int log_level = google::WARNING;
    if (cmd.used('v'))
        log_level = google::INFO;
    if (cmd.used('q'))
        log_level = google::ERROR;
    FLAGS_minloglevel = log_level;

    // Setup SIFT parameters (pure extraction only)
    insight::modules::SiftGPUParams sift_params;
    sift_params.nMaxFeatures = nfeatures;
    sift_params.dPeak = threshold;
    sift_params.nOctives = octaves;
    sift_params.nLevel = levels;
    sift_params.adaptDarkness = adapt_darkness;
    
    // Setup SIFT parameters for retrieval features
    insight::modules::SiftGPUParams sift_params_retrieval;
    sift_params_retrieval.nMaxFeatures = nfeatures_retrieval;
    sift_params_retrieval.dPeak = threshold;
    sift_params_retrieval.nOctives = octaves;
    sift_params_retrieval.nLevel = levels;
    sift_params_retrieval.adaptDarkness = adapt_darkness;

    // Log configuration
    LOG(INFO) << "Feature extraction configuration:";
    if (process_matching) {
        LOG(INFO) << "  Matching features:";
        LOG(INFO) << "    Max features: " << nfeatures;
        LOG(INFO) << "    Output: " << output_dir;
    }
    if (process_retrieval) {
        LOG(INFO) << "  Retrieval features:";
        LOG(INFO) << "    Max features: " << nfeatures_retrieval;
        LOG(INFO) << "    Resize dimension: " << resize_retrieval;
        LOG(INFO) << "    Output: " << output_retrieval_dir;
    }
    LOG(INFO) << "  Threshold: " << threshold;
    LOG(INFO) << "  Normalization: " << normalization;
    LOG(INFO) << "  uint8 format: " << (use_uint8 ? "yes" : "no");
    LOG(INFO) << "  NMS enabled: " << (enable_nms ? "yes" : "no");
    if (enable_nms) {
        LOG(INFO) << "    NMS radius: " << nms_radius;
        LOG(INFO) << "    Keep orientations: " << (nms_keep_orientation ? "yes" : "no");
    }
    LOG(INFO) << "  Mode: " << (only_retrieval ? "retrieval-only" : 
                                 (enable_dual_output ? "dual-output" : "matching-only"));

    // Create output directories
    if (process_matching) fs::create_directories(output_dir);
    if (process_retrieval) fs::create_directories(output_retrieval_dir);

    // Load image list
    std::vector<ImageTask> image_tasks = loadImageList(input_file);
    int total_images = image_tasks.size();

    if (total_images == 0) {
        LOG(ERROR) << "No images to process";
        return 1;
    }

    // Create pipeline stages
    const int IO_QUEUE_SIZE = 10;
    const int GPU_QUEUE_SIZE = 5;
    const int NUM_IO_THREADS = 4;

    // Stage 1: Image loading (multi-threaded I/O)
    Stage imageLoadStage("ImageLoad", NUM_IO_THREADS, IO_QUEUE_SIZE,
        [&image_tasks, process_retrieval, resize_retrieval](int index) {
            auto& task = image_tasks[index];
            cv::Mat image = cv::imread(task.image_path);
            if (image.empty()) {
                LOG(ERROR) << "Failed to load image: " << task.image_path;
                return;
            }
            LOG(INFO) << "Loaded image [" << index << "]: " << task.image_path
                      << " (" << image.cols << "x" << image.rows << ")";
            task.image = image;
            
            // Prepare retrieval image if needed (dual-output or retrieval-only)
            if (process_retrieval) {
                int max_dim = std::max(image.rows, image.cols);
                if (max_dim > resize_retrieval) {
                    float scale = static_cast<float>(resize_retrieval) / max_dim;
                    cv::Mat image_resized;
                    cv::resize(image, image_resized, cv::Size(), scale, scale, cv::INTER_AREA);
                    task.image_retrieval = image_resized;
                    LOG(INFO) << "  Resized for retrieval: " << image_resized.cols 
                              << "x" << image_resized.rows;
                } else {
                    task.image_retrieval = image.clone();
                    LOG(INFO) << "  Retrieval image (no resize needed): " 
                              << image.cols << "x" << image.rows;
                }
            }
        });

    // Stage 2: SIFT GPU extraction (single GPU, use reconfigure for dual-output)
    // NOTE: SiftGPU uses global state, so we use ONE extractor and reconfigure parameters
    insight::modules::SiftGPUExtractor extractor(sift_params);
    if (!extractor.initialize()) {
        LOG(FATAL) << "Failed to initialize SiftGPU";
    }

    StageCurrent siftGPUStage("SiftGPU", 1, GPU_QUEUE_SIZE,
        [&image_tasks, &extractor, &sift_params, &sift_params_retrieval, 
         process_matching, process_retrieval](int index) {
            auto& task = image_tasks[index];
            auto start = std::chrono::high_resolution_clock::now();
            
            int num_features_matching = 0;
            int num_features_retrieval = 0;
            
            // Extract matching features from original image
            if (process_matching && !task.image.empty()) {
                // Configure for matching features (high count)
                extractor.reconfigure(sift_params);
                
                num_features_matching = extractor.extract(
                    task.image, task.keypoints, task.descriptors);
                task.image_cols = task.image.cols;
                task.image_rows = task.image.rows;
                task.image.release(); // Free original image memory
            }
            
            // Extract retrieval features from resized image
            if (process_retrieval && !task.image_retrieval.empty()) {
                // Reconfigure for retrieval features (lower count)
                extractor.reconfigure(sift_params_retrieval);
                
                num_features_retrieval = extractor.extract(
                    task.image_retrieval, task.keypoints_retrieval, task.descriptors_retrieval);
                task.image_retrieval_cols = task.image_retrieval.cols;
                task.image_retrieval_rows = task.image_retrieval.rows;
                task.image_retrieval.release(); // Free resized image memory
            }

            auto end = std::chrono::high_resolution_clock::now();
            int exec_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            if (process_matching && num_features_matching == 0) {
                LOG(WARNING) << "No matching features extracted from [" << index 
                             << "] - " << task.image_path;
            }
            if (process_retrieval && num_features_retrieval == 0) {
                LOG(WARNING) << "No retrieval features extracted from [" << index 
                             << "] - " << task.image_path;
            }
            
            if (process_matching && process_retrieval) {
                LOG(INFO) << "Extracted [" << index << "] in " << exec_time << "ms: "
                          << num_features_matching << " matching, "
                          << num_features_retrieval << " retrieval features";
            } else if (process_matching) {
                LOG(INFO) << "Extracted " << num_features_matching << " matching features from ["
                          << index << "] in " << exec_time << "ms";
            } else {
                LOG(INFO) << "Extracted " << num_features_retrieval << " retrieval features from ["
                          << index << "] in " << exec_time << "ms";
            }
        });

    // Stage 3: CPU post-processing (normalization, distribution, uint8 conversion)
    Stage postProcessStage("PostProcess", NUM_IO_THREADS, IO_QUEUE_SIZE,
        [&image_tasks, use_uint8, enable_nms, normalization,
            nms_radius, nms_keep_orientation, process_matching, process_retrieval](int index) {
            auto& task = image_tasks[index];

            // Process matching features
            if (process_matching && !task.keypoints.empty()) {
                // Step 1: Apply normalization (CPU)
                if (normalization == "l2") {
                    insight::modules::L2NormalizeDescriptors(task.descriptors, 128);
                } else { // l1root
                    insight::modules::L1RootNormalizeDescriptors(task.descriptors, 128);
                }

                // Step 2: Apply feature distribution if enabled (CPU)
                if (enable_nms) {
                    insight::modules::ApplyFeatureDistribution(
                        task.keypoints, task.descriptors,
                        task.image_cols, task.image_rows,
                        static_cast<int>(nms_radius * 10), // Grid size ~10x radius
                        2, // Max 2 features per cell
                        nms_keep_orientation);
                }

                // Step 3: Convert to uint8 if needed (CPU)
                if (use_uint8) {
                    task.descriptors_uchar = insight::modules::ConvertDescriptorsToUChar(
                        task.descriptors, 128);
                    // Free float descriptors to save memory
                    task.descriptors.clear();
                    task.descriptors.shrink_to_fit();
                }
            }
            
            // Process retrieval features
            if (process_retrieval && !task.keypoints_retrieval.empty()) {
                // Step 1: Apply normalization (CPU)
                if (normalization == "l2") {
                    insight::modules::L2NormalizeDescriptors(task.descriptors_retrieval, 128);
                } else { // l1root
                    insight::modules::L1RootNormalizeDescriptors(task.descriptors_retrieval, 128);
                }

                // Step 2: Apply feature distribution if enabled (CPU)
                if (enable_nms) {
                    insight::modules::ApplyFeatureDistribution(
                        task.keypoints_retrieval, task.descriptors_retrieval,
                        task.image_retrieval_cols, task.image_retrieval_rows,
                        static_cast<int>(nms_radius * 10),
                        2,
                        nms_keep_orientation);
                }

                // Step 3: Convert to uint8 if needed (CPU)
                if (use_uint8) {
                    task.descriptors_uchar_retrieval = insight::modules::ConvertDescriptorsToUChar(
                        task.descriptors_retrieval, 128);
                    task.descriptors_retrieval.clear();
                    task.descriptors_retrieval.shrink_to_fit();
                }
            }
        });

    // Stage 4: Write IDC files (multi-threaded I/O, dual-output support)
    Stage writeStage("WriteIDC", NUM_IO_THREADS, IO_QUEUE_SIZE,
        [&output_dir, &output_retrieval_dir, &image_tasks, use_uint8, enable_nms, normalization,
            nms_radius, nms_keep_orientation, &sift_params, &sift_params_retrieval,
            process_matching, process_retrieval](int index) {
            auto& task = image_tasks[index];
            
            // Use image_id for output filename (NOT filesystem path stem!)
            std::string base_filename = task.image_id + ".isat_feat";
            
            // Helper lambda to write features to file
            auto write_features = [&](const std::string& output_path,
                                      const std::vector<SiftGPU::SiftKeypoint>& keypoints,
                                      const std::vector<float>& descriptors,
                                      const std::vector<unsigned char>& descriptors_uchar,
                                      const insight::modules::SiftGPUParams& params,
                                      const std::string& feature_type) {
                if (keypoints.empty()) return false;
                
                insight::io::IDCWriter writer(output_path);

                json params_json;
                params_json["nfeatures"] = params.nMaxFeatures;
                params_json["threshold"] = params.dPeak;
                params_json["octaves"] = params.nOctives;
                params_json["levels"] = params.nLevel;
                params_json["adapt_darkness"] = params.adaptDarkness;
                params_json["normalization"] = normalization;
                params_json["uint8"] = use_uint8;
                params_json["nms_enabled"] = enable_nms;
                params_json["feature_type"] = feature_type;  // "matching" or "retrieval"
                if (enable_nms) {
                    params_json["nms_radius"] = nms_radius;
                    params_json["nms_keep_orientation"] = nms_keep_orientation;
                }

                auto metadata = insight::io::createFeatureMetadata(
                    task.image_path,
                    "SIFT_GPU",
                    "1.2",  // Version bump for dual-output support
                    params_json,
                    0);

                writer.setMetadata(metadata);

                // Add keypoints (x, y, scale, orientation)
                std::vector<float> kpt_data;
                for (const auto& kp : keypoints) {
                    kpt_data.push_back(kp.x);
                    kpt_data.push_back(kp.y);
                    kpt_data.push_back(kp.s);
                    kpt_data.push_back(kp.o);
                }

                writer.addBlob("keypoints", kpt_data.data(),
                    kpt_data.size() * sizeof(float),
                    "float32", { (int)keypoints.size(), 4 });

                // Add descriptors (uint8 or float32)
                if (use_uint8) {
                    writer.addBlob("descriptors", descriptors_uchar.data(),
                        descriptors_uchar.size() * sizeof(unsigned char),
                        "uint8", { (int)keypoints.size(), 128 });
                } else {
                    writer.addBlob("descriptors", descriptors.data(),
                        descriptors.size() * sizeof(float),
                        "float32", { (int)keypoints.size(), 128 });
                }

                return writer.write();
            };
            
            // Write matching features
            if (process_matching) {
                std::string output_path = fs::path(output_dir) / base_filename;
                if (write_features(output_path, task.keypoints, task.descriptors,
                                   task.descriptors_uchar, sift_params, "matching")) {
                    LOG(INFO) << "Written matching features [" << index << "]: " << output_path;
                }
            }
            
            // Write retrieval features
            if (process_retrieval) {
                std::string output_path = fs::path(output_retrieval_dir) / base_filename;
                if (write_features(output_path, task.keypoints_retrieval, 
                                   task.descriptors_retrieval,
                                   task.descriptors_uchar_retrieval, 
                                   sift_params_retrieval, "retrieval")) {
                    LOG(INFO) << "Written retrieval features [" << index << "]: " << output_path;
                }
            }
            
            // Progress reporting
            std::cerr << "PROGRESS: " << (float)(index + 1) / image_tasks.size() << "\n";

            // Clear memory
            task = ImageTask(); // release all memory
        });

    // Chain stages
    chain(imageLoadStage, siftGPUStage);
    chain(siftGPUStage, postProcessStage);
    chain(postProcessStage, writeStage);

    // Set task counts
    imageLoadStage.setTaskCount(total_images);
    siftGPUStage.setTaskCount(total_images);
    postProcessStage.setTaskCount(total_images);
    writeStage.setTaskCount(total_images);

    // Start processing
    auto start_time = std::chrono::high_resolution_clock::now();

    // Push tasks in background thread, process GPU in main thread
    std::thread push_thread([&]() {
        for (int i = 0; i < total_images; ++i) {
            imageLoadStage.push(i);
        }
    });

    // Run GPU stage in main thread (OpenGL context requirement)
    siftGPUStage.run();

    // Wait for all stages to complete
    push_thread.join();
    imageLoadStage.wait();
    postProcessStage.wait();
    writeStage.wait();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    LOG(INFO) << "Feature extraction completed in " << total_time << "s";
    LOG(INFO) << "Average time per image: " << (float)total_time / total_images << "s";

    return 0;
}
