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
    std::string image_path;
    cv::Mat image;
    int image_cols; // cache image size
    int image_rows;
    int camera_id;
    int index;

    std::vector<SiftGPU::SiftKeypoint> keypoints;
    std::vector<float> descriptors;
    std::vector<unsigned char> descriptors_uchar;
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
    int nfeatures = 8000;
    float threshold = 0.02f;
    int octaves = -1;
    int levels = 3;
    std::string normalization = "l1root";
    float nms_radius = 3.0f;

    // Required arguments
    cmd.add(make_option('i', input_file, "input")
            .doc("Input image list (JSON format)"));
    cmd.add(make_option('o', output_dir, "output")
            .doc("Output directory for .isat_feat files"));

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

    // Log configuration
    LOG(INFO) << "Feature extraction configuration:";
    LOG(INFO) << "  Max features: " << nfeatures;
    LOG(INFO) << "  Threshold: " << threshold;
    LOG(INFO) << "  Normalization: " << normalization;
    LOG(INFO) << "  uint8 format: " << (use_uint8 ? "yes" : "no");
    LOG(INFO) << "  NMS enabled: " << (enable_nms ? "yes" : "no");
    if (enable_nms) {
        LOG(INFO) << "    NMS radius: " << nms_radius;
        LOG(INFO) << "    Keep orientations: " << (nms_keep_orientation ? "yes" : "no");
    }

    // Create output directory
    fs::create_directories(output_dir);

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
        [&image_tasks](int index) {
            auto& task = image_tasks[index];
            cv::Mat image = cv::imread(task.image_path);
            if (image.empty()) {
                LOG(ERROR) << "Failed to load image: " << task.image_path;
                return;
            }
            LOG(INFO) << "Loaded image [" << index << "]: " << task.image_path
                      << " (" << image.cols << "x" << image.rows << ")";
            task.image = image;
        });

    // Stage 2: SIFT GPU extraction (single GPU, pure extraction only)
    insight::modules::SiftGPUExtractor extractor(sift_params);
    if (!extractor.initialize()) {
        LOG(FATAL) << "Failed to initialize SiftGPU";
    }

    StageCurrent siftGPUStage("SiftGPU", 1, GPU_QUEUE_SIZE,
        [&image_tasks, &extractor](int index) {
            auto& task = image_tasks[index];
            cv::Mat image = task.image;
            if (image.empty())
                return;

            auto start = std::chrono::high_resolution_clock::now();

            // GPU: pure feature extraction → float descriptors
            int num_features = extractor.extract(image, task.keypoints, task.descriptors);

            task.image_cols = image.cols;
            task.image_rows = image.rows;
            task.image.release(); // Free image memory

            auto end = std::chrono::high_resolution_clock::now();
            int exec_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            if (num_features == 0) {
                LOG(WARNING) << "No features extracted from [" << index << "] - " << task.image_path;
            } else {
                LOG(INFO) << "Extracted " << num_features << " features from [" << index
                          << "] in " << exec_time << "ms";
            }
        });

    // Stage 3: CPU post-processing (normalization, distribution, uint8 conversion)
    Stage postProcessStage("PostProcess", NUM_IO_THREADS, IO_QUEUE_SIZE,
        [&image_tasks, use_uint8, enable_nms, normalization,
            nms_radius, nms_keep_orientation](int index) {
            auto& task = image_tasks[index];

            if (task.keypoints.empty())
                return;

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
        });

    // Stage 4: Write IDC files (multi-threaded I/O)
    Stage writeStage("WriteIDC", NUM_IO_THREADS, IO_QUEUE_SIZE,
        [&output_dir, &image_tasks, use_uint8, enable_nms, normalization,
            nms_radius, nms_keep_orientation, &sift_params](int index) {
            auto& task = image_tasks[index];

            // Write IDC file
            std::string output_filename = fs::path(task.image_path).stem().string() + ".isat_feat";
            std::string output_path = fs::path(output_dir) / output_filename;

            insight::io::IDCWriter writer(output_path);

            json params_json;
            params_json["nfeatures"] = sift_params.nMaxFeatures;
            params_json["threshold"] = sift_params.dPeak;
            params_json["octaves"] = sift_params.nOctives;
            params_json["levels"] = sift_params.nLevel;
            params_json["adapt_darkness"] = sift_params.adaptDarkness;
            params_json["normalization"] = normalization;
            params_json["uint8"] = use_uint8;
            params_json["nms_enabled"] = enable_nms;
            if (enable_nms) {
                params_json["nms_radius"] = nms_radius;
                params_json["nms_keep_orientation"] = nms_keep_orientation;
            }

            auto metadata = insight::io::createFeatureMetadata(
                task.image_path,
                "SIFT_GPU",
                "1.1",
                params_json,
                0);

            writer.setMetadata(metadata);

            // Add keypoints (x, y, scale, orientation)
            std::vector<float> kpt_data;
            for (const auto& kp : task.keypoints) {
                kpt_data.push_back(kp.x);
                kpt_data.push_back(kp.y);
                kpt_data.push_back(kp.s);
                kpt_data.push_back(kp.o);
            }

            writer.addBlob("keypoints", kpt_data.data(),
                kpt_data.size() * sizeof(float),
                "float32", { (int)task.keypoints.size(), 4 });

            // Add descriptors (uint8 or float32)
            if (use_uint8) {
                writer.addBlob("descriptors", task.descriptors_uchar.data(),
                    task.descriptors_uchar.size() * sizeof(unsigned char),
                    "uint8", { (int)task.keypoints.size(), 128 });
            } else {
                writer.addBlob("descriptors", task.descriptors.data(),
                    task.descriptors.size() * sizeof(float),
                    "float32", { (int)task.keypoints.size(), 128 });
            }

            if (writer.write()) {
                LOG(INFO) << "Written features [" << index << "]: " << output_path;
                std::cerr << "PROGRESS: " << (float)(index + 1) / image_tasks.size() << "\n";
            }

            // Clear memory
            // task.keypoints.clear();
            // task.descriptors.clear();
            // task.descriptors_uchar.clear();
            // task.keypoints.shrink_to_fit();
            // task.descriptors.shrink_to_fit();
            // task.descriptors_uchar.shrink_to_fit();
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
