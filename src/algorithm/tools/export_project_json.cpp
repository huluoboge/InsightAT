#include <chrono>
#include <filesystem>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/optional.hpp>

#include "cmdLine/cmdLine.h"
#include "../database/database_types.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace insight::database;

/**
 * Load project from JSON file (matching ProjectDocument.cpp format)
 */
bool loadProjectFromFile(const std::string& filepath, Project& project) {
    std::ifstream ifs(filepath);  // Text mode for JSON
    if (!ifs.is_open()) {
        LOG(ERROR) << "Failed to open project file: " << filepath;
        return false;
    }
    
    try {
        cereal::JSONInputArchive archive(ifs);
        archive(cereal::make_nvp("project", project));
        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to deserialize project: " << e.what();
        return false;
    }
}

/**
 * Export Project images to CLI-compatible JSON (v2.0 format)
 * 
 * Usage:
 *   ./export_project_json -p project.iat -o image_list.json
 */
int main(int argc, char* argv[]) {
    // Initialize glog
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    
    // Command line options
    CmdLine cmd("InsightAT Project JSON Exporter - Export Project to CLI-compatible JSON format");
    
    std::string project_file;
    std::string output_file;
    std::string task_id = "";        // Export from ATTask (optional)
    std::string image_root = "";  // Optional: rewrite image paths relative to this root
    bool export_gnss = true;
    bool export_imu = true;
    bool export_all_groups = false;  // Export all groups or only first group
    int target_group_id = -1;        // Export specific group ID (-1 = all/first)
    
    // Required arguments
    cmd.add(make_option('p', project_file, "project")
        .doc("Input project file (.iat)"));
    cmd.add(make_option('o', output_file, "output")
        .doc("Output JSON file (v2.0 format)"));
    
    // Optional arguments
    cmd.add(make_option('t', task_id, "task-id")
        .doc("Export from ATTask UUID (default: use Project data)"));  
    cmd.add(make_option('r', image_root, "root")
        .doc("Image root path (rewrite paths relative to this)"));
    cmd.add(make_option('g', target_group_id, "group-id")
        .doc("Export specific group ID (-1=first, -2=all, default: -1)"));
    cmd.add(make_switch('a', "all-groups")
        .doc("Export all image groups (instead of first only)"));
    cmd.add(make_switch(0, "no-gnss")
        .doc("Exclude GNSS data from export"));
    cmd.add(make_switch(0, "no-imu")
        .doc("Exclude IMU data from export"));
    
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
    if (project_file.empty() || output_file.empty()) {
        std::cerr << "Error: -p/--project and -o/--output are required\n\n";
        cmd.printHelp(std::cerr, argv[0]);
        return 1;
    }
    
    // Set flags
    export_gnss = !cmd.used("no-gnss");
    export_imu = !cmd.used("no-imu");
    export_all_groups = cmd.used('a');
    
    // Set logging level
    if (cmd.used('v')) FLAGS_minloglevel = 0;  // INFO
    else if (cmd.used('q')) FLAGS_minloglevel = 2;  // ERROR
    else FLAGS_minloglevel = 1;  // WARNING
    
    LOG(INFO) << "=== Project JSON Exporter ===";
    LOG(INFO) << "Project file: " << project_file;
    LOG(INFO) << "Output file: " << output_file;
    if (!image_root.empty()) {
        LOG(INFO) << "Image root: " << image_root;
    }
    
    // Load project
    LOG(INFO) << "Loading project...";
    Project project;
    if (!loadProjectFromFile(project_file, project)) {
        LOG(ERROR) << "Failed to load project from " << project_file;
        return 1;
    }
    
    LOG(INFO) << "Project loaded: " << project.name;
    LOG(INFO) << "  Image groups: " << project.image_groups.size();
    LOG(INFO) << "  Total images: " << project.GetTotalImageCount();
    
    // ────────────────────────────────────────────────────────────
    // Determine data source: ATTask or Project
    // ────────────────────────────────────────────────────────────
    std::vector<ImageGroup> source_groups_storage;  // Own the data if from ATTask
    const std::vector<ImageGroup>* source_groups = nullptr;
    const CoordinateSystem* source_coord_sys = nullptr;
    std::string export_source_name;
    
    if (!task_id.empty()) {
        // Export from ATTask.input_snapshot
        const ATTask* task = nullptr;
        for (const auto& t : project.at_tasks) {
            if (t.id == task_id) {
                task = &t;
                break;
            }
        }
        if (!task) {
            LOG(ERROR) << "ATTask not found: " << task_id;
            return 1;
        }
        LOG(INFO) << "Exporting from ATTask: " << task->task_name 
                  << " (ID: " << task->task_id << ")";
        
        // Copy data from input_snapshot
        source_groups_storage = task->input_snapshot.image_groups;
        source_groups = &source_groups_storage;
        source_coord_sys = &task->input_snapshot.input_coordinate_system;
        export_source_name = task->task_name;
    } else {
        // Export from Project
        LOG(INFO) << "Exporting from Project (no task specified)";
        source_groups = &project.image_groups;
        source_coord_sys = &project.input_coordinate_system;
        export_source_name = project.name;
    }
    
    // Build JSON
    json output;
    output["$schema"] = "InsightAT Image List Format v2.0";
    output["images"] = json::array();
    
    // Determine which groups to export
    std::vector<const ImageGroup*> groups_to_export;
    if (export_all_groups || target_group_id == -2) {
        // Export all groups
        for (const auto& group : *source_groups) {
            groups_to_export.push_back(&group);
        }
    } else if (target_group_id >= 0) {
        // Export specific group
        bool found = false;
        for (const auto& group : *source_groups) {
            if (group.group_id == static_cast<uint32_t>(target_group_id)) {
                groups_to_export.push_back(&group);
                found = true;
                break;
            }
        }
        if (!found) {
            LOG(ERROR) << "Group ID " << target_group_id << " not found";
            return 1;
        }
    } else {
        // Export first group (default)
        if (!source_groups->empty()) {
            groups_to_export.push_back(&(*source_groups)[0]);
        }
    }
    
    if (groups_to_export.empty()) {
        LOG(ERROR) << "No image groups to export";
        return 1;
    }
    
    // Export images
    int total_exported = 0;
    int gnss_count = 0;
    int imu_count = 0;
    
    for (const auto* group : groups_to_export) {
        LOG(INFO) << "Exporting group: " << group->group_name 
                  << " (ID: " << group->group_id << ", " << group->images.size() << " images)";
        
        for (const auto& image : group->images) {
            json img_obj;
            
            // Required fields
            img_obj["id"] = image.image_id;
            
            // Image path
            std::string image_path = image.filename;
            if (!image_root.empty()) {
                // Rewrite path relative to root
                fs::path full_path = fs::path(image_root) / fs::path(image.filename).filename();
                image_path = full_path.string();
            }
            img_obj["path"] = image_path;
            
            // Camera ID (from group or image)
            img_obj["camera_id"] = group->group_id;  // Use group_id as camera_id for now
            
            // Export GNSS data
            if (export_gnss && image.gnss_data.has_value()) {
                const auto& gnss = image.gnss_data.value();
                json gnss_obj;
                gnss_obj["x"] = gnss.x;
                gnss_obj["y"] = gnss.y;
                gnss_obj["z"] = gnss.z;
                gnss_obj["cov_xx"] = gnss.cov_xx;
                gnss_obj["cov_yy"] = gnss.cov_yy;
                gnss_obj["cov_zz"] = gnss.cov_zz;
                gnss_obj["cov_xy"] = gnss.cov_xy;
                gnss_obj["cov_xz"] = gnss.cov_xz;
                gnss_obj["cov_yz"] = gnss.cov_yz;
                gnss_obj["num_satellites"] = gnss.num_satellites;
                gnss_obj["hdop"] = gnss.hdop;
                gnss_obj["vdop"] = gnss.vdop;
                img_obj["gnss"] = gnss_obj;
                gnss_count++;
            }
            
            // Export IMU data (if available in input_pose)
            if (export_imu && image.input_pose.has_rotation) {
                json imu_obj;
                // Convert to degrees if stored in radians
                double omega_deg = image.input_pose.omega;
                double phi_deg = image.input_pose.phi;
                double kappa_deg = image.input_pose.kappa;
                
                if (image.input_pose.angle_unit == InputPose::AngleUnit::kRadians) {
                    omega_deg *= 180.0 / M_PI;
                    phi_deg *= 180.0 / M_PI;
                    kappa_deg *= 180.0 / M_PI;
                }
                
                imu_obj["roll"] = omega_deg;
                imu_obj["pitch"] = phi_deg;
                imu_obj["yaw"] = kappa_deg;
                // Note: Covariance not currently stored in InputPose
                imu_obj["cov_att_xx"] = 0.1;  // Default values
                imu_obj["cov_att_yy"] = 0.1;
                imu_obj["cov_att_zz"] = 0.1;
                img_obj["imu"] = imu_obj;
                imu_count++;
            }
            
            output["images"].push_back(img_obj);
            total_exported++;
        }
    }
    
    // Add metadata
    json metadata;
    metadata["format_version"] = "2.0";
    metadata["exported_from"] = export_source_name;
    if (!task_id.empty()) {
        metadata["task_id"] = task_id;
    }
    metadata["exported_at"] = std::chrono::system_clock::now().time_since_epoch().count();
    metadata["coordinate_system"] = source_coord_sys->definition.empty()
        ? "Unknown" 
        : source_coord_sys->definition;
    metadata["angle_unit"] = "degrees";
    metadata["num_groups_exported"] = groups_to_export.size();
    output["metadata"] = metadata;
    
    // Write to file
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        LOG(ERROR) << "Failed to open output file: " << output_file;
        return 1;
    }
    
    out_file << output.dump(2);  // Pretty print with 2-space indent
    out_file.close();
    
    LOG(INFO) << "=== Export Complete ===";
    LOG(INFO) << "Exported images: " << total_exported;
    LOG(INFO) << "  With GNSS: " << gnss_count;
    LOG(INFO) << "  With IMU: " << imu_count;
    LOG(INFO) << "Output file: " << output_file;
    
    return 0;
}
