/**
 * @file test_database_serialization.cpp
 * @brief 数据库序列化单元测试
 */

#include <iostream>
#include <fstream>
#include <cereal/archives/binary.hpp>
#include "database/database_types.h"

using namespace insight::database;

int main() {
    std::cout << "Testing database serialization..." << std::endl;
    
    try {
        // 创建一个简单的项目
        Project project;
        project.name = "Test Project";
        project.author = "Test Author";
        project.uuid = "12345-67890";
        project.creation_time = std::time(nullptr);
        
        // 添加一个图像分组
        ImageGroup group;
        group.group_id = 1;
        group.group_name = "Test Group";
        group.camera_mode = ImageGroup::CameraMode::kGroupLevel;
        
        // 添加相机参数
        CameraModel camera;
        camera.width = 1920;
        camera.height = 1440;
        camera.focal_length = 1000.0;
        camera.principal_point_x = 960.0;
        camera.principal_point_y = 720.0;
        
        group.group_camera = camera;
        project.image_groups.push_back(group);
        
        // 测试序列化
        {
            std::ofstream ofs("/tmp/test_project.bin", std::ios::binary);
            cereal::BinaryOutputArchive archive(ofs);
            archive(project);
            std::cout << "✓ Serialization successful" << std::endl;
        }
        
        // 测试反序列化
        Project loaded_project;
        {
            std::ifstream ifs("/tmp/test_project.bin", std::ios::binary);
            cereal::BinaryInputArchive archive(ifs);
            archive(loaded_project);
            std::cout << "✓ Deserialization successful" << std::endl;
        }
        
        // 验证数据
        if (loaded_project.name == project.name &&
            loaded_project.author == project.author &&
            loaded_project.uuid == project.uuid) {
            std::cout << "✓ Data integrity verified" << std::endl;
            return 0;
        } else {
            std::cout << "✗ Data integrity check failed" << std::endl;
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }
}
