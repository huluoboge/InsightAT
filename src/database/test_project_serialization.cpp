/**
 * @file test_project_serialization.cpp
 * @brief 项目序列化集成测试
 */

#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>

// 包含数据库类型
#include "database/database_types.h"

using namespace insight::database;

/**
 * 测试基础 Project 序列化
 */
TEST(ProjectSerialization, BasicProjectSerialization) {
    // 创建临时文件
    char tmpfile[] = "/tmp/test_project_basic_XXXXXX.db";
    FILE* fp = fopen(tmpfile, "w");
    ASSERT_TRUE(fp != nullptr);
    fclose(fp);
    
    // 创建项目
    Project original;
    original.name = "Test Project";
    original.author = "Test Author";
    original.description = "Test Description";
    
    // 保存
    {
        std::ofstream ofs(tmpfile, std::ios::binary);
        ASSERT_TRUE(ofs.is_open());
        cereal::BinaryOutputArchive ar(ofs);
        ar(original);
    }
    
    // 加载
    Project loaded;
    {
        std::ifstream ifs(tmpfile, std::ios::binary);
        ASSERT_TRUE(ifs.is_open());
        cereal::BinaryInputArchive ar(ifs);
        ar(loaded);
    }
    
    // 验证
    EXPECT_EQ(loaded.name, original.name);
    EXPECT_EQ(loaded.author, original.author);
    EXPECT_EQ(loaded.description, original.description);
    
    // 清理
    remove(tmpfile);
}

/**
 * 测试包含 ImageGroup 的 Project 序列化
 */
TEST(ProjectSerialization, ProjectWithImageGroups) {
    char tmpfile[] = "/tmp/test_project_groups_XXXXXX.db";
    FILE* fp = fopen(tmpfile, "w");
    ASSERT_TRUE(fp != nullptr);
    fclose(fp);
    
    // 创建项目并添加 ImageGroup
    Project original;
    original.name = "Project with Groups";
    
    ImageGroup group1;
    group1.group_id = 1;
    group1.group_name = "Group 1";
    group1.camera_mode = ImageGroup::CameraMode::kGroupLevel;
    original.image_groups.push_back(group1);
    
    // 保存
    {
        std::ofstream ofs(tmpfile, std::ios::binary);
        cereal::BinaryOutputArchive ar(ofs);
        ar(original);
    }
    
    // 加载
    Project loaded;
    {
        std::ifstream ifs(tmpfile, std::ios::binary);
        cereal::BinaryInputArchive ar(ifs);
        ar(loaded);
    }
    
    // 验证
    EXPECT_EQ(loaded.name, original.name);
    EXPECT_EQ(loaded.image_groups.size(), 1);
    EXPECT_EQ(loaded.image_groups[0].group_name, "Group 1");
    EXPECT_EQ(loaded.image_groups[0].camera_mode, ImageGroup::CameraMode::kGroupLevel);
    
    // 清理
    remove(tmpfile);
}

/**
 * 测试包含可选值的 Project 序列化
 */
TEST(ProjectSerialization, ProjectWithOptionalCamera) {
    char tmpfile[] = "/tmp/test_project_optional_XXXXXX.db";
    FILE* fp = fopen(tmpfile, "w");
    ASSERT_TRUE(fp != nullptr);
    fclose(fp);
    
    // 创建项目
    Project original;
    original.name = "Project with Optional";
    
    // 创建包含可选相机的 ImageGroup
    ImageGroup group;
    group.group_id = 1;
    group.group_name = "Test Group";
    group.camera_mode = ImageGroup::CameraMode::kGroupLevel;
    
    // 设置一个相机模型
    CameraModel camera;
    camera.width = 1920;
    camera.height = 1080;
    camera.focal_length = 1000.0;
    camera.principal_point_x = 960.0;
    camera.principal_point_y = 540.0;
    group.group_camera = camera;
    
    original.image_groups.push_back(group);
    
    // 保存
    {
        std::ofstream ofs(tmpfile, std::ios::binary);
        cereal::BinaryOutputArchive ar(ofs);
        ar(original);
    }
    
    // 加载
    Project loaded;
    {
        std::ifstream ifs(tmpfile, std::ios::binary);
        cereal::BinaryInputArchive ar(ifs);
        ar(loaded);
    }
    
    // 验证
    EXPECT_EQ(loaded.name, original.name);
    EXPECT_EQ(loaded.image_groups.size(), 1);
    EXPECT_TRUE(loaded.image_groups[0].group_camera.has_value());
    EXPECT_EQ(loaded.image_groups[0].group_camera.value().width, 1920);
    EXPECT_EQ(loaded.image_groups[0].group_camera.value().height, 1080);
    EXPECT_EQ(loaded.image_groups[0].group_camera.value().focal_length, 1000.0);
    
    // 清理
    remove(tmpfile);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
