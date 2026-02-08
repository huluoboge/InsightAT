/**
 * @file test_serialization_comprehensive.cpp
 * @brief 完整的数据库序列化单元测试
 * @description 验证 Cereal 对所有数据类型的支持，包括 optional<T>
 */

#include <gtest/gtest.h>
#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/optional.hpp>  // Cereal 的 optional 支持
#include <cereal/types/vector.hpp>     // vector 支持
#include <cereal/types/map.hpp>        // map 支持
#include <sstream>
#include <optional>
#include <vector>
#include <map>

// 简单的测试类型
struct SimpleData {
    int id = 0;
    double value = 0.0;
    
    template <class Archive>
    void serialize(Archive& ar) {
        ar(id, value);
    }
    
    bool operator==(const SimpleData& other) const {
        return id == other.id && 
               std::abs(value - other.value) < 1e-9;
    }
};

// 包含 optional 的测试类型
struct OptionalData {
    std::optional<int> opt_id;
    std::optional<double> opt_value;
    
    template <class Archive>
    void serialize(Archive& ar) {
        ar(opt_id, opt_value);
    }
    
    bool operator==(const OptionalData& other) const {
        return opt_id == other.opt_id && 
               opt_value == other.opt_value;
    }
};

// 包含 vector 和 map 的复杂类型
struct ComplexData {
    std::vector<SimpleData> items;
    std::map<int, double> mapping;
    std::optional<SimpleData> optional_item;
    
    template <class Archive>
    void serialize(Archive& ar) {
        ar(items, mapping, optional_item);
    }
};

// 测试基础类型序列化
TEST(CerealSerialization, SimpleTypesBinary) {
    SimpleData original{42, 3.14};
    
    // 序列化
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        oar(original);
    }
    
    // 反序列化
    SimpleData loaded;
    {
        ss.seekg(0);
        cereal::BinaryInputArchive iar(ss);
        iar(loaded);
    }
    
    EXPECT_EQ(loaded, original);
}

// 测试 optional 类型序列化
TEST(CerealSerialization, OptionalTypesBinary) {
    OptionalData original;
    original.opt_id = 100;
    original.opt_value = 2.71;
    
    // 序列化
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        oar(original);
    }
    
    // 反序列化
    OptionalData loaded;
    {
        ss.seekg(0);
        cereal::BinaryInputArchive iar(ss);
        iar(loaded);
    }
    
    EXPECT_EQ(loaded, original);
}

// 测试 optional 空值情况
TEST(CerealSerialization, OptionalEmptyBinary) {
    OptionalData original;
    // 不设置任何值，保持 empty
    
    // 序列化
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        oar(original);
    }
    
    // 反序列化
    OptionalData loaded;
    {
        ss.seekg(0);
        cereal::BinaryInputArchive iar(ss);
        iar(loaded);
    }
    
    EXPECT_FALSE(loaded.opt_id.has_value());
    EXPECT_FALSE(loaded.opt_value.has_value());
}

// 测试 vector 序列化
TEST(CerealSerialization, VectorBinary) {
    std::vector<SimpleData> original;
    original.push_back({1, 1.1});
    original.push_back({2, 2.2});
    original.push_back({3, 3.3});
    
    // 序列化
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        oar(original);
    }
    
    // 反序列化
    std::vector<SimpleData> loaded;
    {
        ss.seekg(0);
        cereal::BinaryInputArchive iar(ss);
        iar(loaded);
    }
    
    EXPECT_EQ(loaded.size(), original.size());
    for (size_t i = 0; i < original.size(); ++i) {
        EXPECT_EQ(loaded[i], original[i]);
    }
}

// 测试 map 序列化
TEST(CerealSerialization, MapBinary) {
    std::map<int, double> original;
    original[1] = 1.1;
    original[2] = 2.2;
    original[3] = 3.3;
    
    // 序列化
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        oar(original);
    }
    
    // 反序列化
    std::map<int, double> loaded;
    {
        ss.seekg(0);
        cereal::BinaryInputArchive iar(ss);
        iar(loaded);
    }
    
    EXPECT_EQ(loaded, original);
}

// 测试复杂嵌套类型
TEST(CerealSerialization, ComplexNestedBinary) {
    ComplexData original;
    original.items.push_back({1, 1.1});
    original.items.push_back({2, 2.2});
    original.mapping[10] = 10.5;
    original.mapping[20] = 20.5;
    original.optional_item = SimpleData{99, 9.9};
    
    // 序列化
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        oar(original);
    }
    
    // 反序列化
    ComplexData loaded;
    {
        ss.seekg(0);
        cereal::BinaryInputArchive iar(ss);
        iar(loaded);
    }
    
    EXPECT_EQ(loaded.items.size(), original.items.size());
    EXPECT_EQ(loaded.mapping, original.mapping);
    EXPECT_TRUE(loaded.optional_item.has_value());
    if (loaded.optional_item.has_value()) {
        EXPECT_EQ(loaded.optional_item.value(), original.optional_item.value());
    }
}

// 测试 NVP 包装的序列化
TEST(CerealSerialization, NamedValuePairBinary) {
    SimpleData original{77, 7.7};
    
    // 序列化
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        oar(cereal::make_nvp("data", original));
    }
    
    // 反序列化
    SimpleData loaded;
    {
        ss.seekg(0);
        cereal::BinaryInputArchive iar(ss);
        iar(cereal::make_nvp("data", loaded));
    }
    
    EXPECT_EQ(loaded, original);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

