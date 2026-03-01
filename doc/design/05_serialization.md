# 05 - 数据持久化 (Serialization)

InsightAT 使用 **Cereal** 库进行二进制序列化，所有 `.db` 文件均为版本化的二进制格式。

## 1. 为什么选择 Cereal？
- **零开销**：二进制流非常小，加载速度快。
- **头文件模式**：无需预编译复杂的 schema。
- **标准库支持**：原生支持 `std::vector`, `std::optional`, `std::map`, `std::string`。

## 2. 版本控制 (Versioning)
为了支持软件长期的 schema 演进，每个核心类都必须使用版本号。

### 实现规范
```cpp
// 1. 声明版本号
CEREAL_CLASS_VERSION(MyType, 1);

// 2. 实现 serialize 方法
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    if (version == 0) {
        // 加载/保存旧版本字段
        ar(CEREAL_NVP(old_field));
    } else {
        // 加载/保存新版本字段
        ar(CEREAL_NVP(old_field));
        ar(CEREAL_NVP(new_rotation_field));
        ar(CEREAL_NVP(is_valid));
    }
}
```

## 3. 注意事项
- **二进制不稳定性**：不同操作系统/编译器生成的二进制在底层对齐上可能不同（尽管 Cereal 做了很多工作）。目前主要保证在主流 x64 Linux/Windows 上的兼容性。
- **诊断信息**：加载失败时，通常会抛出 `cereal::Exception`，UI 层需捕获该异常并提示用户版本不匹配。
- **NVP (Name-Value Pair)**：必须使用 `CEREAL_NVP` 宏，这在未来切换到 JSON/XML 存档时非常有用。
