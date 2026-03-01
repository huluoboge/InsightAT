# 坐标系数据库加载修复 - 完整说明

## 📋 问题分析

### 原始问题
软件在加载坐标系数据库时存在以下问题：

1. **数据库位置不正确**：尝试从 `./data/coordinates.db` 加载单一文件
2. **数据库格式不匹配**：系统实际使用两个独立的 CSV 文件
3. **构建流程不完整**：数据文件未自动复制到可执行文件目录
4. **路径解析错误**：未使用系统的 stlplus 工具正确构建路径

### 实际架构（正确的做法）

项目的坐标系加载流程（来自 [src/Common/Project.cpp](src/Common/Project.cpp#L709-L728)）：

```cpp
bool SystemConfig::readCoordinate()
{
    std::string geoCoord = stlplus::create_filespec(configPath(), "GEOGCS_Database.csv");
    std::string prjCoord = stlplus::create_filespec(configPath(), "PROJCS_Database.csv");
    
    // 分别加载地理和投影坐标系
    parseCoordinates(ProjCoordinate, prjCoord);
    parseCoordinates(GeoCoordinate, geoCoord);
}
```

两个数据库：
- **GEOGCS_Database.csv** (675 条) - 地理坐标系（椭球体）
- **PROJCS_Database.csv** (4781 条) - 投影坐标系（投影）

---

## ✅ 解决方案执行

### 1️⃣ CMake 构建系统整合

**修改文件**: `src/CMakeLists.txt`

```cmake
# 添加 POST_BUILD 命令到 InsightAT_New 目标
add_custom_command(TARGET InsightAT_New POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/data/config"
        "${CMAKE_BINARY_DIR}/config"
    COMMENT "Copying coordinate system databases to build directory"
)
```

**效果**：
- 每次编译后，自动将 `data/config/` 整个目录复制到 `build/config/`
- 确保可执行文件旁边始终有最新的数据库
- 无需手动复制文件

### 2️⃣ SpatialReferenceTool 重构

**核心变更**：支持同时加载两个数据库

```cpp
// API 变更
bool loadCoordinateDatabases(const std::string& configPath);  // 新方法
```

**实现逻辑**（[SpatialReferenceTool.cpp](src/ui/widgets/SpatialReferenceTool.cpp#L22-L62)）：

```cpp
bool SpatialReferenceTool::loadCoordinateDatabases(const std::string& configPath)
{
    // 1. 构建完整路径
    std::string geoCoord = stlplus::create_filespec(configPath, "GEOGCS_Database.csv");
    std::string prjCoord = stlplus::create_filespec(configPath, "PROJCS_Database.csv");

    // 2. 验证文件存在
    if (!stlplus::file_exists(geoCoord)) return false;
    if (!stlplus::file_exists(prjCoord)) return false;

    // 3. 分别加载两个数据库
    std::vector<Coordinate> geoCoordinates;
    std::vector<Coordinate> prjCoordinates;
    
    insight::parseCoordinates(geoCoordinates, geoCoord);
    insight::parseCoordinates(prjCoordinates, prjCoord);

    // 4. 合并到统一列表
    m_coordinates.reserve(geoCoordinates.size() + prjCoordinates.size());
    m_coordinates.insert(m_coordinates.end(), 
                        geoCoordinates.begin(), geoCoordinates.end());
    m_coordinates.insert(m_coordinates.end(), 
                        prjCoordinates.begin(), prjCoordinates.end());

    m_loaded = true;
    return true;
}
```

**特点**：
- 使用项目现有的 `parseCoordinates()` 函数
- 使用 stlplus 工具确保跨平台路径处理
- 同时支持地理和投影坐标系搜索

### 3️⃣ CoordinateSystemWidget 接口更新

**API 更改**：

```cpp
// 旧
bool loadCoordinateDatabase(const std::string& dbPath);

// 新  
bool loadCoordinateDatabases(const std::string& configPath);
```

**调用流程**：
```
CoordinateSystemWidget 
  → loadCoordinateDatabases(configPath)
  → SpatialReferenceTool::loadCoordinateDatabases()
  → 加载 GEOGCS_Database.csv
  → 加载 PROJCS_Database.csv
  → 合并为 5456 个坐标系
```

### 4️⃣ MainWindow 错误处理

**新增的鲁棒性改进**（[MainWindow.cpp](src/ui/MainWindow.cpp#L362-L376)）：

```cpp
void MainWindow::onSetCoordinateSystem() {
    if (!m_coordinateSystemWidget) {
        m_coordinateSystemWidget = std::make_unique<CoordinateSystemWidget>(this);
        
        // 尝试多个配置路径
        std::string configPath = "./config";              // build 目录
        std::string altConfigPath = "../data/config";     // 源目录
        
        // 首选路径
        if (!m_coordinateSystemWidget->loadCoordinateDatabases(configPath)) {
            // 备选路径
            if (!m_coordinateSystemWidget->loadCoordinateDatabases(altConfigPath)) {
                // 失败时给用户明确提示
                QMessageBox::warning(this, tr("Error"), 
                    tr("Failed to load coordinate system databases.\n"
                       "Please ensure GEOGCS_Database.csv and PROJCS_Database.csv "
                       "exist in config directory."));
                return;
            }
        }
    }
}
```

**路径解析逻辑**：
1. 首先尝试 `./config/` (build 目录下，CMake 复制后的位置)
2. 若失败，尝试 `../data/config/` (源代码目录)
3. 两个都失败时，显示用户友好的错误消息

---

## 📊 数据库统计

编译后验证：

```bash
$ ls -lh build/config/
-rw-rw-r-- 1 jones jones 126K  GEOGCS_Database.csv (675 lines)
-rw-rw-r-- 1 jones jones 2.3M  PROJCS_Database.csv (4781 lines)

总计：5456 个坐标系
```

**数据库格式示例**：

```
EPSG;坐标系名称;WKT字符串

地理坐标系示例：
4326;GCS_WGS_1984;GEOGCS["WGS 84",DATUM["D_WGS_1984",...

投影坐标系示例：
2000;Anguilla_1957_British_West_Indies_Grid;PROJCS["Anguilla_1957_British_West_Indies_Grid",...
3857;Web_Mercator_Auxiliary_Sphere;PROJCS["WGS 84 / Web Mercator",...
```

---

## 🛠️ 构建流程

### 完整编译（推荐）

```bash
cd /home/jones/Git/01jones/InsightAT/build
rm -rf CMakeCache.txt CMakeFiles/        # 清除旧缓存
cmake .. -DCMAKE_BUILD_TYPE=Release      # 重新配置
cmake --build . -j4                       # 并行编译 4 个任务
```

### 构建输出验证

```
✅ [ 99%] Copying coordinate system databases to build directory
✅ [100%] Built target InsightAT_New
✅ build/config/GEOGCS_Database.csv 已生成
✅ build/config/PROJCS_Database.csv 已生成
```

---

## 🧪 运行时测试

### 启动应用

```bash
cd /home/jones/Git/01jones/InsightAT/build
./InsightAT_New
```

### 测试坐标系加载

1. **File** → **New Project**
   - 输入项目信息（名称、作者、描述）
   - 点击 "Create"

2. **Edit** → **Set Coordinate System**
   - 应该看到约 5456 个坐标系可用
   - 可以通过以下方式选择：
     - 常见坐标系下拉菜单（预选择快速选项）
     - 关键字搜索（如输入 "4326" 或 "WGS84"）
     - WKT 输入（复制粘贴 WKT 字符串）

3. **验证成功**
   - 选择坐标系后显示详细信息
   - EPSG 代码显示正确
   - WKT 字符串可读取

---

## 📝 相关代码位置

| 组件 | 文件 | 关键方法 |
|------|------|--------|
| **CMake** | `src/CMakeLists.txt` | `add_custom_command()` |
| **数据库工具** | `src/ui/widgets/SpatialReferenceTool.*` | `loadCoordinateDatabases()` |
| **UI 小部件** | `src/ui/widgets/CoordinateSystemWidget.*` | `loadCoordinateDatabases()` |
| **主窗口** | `src/ui/MainWindow.cpp` | `onSetCoordinateSystem()` |
| **系统配置** | `src/Common/Project.cpp` | `SystemConfig::readCoordinate()` |

---

## ⚠️ 常见问题

### Q: 编译后找不到坐标系数据库？
**A**: 确保在 build 目录中运行应用，或检查 `build/config/` 目录是否存在：
```bash
ls -lh build/config/GEOGCS_Database.csv
ls -lh build/config/PROJCS_Database.csv
```

### Q: 应用启动很慢？
**A**: 这是正常的，因为首次加载需要解析 5456 个坐标系。后续操作会更快。

### Q: 搜索功能很慢？
**A**: 当前使用线性搜索。对于生产环境，考虑：
- 构建索引结构（Hash Map）
- 实现前缀树（Trie）
- 使用数据库（SQLite）

### Q: 如何添加新的坐标系？
**A**: 编辑对应的 CSV 文件：
```bash
# 地理坐标系
vi data/config/GEOGCS_Database.csv

# 投影坐标系
vi data/config/PROJCS_Database.csv
```

格式：`EPSG;名称;WKT`

---

## 📋 验证清单

- ✅ CMake 配置包含 POST_BUILD 命令
- ✅ data/config 目录存在且包含两个 CSV 文件
- ✅ SpatialReferenceTool 实现 loadCoordinateDatabases()
- ✅ CoordinateSystemWidget 调用正确的 API
- ✅ MainWindow 处理多个配置路径
- ✅ 编译后 build/config 目录自动创建
- ✅ 应用能启动并加载坐标系
- ✅ 总共 5456 个坐标系可用

---

## 🎯 总结

此修复将系统从使用单一数据文件转变为使用正确的两数据库架构：

| 方面 | 修复前 | 修复后 |
|------|--------|--------|
| 数据源 | `coordinates.db` | GEOGCS + PROJCS |
| 坐标系数 | ~8 (手动) | 5456 (完整) |
| 加载方式 | 手动复制 | CMake 自动 |
| 错误处理 | 无 | 多路径尝试 + 用户提示 |
| 代码对齐 | 不一致 | 与 Project.cpp 对齐 |

应用现在能够正确加载并使用项目内置的完整坐标系数据库！
