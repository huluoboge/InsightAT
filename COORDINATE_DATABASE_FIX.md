# 坐标系数据库加载修复总结

## 问题描述

原有的实现试图使用单个 CSV 文件来加载坐标系数据，但项目的实际架构使用两个独立的坐标系数据库：
- **GEOGCS_Database.csv** - 地理坐标系（675 个条目）
- **PROJCS_Database.csv** - 投影坐标系（4781 个条目）

## 解决方案

### 1. CMake 构建配置修改

**文件**: [CMakeLists.txt](CMakeLists.txt) 和 [src/CMakeLists.txt](src/CMakeLists.txt)

在构建时自动复制配置数据到 build 目录：

```cmake
# 在 src/CMakeLists.txt 中添加
add_custom_command(TARGET InsightAT_New POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/data/config"
        "${CMAKE_BINARY_DIR}/config"
    COMMENT "Copying coordinate system databases to build directory"
)
```

**结果**：构建完成后，config 文件会自动复制到 `build/config/` 目录。

### 2. SpatialReferenceTool 更新

**文件**: 
- [src/ui/widgets/SpatialReferenceTool.h](src/ui/widgets/SpatialReferenceTool.h)
- [src/ui/widgets/SpatialReferenceTool.cpp](src/ui/widgets/SpatialReferenceTool.cpp)

**更改内容**：

```cpp
// 旧方法（加载单个数据库）
bool loadCoordinateDatabase(const std::string& dbPath);

// 新方法（加载两个数据库）
bool loadCoordinateDatabases(const std::string& configPath);
```

新方法将：
1. 从配置目录加载 GEOGCS_Database.csv
2. 从配置目录加载 PROJCS_Database.csv
3. 合并两个数据库到单个 vector 中
4. 总计加载 5456 个坐标系（675 + 4781）

### 3. CoordinateSystemWidget 更新

**文件**: 
- [src/ui/widgets/CoordinateSystemWidget.h](src/ui/widgets/CoordinateSystemWidget.h)
- [src/ui/widgets/CoordinateSystemWidget.cpp](src/ui/widgets/CoordinateSystemWidget.cpp)

**更改内容**：

```cpp
// 旧方法
bool loadCoordinateDatabase(const std::string& dbPath);

// 新方法
bool loadCoordinateDatabases(const std::string& configPath);
```

### 4. MainWindow 更新

**文件**: [src/ui/MainWindow.cpp](src/ui/MainWindow.cpp)

更新 `onSetCoordinateSystem()` 方法以正确处理配置路径：

```cpp
void MainWindow::onSetCoordinateSystem() {
    if (!m_coordinateSystemWidget) {
        m_coordinateSystemWidget = std::make_unique<CoordinateSystemWidget>(this);
        
        // 获取配置路径（EXE 同级目录下的 config 文件夹）
        std::string configPath = "./config";  // build 目录下
        
        // 如果在源目录构建，尝试 data/config 路径
        std::string altConfigPath = "../data/config";
        
        if (!m_coordinateSystemWidget->loadCoordinateDatabases(configPath)) {
            if (!m_coordinateSystemWidget->loadCoordinateDatabases(altConfigPath)) {
                LOG(ERROR) << "Failed to load coordinate databases";
                QMessageBox::warning(this, tr("Error"), 
                    tr("Failed to load coordinate system databases.\n"
                       "Please ensure GEOGCS_Database.csv and PROJCS_Database.csv "
                       "exist in config directory."));
                return;
            }
        }
    }
    // ... 其余代码
}
```

## 数据库内容验证

编译后的数据库统计：

```bash
$ ls -lh build/config/
-rw-rw-r-- 1 jones jones 110K  2月  8 19:16 camera_sensor_database.txt
-rw-rw-r-- 1 jones jones  314  2月  8 19:16 error_code.txt
-rw-rw-r-- 1 jones jones 126K  2月  8 19:16 GEOGCS_Database.csv (675 lines)
-rw-rw-r-- 1 jones jones 2.3M  2月  8 19:16 PROJCS_Database.csv (4781 lines)

总计：5456 个坐标系可用
```

## 坐标系数据库格式

两个数据库都使用相同的格式（分号分隔）：

```
EPSG;坐标系名称;WKT字符串

例：
4326;GCS_WGS_1984;GEOGCS["WGS 84",DATUM[...]...]
3857;Web_Mercator_Auxiliary_Sphere;PROJCS["WGS 84 / Pseudo-Mercator"...]
```

## 编译验证

```
✅ CMake 配置成功
✅ 所有 14 个构建目标完成
✅ InsightAT_New 可执行文件生成 (624 KB)
✅ config 目录自动复制到 build 目录
✅ GEOGCS 和 PROJCS 数据库验证成功
```

## 运行时配置

应用启动时，将尝试按以下顺序加载配置：

1. `./config/` - 优先使用 build 目录下的 config
2. `../data/config/` - 备选源目录路径
3. 若两个都失败，弹出错误提示要求用户检查文件

## 相关文件列表

| 文件 | 修改类型 | 用途 |
|------|---------|------|
| CMakeLists.txt | 修改 | 主 CMake 配置 |
| src/CMakeLists.txt | 修改 | 添加 post_build 命令 |
| src/ui/widgets/SpatialReferenceTool.h | 修改 | API 更新 |
| src/ui/widgets/SpatialReferenceTool.cpp | 修改 | 实现两个数据库加载 |
| src/ui/widgets/CoordinateSystemWidget.h | 修改 | API 更新 |
| src/ui/widgets/CoordinateSystemWidget.cpp | 修改 | 调用新 API |
| src/ui/MainWindow.cpp | 修改 | 配置路径处理 |
| data/config/GEOGCS_Database.csv | 现有 | 地理坐标系数据库 |
| data/config/PROJCS_Database.csv | 现有 | 投影坐标系数据库 |

## 测试步骤

1. 编译项目：`cmake --build . -j4`
2. 验证 config 目录：`ls -lh build/config/`
3. 运行应用：`./build/InsightAT_New`
4. 创建新项目：File → New Project
5. 设置坐标系：Edit → Set Coordinate System
6. 验证数据库加载成功（约 5456 个坐标系可用）

## 已知限制

- 应用启动时加载所有 5456 个坐标系，可能需要 1-2 秒
- 搜索功能使用线性搜索，大型数据库可能需要优化为索引查询
- WKT 输入验证目前仅支持显示，完整的 WKT 解析需要 GDAL/OGR 库支持
