# InsightAT 代码编写规范

> 本规范从 `src/algorithm/` 现有代码归纳整理，所有新代码必须遵守。  
> 格式化规则由 `.clang-format` 自动执行，本文聚焦**命名、结构、注释、设计**约定。

---

## 目录

1. [文件组织](#1-文件组织)
2. [命名规范](#2-命名规范)
3. [注释规范](#3-注释规范)
4. [代码格式（补充说明）](#4-代码格式补充说明)
5. [头文件规范](#5-头文件规范)
6. [命名空间](#6-命名空间)
7. [类与结构体](#7-类与结构体)
8. [函数设计](#8-函数设计)
9. [错误处理](#9-错误处理)
10. [GPU 代码规范](#10-gpu-代码规范)
11. [序列化规范](#11-序列化规范)
12. [CLI 工具规范](#12-cli-工具规范)
13. [禁止事项](#13-禁止事项)

---

## 1. 文件组织

### 1.1 目录结构

```
src/
├── algorithm/               # 核心算法（严禁依赖 Qt）
│   ├── modules/
│   │   ├── extraction/      # 特征提取
│   │   ├── retrieval/       # 图像检索
│   │   ├── matching/        # 特征匹配
│   │   ├── geometry/        # 几何计算（RANSAC / 三角化）
│   │   └── sfm/             # SfM 重建
│   ├── io/                  # IDC 文件读写
│   └── tools/               # CLI 可执行程序（isat_*.cpp）
├── database/                # 数据模型 + Cereal 序列化
├── Common/                  # 跨层公共工具（无 Qt，无 CUDA）
├── render/                  # OpenGL 渲染（可依赖 Qt）
└── ui/                      # Qt 界面（唯一允许依赖 Qt 的层）
```

### 1.2 文件命名

| 类型 | 规则 | 示例 |
|------|------|------|
| C++ 实现文件 | `snake_case.cpp` | `two_view_reconstruction.cpp` |
| C++ 头文件 | `snake_case.h` | `gpu_geo_ransac.h` |
| CLI 工具 | `isat_<动词>.cpp` | `isat_geo.cpp` |
| GLSL Shader | `<功能>_<类型>.glsl` | `triangulate_comp.glsl` |

### 1.3 文件头注释

每个 `.cpp` / `.h` 文件必须以 Doxygen 文件头开始：

```cpp
/**
 * @file  gpu_geo_ransac.h
 * @brief GPU-accelerated two-view geometry solver (H / F / E) via
 *        OpenGL 4.3 Compute Shaders + EGL headless context.
 *
 * Architecture
 * ─────────────
 *  说明模块内部分层结构 ...
 *
 * Usage
 * ─────
 *   代码示例 ...
 */
```

CLI 工具（`isat_*.cpp`）使用简化块注释，顶部列出：功能概述、输入输出格式、用法示例：

```cpp
/**
 * isat_geo.cpp
 * InsightAT Two-View Geometry Estimation Tool
 *
 * Pipeline:
 *   Stage 1  [multi-thread I/O]  ...
 *   Stage 2  [main thread, EGL]  ...
 *
 * Usage:
 *   isat_geo -i pairs.json -m match_dir/ -o geo_dir/
 */
```

---

## 2. 命名规范

### 2.1 总览

| 元素 | 规则 | 示例 |
|------|------|------|
| 变量（局部 / 全局） | `snake_case` | `num_matches`, `input_dir` |
| 函数 / 方法 | `snake_case` 或 `camelCase`（仅 Qt slots） | `gpu_ransac_F()`, `onProjectChanged()` |
| 结构体 / 类 | `PascalCase` | `TwoViewTask`, `GeoRansacConfig` |
| 枚举类型 | `PascalCase` | `Measurement::Type` |
| 枚举值 | `kPascalCase` | `kGNSS`, `kOmegaPhiKappa` |
| 常量 / constexpr | `kCamelCase` | `kEventPrefix`, `kMaxIterations` |
| 宏 | `UPPER_SNAKE_CASE` | `GPU_GEO_RANSAC_H` |
| 命名空间 | `snake_case` | `insight::sfm`, `insight::io` |
| 模板参数 | `PascalCase` 或单字母大写 | `T`, `Archive` |
| 私有成员 | `snake_case_`（尾下划线） | `context_`, `shader_id_` |

### 2.2 语义清晰原则

- **布尔变量**以 `is_`、`has_`、`can_`、`ok` 开头：`is_valid`, `has_gnss`, `F_ok`
- **计数变量**使用 `n_` 前缀或 `_count` 后缀：`n_inliers`, `num_matches`
- **索引变量**使用 `idx`、`k`、`i`、`j`（循环）
- **输出参数**指针以 `_out` 结尾：`X_out`, `residuals_out`
- **阶段数据**遵循 `Stage N output` 注释分区（见 §7.2）

---

## 3. 注释规范

### 3.1 Doxygen 文档注释

用于 **所有公开 API**（头文件中的函数、结构体成员）：

```cpp
/**
 * Triangulate @p n correspondences in parallel on GPU.
 *
 * @param pts_n   Normalised image coordinates, row-major float[4n].
 * @param n       Number of correspondences (≥ 1).
 * @param R       Camera-2 rotation matrix, row-major float[9].
 * @param t       Camera-2 translation, float[3].
 * @param X_out   Output 3-D points, float[3n].
 * @return        Number of valid (finite, positive-depth) points.
 */
int gpu_triangulate(const float* pts_n, int n,
                    const float  R[9],
                    const float  t[3],
                    float*       X_out);
```

单行成员使用 `///< 说明`：

```cpp
struct TwoViewTask {
    std::string geo_file;    ///< .isat_geo 路径
    uint32_t camera1_id = 1; ///< image1 所属 group_id
    bool F_ok = false;       ///< isat_geo 是否成功估计 F
};
```

### 3.2 段落分隔线

使用 Unicode 横线字符（`─`，U+2500）将逻辑段落分隔：

```cpp
// ─────────────────────────────────────────────────────────────────────────────
// Per-pair task
// ─────────────────────────────────────────────────────────────────────────────
```

总宽度为 81 个字符（`// ` + 78 个 `─`）。

### 3.3 步骤注释

多阶段算法用 `── Step N: 描述 ──` 行内标注：

```cpp
// ── Step 4: normalise pixel coords (K1 for img1, K2 for img2) ────────────────
// ── Step 5: decompose E → (R, t) ─────────────────────────────────────────────
// ── Step 6: GPU triangulation (normalised coords) ─────────────────────────────
```

### 3.4 禁止无意义注释

```cpp
// 错误 ✗
int count = 0;  // count is zero

// 正确 ✓
int n_inliers = 0;  // reset before GPU readback
```

---

## 4. 代码格式（补充说明）

格式化由 `.clang-format` 自动执行，以下规则需**手动遵守**：

### 4.1 `using` 别名声明

放在文件 `namespace` 块外，实现文件顶部：

```cpp
namespace fs = std::filesystem;
using json   = nlohmann::json;       // 对齐 '='
using namespace insight::io;
using namespace insight::sfm;
```

`using namespace` **仅允许**在 `.cpp` 文件中使用，**禁止**出现在头文件。

### 4.2 结构化绑定与初始化列表

优先使用 C++17 语法：

```cpp
// 推荐
for (const auto& [id, rig] : project.camera_rigs) { ... }
auto [R, t] = decomposeEssential(E, pts1, pts2);

// 初始化列表对齐
Image img{
    .image_id = project.next_image_id++,
    .filename = abs_path.string(),
};
```

### 4.3 Lambda 格式

短 lambda 单行，长 lambda 换行后缩进 4 空格，捕获列表显式写出：

```cpp
// 短 lambda（≤ 50 字符）
auto lower = [](std::string s) { return /* ... */; };

// 长 lambda
auto accept = [&ext_list, &ext_re, ext_is_regex](const fs::path& p) -> bool {
    if (ext_is_regex)
        return std::regex_search(p.filename().string(), ext_re);
    // ...
};
```

### 4.4 多返回值 / 输出参数

优先使用 `std::optional` 或结构体返回；仅在性能关键路径使用裸指针输出：

```cpp
// 推荐
std::optional<TwoViewResult> reconstructPair(...);

// 性能关键时允许
int gpu_ba_residuals(..., float* wrss_out, int* ninliers_out);
```

---

## 5. 头文件规范

### 5.1 Include 守卫

所有头文件使用 `#pragma once` + 传统守卫双保险：

```cpp
#pragma once
#ifndef GPU_GEO_RANSAC_H
#define GPU_GEO_RANSAC_H
// ...
#endif  // GPU_GEO_RANSAC_H
```

纯 C 接口文件（`.h` 被 C 代码引用）在 `#pragma once` 之外加 `extern "C"` 块。

### 5.2 Include 分组顺序

```cpp
// 1. 对应的 .h（仅 .cpp 中）
#include "gpu_geo_ransac.h"

// 2. 标准库（字母序）
#include <algorithm>
#include <filesystem>
#include <vector>

// 3. 第三方库
#include <Eigen/Core>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

// 4. 项目内部头（相对路径）
#include "../io/idc_reader.h"
#include "cmdLine/cmdLine.h"
```

各组之间**空一行**，组内字母序排列。

---

## 6. 命名空间

```cpp
namespace insight {
namespace sfm {

// ── 不缩进命名空间内容 ────────────────────────────────────────────────────────
struct TwoViewResult { ... };

double FocalFromFundamental(const Eigen::Matrix3d& F,
                             double cx, double cy);

}  // namespace sfm
}  // namespace insight
```

- 命名空间内容**不缩进**
- 结束括号写 `}  // namespace <name>`（两空格）
- 匿名命名空间（`namespace {}`）用于文件作用域静态函数，替代 `static`

---

## 7. 类与结构体

### 7.1 纯数据结构使用 `struct`

```cpp
struct GeoTask {
    // ── 标识 ──────────────────────────────────────────────────────────────────
    std::string image1_id;
    std::string image2_id;
    int         index = 0;

    // ── Stage 1 output ───────────────────────────────────────────────────────
    std::vector<float>   coords_pixel;  ///< [x1,y1,x2,y2] × n_matches
    std::vector<uint8_t> F_mask;
    int  num_matches = 0;
    bool F_ok        = false;

    // ── Stage 2 output ───────────────────────────────────────────────────────
    TwoViewResult recon;
    bool processed = false;
};
```

规则：
- 成员按**阶段/语义分组**，使用段落分隔线
- 所有成员**原地初始化**（= 默认值）
- `///< 注释` 对齐

### 7.2 类（带行为）

```cpp
class ProjectDocument : public QObject {
    Q_OBJECT
public:
    explicit ProjectDocument(QObject* parent = nullptr);
    ~ProjectDocument() override;

    // ── 查询 ──────────────────────────────────────────────────────────────────
    const Project& project() const { return project_; }
    bool           isDirty()  const { return dirty_; }

signals:
    void projectChanged();

public slots:
    void onActionTriggered();

private:
    Project     project_;
    bool        dirty_ = false;
};
```

- `public` → `signals` → `public slots` → `protected` → `private` 顺序
- 私有成员用尾下划线：`project_`, `dirty_`

---

## 8. 函数设计

### 8.1 单一职责

每个函数只做一件事。超过 80 行的函数应拆分。

### 8.2 参数顺序

```
[输入 const 引用/指针] [输入值] [输出指针]
```

```cpp
int gpu_ba_residuals(const float* pts_px,   // 输入观测
                     const float* X,         // 输入 3D 点
                     int          n,          // 输入计数
                     const float  R[9],       // 输入旋转
                     const float  t[3],       // 输入平移
                     float        f,           // 内参
                     float        cx,
                     float        cy,
                     float        huber_k,
                     float*       wrss_out,    // 输出
                     int*         ninliers_out);
```

### 8.3 `static` 文件作用域函数

`.cpp` 内部辅助函数一律加 `static`：

```cpp
static std::vector<TwoViewTask> loadPairs(const std::string& json_path, ...);
static bool writeTwoViewFile(const TwoViewTask& task, const std::string& out_dir);
```

---

## 9. 错误处理

### 9.1 算法层（无 Qt）

| 场景 | 做法 |
|------|------|
| 不可恢复的编程错误 | `LOG(FATAL) << "..."` |
| 运行时失败（文件不存在等） | `LOG(ERROR) << "..."` + 返回 `false` / `std::nullopt` |
| 调试信息 | `LOG(INFO) <<` / `VLOG(1) <<` |
| 断言（仅 Debug）| `DCHECK(condition) << "..."` |

### 9.2 CLI 工具

```cpp
if (!fs::is_directory(input_dir)) {
    printEvent({{"type", "project.add_images"},
                {"ok",   false},
                {"error","input directory not found"}});
    return 1;
}
```

- `stdout` **只输出** `ISAT_EVENT <single-line-json>`
- 人类可读日志输出到 `stderr`（通过 glog）
- 详见 `doc/design/CLI_IO_CONVENTIONS.md`

### 9.3 禁止裸 `throw` 到顶层

CLI `main()` 顶部必须有 try-catch 包裹 `cmd.process()`：

```cpp
try { cmd.process(argc, argv); }
catch (const std::string& s) {
    std::cerr << "Error: " << s << "\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
}
```

---

## 10. GPU 代码规范

### 10.1 OpenGL Compute Shader 模式

所有 GPU 模块须遵循"CPU–GPU 异步 Stage 流水线"模式：

```
Stage 1  [多线程 I/O]   读磁盘 → 填充 Task 的输入字段
Stage 2  [主线程 EGL]   上传 SSBO → dispatch → readback → 写 Task 输出字段
Stage 3  [多线程 I/O]   写磁盘
```

### 10.2 GPU 初始化 / 销毁

```cpp
// 进程级别单次调用
gpu_geo_init(nullptr);   // EGL + GLEW + 编译 shader
// ... 处理所有 pair ...
gpu_geo_shutdown();
```

- 非线程安全，所有 GPU 调用必须在**持有 EGL context 的同一线程**执行
- 并发调用须加外部互斥锁（`std::mutex`）

### 10.3 数据布局约定

| 数据 | 类型 | 布局 |
|------|------|------|
| 对应点 | `float[4N]` | `[x1,y1,x2,y2, ...]` 行主序 |
| 旋转矩阵 | `float[9]` | 行主序 |
| 3D 点 | `float[3N]` | `[X,Y,Z, ...]` |
| 残差 | `float[4N]` | `[r1u,r1v,r2u,r2v, ...]` |

无效点（摄影机后方）写为 `NaN`，不得改变数组长度。

---

## 11. 序列化规范

所有持久化数据结构使用 **Cereal + 版本号**：

```cpp
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    ar(CEREAL_NVP(image_id),
       CEREAL_NVP(filename));
    if (version >= 1) {
        ar(CEREAL_NVP(pose));      // v1 新增字段
    }
    if (version >= 2) {
        ar(CEREAL_NVP(metadata));  // v2 新增字段
    }
}
CEREAL_CLASS_VERSION(Image, 2);
```

规则：
- 每次增加字段必须**递增版本号**
- 旧字段**不得删除**，只能在新 version 分支新增
- `std::optional<T>` 用于稀疏字段，空值不占序列化空间
- 生产格式为 **JSON**（`.iat` 文件），测试时允许二进制

---

## 12. CLI 工具规范

### 12.1 选项定义模式

```cpp
CmdLine cmd("工具简短描述。");
std::string project_file;
int count = 100;

cmd.add(make_option('p', project_file, "project")
    .doc("项目文件路径 (.iat)"));
cmd.add(make_option('n', count, "count")
    .doc("处理数量（默认：100）"));
cmd.add(make_switch('v', "verbose")
    .doc("详细日志输出"));
cmd.add(make_switch('h', "help")
    .doc("显示帮助信息"));
```

### 12.2 必选参数验证

```cpp
if (cmd.checkHelp(argv[0])) return 0;
if (project_file.empty()) {
    std::cerr << "Error: -p/--project is required\n\n";
    cmd.printHelp(std::cerr, argv[0]);
    return 2;
}
```

### 12.3 退出码

| 退出码 | 含义 |
|--------|------|
| `0` | 成功 |
| `1` | 运行时错误（文件不存在、处理失败等） |
| `2` | 参数错误（缺少必选参数、无效值等） |

---

## 13. 禁止事项

| 禁止 | 原因 |
|------|------|
| `algorithm/` 层引入 Qt 头文件 | 破坏 headless 运行能力 |
| `#define` 替代 `constexpr` / `inline` | 无类型安全 |
| 头文件中使用 `using namespace std` | 污染引用方命名空间 |
| 裸 `new` / `delete`（非必要） | 优先 `std::unique_ptr` / `std::shared_ptr` |
| 魔法数字（未命名常量） | 使用 `constexpr` 或 `enum` |
| `printf` / `scanf`（新代码） | 使用 `LOG()` + `std::cout/cin` |
| 在 GPU Stage 2 中读写磁盘 | Stage 2 仅允许 GPU 操作（主线程 EGL）|
| Cereal 序列化时删除已有字段 | 破坏向后兼容性 |
| `stdout` 输出非 `ISAT_EVENT` 格式内容（CLI 工具） | 破坏管道/脚本解析 |

---

*最后更新：通过代码审查自动同步，勿手动修改字段列表。*
