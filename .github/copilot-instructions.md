
# InsightAT Copilot Instructions

InsightAT 是一个面向航空摄影测量的**空三（Aerial Triangulation）处理软件**，覆盖从特征提取、图像检索、特征匹配、几何验证到两视图重建的完整 GPU 加速流水线，并通过 CLI-First 架构支持 headless/Docker/云端运行。

---

## 架构概述

### 三层架构

```
Project Layer   → 全局配置、坐标系、相机、图像组、测量数据（GNSS/IMU/GCP）
ImageGroup Layer→ 图像逻辑分组，共享相机参数
AT Task Layer   → 空三任务（冻结输入快照，树形嵌套，BA 优化）
```

**严格分层规则**：`src/algorithm/`（含所有 CLI 工具）**禁止依赖 Qt**，Qt 只允许出现在 `src/ui/` 和 `src/render/`。

### SfM 两层设计

- **第一层（粗粒度）**：≤4000×4000 分辨率，≤10000 特征点，RANSAC+F/E/H，GPU 三角化，层级式 SfM，PBA 优化
- **第二层（细粒度）**：原始分辨率，光流+子像素匹配，完整畸变模型，全局 BA

---

## 项目文件格式

**项目文件**：`.iat`（Cereal `JSONOutputArchive`，文本 JSON，非二进制）  
**算法数据文件**：IDC 格式（InsightAT Data Container）

```
[Magic "ISAT" 4B][Version uint32][JSON Size uint64][JSON Descriptor UTF-8][Padding 8B对齐][Binary Payload]
```

IDC 扩展名约定：`.isat_feat`（特征）/ `.isat_match`（匹配）/ `.isat_geo`（几何）/ `.isat_twoview`（两视图重建）

---

## 核心数据结构（`src/database/database_types.h`）

| 类型 | 说明 |
|------|------|
| `CoordinateSystem` | EPSG/WKT/ENU/Local + 旋转约定（OmegaPhiKappa/YawPitchRoll） |
| `Measurement` | GNSS/IMU/GCP/SLAM，含协方差，用于 BA 先验约束 |
| `InputPose` | 位置（optional）+ 旋转（optional） |
| `ImageGroup` | 图像集合 + CameraRig 引用 |
| `Project` | 顶层容器：元数据、相机库、图像组、GCP、坐标系、AT 任务 |
| `ATTask` | 冻结的 InputSnapshot + Initialization（前任务结果）+ child_tasks |

**序列化模式（必须带版本号）**：

```cpp
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    ar(CEREAL_NVP(field_v0));
    if (version >= 1) ar(CEREAL_NVP(field_v1));  // 新字段只能在新 version 分支追加
}
CEREAL_CLASS_VERSION(MyType, 1);
```

---

## CLI 工具链

所有工具在 `src/algorithm/tools/`，构建产物在 `build/isat_*`。

| 工具 | 功能 | GPU | 输出 |
|------|------|-----|------|
| `isat_project` | 项目管理（创建/分组/导入图像/设相机/导出） | — | `.iat` |
| `isat_extract` | 特征提取（SIFT/SuperPoint） | ✅ | `.isat_feat` |
| `isat_retrieve` | 图像检索（GNSS/VLAD/BoW/Sequence） | CPU 为主 | `.json` |
| `isat_match` | 特征匹配 | ✅ | `.isat_match` |
| `isat_geo` | 几何验证（F/E/H RANSAC） | ✅ | `.isat_geo` |
| `isat_twoview` | 两视图重建（三角化+BA） | ✅ | `.isat_twoview` |
| `isat_calibrate` | 内参估计（BA 优化 fx/fy/cx/cy） | CPU | `K_refined.json` |
| `isat_camera_estimator` | EXIF/sensor_db 初始内参估计，写回 `.iat` | — | `.iat`（就地修改）|

### CLI I/O 规范

- **stdout**：只输出 `ISAT_EVENT <single-line-json>`，机器可读
- **stderr**：glog 日志（人类可读）
- 退出码：`0` 成功 / `1` 运行时错误 / `2` 参数错误

```cpp
printEvent({{"type","project.add_images"},{"ok",true},
            {"data",{{"added",100},{"skipped",20},{"total_in_group",100}}}});
```

### isat_project set-camera --from-k

`isat_project set-camera -p project.iat -g <group_id> --from-k K.json`  
读取 `isat_calibrate` 输出的 K.json（字段：`fx`/`fy`/`cx`/`cy`，可选 `width`/`height`），写入该 Group 的 `CameraModel`，**保留已有的** `make/model/sensor_width_mm` 等元数据字段。

### isat_camera_estimator

```
isat_camera_estimator -p project.iat [-g <group_id> | -a/--all-groups]
                      [-d sensor_db] [--max-sample 5] [-v/-q]
```

逐 Group 采样 ≤max_sample 张图像，通过以下三种方法估计焦距（优先级递减）：
1. `focal_35mm` EXIF → `f_px = f35 × diag_px / 43.26661`（source=`exif_focal35mm`）
2. sensor_db 查询 + 物理焦距 → `f_px = f_mm × width / sw_mm`（source=`sensor_db`）
3. 假设 35mm 等效焦距=35mm（source=`fallback`）

每个 Group 输出一个 ISAT_EVENT：
```json
{"type":"camera_estimator.estimate","ok":true,"data":{"group_id":0,"group_name":"strip_01","images_sampled":5,"width":4000,"height":3000,"fx":3612.5,"fy":3612.5,"cx":2000.0,"cy":1500.0,"source":"exif_focal35mm","make":"DJI","model":"FC6310","written_to_project":true}}
```

### isat_project add-images 特性

`--ext` 支持三种形式：
- 单个扩展名：`--ext .jpg`
- 逗号分隔：`--ext .jpg,.tif,.png`
- 正则表达式：`--ext "\.(jpg|tif|png)$"`（含 `(|*+[{^$` 时自动识别为正则）

去重（`--no-dedup` 可关闭）：跨**项目全部 Group**，以 `fs::canonical()` 规范化绝对路径为 key，同批次内也去重。图像 `filename` 始终存储为绝对路径。

---

## GPU 模块模式

所有 GPU 模块遵循 **CPU–GPU 异步 Stage 流水线**：

```
Stage 1  [多线程 I/O]    读磁盘 → 填充 Task 输入字段
Stage 2  [主线程 EGL]    上传 SSBO → dispatch compute shader → readback → 写 Task 输出字段
Stage 3  [多线程 I/O]    写磁盘
```

- GPU 初始化：进程级单次调用 `gpu_*_init()` / `gpu_*_shutdown()`
- 非线程安全，所有 GPU 调用须在持有 EGL context 的同一线程执行
- 无效点写为 `NaN`，不改变数组长度

**数据布局约定**：

| 数据 | 类型 | 布局 |
|------|------|------|
| 对应点 | `float[4N]` | `[x1,y1,x2,y2, ...]` 行主序 |
| 旋转矩阵 | `float[9]` | 行主序 |
| 3D 点 | `float[3N]` | `[X,Y,Z, ...]` |

---

## 代码风格

详细规范见 [`doc/design/CODING_STYLE.md`](doc/design/CODING_STYLE.md)，格式化由根目录 [`.clang-format`](.clang-format) 自动执行（clang-format 14，列宽 100，4 空格缩进）。

**关键命名规则**：

| 元素 | 规则 | 示例 |
|------|------|------|
| 变量 / 函数 | `snake_case` | `num_matches`, `gpu_ransac_F()` |
| 类 / 结构体 | `PascalCase` | `TwoViewTask`, `GeoRansacConfig` |
| 枚举值 | `kPascalCase` | `kGNSS`, `kOmegaPhiKappa` |
| 常量 | `kCamelCase` | `kEventPrefix` |
| 私有成员 | `snake_case_`（尾下划线） | `context_`, `dirty_` |

段落分隔使用 Unicode 横线（`─` U+2500）：

```cpp
// ─────────────────────────────────────────────────────────────────────────────
// Per-pair task
// ─────────────────────────────────────────────────────────────────────────────
```

---

## Python Pipeline（`src/pipeline/`）

全自动内参标定流水线，一键处理：扫描子目录 → 创建项目 → 每子目录一个分组 → 估计初始内参 → 特征提取/检索/匹配/几何验证/两视图重建/标定 → 精化内参写回 `.iat`。

```
src/pipeline/
  __init__.py
  runner.py          # subprocess 封装，parse ISAT_EVENT，ToolError
  project_utils.py   # isat_* 工具的 Python 高层封装
  pipelines/
    __init__.py
    auto_calibrate.py  # 主流水线 + CLI 入口
```

### 运行

```bash
python -m src.pipeline.pipelines.auto_calibrate \
    --input /data/flight01 \
    --project /out/flight01.iat \
    --work-dir /out/flight01_work \
    --ext ".jpg,.tif" \
    --min-images 5 --max-sample 5
```

### 流水线步骤（每个分组）

```
1. isat_project extract -g <id> -t 0 -o images.json
2. isat_extract -i images.json -o feat/
3. isat_retrieve -i images.json -f feat/ -o pairs.json
4. isat_match -i pairs.json -f feat/ -o match/
5. isat_project intrinsics -g <id> -t 0 -o K_initial.json
6. isat_geo -i pairs.json -m match/ -o geo/ --estimate-h -k K_initial.json
7. isat_twoview -i pairs.json -g geo/ -m match/ -o twoview/ -k K_initial.json
8. isat_calibrate -t twoview/ -o K_refined.json
9. isat_project set-camera -g <id> --from-k K_refined.json
```

**stdout**：`PIPELINE_EVENT <json>`（step/group_id/group_name/ok/data）  
**stderr**：Python logging  
**exit 0/1/2**：成功/运行错误/参数错误

环境变量 **`ISAT_BIN_DIR`**：指定 isat_* 可执行文件目录（默认 `<repo_root>/build/`）。

---

## 构建与测试

```bash
cd build && cmake .. && make -j10    # 首次或新增target后需 cmake
cd build && make -j10               # 增量构建
make isat_geo -j4                   # 构建单个工具
./test_project_serialization        # 序列化单元测试
./test_serialization_comprehensive
```

---

## 常见陷阱

- 项目文件是 **JSON `.iat`**（`cereal::JSONOutputArchive`），不是二进制 `.db`
- Cereal 序列化字段**只能追加，不能删除**，每次新增字段必须递增版本号
- `OmegaPhiKappa`（Z-Y-X 外旋，摄影测量）≠ `YawPitchRoll`（Z-Y-X 内旋，航空学）
- ATTask 创建后输入快照已冻结，之后添加的测量数据不会影响该任务
- GPU Stage 2（主线程 EGL）中**禁止读写磁盘**

# 文档
详细文档请先阅读 README.md，编码规范请遵守doc/design/CODING_STYLE.md. 

# 其他说明
在agent模式下， 应该先就理解的内容进行总结，并交互确认，甚至讨论， 然后再进行设计和代码编写。 首先需要是一个Structure from Motion (SfM), 摄影测量、计算机视觉多视图几何，SLAM方面的专家，要有非常强的时间经验和算法工程能力， 以这个背景来进行思考、设计和编码