# SIFT GPU 特征提取工具 (isat_extract)

## 概述

`isat_extract` 是基于 SiftGPU 的高性能特征提取命令行工具，使用异步流水线架构实现 I/O、GPU 计算的并行处理，最大化 GPU 利用率。

## 架构特点

### 三阶段异步 Pipeline

```
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  Image I/O  │ ───> │  SIFT GPU   │ ───> │  Write IDC  │
│ (多线程)     │      │ (单GPU线程)  │      │  (多线程)    │
└─────────────┘      └─────────────┘      └─────────────┘
     Stage 1              Stage 2              Stage 3
  4 worker threads    Current thread      4 worker threads
    Queue: 10             Queue: 5            Queue: 10
```

- **Stage 1 (Image Loading)**: 多线程异步读取图像，避免 GPU 等待 I/O
- **Stage 2 (SIFT GPU)**: 在当前线程运行 GPU 特征提取（OpenGL 上下文限制）
- **Stage 3 (IDC Writing)**: 多线程写出 IDC 格式结果文件

### 有界队列 (BoundedTaskQueue)

- 防止内存爆炸：限制缓冲区大小（默认 IO 队列 10，GPU 队列 5）
- 自动流量控制：生产者在队列满时阻塞

## 编译

```bash
cd build
cmake ..
make -j10 isat_extract
```

## 使用方法

### 基本用法

```bash
./isat_extract -i images.json -o features/
```

### 完整选项

```bash
./isat_extract [OPTIONS] -i <image_list.json> -o <output_dir>

OPTIONS:
  -h, --help              打印帮助信息
  -i, --input <file>      输入图像列表 (JSON 格式)
  -o, --output <dir>      输出目录（存放 .isat_feat 文件）
  -n, --nfeatures <int>   每张图像最大特征点数 (默认: 8000)
  -t, --threshold <float> 峰值阈值 (默认: 0.04)
  --octaves <int>         金字塔层数 (-1 = 自动, 默认: -1)
  --levels <int>          每层级别数 (默认: 3)
  --no-adapt              禁用暗图自适应
  -v, --verbose           详细日志
  -q, --quiet             安静模式（仅错误）
```

### 输入格式 (JSON)

```json
{
  "images": [
    {"path": "/data/IMG_0001.jpg", "camera_id": 1},
    {"path": "/data/IMG_0002.jpg", "camera_id": 1},
    {"path": "/data/IMG_0003.jpg", "camera_id": 2}
  ]
}
```

### 输出格式 (IDC)

每张图像生成一个 `.isat_feat` 文件，采用 IDC (Insight Data Container) 格式：

**文件结构**:
```
┌─────────────────────────────────────┐
│ Magic: "ISAT" (4 bytes)             │
├─────────────────────────────────────┤
│ Version: uint32_t (4 bytes)         │
├─────────────────────────────────────┤
│ JSON Size: uint64_t (8 bytes)       │
├─────────────────────────────────────┤
│ JSON Descriptor                     │  ← 元数据（算法、参数、时间戳）
├─────────────────────────────────────┤
│ Binary Blob 1: keypoints            │  ← float32[N, 4] (x, y, scale, orientation)
├─────────────────────────────────────┤
│ Binary Blob 2: descriptors          │  ← float32[N, 128] (SIFT 描述子)
└─────────────────────────────────────┘
```

**JSON 描述符示例**:
```json
{
  "schema_version": "1.0",
  "task_type": "feature_extraction",
  "algorithm": {
    "name": "SIFT_GPU",
    "version": "1.0",
    "parameters": {
      "nfeatures": 8000,
      "threshold": 0.04,
      "octaves": -1,
      "levels": 3,
      "adapt_darkness": true
    }
  },
  "metadata": {
    "image_path": "/data/IMG_0001.jpg",
    "timestamp": "2026-02-11T10:30:00Z",
    "execution_time_ms": 1250
  },
  "blobs": [
    {
      "name": "keypoints",
      "dtype": "float32",
      "shape": [8000, 4],
      "offset": 0,
      "size": 128000
    },
    {
      "name": "descriptors",
      "dtype": "float32",
      "shape": [8000, 128],
      "offset": 128000,
      "size": 4096000
    }
  ]
}
```

## 进度监控

工具输出进度到 `stderr`，格式：
```
PROGRESS: 0.333333
PROGRESS: 0.666667
PROGRESS: 1.000000
```

可以用脚本解析进度：
```bash
./isat_extract -i images.json -o features/ 2>&1 | grep PROGRESS
```

## 示例

### 提取 10000 个特征点
```bash
./isat_extract -i dataset.json -o features/ -n 10000 -v
```

### 静默模式处理
```bash
./isat_extract -i large_dataset.json -o output/ -q
```

### 自定义参数
```bash
./isat_extract -i images.json -o features/ \
  --nfeatures 12000 \
  --threshold 0.03 \
  --octaves 4 \
  --levels 5
```

## 性能优化建议

1. **GPU 利用率**: 调整 `GPU_QUEUE_SIZE`（默认 5），根据 GPU 内存和图像大小调整
2. **I/O 线程数**: 根据存储速度调整 `NUM_IO_THREADS`（默认 4）
3. **内存控制**: 降低 `IO_QUEUE_SIZE` 可减少内存占用，但可能降低吞吐量

## 技术细节

### task_queue 使用

```cpp
// Stage 1: 多线程 I/O
Stage imageLoadStage("ImageLoad", 4, 10, [](int index) { /* 读图 */ });

// Stage 2: GPU 处理（当前线程）
StageCurrent siftGPUStage("SiftGPU", 1, 5, [](int index) { /* GPU提取 */ });

// Stage 3: 多线程写出
Stage writeStage("WriteIDC", 4, 10, [](int index) { /* 写IDC */ });

// 链接流水线
chain(imageLoadStage, siftGPUStage);
chain(siftGPUStage, writeStage);

// 执行
for (int i = 0; i < n; ++i) imageLoadStage.push(i);
std::thread gpu_thread([&]() { siftGPUStage.run(); });
imageLoadStage.wait();
gpu_thread.join();
writeStage.wait();
```

### SiftGPU 参数映射

| CLI 参数 | SiftGPU 参数 | 说明 |
|----------|--------------|------|
| `--nfeatures` | `-tc2` | 最大特征点数 |
| `--threshold` | `-t` | 峰值阈值（除以 levels） |
| `--octaves` | `-no` | 金字塔层数 |
| `--levels` | `-d` | 每层级别数 |
| `--no-adapt` | 不传 `-da` | 禁用暗图自适应 |

### OpenGL 上下文限制

SiftGPU 的 OpenGL 上下文不能跨线程共享，因此：
- Stage 2 必须使用 `StageCurrent`（在主线程运行）
- 每个线程需要独立的 `SiftGPUExtractor` 实例（使用 `thread_local`）

## 故障排查

### "SIFTGPU_FULL_SUPPORTED Error"
- 检查是否安装了 GLEW 和 OpenGL 驱动
- 尝试运行 `glxinfo | grep OpenGL` 验证 OpenGL 可用性

### "Failed to load image"
- 检查 JSON 中的路径是否正确
- 确认图像文件可读且格式支持（JPEG/PNG/TIFF）

### 内存不足
- 减少 `IO_QUEUE_SIZE` 和 `GPU_QUEUE_SIZE`
- 降低 `--nfeatures` 或使用图像降采样

## 未来改进

- [ ] 支持批量 GPU 处理（多图像并行）
- [ ] 支持分布式处理（Docker + 文件共享）
- [ ] 添加 IDC 读取器用于后续处理
- [ ] 支持更多特征提取算法（ORB/AKAZE）
- [ ] 自动参数调优

## 许可证

与 InsightAT 主项目相同。
