# SuperPoint ONNX C++

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![CUDA](https://img.shields.io/badge/CUDA-11.8-green.svg)](https://developer.nvidia.com/cuda-toolkit)
[![ONNX Runtime](https://img.shields.io/badge/ONNX%20Runtime-1.16.3-orange.svg)](https://onnxruntime.ai/)

高性能 SuperPoint 关键点检测 C++ 实现，基于 ONNX Runtime GPU 加速。

## 🚀 快速开始

```bash
# 1. 克隆仓库
git clone <your-repo-url>
cd SuperPointONNX-Cpp

# 2. 安装依赖（需要 sudo）
./scripts/install_cuda11.sh
./scripts/download_onnxruntime.sh

# 3. 编译
./scripts/build.sh

# 4. 运行
./build/superpoint_inference examples/IMG_0926.JPG gpu
```

## 📊 性能

GTX 1060 6GB @ 640×480:
- **GPU**: ~104ms (9.5 FPS)
- **CPU**: ~180ms (5.6 FPS)

RTX 3060 @ 640×480 (预估):
- **GPU**: ~35ms (28.5 FPS)

## 📖 文档

- [完整使用指南](docs/CPP_API_GUIDE.md)
- [性能报告](docs/FINAL_REPORT.md)
- [GPU 兼容性](docs/GTX1060_COMPATIBILITY.md)

## 🔧 系统要求

- **OS**: Ubuntu 20.04+
- **GPU**: NVIDIA (计算能力 6.1+)
- **CUDA**: 11.8.89
- **cuDNN**: 8.9.7
- **OpenCV**: 4.5+

## 📁 项目结构

```
.
├── CMakeLists.txt          # CMake 配置
├── README.md               # 快速开始
├── superpoint.onnx         # ONNX 模型 (5.0 MB)
├── src/                    # 源代码
│   └── superpoint_inference.cpp
├── scripts/                # 构建脚本
│   ├── build.sh
│   ├── benchmark.sh
│   ├── install_cuda11.sh
│   └── download_onnxruntime.sh
├── docs/                   # 详细文档
└── examples/               # 示例图像
```

## 🎯 使用示例

### 基本推理

```cpp
SuperPointONNX superpoint("superpoint.onnx", true);  // GPU
cv::Mat image = cv::imread("image.jpg");

auto detection = superpoint.infer(image, 0.005, 4);
// detection.keypoints: 关键点位置
// detection.scores: 置信度
// detection.descriptors: 256维描述符

cv::Mat result = superpoint.visualize(image, detection);
```

### 调整参数

```cpp
// 更多关键点 (降低阈值)
auto detection = superpoint.infer(image, 0.001, 4);

// 更稀疏关键点 (增大 NMS)
auto detection = superpoint.infer(image, 0.005, 8);

// CPU 模式
SuperPointONNX superpoint("superpoint.onnx", false);
```

## 🛠️ 高级配置

### 自定义编译

```bash
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \
    -DONNXRUNTIME_ROOT=/opt/onnxruntime-gpu

make -j$(nproc)
```

### 性能测试

```bash
./scripts/benchmark.sh
```

## 📝 TODO

- [ ] 提取为共享库 (.so)
- [ ] Python Bindings (pybind11)
- [ ] 批处理支持
- [ ] TensorRT 后端选项
- [ ] Docker 镜像

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

MIT License

## 🔗 相关项目

- [SuperPoint 论文](https://arxiv.org/abs/1712.07629)
- [原始 PyTorch 实现](https://github.com/magicleap/SuperPointPretrainedNetwork)

---

**版本**: 1.0.0  
**作者**: jones  
**日期**: 2026-02-15
