# SuperPoint ONNX C++ Inference

高性能 SuperPoint 关键点检测 C++ 实现，基于 ONNX Runtime GPU 加速。

## ✨ 特性

- **GPU 加速**: CUDA 11.8 + cuDNN 8 支持
- **高性能**: ~104ms @ 640×480 (GTX 1060)
- **动态分辨率**: 支持任意图像尺寸
- **完整 NMS**: 非极大值抑制算法
- **跨平台**: CPU/GPU 自动切换

## 📋 依赖

### 系统要求
- Ubuntu 20.04+ / Linux
- NVIDIA GPU (计算能力 6.1+)
- GCC 9+
- CMake 3.18+

### 软件依赖
- **CUDA**: 11.8.89
- **cuDNN**: 8.9.7
- **ONNX Runtime**: 1.16.3 (GPU 版本)
- **OpenCV**: 4.5.4+

## 🚀 快速开始

### 1. 安装依赖

```bash
# 安装 CUDA 11.8 + cuDNN 8
cd scripts
./install_cuda11.sh

# 下载 ONNX Runtime C++ GPU
./download_onnxruntime.sh
```

### 2. 编译

```bash
# 激活 CUDA 11 环境
source scripts/setup_env.sh

# 编译项目
./scripts/build.sh
```

### 3. 运行

```bash
# GPU 推理
./build/superpoint_inference examples/IMG_0926.JPG gpu

# CPU 推理
./build/superpoint_inference examples/IMG_0926.JPG cpu
```

## 📖 使用方法

### 基本用法

```cpp
#include "superpoint.hpp"

// 创建推理器（GPU）
SuperPointONNX superpoint("superpoint.onnx", true);

// 读取图像
cv::Mat image = cv::imread("image.jpg");

// 推理
auto detection = superpoint.infer(image, 0.005, 4);

// 结果
std::cout << "关键点数: " << detection.keypoints.size() << std::endl;
std::cout << "描述符维度: " << detection.descriptors[0].size() << std::endl;

// 可视化
cv::Mat result = superpoint.visualize(image, detection);
cv::imwrite("result.jpg", result);
```

### 参数调整

```cpp
// infer(image, threshold, nms_radius)
auto detection = superpoint.infer(
    image,
    0.005,  // 置信度阈值 (越小检测越多)
    4       // NMS 半径 (越大抑制越强)
);
```

## 📊 性能基准

GTX 1060 6GB @ 640×480:

| 配置 | 推理时间 | FPS | 关键点数 |
|------|----------|-----|----------|
| GPU (NMS=4) | 104.5ms | 9.5 | ~5000 |
| CPU (NMS=4) | 180ms | 5.6 | ~5000 |

## 🔧 构建选项

### 自定义 CUDA 路径

```bash
cmake -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 ..
```

### 自定义 ONNX Runtime 路径

```bash
cmake -DONNXRUNTIME_ROOT=/path/to/onnxruntime ..
```

### Release 模式

```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
```

## 📁 项目结构

```
cpp/
├── CMakeLists.txt          # CMake 配置
├── README.md               # 本文档
├── superpoint.onnx         # ONNX 模型
├── src/
│   └── superpoint_inference.cpp  # 主程序
├── include/
│   └── (头文件)
├── scripts/
│   ├── install_cuda11.sh   # CUDA 安装脚本
│   ├── download_onnxruntime.sh  # ONNX Runtime 下载
│   ├── build.sh            # 编译脚本
│   ├── benchmark.sh        # 性能测试
│   └── setup_env.sh        # 环境配置
├── docs/
│   └── (文档)
├── examples/
│   └── IMG_0926.JPG        # 示例图像
└── build/                  # 构建输出
```

## 🐛 故障排除

### CUDA 版本问题

```bash
# 检查 CUDA 版本
nvcc --version

# 应该显示: release 11.8, V11.8.89

# 如果不是，激活 CUDA 11 环境
source scripts/setup_env.sh
```

### ONNX Runtime 找不到

```bash
# 设置环境变量
export LD_LIBRARY_PATH=/opt/onnxruntime-gpu/lib:$LD_LIBRARY_PATH

# 或在 ~/.bashrc 中添加
echo 'export LD_LIBRARY_PATH=/opt/onnxruntime-gpu/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```

### GPU 不工作

```bash
# 检查 GPU
nvidia-smi

# 检查 CUDA 库
ldconfig -p | grep cuda

# 运行时应该看到：
# ✓ 启用GPU推理 (CUDA)
```

## 📝 许可证

本项目基于原始 SuperPoint 实现。

## 🔗 相关链接

- **SuperPoint 论文**: https://arxiv.org/abs/1712.07629
- **ONNX Runtime**: https://onnxruntime.ai/
- **原始 PyTorch 实现**: ../superpoint/

## 📧 联系方式

如有问题，请提交 Issue。

---

**版本**: 1.0.0  
**更新日期**: 2026-02-15
