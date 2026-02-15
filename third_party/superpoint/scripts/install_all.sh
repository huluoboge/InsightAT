#!/bin/bash
# SuperPoint ONNX C++ - 一键安装脚本

set -e

echo "=========================================="
echo "SuperPoint ONNX C++ - 一键安装"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 检查是否在正确的目录
if [ ! -f "$PROJECT_ROOT/CMakeLists.txt" ]; then
    echo "✗ 错误: 请在项目根目录运行此脚本"
    exit 1
fi

cd "$PROJECT_ROOT"

# 步骤 1: 安装 CUDA 11.8 + cuDNN 8
echo "[1/3] 安装 CUDA 11.8 + cuDNN 8..."
if [ ! -d "/usr/local/cuda-11.8" ]; then
    ./scripts/install_cuda11.sh
else
    echo "  ✓ CUDA 11.8 已安装"
fi
echo ""

# 步骤 2: 下载 ONNX Runtime
echo "[2/3] 下载 ONNX Runtime C++ GPU..."
if [ ! -d "/opt/onnxruntime-gpu" ]; then
    ./scripts/download_onnxruntime.sh
else
    echo "  ✓ ONNX Runtime 已安装"
fi
echo ""

# 步骤 3: 编译项目
echo "[3/3] 编译项目..."
./scripts/build.sh
echo ""

# 验证安装
echo "=========================================="
echo "验证安装..."
echo "=========================================="
echo ""

if [ -f "build/superpoint_inference" ]; then
    echo "✓ 可执行文件: build/superpoint_inference"
    
    if [ -f "examples/IMG_0926.JPG" ]; then
        echo ""
        echo "运行测试..."
        ./build/superpoint_inference examples/IMG_0926.JPG gpu
    fi
else
    echo "✗ 编译失败"
    exit 1
fi

echo ""
echo "=========================================="
echo "✓ 安装完成！"
echo "=========================================="
echo ""
echo "快速使用:"
echo "  ./build/superpoint_inference <image> gpu"
echo ""
echo "性能测试:"
echo "  ./scripts/benchmark.sh"
echo ""
echo "文档:"
echo "  docs/CPP_API_GUIDE.md"
echo ""
