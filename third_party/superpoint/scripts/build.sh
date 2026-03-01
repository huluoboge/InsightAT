#!/bin/bash

echo "=========================================="
echo "SuperPoint ONNX C++ - 编译脚本"
echo "=========================================="
echo ""

# 激活 CUDA 11.8 环境
if [ -f "$HOME/.cuda11_env" ]; then
    echo "加载 CUDA 11.8 环境..."
    source "$HOME/.cuda11_env"
else
    echo "⚠ 警告: CUDA 11 环境文件未找到"
    echo "  设置默认路径..."
    export CUDA_HOME=/usr/local/cuda-11.8
    export PATH=/usr/local/cuda-11.8/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
fi

# 检查 CUDA
echo ""
echo "检查 CUDA..."
nvcc --version | grep "release" || {
    echo "✗ CUDA 未找到或版本错误"
    exit 1
}

# 检查 ONNX Runtime
echo ""
echo "检查 ONNX Runtime..."
if [ ! -d "/opt/onnxruntime-gpu" ]; then
    echo "✗ ONNX Runtime 未安装"
    echo "  请运行: ./scripts/download_onnxruntime.sh"
    exit 1
fi

# 创建构建目录
cd "$(dirname "$0")/.." || exit 1
mkdir -p build
cd build || exit 1

# CMake 配置
echo ""
echo "CMake 配置..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \
    -DONNXRUNTIME_ROOT=/opt/onnxruntime-gpu

if [ $? -ne 0 ]; then
    echo "✗ CMake 配置失败"
    exit 1
fi

# 编译
echo ""
echo "编译中..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "✗ 编译失败"
    exit 1
fi

echo ""
echo "=========================================="
echo "✓ 编译成功！"
echo "=========================================="
echo ""
echo "可执行文件: build/superpoint_inference"
echo ""
echo "运行示例:"
echo "  ./build/superpoint_inference examples/IMG_0926.JPG gpu"
echo ""
