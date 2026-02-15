#!/bin/bash
# SuperPoint C++ - CUDA 11.8 环境配置

# CUDA 11.8
export CUDA_HOME=/usr/local/cuda-11.8
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH

# ONNX Runtime
export LD_LIBRARY_PATH=/opt/onnxruntime-gpu/lib:$LD_LIBRARY_PATH

echo "✓ CUDA 11.8 环境已加载"
echo "  CUDA_HOME: $CUDA_HOME"
echo "  nvcc: $(which nvcc)"
