#!/bin/bash
# 安装CUDA 11.8 + cuDNN 8 (与CUDA 12共存)
# 用于GTX 1060 GPU加速

set -e

echo "========================================================================"
echo "GTX 1060 GPU加速环境配置"
echo "安装CUDA 11.8 + cuDNN 8 (与CUDA 12共存)"
echo "========================================================================"

# 检查当前CUDA版本
echo ""
echo "[1/6] 检查当前CUDA版本..."
if command -v nvcc &> /dev/null; then
    current_cuda=$(nvcc --version | grep "release" | awk '{print $5}' | cut -d',' -f1)
    echo "  当前CUDA: $current_cuda"
else
    echo "  未找到CUDA"
fi

nvidia-smi | grep "CUDA Version" || echo "  检查nvidia-smi..."

# 下载CUDA 11.8
echo ""
echo "[2/6] 下载CUDA 11.8 Toolkit..."
CUDA_11_8_URL="https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run"
CUDA_11_8_FILE="cuda_11.8.0_520.61.05_linux.run"

if [ -f "$CUDA_11_8_FILE" ]; then
    echo "  ✓ 安装包已存在: $CUDA_11_8_FILE"
else
    echo "  下载中... (约3GB，需要几分钟)"
    wget -c $CUDA_11_8_URL
    echo "  ✓ 下载完成"
fi

# 安装CUDA 11.8 (不覆盖驱动，与CUDA 12共存)
echo ""
echo "[3/6] 安装CUDA 11.8..."
echo "  注意: 不会覆盖CUDA 12或NVIDIA驱动"
read -p "  继续安装? (Y/n): " confirm
if [[ ! "$confirm" =~ ^[Nn]$ ]]; then
    sudo sh $CUDA_11_8_FILE \
        --toolkit \
        --installpath=/usr/local/cuda-11.8 \
        --no-opengl-libs \
        --no-drm \
        --silent \
        --override
    
    echo "  ✓ CUDA 11.8 安装完成"
    echo "  位置: /usr/local/cuda-11.8"
else
    echo "  跳过CUDA 11.8安装"
fi

# 验证CUDA 11.8
if [ -f /usr/local/cuda-11.8/bin/nvcc ]; then
    echo ""
    echo "  验证CUDA 11.8:"
    /usr/local/cuda-11.8/bin/nvcc --version | grep "release"
else
    echo "  ✗ CUDA 11.8未正确安装"
    exit 1
fi

# 卸载cuDNN 9
echo ""
echo "[4/6] 处理cuDNN..."
if dpkg -l | grep -q libcudnn9-cuda-12; then
    echo "  检测到cuDNN 9 (CUDA 12)，需要安装cuDNN 8 (CUDA 11)"
    read -p "  是否继续? (Y/n): " confirm_cudnn
    if [[ ! "$confirm_cudnn" =~ ^[Nn]$ ]]; then
        # 不卸载cuDNN 9，而是同时安装cuDNN 8
        echo "  保留cuDNN 9，同时安装cuDNN 8..."
    fi
fi

# 安装cuDNN 8 for CUDA 11
echo ""
echo "[5/6] 安装cuDNN 8 for CUDA 11.8..."

# 添加NVIDIA仓库
if [ ! -f /usr/share/keyrings/cuda-archive-keyring.gpg ]; then
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
    sudo dpkg -i cuda-keyring_1.1-1_all.deb
    sudo apt update
fi

# 安装cuDNN 8
echo "  安装cuDNN 8.9.7 for CUDA 11.8..."
sudo apt install -y \
    libcudnn8=8.9.7.*-1+cuda11.8 \
    libcudnn8-dev=8.9.7.*-1+cuda11.8 \
    libcudnn8-samples=8.9.7.*-1+cuda11.8

# 防止自动更新
sudo apt-mark hold libcudnn8 libcudnn8-dev

# 更新库缓存
sudo ldconfig

echo "  ✓ cuDNN 8 安装完成"

# 验证cuDNN
echo ""
echo "  验证cuDNN安装:"
ldconfig -p | grep cudnn

# 环境变量设置
echo ""
echo "[6/6] 配置环境变量..."

cat > ~/.cuda11_env << 'EOF'
# CUDA 11.8 环境变量
# 使用方法: source ~/.cuda11_env

export CUDA_HOME=/usr/local/cuda-11.8
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
export CUDNN_PATH=/usr/lib/x86_64-linux-gnu

echo "✓ CUDA 11.8 环境已激活"
echo "  - CUDA_HOME: $CUDA_HOME"
echo "  - nvcc: $(which nvcc)"
echo "  - version: $(nvcc --version | grep release | awk '{print $5}')"
EOF

echo "  ✓ 环境配置文件已创建: ~/.cuda11_env"
echo ""
echo "  使用CUDA 11.8时，运行:"
echo "    source ~/.cuda11_env"
echo ""
echo "  使用CUDA 12.8时，运行:"
echo "    export CUDA_HOME=/usr/local/cuda-12.8"
echo "    export PATH=/usr/local/cuda-12.8/bin:\$PATH"

# 创建快捷切换脚本
cat > switch_cuda.sh << 'EOF'
#!/bin/bash
# CUDA版本切换脚本

if [ "$1" == "11" ] || [ "$1" == "11.8" ]; then
    export CUDA_HOME=/usr/local/cuda-11.8
    export PATH=/usr/local/cuda-11.8/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
    echo "✓ 切换到 CUDA 11.8"
    nvcc --version | grep release
elif [ "$1" == "12" ] || [ "$1" == "12.8" ]; then
    export CUDA_HOME=/usr/local/cuda-12.8
    export PATH=/usr/local/cuda-12.8/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH
    echo "✓ 切换到 CUDA 12.8"
    nvcc --version | grep release
else
    echo "用法: source switch_cuda.sh [11|12]"
    echo "当前CUDA版本:"
    nvcc --version | grep release || echo "未设置CUDA环境"
fi
EOF

chmod +x switch_cuda.sh

echo ""
echo "========================================================================"
echo "安装完成！"
echo "========================================================================"
echo ""
echo "系统现在有两个CUDA版本:"
echo "  - CUDA 12.8: /usr/local/cuda-12.8 (PyTorch等)"
echo "  - CUDA 11.8: /usr/local/cuda-11.8 (ONNX Runtime GPU)"
echo ""
echo "cuDNN版本:"
dpkg -l | grep cudnn | awk '{print "  - " $2 " " $3}'
echo ""
echo "切换CUDA版本:"
echo "  source switch_cuda.sh 11  # 切换到CUDA 11.8"
echo "  source switch_cuda.sh 12  # 切换到CUDA 12.8"
echo ""
echo "下一步:"
echo "  1. 激活CUDA 11.8环境: source ~/.cuda11_env"
echo "  2. 编译ONNX Runtime C++ API"
echo "  3. 测试GPU推理"
echo ""
echo "========================================================================"
