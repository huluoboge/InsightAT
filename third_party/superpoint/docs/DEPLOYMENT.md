# 部署指南

本指南说明如何将 SuperPoint ONNX C++ 部署到新机器。

## 📦 方式 1: 从源码安装（推荐）

### 前提条件
- Ubuntu 20.04+
- NVIDIA GPU (计算能力 6.1+)
- sudo 权限

### 步骤

```bash
# 1. 克隆仓库
git clone <your-repo-url> SuperPointONNX-Cpp
cd SuperPointONNX-Cpp

# 2. 一键安装所有依赖
./scripts/install_all.sh

# 完成！可执行文件: build/superpoint_inference
```

## 🐳 方式 2: Docker（规划中）

```bash
# 构建镜像
docker build -t superpoint-onnx-cpp .

# 运行
docker run --gpus all -v $(pwd):/workspace superpoint-onnx-cpp \
    /workspace/image.jpg gpu
```

## 📋 方式 3: 手动安装

### 1. 安装 CUDA 11.8

```bash
# 下载 CUDA 11.8
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run

# 安装
sudo sh cuda_11.8.0_520.61.05_linux.run \
    --silent \
    --toolkit \
    --installpath=/usr/local/cuda-11.8

# 设置环境变量
export CUDA_HOME=/usr/local/cuda-11.8
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
```

### 2. 安装 cuDNN 8

```bash
# 下载 cuDNN 8.9.7 for CUDA 11.x
# 从 https://developer.nvidia.com/cudnn
# 需要 NVIDIA 账号

# 解压并安装
tar -xzvf cudnn-linux-x86_64-8.9.7.29_cuda11-archive.tar.xz
sudo cp cudnn-linux-x86_64-8.9.7.29_cuda11-archive/include/cudnn*.h /usr/local/cuda-11.8/include
sudo cp cudnn-linux-x86_64-8.9.7.29_cuda11-archive/lib/libcudnn* /usr/local/cuda-11.8/lib64
sudo chmod a+r /usr/local/cuda-11.8/include/cudnn*.h /usr/local/cuda-11.8/lib64/libcudnn*
```

### 3. 安装 ONNX Runtime

```bash
# 下载
wget https://github.com/microsoft/onnxruntime/releases/download/v1.16.3/onnxruntime-linux-x64-gpu-1.16.3.tgz

# 解压
tar -xzf onnxruntime-linux-x64-gpu-1.16.3.tgz

# 安装到系统目录
sudo mkdir -p /opt/onnxruntime-gpu
sudo mv onnxruntime-linux-x64-gpu-1.16.3/* /opt/onnxruntime-gpu/

# 设置库路径
export LD_LIBRARY_PATH=/opt/onnxruntime-gpu/lib:$LD_LIBRARY_PATH
```

### 4. 安装 OpenCV

```bash
sudo apt update
sudo apt install -y libopencv-dev
```

### 5. 编译项目

```bash
# 克隆项目
git clone <your-repo-url> SuperPointONNX-Cpp
cd SuperPointONNX-Cpp

# 编译
./scripts/build.sh
```

## 🔍 验证安装

```bash
# 检查 CUDA
nvcc --version

# 检查 GPU
nvidia-smi

# 运行测试
./build/superpoint_inference examples/IMG_0926.JPG gpu

# 应该输出:
# ✓ 启用GPU推理 (CUDA)
# 推理时间: ~100ms (取决于GPU)
```

## 🚀 生产环境建议

### 1. 系统服务

创建 `/etc/systemd/system/superpoint.service`:

```ini
[Unit]
Description=SuperPoint Inference Service
After=network.target

[Service]
Type=simple
User=superpoint
WorkingDir=/opt/SuperPointONNX-Cpp
Environment="CUDA_HOME=/usr/local/cuda-11.8"
Environment="LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:/opt/onnxruntime-gpu/lib"
ExecStart=/opt/SuperPointONNX-Cpp/build/superpoint_inference

[Install]
WantedBy=multi-user.target
```

启动服务:
```bash
sudo systemctl enable superpoint
sudo systemctl start superpoint
```

### 2. 性能优化

```bash
# CPU 固定
taskset -c 0-3 ./build/superpoint_inference image.jpg gpu

# GPU 锁频 (避免降频)
sudo nvidia-smi -pm 1
sudo nvidia-smi -lgc 1800

# 预加载模型 (避免首次延迟)
# 在代码中添加 warm-up 推理
```

### 3. 监控

```bash
# GPU 使用率
nvidia-smi dmon -s u

# 内存使用
nvidia-smi dmon -s m

# 进程监控
watch -n 1 nvidia-smi
```

## 📊 不同平台性能

| GPU | CUDA | 推理时间 @ 640×480 | FPS |
|-----|------|-------------------|-----|
| GTX 1060 6GB | 11.8 | 104ms | 9.5 |
| RTX 3060 12GB | 11.8 | ~35ms | ~28.5 |
| RTX 4060 8GB | 11.8 | ~25ms | ~40 |
| CPU (i7-9700) | - | 180ms | 5.6 |

## 🐛 常见问题

### 1. libcudnn.so.9 找不到

**问题**: CUDA 12 的 cuDNN 9 冲突  
**解决**: 确保使用 CUDA 11.8 + cuDNN 8

```bash
# 检查当前 CUDA 版本
nvcc --version

# 应该显示: release 11.8
```

### 2. GPU 不工作（使用 CPU）

**问题**: ONNX Runtime 找不到 CUDA  
**解决**: 设置库路径

```bash
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:/opt/onnxruntime-gpu/lib:$LD_LIBRARY_PATH
```

### 3. 编译错误

**问题**: OpenCV 或其他依赖缺失  
**解决**:

```bash
sudo apt install -y build-essential cmake libopencv-dev
```

## 📦 打包分发

### 创建二进制包

```bash
# 收集所有依赖
mkdir -p package/lib
cp build/superpoint_inference package/
cp /opt/onnxruntime-gpu/lib/*.so* package/lib/
cp /usr/local/cuda-11.8/lib64/libcudart.so* package/lib/
cp /usr/local/cuda-11.8/lib64/libcudnn.so* package/lib/

# 创建启动脚本
cat > package/run.sh << 'EOF'
#!/bin/bash
export LD_LIBRARY_PATH=$(dirname $0)/lib:$LD_LIBRARY_PATH
$(dirname $0)/superpoint_inference "$@"
EOF

chmod +x package/run.sh

# 打包
tar -czf superpoint-onnx-cpp-$(uname -m).tar.gz package/

# 使用
tar -xzf superpoint-onnx-cpp-x86_64.tar.gz
cd package
./run.sh image.jpg gpu
```

## 🔐 安全建议

- 不要以 root 用户运行
- 限制文件访问权限
- 使用防火墙保护服务端口
- 定期更新依赖库

---

**版本**: 1.0.0  
**更新**: 2026-02-15
