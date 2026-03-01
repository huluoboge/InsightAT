#!/bin/bash
# 下载和配置ONNX Runtime C++ GPU库 (CUDA 11.8)

set -e

echo "========================================================================"
echo "下载ONNX Runtime C++ GPU库"
echo "========================================================================"

# ONNX Runtime版本
ONNXRUNTIME_VERSION="1.16.3"  # 最后支持CUDA 11的稳定版本
INSTALL_DIR="/opt/onnxruntime-gpu"

echo ""
echo "配置:"
echo "  版本: ${ONNXRUNTIME_VERSION}"
echo "  安装路径: ${INSTALL_DIR}"
echo "  CUDA: 11.8"
echo ""

# 检查是否已安装
if [ -d "$INSTALL_DIR" ]; then
    echo "检测到已安装的ONNX Runtime: $INSTALL_DIR"
    read -p "是否重新下载? (y/N): " reinstall
    if [[ ! "$reinstall" =~ ^[Yy]$ ]]; then
        echo "使用现有安装"
        exit 0
    fi
    sudo rm -rf $INSTALL_DIR
fi

# 下载URL
DOWNLOAD_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz"
DOWNLOAD_FILE="onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz"

echo "下载ONNX Runtime..."
echo "  URL: $DOWNLOAD_URL"

if [ -f "$DOWNLOAD_FILE" ]; then
    echo "  ✓ 文件已存在: $DOWNLOAD_FILE"
else
    wget -c $DOWNLOAD_URL
    echo "  ✓ 下载完成"
fi

# 解压
echo ""
echo "解压..."
tar -xzf $DOWNLOAD_FILE

# 移动到安装目录
EXTRACTED_DIR="onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}"
echo ""
echo "安装到 $INSTALL_DIR ..."
sudo mkdir -p /opt
sudo mv $EXTRACTED_DIR $INSTALL_DIR

# 验证安装
echo ""
echo "验证安装..."
if [ -f "$INSTALL_DIR/lib/libonnxruntime.so" ]; then
    echo "  ✓ 找到库文件: $INSTALL_DIR/lib/libonnxruntime.so"
    ls -lh $INSTALL_DIR/lib/libonnxruntime.so*
else
    echo "  ✗ 未找到库文件"
    exit 1
fi

if [ -d "$INSTALL_DIR/include" ]; then
    echo "  ✓ 找到头文件: $INSTALL_DIR/include"
    ls $INSTALL_DIR/include/onnxruntime/
else
    echo "  ✗ 未找到头文件"
    exit 1
fi

# 设置库路径
echo ""
echo "配置库路径..."
echo "$INSTALL_DIR/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig

# 验证库可以找到
if ldconfig -p | grep -q onnxruntime; then
    echo "  ✓ 库路径已配置"
    ldconfig -p | grep onnxruntime
else
    echo "  ⚠ 库路径配置可能有问题"
fi

# 清理下载文件
echo ""
read -p "删除下载的压缩包? (Y/n): " cleanup
if [[ ! "$cleanup" =~ ^[Nn]$ ]]; then
    rm -f $DOWNLOAD_FILE
    echo "  ✓ 已清理"
fi

echo ""
echo "========================================================================"
echo "ONNX Runtime C++ 安装完成！"
echo "========================================================================"
echo ""
echo "安装信息:"
echo "  路径: $INSTALL_DIR"
echo "  版本: $ONNXRUNTIME_VERSION"
echo "  头文件: $INSTALL_DIR/include"
echo "  库文件: $INSTALL_DIR/lib"
echo ""
echo "使用方法:"
echo "  在CMakeLists.txt中设置:"
echo "    set(ONNXRUNTIME_ROOT \"$INSTALL_DIR\")"
echo ""
echo "下一步:"
echo "  ./build_cpp_api.sh"
echo ""
echo "========================================================================"
