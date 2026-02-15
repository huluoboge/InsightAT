#!/bin/bash

echo "=========================================="
echo "SuperPoint ONNX C++ - 性能测试"
echo "=========================================="
echo ""

# 激活环境
if [ -f "scripts/setup_env.sh" ]; then
    source scripts/setup_env.sh
fi

# 检查可执行文件
if [ ! -f "build/superpoint_inference" ]; then
    echo "✗ 未找到可执行文件"
    echo "  请先运行: ./scripts/build.sh"
    exit 1
fi

# 检查示例图像
IMAGE="examples/IMG_0926.JPG"
if [ ! -f "$IMAGE" ]; then
    echo "✗ 示例图像未找到: $IMAGE"
    exit 1
fi

# 运行测试
ITERATIONS=10
echo "运行 $ITERATIONS 次测试..."
echo ""

total_time=0

for i in $(seq 1 $ITERATIONS); do
    echo -n "[$i/$ITERATIONS] "
    
    output=$(./build/superpoint_inference "$IMAGE" gpu 2>&1)
    time=$(echo "$output" | grep "推理时间:" | grep -oP '\d+(?=ms)')
    
    if [ -n "$time" ]; then
        echo "推理时间: ${time}ms"
        total_time=$((total_time + time))
    else
        echo "失败"
    fi
done

# 计算平均值
if [ $total_time -gt 0 ]; then
    avg_time=$((total_time / ITERATIONS))
    avg_fps=$((1000 / avg_time))
    
    echo ""
    echo "=========================================="
    echo "性能统计:"
    echo "  平均推理时间: ${avg_time} ms"
    echo "  平均 FPS: ${avg_fps}"
    echo "  总迭代次数: ${ITERATIONS}"
    echo "=========================================="
fi
