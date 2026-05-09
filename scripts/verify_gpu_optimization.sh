#!/bin/bash
# GPU 显存优化验证脚本
# 使用：./verify_gpu_optimization.sh <dataset.iat> [output_dir]

set -e

DATASET="${1:?Usage: $0 <dataset.iat> [output_dir]}"
OUTPUT_DIR="${2:-.}"

if [ ! -f "$DATASET" ]; then
    echo "❌ 错误：数据集文件不存在 $DATASET"
    exit 1
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "GPU 显存优化验证"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检查 NVIDIA GPU
if ! command -v nvidia-smi &> /dev/null; then
    echo "⚠️  警告：未检测到 nvidia-smi，跳过 GPU 监控"
    USE_GPU_MONITOR=0
else
    USE_GPU_MONITOR=1
    echo "✅ 检测到 nvidia-smi"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
fi

# 检查可执行文件
if [ ! -f build/isat_geo ]; then
    echo "❌ 错误：isat_geo 不存在，请先编译"
    exit 1
fi

echo ""
echo "📋 测试参数"
echo "  数据集: $DATASET"
echo "  可执行文件: ./build/isat_geo"
echo "  输出目录: $OUTPUT_DIR"
echo ""

# 创建临时目录
TMP_DIR=$(mktemp -d)
trap "rm -rf $TMP_DIR" EXIT

echo "🚀 启动 isat_geo..."
echo ""

# 后台启动 GPU 监控
if [ "$USE_GPU_MONITOR" -eq 1 ]; then
    (
        while true; do
            echo "[$(date '+%H:%M:%S')]" $(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader)
            sleep 0.5
        done
    ) > "$TMP_DIR/gpu_stats.log" 2>&1 &
    GPU_MONITOR_PID=$!
    trap "kill $GPU_MONITOR_PID 2>/dev/null || true; rm -rf $TMP_DIR" EXIT
fi

# 运行 isat_geo 并测时
START_TIME=$(date +%s%N)
./build/isat_geo -i "$DATASET" -t 0 -v > "$TMP_DIR/isat_geo.log" 2>&1
END_TIME=$(date +%s%N)

ELAPSED_MS=$(( (END_TIME - START_TIME) / 1000000 ))
ELAPSED_S=$(echo "scale=3; $ELAPSED_MS / 1000" | bc)

# 停止 GPU 监控
if [ "$USE_GPU_MONITOR" -eq 1 ]; then
    sleep 1
    kill $GPU_MONITOR_PID 2>/dev/null || true
fi

echo ""
echo "✅ isat_geo 完成"
echo ""

# 解析日志
echo "📊 结果分析"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if grep -q "pairs processed" "$TMP_DIR/isat_geo.log"; then
    PAIRS=$(grep "pairs processed" "$TMP_DIR/isat_geo.log" | tail -1 | grep -oE '[0-9]+' | head -1)
    echo "✅ 处理对数: $PAIRS"
    THROUGHPUT=$(echo "scale=1; $PAIRS / $ELAPSED_S" | bc)
    echo "✅ 吞吐量: $THROUGHPUT pairs/sec"
fi

echo "⏱️  总耗时: ${ELAPSED_S}s (${ELAPSED_MS}ms)"

# 输出 GPU 统计
if [ "$USE_GPU_MONITOR" -eq 1 ] && [ -f "$TMP_DIR/gpu_stats.log" ]; then
    echo ""
    echo "📈 GPU 使用统计"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # 提取最大值
    MAX_UTIL=$(awk -F',' '{print $1}' "$TMP_DIR/gpu_stats.log" | grep -oE '[0-9]+' | sort -n | tail -1)
    MAX_MEM=$(awk -F',' '{print $2}' "$TMP_DIR/gpu_stats.log" | grep -oE '[0-9]+' | sort -n | tail -1)
    
    echo "  最大 GPU 利用率: ${MAX_UTIL}%"
    echo "  最大 VRAM 占用: ${MAX_MEM}MB"
    
    echo ""
    echo "🔍 完整 GPU 监控日志 (前 20 行)："
    head -20 "$TMP_DIR/gpu_stats.log"
fi

# 保存输出
echo ""
echo "💾 详细日志"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cp "$TMP_DIR/isat_geo.log" "$OUTPUT_DIR/isat_geo_latest.log"
echo "✅ isat_geo 主日志: $OUTPUT_DIR/isat_geo_latest.log"

if [ "$USE_GPU_MONITOR" -eq 1 ] && [ -f "$TMP_DIR/gpu_stats.log" ]; then
    cp "$TMP_DIR/gpu_stats.log" "$OUTPUT_DIR/gpu_stats_latest.log"
    echo "✅ GPU 监控日志: $OUTPUT_DIR/gpu_stats_latest.log"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✨ 验证完成！"
echo ""

# 给出建议
if [ "$USE_GPU_MONITOR" -eq 1 ]; then
    if [ -n "$MAX_UTIL" ] && [ "$MAX_UTIL" -lt 50 ]; then
        echo "⚠️  建议：GPU 利用率偏低 (${MAX_UTIL}%)，可考虑"
        echo "   - 增加批处理数据"
        echo "   - 减少其他后台进程"
        echo "   - 检查 CPU 是否为瓶颈"
    elif [ -n "$MAX_UTIL" ] && [ "$MAX_UTIL" -gt 90 ]; then
        echo "✨ 优秀：GPU 利用率很高 (${MAX_UTIL}%)，优化成功！"
    fi
fi

echo ""
