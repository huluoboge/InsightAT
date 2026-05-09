#!/bin/bash
# GPU 优化实施检查清单
# 使用：bash check_gpu_optimization.sh

set -e

REPO_ROOT="/home/jones/Git/01jones/InsightAT"
cd "$REPO_ROOT"

echo "╔═══════════════════════════════════════════════════════╗"
echo "║    GPU 显存优化实施检查清单                          ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo ""

# 检查项计数
PASSED=0
FAILED=0

# 1. 检查代码改动
echo "📝 代码改动检查"
echo "─────────────────────────────────────────────────────────"

# 检查 batch 大小
BATCH=$(grep "^#define INSIGHTAT_CUDA_GEO_BATCH_MAX_PAIRS" \
        src/algorithm/modules/geometry/cuda_geo_ransac.h | awk '{print $3}')
if [ "$BATCH" = "10000" ]; then
    echo "✅ BATCH_MAX_PAIRS = $BATCH (正确)"
    ((PASSED++))
else
    echo "❌ BATCH_MAX_PAIRS = $BATCH (应是 10000)"
    ((FAILED++))
fi

# 检查 warp 线程
WARP=$(grep "cfg.local_size_x = " src/algorithm/tools/isat_geo.cpp | head -1 | grep -oE "[0-9]+")
if [ "$WARP" = "128" ]; then
    echo "✅ local_size_x = $WARP (正确)"
    ((PASSED++))
else
    echo "❌ local_size_x = $WARP (应是 128)"
    ((FAILED++))
fi

# 检查 GPU 队列
QUEUE=$(grep "const int GPU_Q = " src/algorithm/tools/isat_geo.cpp | tail -1 | grep -oE "[0-9]+")
if [ "$QUEUE" = "256" ]; then
    echo "✅ GPU_Q = $QUEUE (正确)"
    ((PASSED++))
else
    echo "❌ GPU_Q = $QUEUE (应是 256)"
    ((FAILED++))
fi

echo ""

# 2. 检查编译
echo "🔨 编译状态检查"
echo "─────────────────────────────────────────────────────────"

if [ -f "build/isat_geo" ]; then
    SIZE=$(ls -lh build/isat_geo | awk '{print $5}')
    TIME=$(stat -c %y build/isat_geo | cut -d' ' -f1-2)
    echo "✅ isat_geo 可执行文件存在"
    echo "   大小：$SIZE"
    echo "   修改时间：$TIME"
    ((PASSED++))
else
    echo "❌ build/isat_geo 不存在，需要编译"
    echo "   运行：make -j10 isat_geo"
    ((FAILED++))
fi

echo ""

# 3. 检查文档
echo "📚 文档完整性检查"
echo "─────────────────────────────────────────────────────────"

DOCS=(
    "GPU_OPTIMIZATION_FINAL_SUMMARY.md"
    "GPU_OPTIMIZATION_QUICK_START.md" 
    "GPU_VRAM_OPTIMIZATION_1G_TO_5G.md"
    "CHANGELOG_GPU_OPTIMIZATION_v5.md"
)

for doc in "${DOCS[@]}"; do
    if [ -f "$doc" ]; then
        LINES=$(wc -l < "$doc")
        echo "✅ $doc ($LINES 行)"
        ((PASSED++))
    else
        echo "❌ $doc 缺失"
        ((FAILED++))
    fi
done

echo ""

# 4. 检查工具脚本
echo "🛠️  工具脚本检查"
echo "─────────────────────────────────────────────────────────"

SCRIPTS=(
    "scripts/verify_gpu_optimization.sh"
    "scripts/gpu_diagnostic.py"
)

for script in "${SCRIPTS[@]}"; do
    if [ -f "$script" ]; then
        if [ -x "$script" ] || [ "${script##*.}" = "py" ]; then
            echo "✅ $script"
            ((PASSED++))
        else
            echo "⚠️  $script 存在但不可执行"
            chmod +x "$script"
            echo "   已设置为可执行"
        fi
    else
        echo "❌ $script 缺失"
        ((FAILED++))
    fi
done

echo ""

# 5. 环境检查
echo "🔧 环境检查"
echo "─────────────────────────────────────────────────────────"

# NVIDIA GPU
if command -v nvidia-smi &> /dev/null; then
    GPU_COUNT=$(nvidia-smi --list-gpus | wc -l)
    echo "✅ NVIDIA GPU 检测到 ($GPU_COUNT 个)"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | \
        while read line; do 
            echo "   - $line"
        done
    ((PASSED++))
else
    echo "⚠️  nvidia-smi 未找到（可能没有 NVIDIA GPU）"
fi

# CUDA
if command -v nvcc &> /dev/null; then
    CUDA_VER=$(nvcc --version | grep release | awk '{print $5}' | tr -d ',')
    echo "✅ CUDA 版本: $CUDA_VER"
    ((PASSED++))
else
    echo "❌ nvcc 未找到，CUDA 可能未安装"
    ((FAILED++))
fi

# Python
if command -v python3 &> /dev/null; then
    PY_VER=$(python3 --version | awk '{print $2}')
    echo "✅ Python 版本: $PY_VER"
    ((PASSED++))
else
    echo "❌ Python 3 未找到"
    ((FAILED++))
fi

echo ""

# 6. 快速功能测试
echo "⚡ 快速功能测试"
echo "─────────────────────────────────────────────────────────"

if [ -f "build/isat_geo" ]; then
    # 测试帮助输出
    if build/isat_geo --help &> /dev/null; then
        echo "✅ isat_geo 帮助可用"
        ((PASSED++))
    fi
    
    # 测试诊断工具
    if python3 scripts/gpu_diagnostic.py --gpu-info &> /dev/null; then
        echo "✅ gpu_diagnostic.py 可运行"
        ((PASSED++))
    fi
fi

echo ""

# 总结
echo "╔═══════════════════════════════════════════════════════╗"
TOTAL=$((PASSED + FAILED))
PERCENT=$((PASSED * 100 / TOTAL))

if [ $FAILED -eq 0 ]; then
    echo "║ ✅ 检查完成 - 全部通过！"
else
    echo "║ ⚠️  检查完成 - 有 $FAILED 项失败"
fi

echo "║"
echo "║ 通过: $PASSED / $TOTAL ($PERCENT%)"
echo "╚═══════════════════════════════════════════════════════╝"

echo ""

# 后续步骤
if [ $FAILED -eq 0 ]; then
    echo "🚀 下一步："
    echo ""
    echo "1. 准备测试数据（.iat 项目文件或图像目录）"
    echo ""
    echo "2. 运行自动化验证："
    echo "   bash scripts/verify_gpu_optimization.sh <dataset.iat> ./results"
    echo ""
    echo "3. 查看诊断报告："
    echo "   python3 scripts/gpu_diagnostic.py --log results/isat_geo_latest.log"
    echo ""
    echo "4. 查看详细文档："
    echo "   cat GPU_OPTIMIZATION_QUICK_START.md"
else
    echo "❌ 需要修复失败项，然后重新运行此脚本"
fi

exit $FAILED
