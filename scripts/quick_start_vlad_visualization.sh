#!/bin/bash
# VLAD检索可视化 - 快速开始示例
# 此脚本演示了从特征提取到可视化的完整流程

set -e  # 遇到错误立即退出

# ============================================================================
# 命令行参数解析
# ============================================================================

function print_usage() {
    cat << EOF
用法: $0 -i IMAGE_LIST [-w WORK_DIR] [选项]

必需参数:
  -i IMAGE_LIST     图像列表JSON文件路径

可选参数:
  -w WORK_DIR       工作目录（默认: IMAGE_LIST所在目录/vlad_visualization_test）
  -k NUM_CLUSTERS   VLAD聚类数量（默认: 128）
  -p PCA_DIM        PCA降维维度（默认: 512）
  -t TOP_K          检索top-k个最相似图像（默认: 10）
  -m MAX_QUERIES    最大查询图像数量（默认: 20）
  -n NUM_FEATURES   每张图像提取的特征数量（默认: 1500）
  -r RESIZE_RES     降采样分辨率（默认: 1024）
  -h                显示此帮助信息

示例:
  # 使用默认工作目录
  $0 -i /path/to/images.json

  # 指定工作目录
  $0 -i /path/to/images.json -w /path/to/output

  # 自定义参数
  $0 -i /path/to/images.json -k 256 -p 1024 -t 15
EOF
}

# 默认参数
IMAGE_LIST=""
WORK_DIR=""
NUM_CLUSTERS=128
PCA_DIM=512
TOP_K=10
MAX_QUERIES=20
NUM_FEATURES=1500
RESIZE_RES=1024

# 解析命令行参数
while getopts "i:w:k:p:t:m:n:r:h" opt; do
    case $opt in
        i) IMAGE_LIST="$OPTARG" ;;
        w) WORK_DIR="$OPTARG" ;;
        k) NUM_CLUSTERS="$OPTARG" ;;
        p) PCA_DIM="$OPTARG" ;;
        t) TOP_K="$OPTARG" ;;
        m) MAX_QUERIES="$OPTARG" ;;
        n) NUM_FEATURES="$OPTARG" ;;
        r) RESIZE_RES="$OPTARG" ;;
        h) print_usage; exit 0 ;;
        \?) echo "无效选项: -$OPTARG" >&2; print_usage; exit 1 ;;
        :) echo "选项 -$OPTARG 需要参数" >&2; print_usage; exit 1 ;;
    esac
done

# 检查必需参数
if [ -z "$IMAGE_LIST" ]; then
    echo "错误: 必须指定图像列表文件 (-i IMAGE_LIST)"
    echo ""
    print_usage
    exit 1
fi

# 如果WORK_DIR未指定，在IMAGE_LIST所在目录创建vlad_visualization_test
if [ -z "$WORK_DIR" ]; then
    IMAGE_LIST_DIR=$(dirname "$(readlink -f "$IMAGE_LIST")")
    WORK_DIR="$IMAGE_LIST_DIR/vlad_visualization_test"
    echo "未指定工作目录，使用默认: $WORK_DIR"
fi

# ============================================================================
# 配置参数
# ============================================================================

# 输出文件路径
FEATURES_DIR="$WORK_DIR/features_retrieval"
CODEBOOK_FILE="$WORK_DIR/codebook_${NUM_CLUSTERS}.vlad"
VLAD_CACHE_DIR="$WORK_DIR/vlad_cache"
PAIRS_FILE="$WORK_DIR/pairs.json"
HTML_REPORT="$WORK_DIR/retrieval_report.html"

# 可执行文件路径
EXTRACT_TOOL="./build/isat_extract"
TRAIN_VLAD_TOOL="./build/isat_train_vlad"
RETRIEVE_TOOL="./build/isat_retrieve"
VISUALIZE_SCRIPT="./scripts/visualize_vlad_retrieval.py"

# ============================================================================
# 预检查
# ============================================================================

echo "=========================================="
echo "VLAD检索可视化 - 快速开始"
echo "=========================================="
echo ""
echo "📋 配置参数:"
echo "  - 图像列表: $IMAGE_LIST"
echo "  - 工作目录: $WORK_DIR"
echo "  - VLAD聚类数: $NUM_CLUSTERS"
echo "  - PCA降维: $PCA_DIM"
echo "  - Top-K: $TOP_K"
echo "  - 最大查询数: $MAX_QUERIES"
echo "  - 特征数量: $NUM_FEATURES"
echo "  - 降采样分辨率: ${RESIZE_RES}px"
echo ""

# 检查必要的工具是否存在
echo "检查必要工具..."

if [ ! -f "$EXTRACT_TOOL" ]; then
    echo "错误: 未找到 $EXTRACT_TOOL"
    echo "请先编译项目: cd build && make -j10"
    exit 1
fi

if [ ! -f "$TRAIN_VLAD_TOOL" ]; then
    echo "错误: 未找到 $TRAIN_VLAD_TOOL"
    exit 1
fi

if [ ! -f "$RETRIEVE_TOOL" ]; then
    echo "错误: 未找到 $RETRIEVE_TOOL"
    exit 1
fi

if [ ! -f "$VISUALIZE_SCRIPT" ]; then
    echo "错误: 未找到 $VISUALIZE_SCRIPT"
    exit 1
fi

if [ ! -f "$IMAGE_LIST" ]; then
    echo "警告: 未找到示例图像列表 $IMAGE_LIST"
    echo "请修改脚本中的 IMAGE_LIST 和 IMAGE_DIR 变量"
    echo ""
    echo "图像列表JSON格式示例:"
    echo '{'
    echo '  "images": ['
    echo '    {"id": 1, "path": "/path/to/image1.jpg"},'
    echo '    {"id": 2, "path": "/path/to/image2.jpg"}'
    echo '  ]'
    echo '}'
    exit 1
fi

echo "✅ 所有工具检查通过"
echo ""

# 创建工作目录
mkdir -p "$WORK_DIR"
mkdir -p "$FEATURES_DIR"
mkdir -p "$VLAD_CACHE_DIR"

# ============================================================================
# Step 1: 提取SIFT特征（检索流，降采样到1024）
# ============================================================================

echo "=========================================="
echo "Step 1: 提取SIFT特征（检索流）"
echo "=========================================="
echo "  - 降采样分辨率: ${RESIZE_RES}px"
echo "  - 特征数量: $NUM_FEATURES"
echo "  - 输出目录: $FEATURES_DIR"
echo ""

if [ -d "$FEATURES_DIR" ] && [ "$(ls -A $FEATURES_DIR/*.isat_feat 2>/dev/null)" ]; then
    echo "⚠️  特征文件已存在，跳过提取步骤"
    echo "   如需重新提取，请删除: $FEATURES_DIR"
else
    $EXTRACT_TOOL \
        -i "$IMAGE_LIST" \
        -o "$FEATURES_DIR" \
        --output-retrieval "$FEATURES_DIR" \
        --nfeatures-retrieval $NUM_FEATURES \
        --resize-retrieval $RESIZE_RES \
        --only-retrieval
    
    echo "✅ 特征提取完成"
fi

echo ""

# ============================================================================
# Step 2: 训练VLAD codebook
# ============================================================================

echo "=========================================="
echo "Step 2: 训练VLAD Codebook"
echo "=========================================="
echo "  - 聚类数量: $NUM_CLUSTERS"
echo "  - PCA降维: $PCA_DIM"
echo "  - 输出文件: $CODEBOOK_FILE"
echo ""

if [ -f "$CODEBOOK_FILE" ]; then
    echo "⚠️  Codebook文件已存在，跳过训练步骤"
    echo "   如需重新训练，请删除: $CODEBOOK_FILE"
else
    $TRAIN_VLAD_TOOL \
        -f "$FEATURES_DIR" \
        -o "$CODEBOOK_FILE" \
        -k $NUM_CLUSTERS \
        --pca-dims $PCA_DIM \
        --scale-weighted
    
    echo "✅ Codebook训练完成"
fi

echo ""

# ============================================================================
# Step 3: 运行检索，生成VLAD缓存
# ============================================================================

echo "=========================================="
echo "Step 3: 运行VLAD检索"
echo "=========================================="
echo "  - 检索策略: VLAD"
echo "  - Top-K: $TOP_K"
echo "  - VLAD缓存目录: $VLAD_CACHE_DIR"
echo ""

$RETRIEVE_TOOL \
    -f "$FEATURES_DIR" \
    -i "$IMAGE_LIST" \
    -o "$PAIRS_FILE" \
    --strategy vlad \
    --vlad-codebook "$CODEBOOK_FILE" \
    --vlad-cache "$VLAD_CACHE_DIR" \
    --vlad-top-k $TOP_K

echo "✅ 检索完成"
echo ""

# ============================================================================
# Step 4: 生成可视化报告
# ============================================================================

echo "=========================================="
echo "Step 4: 生成可视化报告"
echo "=========================================="
echo "  - 最大查询数: $MAX_QUERIES"
echo "  - Top-K显示: $TOP_K"
echo "  - 输出文件: $HTML_REPORT"
echo ""

python3 "$VISUALIZE_SCRIPT" \
    --vlad-dir "$VLAD_CACHE_DIR" \
    --images "$IMAGE_LIST" \
    --output "$HTML_REPORT" \
    --top-k $TOP_K \
    --max-queries $MAX_QUERIES

echo ""
echo "=========================================="
echo "✅ 全部完成!"
echo "=========================================="
echo ""
echo "📊 工作目录: $WORK_DIR"
echo "📝 HTML报告: $HTML_REPORT"
echo ""
echo "在浏览器中打开查看结果:"
echo "  firefox $HTML_REPORT"
echo "  # 或"
echo "  google-chrome $HTML_REPORT"
echo ""
echo "=========================================="
