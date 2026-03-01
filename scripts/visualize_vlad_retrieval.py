#!/usr/bin/env python3
"""
VLAD检索结果可视化工具

对每个reference image检索top-k个最相似的图像，生成HTML可视化报告。
用于人工检查VLAD检索质量。

用法:
    python visualize_vlad_retrieval.py \
        --vlad-dir ./vlad_cache \
        --images ./images.json \
        --output ./retrieval_report.html \
        --top-k 10

VLAD文件格式 (.isat_vlad):
    - Magic: 0x44414C56 ("VLAD")
    - Version: uint32
    - Size: uint32 (VLAD向量维度)
    - Data: float32[Size]
"""

import argparse
import json
import struct
import numpy as np
from pathlib import Path
from typing import List, Tuple, Dict, Optional
import html
from dataclasses import dataclass


@dataclass
class ImageInfo:
    """图像信息"""
    image_id: str
    image_path: str
    vlad_file: str = ""
    vlad_vector: Optional[np.ndarray] = None


@dataclass
class RetrievalResult:
    """检索结果"""
    query_image: ImageInfo
    retrieved_images: List[Tuple[ImageInfo, float]]  # (image, distance)


def load_vlad_cache(vlad_path: str) -> Optional[np.ndarray]:
    """
    加载VLAD缓存文件
    
    Args:
        vlad_path: .isat_vlad文件路径
    
    Returns:
        VLAD向量 (numpy array) 或 None
    """
    try:
        with open(vlad_path, 'rb') as f:
            # Read header
            magic = struct.unpack('I', f.read(4))[0]
            version = struct.unpack('I', f.read(4))[0]
            vlad_size = struct.unpack('I', f.read(4))[0]
            
            # Validate magic number (ASCII "VLAD" = 0x44414C56)
            if magic != 0x44414C56:
                print(f"Warning: Invalid VLAD file (wrong magic): {vlad_path}")
                return None
            
            # Read VLAD vector
            vlad_data = f.read(vlad_size * 4)  # float32 = 4 bytes
            vlad = np.frombuffer(vlad_data, dtype=np.float32)
            
            if len(vlad) != vlad_size:
                print(f"Warning: VLAD size mismatch in {vlad_path}")
                return None
            
            return vlad
    except Exception as e:
        print(f"Error loading VLAD file {vlad_path}: {e}")
        return None


def compute_l2_distance(v1: np.ndarray, v2: np.ndarray) -> float:
    """计算L2距离"""
    return float(np.linalg.norm(v1 - v2))


def load_images_from_json(json_path: str, vlad_dir: str) -> List[ImageInfo]:
    """
    从JSON文件加载图像列表
    
    Args:
        json_path: 图像列表JSON文件
        vlad_dir: VLAD缓存目录
    
    Returns:
        图像信息列表
    """
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    images = []
    vlad_dir_path = Path(vlad_dir)
    
    for img in data.get('images', []):
        image_id = str(img.get('id', ''))
        image_path = img.get('path', '')
        
        if not image_id or not image_path:
            continue
        
        # 构造VLAD文件路径
        vlad_file = vlad_dir_path / f"{image_id}.isat_vlad"
        
        info = ImageInfo(
            image_id=image_id,
            image_path=image_path,
            vlad_file=str(vlad_file) if vlad_file.exists() else ""
        )
        
        images.append(info)
    
    print(f"Loaded {len(images)} images from {json_path}")
    return images


def load_vlad_vectors(images: List[ImageInfo]) -> List[ImageInfo]:
    """
    为所有图像加载VLAD向量
    
    Args:
        images: 图像信息列表
    
    Returns:
        加载了VLAD向量的图像列表
    """
    valid_images = []
    
    for img in images:
        if not img.vlad_file or not Path(img.vlad_file).exists():
            print(f"Warning: VLAD file not found for {img.image_id}: {img.vlad_file}")
            continue
        
        vlad = load_vlad_cache(img.vlad_file)
        if vlad is None:
            continue
        
        img.vlad_vector = vlad
        valid_images.append(img)
    
    print(f"Successfully loaded VLAD vectors for {len(valid_images)}/{len(images)} images")
    return valid_images


def retrieve_top_k(
    query_image: ImageInfo,
    database_images: List[ImageInfo],
    top_k: int
) -> List[Tuple[ImageInfo, float]]:
    """
    检索top-k个最相似的图像
    
    Args:
        query_image: 查询图像
        database_images: 数据库图像列表（包含查询图像本身）
        top_k: 返回的相似图像数量
    
    Returns:
        [(image, distance), ...] 按距离排序
    """
    if query_image.vlad_vector is None:
        return []
    
    distances = []
    
    for img in database_images:
        # 跳过查询图像本身
        if img.image_id == query_image.image_id:
            continue
        
        if img.vlad_vector is None:
            continue
        
        dist = compute_l2_distance(query_image.vlad_vector, img.vlad_vector)
        distances.append((img, dist))
    
    # 按距离排序，取top-k
    distances.sort(key=lambda x: x[1])
    return distances[:top_k]


def generate_html_report(
    results: List[RetrievalResult],
    output_path: str,
    top_k: int
):
    """
    生成HTML可视化报告
    
    Args:
        results: 检索结果列表
        output_path: 输出HTML文件路径
        top_k: 每个查询的检索结果数量
    """
    html_parts = []
    
    # HTML header
    html_parts.append("""
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VLAD检索结果可视化</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        h1 {
            color: #333;
            text-align: center;
            margin-bottom: 30px;
        }
        .summary {
            background: white;
            padding: 20px;
            border-radius: 8px;
            margin-bottom: 30px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .summary h2 {
            margin-top: 0;
            color: #2c3e50;
        }
        .retrieval-result {
            background: white;
            margin-bottom: 30px;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .query-section {
            border-bottom: 3px solid #3498db;
            padding-bottom: 15px;
            margin-bottom: 20px;
        }
        .query-section h2 {
            margin: 0 0 10px 0;
            color: #2c3e50;
        }
        .image-container {
            display: flex;
            flex-wrap: wrap;
            gap: 15px;
        }
        .query-image-box {
            flex: 0 0 auto;
            border: 3px solid #3498db;
            border-radius: 8px;
            padding: 10px;
            background: #ecf0f1;
        }
        .result-image-box {
            flex: 0 0 auto;
            border: 2px solid #95a5a6;
            border-radius: 8px;
            padding: 10px;
            background: white;
            transition: transform 0.2s, box-shadow 0.2s;
        }
        .result-image-box:hover {
            transform: translateY(-5px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        }
        .good-match {
            border-color: #27ae60;
        }
        .medium-match {
            border-color: #f39c12;
        }
        .poor-match {
            border-color: #e74c3c;
        }
        .image-box img {
            display: block;
            max-width: 280px;
            max-height: 280px;
            width: auto;
            height: auto;
            border-radius: 4px;
        }
        .query-image-box img {
            max-width: 350px;
            max-height: 350px;
        }
        .image-info {
            margin-top: 10px;
            font-size: 12px;
            color: #555;
        }
        .image-id {
            font-weight: bold;
            color: #2c3e50;
            margin-bottom: 5px;
        }
        .distance {
            color: #7f8c8d;
        }
        .distance-value {
            font-weight: bold;
        }
        .rank {
            display: inline-block;
            background: #3498db;
            color: white;
            padding: 2px 8px;
            border-radius: 4px;
            font-size: 11px;
            font-weight: bold;
        }
        .results-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }
        .legend {
            margin-top: 20px;
            padding: 15px;
            background: #ecf0f1;
            border-radius: 8px;
        }
        .legend h3 {
            margin-top: 0;
            color: #2c3e50;
        }
        .legend-item {
            display: inline-block;
            margin-right: 20px;
            padding: 5px 10px;
            border-radius: 4px;
            border: 2px solid;
        }
    </style>
</head>
<body>
    <h1>🔍 VLAD检索结果可视化</h1>
""")
    
    # Summary section
    html_parts.append(f"""
    <div class="summary">
        <h2>📊 检索统计</h2>
        <p><strong>查询图像数量:</strong> {len(results)}</p>
        <p><strong>每个查询的检索结果:</strong> Top-{top_k}</p>
        <p><strong>总计检索对数:</strong> {sum(len(r.retrieved_images) for r in results)}</p>
    </div>
    
    <div class="legend">
        <h3>距离颜色编码</h3>
        <span class="legend-item good-match">距离 &lt; 0.5 (优秀匹配)</span>
        <span class="legend-item medium-match">0.5 ≤ 距离 &lt; 1.0 (中等匹配)</span>
        <span class="legend-item poor-match">距离 ≥ 1.0 (较差匹配)</span>
    </div>
""")
    
    # Individual retrieval results
    for idx, result in enumerate(results, 1):
        query = result.query_image
        
        html_parts.append(f"""
    <div class="retrieval-result">
        <div class="query-section">
            <h2>查询 #{idx}: {html.escape(query.image_id)}</h2>
        </div>
        
        <div class="image-container">
            <!-- Query Image -->
            <div class="query-image-box image-box">
                <img src="file://{html.escape(query.image_path)}" 
                     alt="{html.escape(query.image_id)}"
                     onerror="this.src='data:image/svg+xml,%3Csvg xmlns=%22http://www.w3.org/2000/svg%22 width=%22280%22 height=%22280%22%3E%3Crect fill=%22%23ddd%22 width=%22280%22 height=%22280%22/%3E%3Ctext x=%2250%25%22 y=%2250%25%22 text-anchor=%22middle%22 dy=%22.3em%22 fill=%22%23999%22%3E图像未找到%3C/text%3E%3C/svg%3E'">
                <div class="image-info">
                    <div class="image-id">🎯 查询图像</div>
                    <div>ID: {html.escape(query.image_id)}</div>
                    <div style="font-size: 10px; color: #999; margin-top: 5px;">
                        {html.escape(query.image_path)}
                    </div>
                </div>
            </div>
        </div>
        
        <div class="results-grid">
""")
        
        # Retrieved images
        for rank, (retrieved_img, distance) in enumerate(result.retrieved_images, 1):
            # Determine match quality based on distance
            if distance < 0.5:
                quality_class = "good-match"
            elif distance < 1.0:
                quality_class = "medium-match"
            else:
                quality_class = "poor-match"
            
            html_parts.append(f"""
            <div class="result-image-box image-box {quality_class}">
                <img src="file://{html.escape(retrieved_img.image_path)}" 
                     alt="{html.escape(retrieved_img.image_id)}"
                     onerror="this.src='data:image/svg+xml,%3Csvg xmlns=%22http://www.w3.org/2000/svg%22 width=%22280%22 height=%22280%22%3E%3Crect fill=%22%23ddd%22 width=%22280%22 height=%22280%22/%3E%3Ctext x=%2250%25%22 y=%2250%25%22 text-anchor=%22middle%22 dy=%22.3em%22 fill=%22%23999%22%3E图像未找到%3C/text%3E%3C/svg%3E'">
                <div class="image-info">
                    <span class="rank">#{rank}</span>
                    <div class="image-id">{html.escape(retrieved_img.image_id)}</div>
                    <div class="distance">距离: <span class="distance-value">{distance:.4f}</span></div>
                    <div style="font-size: 10px; color: #999; margin-top: 5px;">
                        {html.escape(Path(retrieved_img.image_path).name)}
                    </div>
                </div>
            </div>
""")
        
        html_parts.append("""
        </div>
    </div>
""")
    
    # HTML footer
    html_parts.append("""
</body>
</html>
""")
    
    # Write to file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(''.join(html_parts))
    
    print(f"✅ HTML report generated: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="VLAD检索结果可视化工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本用法
  %(prog)s --vlad-dir ./vlad_cache --images ./images.json --output report.html
  
  # 只显示每个查询的top-5结果
  %(prog)s --vlad-dir ./vlad_cache --images ./images.json --output report.html --top-k 5
  
  # 只处理前10个查询图像（快速预览）
  %(prog)s --vlad-dir ./vlad_cache --images ./images.json --output report.html --max-queries 10
"""
    )
    
    parser.add_argument(
        '--vlad-dir',
        required=True,
        help='VLAD缓存文件目录 (包含 *.isat_vlad 文件)'
    )
    parser.add_argument(
        '--images',
        required=True,
        help='图像列表JSON文件'
    )
    parser.add_argument(
        '--output',
        required=True,
        help='输出HTML文件路径'
    )
    parser.add_argument(
        '--top-k',
        type=int,
        default=10,
        help='每个查询返回的检索结果数量 (默认: 10)'
    )
    parser.add_argument(
        '--max-queries',
        type=int,
        default=None,
        help='处理的最大查询图像数量 (用于快速预览，默认: 全部)'
    )
    
    args = parser.parse_args()
    
    # Validate inputs
    if not Path(args.vlad_dir).exists():
        print(f"Error: VLAD directory not found: {args.vlad_dir}")
        return 1
    
    if not Path(args.images).exists():
        print(f"Error: Image list file not found: {args.images}")
        return 1
    
    print("=" * 60)
    print("VLAD检索可视化工具")
    print("=" * 60)
    
    # Load images
    print(f"\n📂 Loading images from {args.images}...")
    images = load_images_from_json(args.images, args.vlad_dir)
    
    if not images:
        print("Error: No images loaded")
        return 1
    
    # Load VLAD vectors
    print(f"\n📊 Loading VLAD vectors from {args.vlad_dir}...")
    images = load_vlad_vectors(images)
    
    if not images:
        print("Error: No valid VLAD vectors loaded")
        return 1
    
    # Limit number of queries if specified
    query_images = images
    if args.max_queries is not None and args.max_queries > 0:
        query_images = images[:args.max_queries]
        print(f"\n⚠️  Limiting to first {len(query_images)} query images")
    
    # Perform retrieval for each query image
    print(f"\n🔍 Performing retrieval (top-{args.top_k}) for {len(query_images)} query images...")
    results = []
    
    for i, query_img in enumerate(query_images, 1):
        if i % 10 == 0:
            print(f"  Processing query {i}/{len(query_images)}...")
        
        retrieved = retrieve_top_k(query_img, images, args.top_k)
        results.append(RetrievalResult(
            query_image=query_img,
            retrieved_images=retrieved
        ))
    
    # Generate HTML report
    print(f"\n📝 Generating HTML report...")
    generate_html_report(results, args.output, args.top_k)
    
    # Print summary
    print("\n" + "=" * 60)
    print("✅ 完成!")
    print("=" * 60)
    print(f"查询图像数: {len(results)}")
    print(f"Top-K: {args.top_k}")
    print(f"输出文件: {args.output}")
    print(f"\n在浏览器中打开查看结果:")
    print(f"  file://{Path(args.output).absolute()}")
    print("=" * 60)
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
