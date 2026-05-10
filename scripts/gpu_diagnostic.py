#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPU 显存优化诊断分析工具

用法：
    python3 gpu_diagnostic.py --device 0 --log isat_geo_latest.log --gpu-stats gpu_stats_latest.log
    
功能：
    1. 编译参数验证
    2. GPU 显存占用分析
    3. 吞吐量计算
    4. 性能建议
"""

import argparse
import json
import re
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Dict, Tuple
import subprocess

@dataclass
class GPUStats:
    timestamp: str
    utilization: int  # %
    memory_used: int  # MB
    memory_total: int  # MB
    
@dataclass
class DiagnosticResult:
    pairs_processed: int
    elapsed_time_s: float
    throughput_pairs_per_s: float
    max_gpu_util: int
    max_vram_mb: int
    avg_gpu_util: int
    batch_efficiency: float  # (expected / actual) throughput ratio

class GPUDiagnostic:
    def __init__(self, device: int = 0):
        self.device = device
        self.gpu_info = self._get_gpu_info()
        
    def _get_gpu_info(self) -> Dict:
        """获取 GPU 信息"""
        try:
            result = subprocess.run(
                ["nvidia-smi", "--query-gpu=index,name,memory.total", 
                 "--format=csv,noheader", "-i", str(self.device)],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                parts = result.stdout.strip().split(",")
                return {
                    "name": parts[1].strip(),
                    "total_memory_mb": int(parts[2].strip().split()[0]),
                }
        except Exception as e:
            print(f"⚠️  无法获取 GPU 信息: {e}", file=sys.stderr)
        
        return {"name": "Unknown", "total_memory_mb": 8192}
    
    def parse_isat_log(self, log_file: str) -> Optional[Tuple[int, float]]:
        """解析 isat_geo 日志提取处理对数和耗时"""
        try:
            with open(log_file, 'r') as f:
                content = f.read()
            
            # 查找'pairs processed'或相似输出
            pairs_match = re.search(r'Processed (\d+) .*pairs', content, re.IGNORECASE)
            if not pairs_match:
                pairs_match = re.search(r'(\d+).*pairs.*processed', content, re.IGNORECASE)
            
            if pairs_match:
                pairs = int(pairs_match.group(1))
                
                # 查找耗时
                time_match = re.search(r'Time[:\s]+(\d+\.?\d*)\s*(?:ms|second|s)', content, re.IGNORECASE)
                if time_match:
                    elapsed_s = float(time_match.group(1))
                    if 'ms' in content[time_match.start():time_match.end()]:
                        elapsed_s /= 1000
                    return pairs, elapsed_s
            
            # 备选：查找任务计数
            task_match = re.search(r'Total.*(?:tasks|batches)[:\s]+(\d+)', content, re.IGNORECASE)
            if task_match:
                tasks = int(task_match.group(1))
                return tasks * 200, None  # 假设每个 task 200 对
                
        except Exception as e:
            print(f"❌ 解析日志失败 {log_file}: {e}", file=sys.stderr)
        
        return None
    
    def parse_gpu_stats(self, stats_file: str) -> List[GPUStats]:
        """解析 GPU 统计日志"""
        stats = []
        try:
            with open(stats_file, 'r') as f:
                for line in f:
                    # 格式: [HH:MM:SS] GPU_UTIL[%],MEM_USED[MB],MEM_TOTAL[MB]
                    match = re.search(
                        r'\[([^\]]+)\]\s+(\d+),(\d+),(\d+)',
                        line
                    )
                    if match:
                        stats.append(GPUStats(
                            timestamp=match.group(1),
                            utilization=int(match.group(2)),
                            memory_used=int(match.group(3)),
                            memory_total=int(match.group(4)),
                        ))
        except Exception as e:
            print(f"⚠️  解析 GPU 统计失败 {stats_file}: {e}", file=sys.stderr)
        
        return stats
    
    def diagnose(self, log_file: Optional[str] = None, 
                 stats_file: Optional[str] = None) -> DiagnosticResult:
        """运行完整诊断"""
        
        pairs_result = None
        if log_file and Path(log_file).exists():
            pairs_result = self.parse_isat_log(log_file)
        
        gpu_stats = []
        if stats_file and Path(stats_file).exists():
            gpu_stats = self.parse_gpu_stats(stats_file)
        
        # 计算指标
        pairs = pairs_result[0] if pairs_result else 0
        elapsed = pairs_result[1] if pairs_result and len(pairs_result) > 1 else None
        
        max_util = max([s.utilization for s in gpu_stats], default=0)
        avg_util = sum([s.utilization for s in gpu_stats]) // len(gpu_stats) if gpu_stats else 0
        max_mem = max([s.memory_used for s in gpu_stats], default=0)
        
        throughput = (pairs / elapsed) if elapsed and elapsed > 0 else 0
        
        # 计算批效率 (理想 vs 实际)
        batch_efficiency = 1.0
        if max_util > 0:
            batch_efficiency = min(max_util / 95, 1.0)  # 95% 是目标利用率
        
        return DiagnosticResult(
            pairs_processed=pairs,
            elapsed_time_s=elapsed or 0,
            throughput_pairs_per_s=throughput,
            max_gpu_util=max_util,
            max_vram_mb=max_mem,
            avg_gpu_util=avg_util,
            batch_efficiency=batch_efficiency,
        )
    
    def generate_report(self, result: DiagnosticResult) -> str:
        """生成诊断报告"""
        report = []
        report.append("╔═══════════════════════════════════════════════════════════╗")
        report.append("║     GPU 显存优化诊断报告                                  ║")
        report.append("╚═══════════════════════════════════════════════════════════╝")
        report.append("")
        
        # GPU 信息
        report.append("📱 GPU 信息")
        report.append(f"  名称: {self.gpu_info.get('name', 'Unknown')}")
        report.append(f"  总显存: {self.gpu_info.get('total_memory_mb', 0) // 1024} GB")
        report.append("")
        
        # 性能指标
        report.append("⚡ 性能指标")
        report.append(f"  处理对数: {result.pairs_processed:,}")
        report.append(f"  耗时: {result.elapsed_time_s:.3f}s")
        report.append(f"  吞吐量: {result.throughput_pairs_per_s:.0f} pairs/sec")
        report.append("")
        
        # GPU 占用
        report.append("📊 GPU 占用")
        report.append(f"  最大利用率: {result.max_gpu_util}%")
        report.append(f"  平均利用率: {result.avg_gpu_util}%")
        report.append(f"  最大显存: {result.max_vram_mb}MB ({result.max_vram_mb // 1024}GB)")
        report.append("")
        
        # 性能评估
        report.append("📈 性能评估")
        
        # 利用率评级
        util_score = "✅ 优秀" if result.max_gpu_util >= 90 else \
                     "⚠️  中等" if result.max_gpu_util >= 50 else \
                     "❌ 低效"
        report.append(f"  GPU 利用率: {util_score} ({result.max_gpu_util}%)")
        
        # 吞吐评级
        if result.throughput_pairs_per_s > 50000:
            throughput_score = "✅ 优秀"
        elif result.throughput_pairs_per_s > 10000:
            throughput_score = "⚠️  中等"
        else:
            throughput_score = "❌ 低效"
        report.append(f"  吞吐量: {throughput_score} ({result.throughput_pairs_per_s:.0f} pairs/s)")
        
        # 显存评级
        mem_ratio = result.max_vram_mb / self.gpu_info.get('total_memory_mb', 1)
        if mem_ratio > 0.8:
            mem_score = "✅ 充分利用"
        elif mem_ratio > 0.3:
            mem_score = "⚠️  部分利用"
        else:
            mem_score = "❌ 显存浪费"
        report.append(f"  显存利用: {mem_score} ({mem_ratio*100:.1f}%)")
        report.append("")
        
        # 建议
        report.append("💡 优化建议")
        suggestions = []
        
        if result.max_gpu_util < 50:
            suggestions.append("  → GPU 利用率低，考虑：")
            suggestions.append("    - 增加批处理大小")
            suggestions.append("    - 检查 CPU 是否为瓶颈")
            suggestions.append("    - 关闭其他 GPU 应用")
        elif result.max_gpu_util >= 90:
            suggestions.append("  → GPU 充分利用，表现优秀！")
        
        if result.max_vram_mb < self.gpu_info.get('total_memory_mb', 1) * 0.3:
            suggestions.append("  → 显存利用不足，可继续扩大批处理")
        
        if result.throughput_pairs_per_s < 10000:
            suggestions.append("  → 吞吐量偏低，检查是否有其他瓶颈")
        
        if suggestions:
            for s in suggestions:
                report.append(s)
        else:
            report.append("  → 配置最优，无需改动 ✨")
        
        report.append("")
        
        return "\n".join(report)

def main():
    parser = argparse.ArgumentParser(
        description="GPU 显存优化诊断工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  python3 gpu_diagnostic.py --log isat_geo_latest.log --gpu-stats gpu_stats_latest.log
  python3 gpu_diagnostic.py --device 0 --gpu-info  # 仅显示 GPU 信息
        """
    )
    
    parser.add_argument("--device", type=int, default=0, help="GPU 设备编号")
    parser.add_argument("--log", type=str, help="isat_geo 日志文件路径")
    parser.add_argument("--gpu-stats", type=str, help="GPU 统计日志文件路径")
    parser.add_argument("--gpu-info", action="store_true", help="仅显示 GPU 信息")
    parser.add_argument("--json", action="store_true", help="输出 JSON 格式")
    
    args = parser.parse_args()
    
    # 创建诊断工具
    diag = GPUDiagnostic(device=args.device)
    
    # 仅显示 GPU 信息
    if args.gpu_info:
        print(f"GPU {args.device}: {diag.gpu_info['name']}")
        print(f"Total Memory: {diag.gpu_info['total_memory_mb']} MB")
        return
    
    # 运行诊断
    result = diag.diagnose(log_file=args.log, stats_file=args.gpu_stats)
    
    if args.json:
        json_result = {
            "pairs_processed": result.pairs_processed,
            "elapsed_time_s": result.elapsed_time_s,
            "throughput_pairs_per_s": result.throughput_pairs_per_s,
            "max_gpu_utilization_percent": result.max_gpu_util,
            "avg_gpu_utilization_percent": result.avg_gpu_util,
            "max_vram_mb": result.max_vram_mb,
            "batch_efficiency": result.batch_efficiency,
        }
        print(json.dumps(json_result, indent=2))
    else:
        report = diag.generate_report(result)
        print(report)

if __name__ == "__main__":
    main()
