"""图 4：ETH3D 训练子集端到端墙钟耗时对比（COLMAP / v0.1 / v0.2）。

数据来源：docs/TECHNICAL_STATUS.md 第 9.1 节实测表，真实数据。
注意：COLMAP 列口径为特征+匹配+建图，不含格式转换；InsightAT 列为端到端
wall time；两者分段口径不完全一致，图中据此仅作量级参考，报告正文亦说明。
"""
import matplotlib.pyplot as plt
import numpy as np

from common import PALETTE, save

scenes = [
    "courtyard", "delivery_area", "electro", "facade", "kicker", "meadow",
    "office", "pipes", "playground", "relief", "relief_2", "terrace", "terrains",
]
colmap = [117.1, 129.1, 104.0, 331.7, 76.5, 24.8, 42.2, 24.0, 85.6, 89.9, 87.8, 49.5, 102.5]
v01 = [60.6, 72.7, 70.6, 249.6, 32.0, 7.9, 22.5, 10.6, 51.0, 55.2, 58.1, 25.4, 45.1]
v02 = [44.3, 57.0, 40.7, 129.7, 27.1, 9.8, 23.4, 8.3, 37.9, 35.6, 40.7, 21.3, 47.3]

x = np.arange(len(scenes))
w = 0.27

fig, ax = plt.subplots(figsize=(12, 5.2))
ax.bar(x - w, colmap, width=w, label="COLMAP（特征+匹配+建图）", color=PALETTE["muted"])
ax.bar(x, v01, width=w, label="InsightAT v0.1（端到端）", color=PALETTE["secondary"])
ax.bar(x + w, v02, width=w, label="InsightAT v0.2（端到端）", color=PALETTE["primary"])

ax.set_xticks(x)
ax.set_xticklabels(scenes, rotation=35, ha="right", fontsize=9.5)
ax.set_ylabel("耗时（秒）")
ax.set_title("ETH3D 训练子集端到端耗时对比（参考硬件：NVIDIA GTX 1060 6GB）")
ax.legend(fontsize=9.5)
ax.grid(axis="y", linestyle="--", alpha=0.4)

total_colmap, total_v01, total_v02 = sum(colmap), sum(v01), sum(v02)
ax.text(
    0.01, 0.97,
    f"13 场景总计：COLMAP {total_colmap:.1f}s ｜ v0.1 {total_v01:.1f}s ｜ v0.2 {total_v02:.1f}s（v0.2/v0.1 ≈ {total_v02/total_v01:.2f}）",
    transform=ax.transAxes, fontsize=9.5, va="top",
    bbox=dict(boxstyle="round,pad=0.3", fc="#f3f4f6", ec="#d1d5db"),
)

fig.tight_layout()
save(fig, "../figures/fig4_eth3d_benchmark.svg")
save(fig, "../figures/fig4_eth3d_benchmark.png")
print("done")
