"""图：退化模型零空间求解中 Jacobi 特征分解与 Cholesky 逆迭代的耗时对比。

数据来源：几何模块设计文档第 4.4 节的实测对照表（12 行完整数据，
GTX 1060 6GB，N=2048 次采样，workgroup=32，50 次均值）。
同一数据集、同一采样一致性框架，仅零空间求解算法不同。
该对照仅存在于图形计算着色器路径；统一计算架构路径自始只有逆迭代实现。
真实实测数据，非构造数据。
"""
import matplotlib.pyplot as plt
import numpy as np

from common import PALETTE, save

# n, model, jacobi_ms, ipi_ms, speedup —— 完整 12 行，未作抽样
rows = [
    (100, "H", 46.2, 0.61, 75.7),
    (100, "F", 46.2, 0.73, 63.3),
    (100, "E", 46.3, 0.75, 61.7),
    (300, "H", 46.4, 0.74, 62.7),
    (300, "F", 46.3, 0.84, 55.1),
    (300, "E", 46.4, 0.86, 53.9),
    (500, "H", 46.6, 0.92, 50.7),
    (500, "F", 46.6, 1.01, 46.1),
    (500, "E", 46.5, 0.99, 47.0),
    (1000, "H", 46.6, 1.26, 37.0),
    (1000, "F", 46.7, 1.37, 34.1),
    (1000, "E", 46.9, 1.36, 34.5),
]
labels = [f"{r[0]}\n{r[1]}" for r in rows]
jacobi = [r[2] for r in rows]
ipi = [r[3] for r in rows]
speedup = [r[4] for r in rows]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.4, 4.6))

x = np.arange(len(rows))
w = 0.38
ax1.bar(x - w / 2, jacobi, width=w, color=PALETTE["muted"], label="对称特征分解（Jacobi 旋转）")
ax1.bar(x + w / 2, ipi, width=w, color=PALETTE["primary"], label="正则化 Cholesky 逆迭代")
ax1.set_yscale("log")
ax1.set_xticks(x)
ax1.set_xticklabels(labels, fontsize=8.4)
ax1.set_xlabel("匹配点数 n / 几何模型", fontsize=9.5)
ax1.set_ylabel("单次求解耗时（毫秒，对数坐标）")
ax1.set_title("(a) 求解耗时对比")
ax1.legend(fontsize=8.6, loc="center right")
ax1.grid(axis="y", linestyle="--", alpha=0.4)

bars = ax2.bar(x, speedup, color=PALETTE["accent"])
ax2.set_xticks(x)
ax2.set_xticklabels(labels, fontsize=8.4)
ax2.set_xlabel("匹配点数 n / 几何模型", fontsize=9.5)
ax2.set_ylabel("加速比（倍）")
ax2.set_ylim(0, 88)
ax2.set_title("(b) 逆迭代相对特征分解的加速比")
ax2.grid(axis="y", linestyle="--", alpha=0.4)
for b, v in zip(bars, speedup):
    ax2.text(b.get_x() + b.get_width() / 2, v + 1.5, f"{v:.1f}", ha="center", fontsize=8.0)
ax2.text(0.98, 0.955, "全部 12 组：34.1×–75.7×", transform=ax2.transAxes,
         ha="right", va="top", fontsize=9, color=PALETTE["danger"])

fig.suptitle(
    "零空间求解的数值方法替换效果（图形计算着色器路径）\n"
    "GTX 1060 6GB，N=2048 次采样，workgroup=32，50 次均值",
    fontsize=11,
)
fig.tight_layout(rect=[0, 0, 1, 0.88])

save(fig, "../figures/fig3_geo_solver_speedup.svg")
save(fig, "../figures/fig3_geo_solver_speedup.png")
print("done")
