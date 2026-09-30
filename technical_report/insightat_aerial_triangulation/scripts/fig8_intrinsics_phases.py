"""图：内参自由度的渐进解锁相位。

相位阈值按「该相机自身已注册影像数」划分，取自默认流程实际取值：
3 / 10 / 50。参数硬边界与焦距软先验同为实际取值。
抽象方法示意图，不含任何源码标识。
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

from common import PALETTE, save

fig, (ax, ax2) = plt.subplots(
    2, 1, figsize=(11.0, 6.8), gridspec_kw={"height_ratios": [2.05, 1.0]}
)

# ── 上半：相位 × 参数 解锁矩阵 ────────────────────────────────────────
params = ["纵横焦距比", "主点 $c_x,c_y$", "$k_3$", "$p_1,p_2$", "$k_2$", "$k_1$", "焦距 $f_x$"]
phases = [
    "相位 0\n$n_c < 3$\n全部冻结",
    "相位 1\n$3 \\leq n_c < 10$\n放开 $f_x, k_1$",
    "相位 2\n$10 \\leq n_c < 50$\n增开 $k_2$",
    "相位 3\n$n_c \\geq 50$\n全部放开",
]
# free[phase][param]
free = [
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 1],
    [0, 0, 0, 0, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1],
]

ax.set_xlim(-0.55, 4.05)
ax.set_ylim(-0.75, len(params) + 0.3)
ax.axis("off")
ax.text(1.75, len(params) + 0.02,
        "内参自由度的渐进解锁（按该相机自身已注册影像数 $n_c$ 分相位）",
        ha="center", fontsize=12.2, fontweight="bold")

for j, p in enumerate(params):
    ax.text(-0.08, j + 0.5, p, ha="right", va="center", fontsize=9.6)

for i, ph in enumerate(phases):
    ax.text(i + 0.5, -0.38, ph, ha="center", va="center", fontsize=8.9,
            color="#111827")
    for j in range(len(params)):
        is_free = free[i][j]
        fc = PALETTE["accent"] if is_free else "#e5e7eb"
        ax.add_patch(Rectangle((i + 0.06, j + 0.1), 0.88, 0.8,
                               facecolor=fc, edgecolor="#9ca3af", linewidth=0.7))
        ax.text(i + 0.5, j + 0.5, "自由" if is_free else "固定",
                ha="center", va="center", fontsize=8.6,
                color="white" if is_free else "#6b7280")

for i in range(1, len(phases)):
    ax.plot([i, i], [0.05, len(params) - 0.02], color="#374151",
            linewidth=0.9, linestyle=":")

# ── 下半：约束与配套机制 ──────────────────────────────────────────────
ax2.set_xlim(0, 11.0)
ax2.set_ylim(0, 2.0)
ax2.axis("off")


def box(x, y, w, h, text, fc, fontsize=8.8):
    ax2.add_patch(FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.06,rounding_size=0.07",
        linewidth=1.0, edgecolor="#1f2937", facecolor=fc,
    ))
    ax2.text(x + w / 2, y + h / 2, text, ha="center", va="center",
             fontsize=fontsize, color="#111827")


box(0.15, 0.95, 3.4, 0.9,
    "参数硬边界\n纵横焦距比 $\\in[0.95,1.05]$\n"
    "$k_1,k_2,k_3$ 按观测量取紧 / 松两档\n$p_1,p_2\\in[-0.05,0.05]$",
    PALETTE["bg_box"])

box(3.8, 0.95, 3.4, 0.9,
    "焦距软先验\n残差 $\\sqrt{w}\\,(f_x-f_x^{0})/f_x^{0}$\n"
    "锚点 $f_x^{0}$ 取本轮平差入口值\n即逐轮重新锚定，非原始先验",
    PALETTE["bg_box"])

box(7.45, 0.95, 3.4, 0.9,
    "局部平差不动内参\n三种局部策略一律固定全部内参\n"
    "内参只在观测更充分的\n全局平差步骤中更新",
    PALETTE["bg_box"])

box(0.15, 0.06, 10.7, 0.72,
    "焦距与畸变、结构质量互为因果：内参不准则结构带系统性偏差，结构不足则内参无法估准。"
    "渐进解锁与逐轮重锚定共同约束了这条回路的推进速度，\n"
    "使早期迭代不会把全部内参自由度一次交给求解器；代价是早期迭代次数增加。",
    "#f3f4f6", fontsize=9.0)

fig.tight_layout()
save(fig, "../figures/fig8_intrinsics_phases.svg")
save(fig, "../figures/fig8_intrinsics_phases.png")
print("done")
