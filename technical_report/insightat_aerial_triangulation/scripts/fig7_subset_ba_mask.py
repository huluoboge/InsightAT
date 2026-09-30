"""图：轨迹存储的可逆逻辑删除与子集光束法平差的协同关系。

图中标志位数量、剔除判据与恢复门限均取自默认流程实际取值：
轨迹四个状态位、观测两个状态位、网格非极大值抑制目标密度每幅
1000 点、评分权重 0.6/0.4 与度数封顶 8、焦距相对变化 2% 触发恢复、
恢复判据重投影 4 像素。抽象方法示意图，不含任何源码标识。
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from common import PALETTE, save

fig, ax = plt.subplots(figsize=(11.6, 7.6))
ax.set_xlim(0, 11.6)
ax.set_ylim(0, 7.6)
ax.axis("off")


def box(x, y, w, h, text, fc, fontsize=9.0):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.07,rounding_size=0.08",
        linewidth=1.1, edgecolor="#1f2937", facecolor=fc,
    ))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, color="#111827")


def arrow(x1, y1, x2, y2, color="#374151", ls="solid"):
    ax.add_patch(FancyArrowPatch(
        (x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=12,
        linewidth=1.25, color=color, linestyle=ls,
    ))


ax.text(5.8, 7.25, "可逆逻辑删除与子集光束法平差", ha="center",
        fontsize=13, fontweight="bold")

# ── 顶部：完整轨迹存储与状态位 ────────────────────────────────────────
box(1.5, 6.15, 8.6, 0.85,
    "完整轨迹存储：并查集一次建成，结构化数组布局，全程不物理搬移\n"
    "轨迹四状态位｜存活・待重三角化・已三角化・本轮排除平差　　观测两状态位｜存活・可恢复",
    PALETTE["bg_box"], fontsize=8.8)

arrow(5.8, 6.15, 5.8, 5.75)

# ── 中层左：子集选择 ──────────────────────────────────────────────────
box(0.3, 4.35, 5.2, 1.35,
    "子集选择（注册数 > 50 时逐轮重算）\n"
    "每幅影像按自适应网格做非极大值抑制\n"
    "网格边长 $\\lceil\\sqrt{WH/1000}\\,\\rceil$，每格留最高分轨迹\n"
    "$\\mathrm{score}=0.6\\min(1,\\deg/8)+0.4\\sin^{2}\\theta$",
    PALETTE["bg_box2"], fontsize=8.8)

# ── 中层右：未入选轨迹 ────────────────────────────────────────────────
box(6.1, 4.35, 5.2, 1.35,
    "未入选轨迹：置「本轮排除平差」位\n"
    "仅影响平差的点与观测集合，\n"
    "不影响三角化、外点剔除与重三角化\n"
    "叠加规则：二度轨迹剪枝、每轨迹观测上限 12",
    "#fee2e2", fontsize=8.8)

arrow(2.9, 4.35, 2.9, 3.95)
arrow(8.7, 4.35, 8.7, 3.95)

# ── 下层左：平差三步 ──────────────────────────────────────────────────
box(0.3, 2.45, 5.2, 1.45,
    "① 入选集联合平差（解析雅可比 + Huber，δ = 4 像素）\n"
    "② 固定位姿，单独优化被排除轨迹的三维点\n"
    "③ 重投影 / 夹角 / 深度三类剔除\n"
    "④ 重三角化，恢复被判为内点的已删观测",
    "#dcfce7", fontsize=8.8)

# ── 下层右：可恢复性判定 ──────────────────────────────────────────────
box(6.1, 2.45, 5.2, 1.45,
    "剔除的可恢复性由失败原因决定：\n"
    "残差超阈值 / 超深度 / 夹角不足 / 三角化外点 → 可恢复\n"
    "深度为负 / 初始像对非内点 / 姿态解算外点 → 不可恢复\n"
    "恢复触发：某相机焦距相对变化 > 2%，判据重投影 ≤ 4 像素",
    "#e0f2fe", fontsize=8.8)

arrow(2.9, 2.45, 2.9, 2.05)
arrow(8.7, 2.45, 8.7, 2.05)
ax.add_patch(FancyArrowPatch((8.7, 1.85), (2.9, 1.85), arrowstyle="-|>",
                             mutation_scale=12, linewidth=1.4,
                             color=PALETTE["primary"]))
ax.text(5.8, 1.98, "内参精化后回流：把早期误删的结构线索重新接上",
        ha="center", fontsize=8.8, color=PALETTE["primary"])

box(1.0, 0.35, 9.6, 1.15,
    "设计动机：冗余观测主要买鲁棒性而非信息量，故逐轮只取子集求解，在规模与质量间折中；\n"
    "早期焦距与畸变不可靠时结构必然带偏差，因此一切剔除先做可逆标记而非硬删，\n"
    "待内参随迭代变好再重新评估。轨迹存储因此完整保留了整个估计过程的最终状态。",
    "#f3f4f6", fontsize=9.0)

save(fig, "../figures/fig7_subset_ba_mask.svg")
save(fig, "../figures/fig7_subset_ba_mask.png")
print("done")
