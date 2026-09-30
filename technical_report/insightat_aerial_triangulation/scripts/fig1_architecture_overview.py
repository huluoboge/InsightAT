"""图 1：任务描述、自描述容器与空三处理阶段的关系（方法示意图，无实测数据）。"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from common import PALETTE, save

fig, ax = plt.subplots(figsize=(10.4, 6.6))
ax.set_xlim(0, 10.4)
ax.set_ylim(0, 6.6)
ax.axis("off")


def box(x, y, w, h, text, fc, fontsize=9.4, weight="normal"):
    b = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.06,rounding_size=0.08",
        linewidth=1.15,
        edgecolor="#1f2937",
        facecolor=fc,
    )
    ax.add_patch(b)
    ax.text(
        x + w / 2,
        y + h / 2,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
        fontweight=weight,
        color="#111827",
    )


def arrow(x1, y1, x2, y2, color="#374151"):
    ax.add_patch(
        FancyArrowPatch(
            (x1, y1),
            (x2, y2),
            arrowstyle="-|>",
            mutation_scale=13,
            linewidth=1.3,
            color=color,
        )
    )


ax.text(5.2, 6.3, "一次可调度的工作：文件 + 命令 + 参数", ha="center", fontsize=13, fontweight="bold")

box(0.35, 5.15, 3.0, 0.85, "输入文件\n（全部任务或其中一批）", PALETTE["bg_box"], fontsize=9.2)
box(3.7, 5.15, 3.0, 0.85, "执行命令与参数\n（阶段算法本身）", PALETTE["bg_box"], fontsize=9.2)
box(7.05, 5.15, 3.0, 0.85, "输出文件\n（下一批任务的输入）", PALETTE["bg_box"], fontsize=9.2)
arrow(3.35, 5.55, 3.7, 5.55)
arrow(6.7, 5.55, 7.05, 5.55)

ax.text(5.2, 4.75, "模块内部通常拆成三段", ha="center", fontsize=11.5, fontweight="bold")

box(0.45, 3.55, 2.9, 0.9, "任务生成\n写出本批要处理的清单", PALETTE["bg_box2"], fontsize=9.1)
box(3.75, 3.55, 2.9, 0.9, "批次并发处理\n异步读入、计算、写出", "#dcfce7", fontsize=9.1)
box(7.05, 3.55, 2.9, 0.9, "结果合并\n形成下一阶段输入", PALETTE["bg_box2"], fontsize=9.1)
arrow(3.35, 4.0, 3.75, 4.0)
arrow(6.65, 4.0, 7.05, 4.0)

arrow(5.2, 3.55, 5.2, 3.15)

box(
    1.3,
    2.25,
    7.8,
    0.8,
    "自描述容器：JSON 描述各数据块的类型、形状、偏移与长度；二进制块按结构化数组存放",
    "#e0f2fe",
    fontsize=9.2,
)

arrow(5.2, 2.25, 5.2, 1.9)

stages = ["特征", "小图关联\n与匹配", "几何验证", "轨迹", "初始对", "增量重建\n与平差"]
w, gap = 1.45, 0.18
x0 = (10.4 - (6 * w + 5 * gap)) / 2
y = 0.55
for i, s in enumerate(stages):
    x = x0 + i * (w + gap)
    box(x, y, w, 1.15, s, "#f3f4f6", fontsize=8.8)
    if i:
        arrow(x - gap, y + 0.55, x, y + 0.55)

save(fig, "../figures/fig1_architecture_overview.svg")
save(fig, "../figures/fig1_architecture_overview.png")
print("done")
