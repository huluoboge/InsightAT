"""图 5：增量式重建主循环的信息流示意图（抽象方法流程）。"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from common import PALETTE, save

fig, ax = plt.subplots(figsize=(9.5, 6.6))
ax.set_xlim(0, 9.5)
ax.set_ylim(0, 6.6)
ax.axis("off")


def box(x, y, w, h, text, fc, fontsize=9.6):
    b = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.07,rounding_size=0.09",
        linewidth=1.1, edgecolor="#1f2937", facecolor=fc,
    )
    ax.add_patch(b)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fontsize, color="#111827")


def arrow(x1, y1, x2, y2, color="#374151", connectionstyle=None):
    kwargs = dict(arrowstyle="-|>", mutation_scale=13, linewidth=1.3, color=color)
    if connectionstyle:
        kwargs["connectionstyle"] = connectionstyle
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), **kwargs))


def elbow_down(x1, y1, x2, y2, y_mid, color="#374151"):
    """竖直下行到 y_mid，再水平转到 x2，最后竖直箭头指向 (x2, y2)。"""
    ax.plot([x1, x1], [y1, y_mid], color=color, linewidth=1.3, solid_capstyle="round")
    ax.plot([x1, x2], [y_mid, y_mid], color=color, linewidth=1.3, solid_capstyle="round")
    arrow(x2, y_mid, x2, y2, color=color)


ax.text(4.75, 6.3, "增量式重建主循环（每轮注册一张新影像）", ha="center", fontsize=12.5, fontweight="bold")

steps = [
    ("① 候选影像\n选择", "覆盖率打分 + 试解取优\n（至多 8 个候选试算）"),
    ("② 姿态解算\n（PnP）", "采样一致性 4 像素\n内点 ≥ 30 且内点率 ≥ 0.10"),
    ("③ 新增轨迹\n三角化", "夹角 [0.5°, 120°]\n重投影 ≤ 16 像素"),
    ("④ 局部/全局\n光束法平差", "注册数 ≤ 100 全用全局平差\n> 100 转局部 + 周期性全局"),
    ("⑤ 轨迹重三角\n化与观测恢复", "待处理队列每 3 轮\n全表扫描每 10 轮"),
]

w, h, gap, y = 1.55, 1.05, 0.35, 3.6
x0 = (9.5 - (5 * w + 4 * gap)) / 2
positions = []
for i, (title, sub) in enumerate(steps):
    x = x0 + i * (w + gap)
    box(x, y, w, h, title, PALETTE["bg_box2"])
    ax.text(x + w / 2, y - 0.52, sub, ha="center", fontsize=7.6, color="#4b5563")
    positions.append((x, y, w, h))
    if i > 0:
        px, py, pw, ph = positions[i - 1]
        arrow(px + pw, y + h / 2, x, y + h / 2)

# 循环回边
last_x, last_y, last_w, last_h = positions[-1]
first_x, first_y, first_w, first_h = positions[0]
arrow(
    last_x + last_w / 2, last_y + last_h,
    first_x + first_w / 2, first_y + first_h,
    color=PALETTE["primary"],
    connectionstyle="arc3,rad=0.35",
)
ax.text(4.75, 5.35, "存在未注册且可解算的候选影像 → 继续下一轮", fontsize=9, color=PALETTE["primary"], ha="center")

# 终止条件：判断发生在①候选影像选择之后，而非中间步骤
first_cx = first_x + first_w / 2
box(0.55, 1.1, 2.8, 0.75, "无可解算候选\n（连续两轮）", "#fee2e2")
elbow_down(first_cx, first_y, 1.95, 1.85, y_mid=2.35, color="#dc2626")

box(0.55, 0.1, 2.8, 0.7, "收尾：全局平差 +\n待处理轨迹重三角化", "#dcfce7")
arrow(1.95, 1.1, 1.95, 0.8)

save(fig, "../figures/fig5_incremental_loop.svg")
save(fig, "../figures/fig5_incremental_loop.png")
print("done")
