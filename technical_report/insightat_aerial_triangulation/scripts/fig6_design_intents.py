"""图：四项设计初衷如何约束系统组织方式（方法示意图，无实测数据）。"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from common import PALETTE, save

fig, ax = plt.subplots(figsize=(12.4, 5.8))
ax.set_xlim(0, 12.4)
ax.set_ylim(0, 5.8)
ax.axis("off")


def box(x, y, w, h, text, fc, fontsize=9.5, weight="normal"):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.07,rounding_size=0.08",
        linewidth=1.15, edgecolor="#1f2937", facecolor=fc,
    ))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, fontweight=weight, color="#111827")


def arrow(x1, y1, x2, y2, color="#374151"):
    ax.add_patch(FancyArrowPatch(
        (x1, y1), (x2, y2), arrowstyle="-|>",
        mutation_scale=13, linewidth=1.3, color=color,
    ))


ax.text(6.2, 5.45, "四项设计初衷 → 系统组织约束", ha="center",
        fontsize=13, fontweight="bold")

intents = [
    ("云端友好", "无全局内存态\n任务描述驱动\n独立可执行程序", PALETTE["bg_box"]),
    ("傻瓜化自动化", "少暴露算法开关\n默认即可工作\n配置只留给评测", PALETTE["bg_box2"]),
    ("鲁棒性优先", "增量式优于全局式\n多假设 + 试解取优\n可逆剔除 + 降级路径", "#fee2e2"),
    ("性能", "CUDA → GPGPU → CPU\n异步批处理 I/O\nSoA + 自描述容器", "#dcfce7"),
]
w, h, gap = 2.75, 1.45, 0.4
x0 = (12.4 - (4 * w + 3 * gap)) / 2
centers = []
for i, (title, body, fc) in enumerate(intents):
    x = x0 + i * (w + gap)
    box(x, 3.55, w, h, f"{title}\n{body}", fc, fontsize=9.0, weight="bold")
    centers.append(x + w / 2)

for cx in centers:
    arrow(cx, 3.55, cx, 3.05)

ax.text(6.2, 2.85, "每个处理环节的统一组织模式", ha="center",
        fontsize=11.5, fontweight="bold")

stages = [
    ("任务生成", "按批次切分\n写出任务清单"),
    ("并发处理", "独立进程批处理\n异步读—算—写"),
    ("结果合并", "合并中间产物\n形成下一阶段输入"),
]
sw, sh, sgap = 2.6, 1.15, 0.5
sx0 = (12.4 - (3 * sw + 2 * sgap)) / 2
for i, (t, s) in enumerate(stages):
    x = sx0 + i * (sw + sgap)
    box(x, 0.85, sw, sh, f"{t}\n{s}", "#eef2ff", fontsize=9.3)
    if i > 0:
        arrow(x - sgap, 0.85 + sh / 2, x, 0.85 + sh / 2)

ax.text(6.2, 0.35,
        "阶段之间仅通过自描述文件交换数据；编排层只负责调度命令与参数，不链接算法库",
        ha="center", fontsize=9.2, color="#4b5563")

save(fig, "../figures/fig6_design_intents.svg")
save(fig, "../figures/fig6_design_intents.png")
print("done")
