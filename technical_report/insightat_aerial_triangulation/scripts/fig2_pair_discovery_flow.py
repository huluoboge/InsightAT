"""图：候选像对发现流程，含两条独立的召回补全通道。

图中门限与判据均取自默认流程的实际取值：穷举切换阈值 60 幅、
低分辨率长边 1024 像素 / 每幅 1500 特征、低分辨率几何验证基础矩阵
阈值 16 像素且内点下限 6、邻居数下限 5、全分辨率几何验证阈值
16 像素且内点下限 10。抽象方法流程图，不含任何源码标识。
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon

from common import PALETTE, save

fig, ax = plt.subplots(figsize=(11.2, 8.6))
ax.set_xlim(0, 11.2)
ax.set_ylim(-0.6, 8.6)
ax.axis("off")


def box(x, y, w, h, text, fc, fontsize=9.4):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.07,rounding_size=0.08",
        linewidth=1.1, edgecolor="#1f2937", facecolor=fc,
    ))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, color="#111827")


def diamond(cx, cy, w, h, text, fc, fontsize=9.2):
    pts = [(cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2), (cx - w / 2, cy)]
    ax.add_patch(Polygon(pts, closed=True, linewidth=1.1,
                         edgecolor="#1f2937", facecolor=fc))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fontsize, color="#111827")


def arrow(x1, y1, x2, y2, color="#374151", style="-|>", ls="solid"):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style,
                                 mutation_scale=13, linewidth=1.3,
                                 color=color, linestyle=ls))


# ── 顶部：输入与规模判定 ────────────────────────────────────────────────
box(4.2, 7.85, 2.8, 0.6, "影像集合，规模 n", PALETTE["bg_box"])
diamond(5.6, 6.75, 3.3, 1.0, "n < 60 ？", "#fef9c3")
arrow(5.6, 7.85, 5.6, 7.25)

# ── 左支：小规模直接穷举 ──────────────────────────────────────────────
box(0.35, 5.35, 2.9, 0.85, "直接生成全部\n$n(n-1)/2$ 个像对", PALETTE["bg_box2"])
ax.text(3.55, 6.95, "是", fontsize=9.6, color="#374151", ha="center", fontweight="bold")
arrow(3.95, 6.75, 1.8, 6.2)
# ── 右支：低分辨率穷举匹配 ────────────────────────────────────────────
box(7.0, 5.35, 4.0, 0.95,
    "低分辨率关联：长边缩至 1024 像素、\n每幅至多 1500 特征，穷举全部像对匹配",
    PALETTE["bg_box2"])
ax.text(7.55, 6.95, "否", fontsize=9.6, color="#374151", ha="center", fontweight="bold")
arrow(7.25, 6.75, 9.0, 6.3)

box(7.0, 4.15, 4.0, 0.85,
    "低分辨率几何验证：基础矩阵\n阈值 16 像素，内点下限 6",
    PALETTE["bg_box2"])
arrow(9.0, 5.35, 9.0, 5.0)

# ── 召回补全通道一：邻居不足 ──────────────────────────────────────────
box(7.0, 2.85, 4.0, 0.9,
    "通道一 · 邻居不足补全：\n通过验证的邻居数 < 5 的影像，\n补入其与全部其余影像的配对",
    "#dcfce7", fontsize=9.0)
arrow(9.0, 4.15, 9.0, 3.75)

# ── 召回补全通道二：弱纹理影像 ────────────────────────────────────────
box(0.35, 2.85, 4.0, 1.05,
    "通道二 · 弱纹理补全：\n全分辨率提取时特征数不足 10000、\n已降阈值重提取的影像，\n补入其与全部其余影像的配对",
    "#dcfce7", fontsize=9.0)
ax.text(2.35, 4.35, "来自特征提取阶段的标记", fontsize=8.6,
        color="#166534", ha="center", style="italic")
arrow(2.35, 4.15, 2.35, 3.9, color="#166534", ls="dashed")

# ── 汇合 ──────────────────────────────────────────────────────────────
box(4.05, 1.75, 3.1, 0.7, "候选像对集合（并集）", "#dbeafe")
# 左支沿左侧折线绕行，避免斜穿通道二的说明区
ax.plot([0.55, 0.55], [5.35, 2.10], color="#374151", linewidth=1.3,
        solid_capstyle="round")
ax.plot([0.55, 4.05], [2.10, 2.10], color="#374151", linewidth=1.3,
        solid_capstyle="round")
arrow(3.85, 2.10, 4.05, 2.10)
arrow(2.35, 2.85, 4.5, 2.45)
arrow(9.0, 2.85, 6.7, 2.45)

# ── 全分辨率精匹配与几何验证 ──────────────────────────────────────────
box(3.55, 0.7, 4.1, 0.75,
    "全分辨率级联哈希匹配 → 几何验证\n（基础矩阵阈值 16 像素，内点下限 10）",
    PALETTE["bg_box"], fontsize=9.2)
arrow(5.6, 1.75, 5.6, 1.45)

ax.text(5.6, 0.15,
        "输出：通过基础矩阵检验的内点掩码、成对相对几何与视图图属性",
        ha="center", fontsize=9.3, color="#374151")

ax.text(5.6, -0.45,
        "两条补全通道均为「宁可多算不可漏配」的取舍：以匹配量换取默认路径的召回率",
        ha="center", fontsize=9.0, color="#166534")

save(fig, "../figures/fig2_pair_discovery_flow.svg")
save(fig, "../figures/fig2_pair_discovery_flow.png")
print("done")
