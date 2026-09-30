"""绘图公共设置：中文字体、配色与保存工具。"""
import matplotlib
import matplotlib.pyplot as plt

matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "WenQuanYi Micro Hei",
    "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False
matplotlib.rcParams["font.size"] = 11
matplotlib.rcParams["svg.fonttype"] = "none"

PALETTE = {
    "primary": "#2563eb",
    "secondary": "#f59e0b",
    "accent": "#059669",
    "muted": "#6b7280",
    "danger": "#dc2626",
    "bg_box": "#eef2ff",
    "bg_box2": "#fef3c7",
}


def save(fig, path, dpi=180):
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
