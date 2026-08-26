from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FIG_DIR = ROOT / "figures"

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")

import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "SimHei", "WenQuanYi Micro Hei"]
rcParams["axes.unicode_minus"] = False


def _box(ax, xy, width, height, text, fc, ec="#333333"):
    patch = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.02,rounding_size=0.02",
        linewidth=1.5,
        facecolor=fc,
        edgecolor=ec,
    )
    ax.add_patch(patch)
    ax.text(
        xy[0] + width / 2,
        xy[1] + height / 2,
        text,
        ha="center",
        va="center",
        fontsize=12,
        color="#111111",
    )
    return patch


def _arrow(ax, start, end):
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=16,
            linewidth=1.5,
            color="#333333",
        )
    )


def _setup(ax, title):
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.5, 0.95, title, ha="center", va="center", fontsize=15, weight="bold")


def bachelor_architecture():
    fig, ax = plt.subplots(figsize=(12, 7), dpi=200)
    _setup(ax, "本科论文系统图：基于 ROS 2 的麦克纳姆轮平台 + Nav2 验证")

    _box(ax, (0.05, 0.70), 0.18, 0.12, "Xbox 手柄\n遥操作", "#DCEBFA")
    _box(ax, (0.28, 0.70), 0.18, 0.12, "Qt6 上位机\nOpenGL 可视化", "#D7F2E3")
    _box(ax, (0.51, 0.70), 0.18, 0.12, "Raspberry Pi 4B\nROS 2 中控", "#F9E3D6")
    _box(ax, (0.74, 0.70), 0.20, 0.12, "STM32 驱动板\n电机/IMU", "#F7DDEB")

    _box(ax, (0.10, 0.42), 0.22, 0.12, "UniLidar L2\n3D 点云", "#E8F0D9")
    _box(ax, (0.40, 0.42), 0.22, 0.12, "Astra Pro\n深度图像", "#E3E7FA")
    _box(ax, (0.70, 0.42), 0.22, 0.12, "IMU / 轮速\n状态反馈", "#FBE5C8")

    _box(ax, (0.22, 0.15), 0.28, 0.14, "定位层\nSLAM / TF / 里程计", "#DDE8FF")
    _box(ax, (0.56, 0.15), 0.28, 0.14, "Nav2\n规划 / 控制 / 避障", "#D8F1EE")

    _arrow(ax, (0.23, 0.76), (0.28, 0.76))
    _arrow(ax, (0.46, 0.76), (0.51, 0.76))
    _arrow(ax, (0.69, 0.76), (0.74, 0.76))

    _arrow(ax, (0.59, 0.70), (0.59, 0.54))
    _arrow(ax, (0.81, 0.70), (0.81, 0.54))
    _arrow(ax, (0.21, 0.42), (0.30, 0.29))
    _arrow(ax, (0.51, 0.42), (0.42, 0.29))
    _arrow(ax, (0.81, 0.42), (0.70, 0.29))

    _arrow(ax, (0.50, 0.22), (0.56, 0.22))
    _arrow(ax, (0.35, 0.15), (0.35, 0.10))
    ax.text(0.5, 0.05, "评测指标：标定误差、控制延迟、轨迹误差、避障成功率", ha="center", fontsize=10)

    fig.tight_layout()
    fig.savefig(FIG_DIR / "bachelor_architecture.png", bbox_inches="tight")
    plt.close(fig)


def graduate_architecture():
    fig, ax = plt.subplots(figsize=(12, 7), dpi=200)
    _setup(ax, "研究生论文系统图：面向 Autoware 的 Sensor Kit + 标定 + 导航验证")

    _box(ax, (0.05, 0.70), 0.22, 0.12, "UniLidar L2\n主定位传感器", "#E8F0D9")
    _box(ax, (0.31, 0.70), 0.18, 0.12, "Astra Pro\n近场补充感知", "#E3E7FA")
    _box(ax, (0.53, 0.70), 0.16, 0.12, "IMU\n姿态约束", "#FBE5C8")
    _box(ax, (0.73, 0.70), 0.22, 0.12, "STM32 + 底盘\n低层控制", "#F7DDEB")

    _box(ax, (0.13, 0.44), 0.26, 0.14, "时空标定\n外参 + 时间同步", "#DDE8FF")
    _box(ax, (0.43, 0.44), 0.22, 0.14, "Raspberry Pi 4B\nROS 2 边缘桥接", "#F9E3D6")
    _box(ax, (0.69, 0.44), 0.22, 0.14, "Autoware Universe\nLocalization / Planning", "#D8F1EE")

    _box(ax, (0.18, 0.16), 0.24, 0.14, "地图与定位\nNDT / 轨迹评测", "#E6DDFF")
    _box(ax, (0.48, 0.16), 0.18, 0.14, "导航验证\n路径跟踪", "#D9F3E9")
    _box(ax, (0.70, 0.16), 0.18, 0.14, "实验指标\n精度 / 鲁棒性 / 延迟", "#FCE2D4")

    _arrow(ax, (0.16, 0.70), (0.24, 0.58))
    _arrow(ax, (0.40, 0.70), (0.34, 0.58))
    _arrow(ax, (0.61, 0.70), (0.54, 0.58))
    _arrow(ax, (0.84, 0.70), (0.84, 0.58))

    _arrow(ax, (0.39, 0.51), (0.43, 0.51))
    _arrow(ax, (0.65, 0.51), (0.69, 0.51))

    _arrow(ax, (0.54, 0.44), (0.30, 0.30))
    _arrow(ax, (0.80, 0.44), (0.57, 0.30))
    _arrow(ax, (0.84, 0.44), (0.79, 0.30))

    ax.text(0.5, 0.05, "研究问题：低成本传感器套件在室内低速平台上的可标定性、可定位性与可导航性", ha="center", fontsize=10)

    fig.tight_layout()
    fig.savefig(FIG_DIR / "graduate_architecture.png", bbox_inches="tight")
    plt.close(fig)


def main():
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    bachelor_architecture()
    graduate_architecture()


if __name__ == "__main__":
    main()
