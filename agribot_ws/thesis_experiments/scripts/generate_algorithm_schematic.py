#!/usr/bin/env python3
"""Generate the dimensionally controlled thesis navigation schematic."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle
import numpy as np


WORKSPACE = Path(__file__).resolve().parents[2]
OUTPUT_DIR = (
    WORKSPACE
    / "thesis_experiments"
    / "analysis"
    / "success_cases_20260901"
    / "figures"
)

ROW_SPACING_M = 0.60
ROBOT_WIDTH_M = 0.22
ROBOT_HEIGHT_M = 0.30
CORN_HEIGHT_M = 2.00


def configure_plot() -> None:
    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    )
    for candidate in candidates:
        path = Path(candidate)
        if path.exists():
            font_manager.fontManager.addfont(str(path))
            family = font_manager.FontProperties(fname=str(path)).get_name()
            plt.rcParams["font.sans-serif"] = [family]
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 150
    plt.rcParams["savefig.dpi"] = 300


def cubic_bezier(p0, p1, p2, p3, samples=80):
    t = np.linspace(0.0, 1.0, samples)[:, None]
    points = (
        (1 - t) ** 3 * np.asarray(p0)
        + 3 * (1 - t) ** 2 * t * np.asarray(p1)
        + 3 * (1 - t) * t**2 * np.asarray(p2)
        + t**3 * np.asarray(p3)
    )
    return points[:, 0], points[:, 1]


def draw_corn_symbol(ax, x, y, scale=1.0):
    ax.plot([x, x], [y - 0.075 * scale, y + 0.075 * scale],
            color="#477a28", linewidth=1.4, zorder=4)
    ax.plot([x, x - 0.045 * scale], [y + 0.025 * scale, y + 0.065 * scale],
            color="#63a63c", linewidth=1.0, zorder=4)
    ax.plot([x, x + 0.045 * scale], [y - 0.005 * scale, y + 0.040 * scale],
            color="#63a63c", linewidth=1.0, zorder=4)
    ax.add_patch(Circle((x, y - 0.075 * scale), 0.012 * scale,
                        color="#7a5228", zorder=4))


def add_direction_arrow(ax, start, end, color, size=13):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={"arrowstyle": "-|>", "color": color,
                    "lw": 2.4, "mutation_scale": size},
        zorder=8,
    )


def draw_plan_view(ax):
    row_x = np.array([0.0, ROW_SPACING_M, 2 * ROW_SPACING_M])
    plant_y = np.arange(0.0, 3.31, 0.28)

    ax.axhspan(-0.95, 0.0, color="#efe5cf", alpha=0.72, zorder=0)
    ax.text(1.42, -0.86, "地头开放区域", ha="right", va="bottom",
            fontsize=10, color="#6b5a42")

    for x in row_x:
        ax.plot([x, x], [0.0, 3.35], color="#94b86a", linewidth=0.8,
                alpha=0.55, zorder=1)
        for y in plant_y:
            draw_corn_symbol(ax, x, y)

    left_center = ROW_SPACING_M / 2
    right_center = 3 * ROW_SPACING_M / 2
    ax.plot([left_center, left_center], [0.05, 3.25], color="#2ca02c",
            linewidth=2.0, linestyle="--", label="行间中心线", zorder=3)
    ax.plot([right_center, right_center], [0.05, 3.25], color="#2ca02c",
            linewidth=2.0, linestyle="--", zorder=3)

    ax.plot([0.0, 0.0], [0.9, 2.7], color="#d62728", linewidth=1.8,
            alpha=0.85, label="检测左边界", zorder=5)
    ax.plot([ROW_SPACING_M, ROW_SPACING_M], [0.9, 2.7], color="#1f77b4",
            linewidth=1.8, alpha=0.85, label="检测右边界", zorder=5)

    rng = np.random.default_rng(7)
    for x in (0.0, ROW_SPACING_M):
        cloud_y = np.linspace(1.0, 2.65, 46)
        cloud_x = x + rng.normal(0.0, 0.026, len(cloud_y))
        ax.scatter(cloud_x, cloud_y, s=7, color="#19b9c6", alpha=0.65,
                   linewidths=0, zorder=2)

    first_x = np.full(60, left_center)
    first_y = np.linspace(2.65, 0.05, 60)
    bx1, by1 = cubic_bezier(
        (left_center, 0.05), (left_center, -0.50),
        (0.43, -0.72), (ROW_SPACING_M, -0.72)
    )
    bx2, by2 = cubic_bezier(
        (ROW_SPACING_M, -0.72), (0.77, -0.72),
        (right_center, -0.50), (right_center, 0.05)
    )
    last_x = np.full(60, right_center)
    last_y = np.linspace(0.05, 2.65, 60)
    turn_x = np.concatenate([first_x, bx1[1:], bx2[1:], last_x[1:]])
    turn_y = np.concatenate([first_y, by1[1:], by2[1:], last_y[1:]])
    ax.plot(turn_x, turn_y, color="#f28e1c", linewidth=3.2,
            label="相邻行掉头路径", zorder=7)
    add_direction_arrow(ax, (left_center, 1.95), (left_center, 1.55), "#f28e1c")
    add_direction_arrow(ax, (0.49, -0.71), (0.68, -0.71), "#f28e1c")
    add_direction_arrow(ax, (right_center, 0.95), (right_center, 1.35), "#f28e1c")

    robot_length = 0.34
    robot_y = 1.78
    body = Rectangle(
        (left_center - ROBOT_WIDTH_M / 2, robot_y - robot_length / 2),
        ROBOT_WIDTH_M,
        robot_length,
        facecolor="#4d5966",
        edgecolor="#17202a",
        linewidth=1.4,
        zorder=9,
    )
    ax.add_patch(body)
    for wheel_x in (left_center - ROBOT_WIDTH_M / 2 - 0.018,
                    left_center + ROBOT_WIDTH_M / 2 + 0.018):
        for wheel_y in (robot_y - 0.105, robot_y + 0.105):
            ax.add_patch(Circle((wheel_x, wheel_y), 0.027,
                                color="#16191c", zorder=10))
    ax.add_patch(Circle((left_center, robot_y), 0.042,
                        facecolor="#c7d5df", edgecolor="#34495e", zorder=10))
    ax.annotate("行间机器人", (left_center, robot_y), xytext=(0.10, 2.08),
                arrowprops={"arrowstyle": "->", "color": "#34495e"},
                fontsize=10, ha="center")

    dim_y = 3.58
    ax.annotate("", xy=(0.0, dim_y), xytext=(ROW_SPACING_M, dim_y),
                arrowprops={"arrowstyle": "<->", "color": "#222", "lw": 1.3})
    ax.plot([0.0, 0.0], [3.40, 3.66], color="#222", linewidth=0.9)
    ax.plot([ROW_SPACING_M, ROW_SPACING_M], [3.40, 3.66], color="#222", linewidth=0.9)
    ax.text(ROW_SPACING_M / 2, 3.66, "种植行中心距 0.60 m",
            ha="center", va="bottom", fontsize=10)

    width_y = robot_y - robot_length / 2 - 0.10
    ax.annotate(
        "",
        xy=(left_center - ROBOT_WIDTH_M / 2, width_y),
        xytext=(left_center + ROBOT_WIDTH_M / 2, width_y),
        arrowprops={"arrowstyle": "<->", "color": "#222", "lw": 1.2},
    )
    ax.text(left_center, width_y - 0.08, "车宽 0.22 m",
            ha="center", va="top", fontsize=9)

    ax.text(left_center, 3.18, "原行通道", ha="center", fontsize=10,
            color="#2e6b2e")
    ax.text(right_center, 3.18, "相邻行通道", ha="center", fontsize=10,
            color="#2e6b2e")
    ax.set_xlim(-0.24, 1.46)
    ax.set_ylim(-0.97, 3.82)
    ax.set_aspect("equal")
    ax.set_title("(a) 俯视几何与换行路径（横向尺寸按比例）", fontsize=13)
    ax.set_xlabel("横向位置（m）")
    ax.set_ylabel("沿种植行方向（m）")
    ax.set_xticks([0.0, 0.3, 0.6, 0.9, 1.2])
    ax.grid(color="#999", alpha=0.15, linewidth=0.6)
    ax.legend(loc="lower left", fontsize=9, framealpha=0.95)


def draw_side_view(ax):
    ax.axhline(0.0, color="#7b5e3b", linewidth=2.0)

    corn_x = 0.55
    ax.plot([corn_x, corn_x], [0.0, CORN_HEIGHT_M], color="#477a28", linewidth=7)
    for height, side in zip(np.linspace(0.35, 1.75, 7), (-1, 1, -1, 1, -1, 1, -1)):
        ax.plot([corn_x, corn_x + side * 0.30], [height, height + 0.18],
                color="#63a63c", linewidth=4, solid_capstyle="round")
    ax.scatter([corn_x], [CORN_HEIGHT_M], marker="^", s=90, color="#477a28")

    robot_x = 1.35
    robot = Rectangle((robot_x - 0.24, 0.0), 0.48, ROBOT_HEIGHT_M,
                      facecolor="#4d5966", edgecolor="#17202a", linewidth=1.4)
    ax.add_patch(robot)
    ax.add_patch(Circle((robot_x - 0.17, 0.02), 0.07, color="#16191c"))
    ax.add_patch(Circle((robot_x + 0.17, 0.02), 0.07, color="#16191c"))
    ax.plot([robot_x, robot_x], [ROBOT_HEIGHT_M, ROBOT_HEIGHT_M + 0.06],
            color="#34495e", linewidth=2)
    ax.add_patch(Circle((robot_x, ROBOT_HEIGHT_M + 0.07), 0.055,
                        facecolor="#c7d5df", edgecolor="#34495e"))

    ax.annotate("", xy=(0.13, 0.0), xytext=(0.13, CORN_HEIGHT_M),
                arrowprops={"arrowstyle": "<->", "color": "#222", "lw": 1.3})
    ax.text(0.06, CORN_HEIGHT_M / 2, "玉米高度\n2.00 m",
            ha="right", va="center", fontsize=10)
    ax.annotate("", xy=(1.78, 0.0), xytext=(1.78, ROBOT_HEIGHT_M),
                arrowprops={"arrowstyle": "<->", "color": "#222", "lw": 1.3})
    ax.text(1.85, ROBOT_HEIGHT_M / 2, "小车高度\n0.30 m",
            ha="left", va="center", fontsize=10)

    ax.text(corn_x, -0.12, "单株玉米", ha="center", va="top", fontsize=10)
    ax.text(robot_x, -0.12, "机器人", ha="center", va="top", fontsize=10)
    ax.set_xlim(-0.18, 2.25)
    ax.set_ylim(-0.22, 2.22)
    ax.set_aspect("equal")
    ax.set_title("(b) 高度比例示意", fontsize=13)
    ax.set_ylabel("离地高度（m）")
    ax.set_xticks([])
    ax.set_yticks([0.0, 0.3, 1.0, 2.0])
    ax.grid(axis="y", color="#999", alpha=0.18, linewidth=0.6)


def main() -> None:
    configure_plot()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(13.2, 7.8), constrained_layout=True)
    grid = fig.add_gridspec(1, 2, width_ratios=(1.55, 0.85))
    draw_plan_view(fig.add_subplot(grid[0, 0]))
    draw_side_view(fig.add_subplot(grid[0, 1]))
    fig.suptitle("玉米行间机器人感知与相邻行掉头原理简图", fontsize=17,
                 fontweight="bold")
    for suffix in ("png", "svg"):
        output = OUTPUT_DIR / f"00_algorithm_geometry_schematic.{suffix}"
        fig.savefig(output, bbox_inches="tight")
        print(output)
    plt.close(fig)


if __name__ == "__main__":
    main()
