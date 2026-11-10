"""Draw the published map and task zones from the experiment inputs."""
import argparse
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo", type=Path, help="path to the llm_swarm checkout")
    args = parser.parse_args()
    sim = args.repo / "iros_llm_swarm_simulation_lite"
    grid_cfg = yaml.safe_load((sim / "map/amongus.yaml").read_text())
    image = Image.open(sim / "map" / grid_cfg["image"]).convert("L")
    occupied = (255 - np.asarray(image, dtype=float)) / 255 > grid_cfg["occupied_thresh"]
    x0, y0, _ = grid_cfg["origin"]
    step = grid_cfg["resolution"]
    h, w = occupied.shape
    ymax = y0 + h * step

    # Vector rectangles follow the exact occupied-pixel boundaries.
    walls = []
    for i, row in enumerate(occupied):
        endpoints = np.flatnonzero(np.diff(np.r_[False, row, False]))
        top, bottom = ymax - i * step, ymax - (i + 1) * step
        for left, right in endpoints.reshape(-1, 2):
            a, b = x0 + left * step, x0 + right * step
            walls.append([(a, bottom), (b, bottom), (b, top), (a, top)])

    tasks = yaml.safe_load((sim / "scenario/common_scenarios.yaml").read_text())["scenarios"]["amongus"]["tasks"]
    m2 = {"task_electrical", "task_med", "task_hall", "task_comms", "carry_security_to_admin"}
    groups = {}
    for x, y, color in re.findall(
        r'warehouse_robot\s*\(\s*name "robot_\d+"\s+pose \[\s*([-\d.]+)\s+([-\d.]+)[^]]*\]\s+color "([^"]+)"',
        (sim / "world/amongus.world").read_text(),
    ):
        groups.setdefault(color, []).append((float(x), float(y)))
    assert len(groups) == 5 and all(len(g) == 4 for g in groups.values())
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "pdf.fonttype": 42})
    fig, ax = plt.subplots(figsize=(5.35, 3.6))
    fig.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.21)
    ax.add_collection(PolyCollection(walls, facecolors="#737b84", edgecolors="none", antialiaseds=False))
    ax.set_facecolor("#ffffff")
    visit, cargo, ink = "#174a7c", "#9c4300", "#131921"

    for task in tasks:
        if task["type"] == "carry":
            ax.annotate("", xy=task["dropoff"], xytext=task["position"],
                        arrowprops={"arrowstyle": "-|>", "color": cargo, "lw": 1.1,
                                    "shrinkA": 7, "shrinkB": 8},
                        zorder=2)

    def zone(point, radius, marker, color, in_m2):
        x, y = point
        ax.add_patch(Circle((x, y), radius, fill=False, edgecolor=color, lw=1.0, zorder=3))
        ax.plot(x, y, marker=marker, ms=5, mfc=color, mec="white", mew=0.5,
                linestyle="none", zorder=4)
        if in_m2:
            ax.plot(x + 1.2, y + 1.2, marker="*", ms=7, mfc=ink, mec="white", mew=0.6,
                    linestyle="none", zorder=5)

    dropoffs = set()
    for task in tasks:
        if task["type"] == "point":
            zone(task["position"], task["radius"], "o", visit, task["id"] in m2)
        else:
            zone(task["position"], task["radius"], "^", cargo, task["id"] in m2)
            key = tuple(task["dropoff"])
            if key not in dropoffs:
                zone(key, task["radius"], "D", cargo, task["id"] in m2)
                dropoffs.add(key)

    for poses in groups.values():
        x, y = np.mean(poses, axis=0)
        ax.plot(x, y, "s", ms=5, mfc="white", mec=ink, mew=1.0, zorder=6)

    labels = [
        ("o", visit, visit, "Visit"),
        ("^", cargo, cargo, "Pickup"),
        ("D", cargo, cargo, "Drop-off"),
        ("s", "white", ink, "Group home"),
        ("*", ink, ink, "M2 task"),
    ]
    handles = [Line2D([], [], marker=marker, mfc=fill, mec=edge,
                      linestyle="none", ms=6, label=label)
               for marker, fill, edge, label in labels]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.04),
              ncol=3, frameon=False, columnspacing=1.1, handletextpad=0.35)
    ax.set(xlim=(x0, x0 + w * step), ylim=(y0, ymax),
           xticks=[], yticks=[], aspect="equal")
    fig.savefig(Path(__file__).with_name("Fig3.pdf"), bbox_inches="tight", pad_inches=0.035)


if __name__ == "__main__":
    main()
