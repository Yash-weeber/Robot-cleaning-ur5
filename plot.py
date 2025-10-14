#!/usr/bin/env python3
"""
Plot XY trajectories per iteration from trajectory_way.csv.

Each iteration (iter_idx) gets its own PNG plot:
  logs/iteration_plot/iter_###.png

CSV must have columns:
  iter_idx, step_idx, timestamp, x, y, z, q0, q1, q2, q3, q4, q5
"""

import os
import csv
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")   # run without GUI
import matplotlib.pyplot as plt

# ====== EDIT YOUR PROJECT ROOT ======
BASE_DIR = "Y:/models/ur5hanibenpng/final/Robot-cleaning-ur51/Robot-cleaning-ur5/"
# ====================================

LOGS_DIR = os.path.join(BASE_DIR, "logs")
CSV_PATH = os.path.join(LOGS_DIR, "trajectory_feedback.csv")
PLOT_DIR = os.path.join(LOGS_DIR, "iteration_plot")


def load_csv(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"CSV not found: {path}")

    groups = defaultdict(list)
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                it = int(float(row["iter"]))
                st = int(float(row["step"]))
                x = float(row["x"])
                y = float(row["y"])
                groups[it].append((st, x, y))
            except Exception:
                continue

    for it in groups:
        groups[it].sort(key=lambda t: t[0])  # sort by step_idx
    return groups


def plot_iteration(it, pts, outdir):
    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]

    plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, linewidth=2, label="trajectory")
    plt.scatter([xs[0]], [ys[0]], c="green", s=60, marker="o", label="start")
    plt.scatter([xs[-1]], [ys[-1]], c="red", s=60, marker="x", label="end")
    plt.title(f"Iteration {it:03d} ({len(xs)} points)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()

    out_path = os.path.join(outdir, f"iter_{it:03d}.png")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    return out_path


def main():
    os.makedirs(PLOT_DIR, exist_ok=True)
    groups = load_csv(CSV_PATH)
    print(f"Loaded {len(groups)} iterations from {CSV_PATH}")

    for it in sorted(groups.keys()):
        out_path = plot_iteration(it, groups[it], PLOT_DIR)
        print("Saved:", out_path)

    print("\nAll plots saved in:", PLOT_DIR)


if __name__ == "__main__":
    main()
