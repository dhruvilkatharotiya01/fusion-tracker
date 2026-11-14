"""Draw the README figures from fusion_eval output.

    ./build/fusion_eval --runs 50 --seed 2 --trace out/trace.csv > out/seed2.txt
    python tools/plot_results.py out/seed2.txt out/trace.csv docs
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {"radar": "#2a78d6", "camera": "#eb6834", "fused": "#1baf7a"}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def style(ax):
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def lateral_by_range(summary_txt, out):
    rows = {}
    for line in Path(summary_txt).read_text().splitlines():
        parts = line.split()
        if parts and parts[0] in COLORS and " / " in line:
            rows[parts[0]] = [float(v) for v in line.split("   ")[-1].split(" / ")]
    bins = ["5–30 m", "30–55 m", "55–80 m"]
    fig, ax = plt.subplots(figsize=(7, 3.6), dpi=150)
    width = 0.26
    for k, (mode, vals) in enumerate(rows.items()):
        xs = [i + (k - 1) * (width + 0.02) for i in range(3)]
        ax.bar(xs, vals, width, color=COLORS[mode], label=mode)
        for x, v in zip(xs, vals):
            ax.text(x, v + 0.006, f"{v:.2f}", ha="center", va="bottom", fontsize=8, color=INK)
    ax.set_xticks(range(3), bins)
    ax.set_ylabel("lateral position error, RMSE (m)", color=MUTED, fontsize=9)
    ax.set_title("Lateral error by distance ahead (50 drives, held-out seed)", color=INK, fontsize=11, loc="left")
    style(ax)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK, loc="upper left")
    fig.tight_layout()
    fig.savefig(out)


def trace(trace_csv, out, start=20.0, end=32.0):
    truth, tracks = defaultdict(list), defaultdict(list)
    with open(trace_csv) as f:
        for row in csv.DictReader(f):
            t = float(row["t"])
            if not start <= t <= end:
                continue
            store = truth if row["kind"] == "truth" else tracks
            store[int(row["id"])].append((float(row["x"]), float(row["y"])))
    fig, ax = plt.subplots(figsize=(7, 3.2), dpi=150)
    for y in (-5.25, -1.75, 1.75, 5.25):
        ax.axhline(y, color=GRID, linewidth=1, linestyle="-" if abs(y) > 5 else "--", zorder=0)
    for i, pts in enumerate(truth.values()):
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=MUTED, linewidth=2, alpha=0.5, label="true path" if i == 0 else None)
    for i, pts in enumerate(tracks.values()):
        xs, ys = zip(*pts)
        ax.scatter(xs, ys, s=8, color=COLORS["fused"], label="fused track" if i == 0 else None, zorder=3)
    ax.set_xlim(-5, 165)
    ax.set_ylim(-6.5, 6.5)
    ax.set_xlabel("distance ahead of the ego car (m)", color=MUTED, fontsize=9)
    ax.set_ylabel("lateral (m)", color=MUTED, fontsize=9)
    ax.set_title(f"Fused tracks against the truth, {end - start:.0f} s of one drive", color=INK, fontsize=11, loc="left")
    style(ax)
    ax.grid(False)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK, loc="upper right", ncol=2)
    fig.tight_layout()
    fig.savefig(out)


if __name__ == "__main__":
    summary_txt, trace_csv, out_dir = sys.argv[1:4]
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    lateral_by_range(summary_txt, Path(out_dir) / "lateral_error.png")
    trace(trace_csv, Path(out_dir) / "trace.png")
