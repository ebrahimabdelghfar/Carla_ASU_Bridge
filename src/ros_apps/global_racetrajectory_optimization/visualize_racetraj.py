#!/usr/bin/env python3
"""
Visualize a generated race trajectory (outputs/<name>.csv, format:
s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2) against the track
boundaries derived from the source centerline csv (x_m,y_m,w_tr_right_m,w_tr_left_m).
"""
import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_traj(traj_path):
    data = np.loadtxt(traj_path, comments='#', delimiter=';')
    return {
        "s_m": data[:, 0],
        "x_m": data[:, 1],
        "y_m": data[:, 2],
        "psi_rad": data[:, 3],
        "kappa_radpm": data[:, 4],
        "vx_mps": data[:, 5],
        "ax_mps2": data[:, 6],
    }


def load_track_bounds(track_path):
    data = np.loadtxt(track_path, comments='#', delimiter=',')
    x, y = data[:, 0], data[:, 1]
    w_right, w_left = data[:, 2], data[:, 3]

    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.arctan2(dy, dx)

    left_x = x - w_left * np.sin(heading)
    left_y = y + w_left * np.cos(heading)
    right_x = x + w_right * np.sin(heading)
    right_y = y - w_right * np.cos(heading)

    return (x, y), (left_x, left_y), (right_x, right_y)


def main():
    parser = argparse.ArgumentParser(description="Visualize generated race trajectory")
    parser.add_argument("traj_csv", help="path to outputs/traj_race_cl.csv")
    parser.add_argument("--track-csv", default=None, help="path to source centerline csv (for boundaries)")
    args = parser.parse_args()

    if not os.path.exists(args.traj_csv):
        print(f"File not found: {args.traj_csv}")
        print("Generate it first (e.g. `make generate_racetraj`).")
        sys.exit(1)

    traj = load_traj(args.traj_csv)

    fig, ax = plt.subplots(figsize=(10, 10))

    if args.track_csv and os.path.exists(args.track_csv):
        center, left, right = load_track_bounds(args.track_csv)
        ax.plot(*center, "k--", linewidth=0.7, label="Centerline")
        ax.plot(*left, "k-", linewidth=0.7, label="Track bound")
        ax.plot(*right, "k-", linewidth=0.7)

    sc = ax.scatter(traj["x_m"], traj["y_m"], c=traj["vx_mps"], cmap="viridis", s=8, label="Raceline (speed)")
    ax.scatter(traj["x_m"][0], traj["y_m"][0], color="lime", marker="o", s=150,
              edgecolor="black", label="Start", zorder=5)

    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("velocity in m/s")

    ax.set_title(f"Generated Race Trajectory\n{os.path.basename(args.traj_csv)}")
    ax.set_xlabel("east in m")
    ax.set_ylabel("north in m")
    ax.set_aspect("equal", "datalim")
    ax.grid(True, linestyle=":", alpha=0.7)
    ax.legend()

    print("Close the plot window to exit.")
    plt.show()


if __name__ == "__main__":
    main()
