#!/usr/bin/env python3
"""
Plot odometry pose from ROS bag files.
Reads all .bag files in BAG_DIR and plots:
  - 2D trajectory (X vs Y)
  - Position over time (X, Y, Z)
  - Orientation over time (roll, pitch, yaw)
"""

import os
import glob
import math
import rosbag
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 – registers the 3d projection

BAG_DIR = "/root/bags"
ODOM_TOPIC = "/aft_mapped_to_init"


def quat_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw (radians)."""
    # Roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    # Yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def read_bag(bag_path):
    data = {"t": [], "x": [], "y": [], "z": [],
            "roll": [], "pitch": [], "yaw": []}
    with rosbag.Bag(bag_path, "r") as bag:
        t0 = None
        for _, msg, t in bag.read_messages(topics=[ODOM_TOPIC]):
            stamp = t.to_sec()
            if t0 is None:
                t0 = stamp
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
            data["t"].append(stamp - t0)
            data["x"].append(p.x)
            data["y"].append(p.y)
            data["z"].append(p.z)
            data["roll"].append(math.degrees(roll))
            data["pitch"].append(math.degrees(pitch))
            data["yaw"].append(math.degrees(yaw))
    return data


def plot_bag(name, d):
    color = "steelblue"

    # ── Figure 1: 2D Trajectory ───────────────────────────────────────────────
    fig1, ax1 = plt.subplots(figsize=(8, 7))
    ax1.plot(d["x"], d["y"], color=color, linewidth=1.2)
    ax1.plot(d["x"][0],  d["y"][0],  "o", color="green", markersize=8, label="start")
    ax1.plot(d["x"][-1], d["y"][-1], "s", color="red",   markersize=8, label="end")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title(f"2D Trajectory — {name}")
    ax1.legend(fontsize=9)
    ax1.set_aspect("equal")
    ax1.grid(True, linestyle="--", alpha=0.5)

    # ── Figure 2: Position over time ─────────────────────────────────────────
    fig2, axes2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig2.suptitle(f"Position over Time — {name}")
    for ax, key, label in zip(axes2, ["x", "y", "z"], ["X (m)", "Y (m)", "Z (m)"]):
        ax.plot(d["t"], d[key], color=color, linewidth=1)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.5)
    axes2[2].set_xlabel("Time (s)")
    fig2.tight_layout()

    # ── Figure 3: Orientation over time ──────────────────────────────────────
    fig3, axes3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig3.suptitle(f"Orientation over Time — {name}")
    for ax, key, label in zip(axes3, ["roll", "pitch", "yaw"], ["Roll (°)", "Pitch (°)", "Yaw (°)"]):
        ax.plot(d["t"], d[key], color=color, linewidth=1)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.5)
    axes3[2].set_xlabel("Time (s)")
    fig3.tight_layout()

    # ── Figure 4: 3D Trajectory ───────────────────────────────────────────────
    fig4 = plt.figure(figsize=(9, 7))
    ax4 = fig4.add_subplot(111, projection="3d")
    ax4.plot(d["x"], d["y"], d["z"], color=color, linewidth=1)
    ax4.scatter(d["x"][0],  d["y"][0],  d["z"][0],  color="green", s=50, marker="o", label="start")
    ax4.scatter(d["x"][-1], d["y"][-1], d["z"][-1], color="red",   s=50, marker="s", label="end")
    ax4.set_xlabel("X (m)")
    ax4.set_ylabel("Y (m)")
    ax4.set_zlabel("Z (m)")
    ax4.set_title(f"3D Trajectory — {name}")
    ax4.legend(fontsize=9)


def main():
    bag_files = sorted(glob.glob(os.path.join(BAG_DIR, "*.bag")))
    if not bag_files:
        print(f"No .bag files found in {BAG_DIR}")
        return

    print(f"Found {len(bag_files)} bag(s): {[os.path.basename(b) for b in bag_files]}")

    for bf in bag_files:
        name = os.path.basename(bf)
        print(f"  Reading {name} ...")
        d = read_bag(bf)
        print(f"    -> {len(d['t'])} odometry messages")
        plot_bag(name, d)
        plt.show()


if __name__ == "__main__":
    main()
