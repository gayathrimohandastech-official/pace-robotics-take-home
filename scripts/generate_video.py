"""
scripts/generate_video.py
Animated video of the run: robot moving down the corridor, wall panel
shifting at t=15s/30s/45s, ground truth vs fused estimate tracked live.

Requires ffmpeg on PATH to save as .mp4. If ffmpeg isn't available, falls
back to saving an animated .gif instead (still satisfies the "demonstration
video" deliverable).
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scripts.run_pipeline import run_full_pipeline


def make_video(result, out_path="output/simulation_video.mp4", frame_stride=4):
    data = result["data"]
    x_est = result["x_est"]

    t = data["t"][::frame_stride]
    x_true = data["x_true"][::frame_stride]
    y_true = data["y_true"][::frame_stride]
    wall_offset = data["wall_offset"][::frame_stride]
    x_est_s = x_est[::frame_stride]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlim(-1, 22)
    ax.set_ylim(-3, 3)
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.grid(True, alpha=0.3)

    wall_top, = ax.plot([], [], "k-", linewidth=4)
    wall_bottom, = ax.plot([], [], "k-", linewidth=4)
    panel, = ax.plot([], [], "orange", linewidth=6, label="Shifting wall panel")
    gt_dot, = ax.plot([], [], "ko", markersize=8, label="Ground truth")
    est_dot, = ax.plot([], [], "b^", markersize=8, label="Fused estimate")
    gt_trail, = ax.plot([], [], "k-", alpha=0.3)
    est_trail, = ax.plot([], [], "b-", alpha=0.3)
    time_text = ax.text(0.02, 0.92, "", transform=ax.transAxes)
    ax.legend(loc="upper right")

    def init():
        wall_top.set_data([0, 20], [2.0, 2.0])
        wall_bottom.set_data([0, 20], [-2.0, -2.0])
        return wall_top, wall_bottom, panel, gt_dot, est_dot, gt_trail, est_trail, time_text

    def update(frame):
        panel.set_data([9.6, 10.4], [wall_offset[frame], wall_offset[frame]])
        gt_dot.set_data([x_true[frame]], [y_true[frame]])
        est_dot.set_data([x_est_s[frame, 0]], [x_est_s[frame, 1]])
        gt_trail.set_data(x_true[:frame+1], y_true[:frame+1])
        est_trail.set_data(x_est_s[:frame+1, 0], x_est_s[:frame+1, 1])
        time_text.set_text(f"t = {t[frame]:.1f}s")
        return wall_top, wall_bottom, panel, gt_dot, est_dot, gt_trail, est_trail, time_text

    ani = animation.FuncAnimation(fig, update, frames=len(t), init_func=init,
                                    interval=50, blit=True)

    try:
        ani.save(out_path, writer="ffmpeg", fps=20, dpi=120)
        print(f"Saved {out_path}")
    except Exception as e:
        gif_path = out_path.replace(".mp4", ".gif")
        print(f"ffmpeg not available ({e}); falling back to GIF")
        ani.save(gif_path, writer="pillow", fps=15, dpi=100)
        print(f"Saved {gif_path}")

    plt.close()


if __name__ == "__main__":
    os.makedirs("output", exist_ok=True)
    result = run_full_pipeline()
    make_video(result)