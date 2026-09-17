"""
scripts/generate_outputs.py
Generates the three required deliverable outputs:
  1. output/trajectory_plot.png -- GT vs slipping odom vs fused estimate
  2. output/run_log.txt -- timestamped covariance + degeneracy warnings
  3. output/simulation_video.mp4 -- animated run (wall shift + robot response)
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scripts.run_pipeline import run_full_pipeline


def make_trajectory_plot(result, out_path="output/trajectory_plot.png"):
    data = result["data"]
    x_est = result["x_est"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # Top-down X-Y view (kept for completeness, but the real story is in ax2)
    ax1.plot(data["x_true"], data["y_true"], label="Ground truth", linewidth=2, color="black")
    ax1.plot(result["raw_x"], result["raw_y"], label="Slipping (unfused) odometry",
              linestyle="--", color="red", alpha=0.7)
    ax1.plot(x_est[:, 0], x_est[:, 1], label="Fused EKF estimate",
              linestyle="-", color="blue", alpha=0.8)
    ax1.set_xlabel("X position (m)")
    ax1.set_ylabel("Y position (m)")
    ax1.set_title("Top-down view (X-Y)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis("equal")

    # X-position over time -- this is where the slip error and EKF correction
    # are actually visible, since the robot travels along X and slip causes
    # it to fall behind in X, not drift sideways in Y.
    ax2.plot(data["t"], data["x_true"], label="Ground truth", linewidth=2, color="black")
    ax2.plot(data["t"], result["raw_x"], label="Slipping (unfused) odometry",
              linestyle="--", color="red", alpha=0.8)
    ax2.plot(data["t"], x_est[:, 0], label="Fused EKF estimate",
              linestyle="-", color="blue", alpha=0.8)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("X position (m)")
    ax2.set_title("X position vs time -- shows slip error and EKF correction")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved {out_path}")


def make_run_log(result, out_path="output/run_log.txt"):
    data = result["data"]
    P_hist = result["P_hist"]
    warnings = result["warnings"]

    with open(out_path, "w") as f:
        f.write("=== Pace Robotics Take-Home: Run Log ===\n")
        f.write("Simulation substitutes a Python-only kinematic model for Gazebo\n")
        f.write("(see README for rationale). All math implemented per spec.\n\n")

        f.write("--- Pose covariance trace (subsampled every 2s) ---\n")
        for i in range(0, len(data["t"]), 40):  # every ~2s at dt=0.05
            trace = np.trace(P_hist[i][:2, :2])
            f.write(f"t={data['t'][i]:6.2f}s  cov_trace={trace:.5f}\n")

        f.write(f"\n--- Degeneracy warnings ({len(warnings)} total) ---\n")
        for t, eig, axis in warnings[::20]:  # subsample for readability
            f.write(f"t={t:6.2f}s  LOCALIZATION_DEGENERACY_WARNING axis={axis} eig={eig:.4f}\n")

        f.write(f"\n--- Ghost-obstacle clear events ({len(result['grid'].clear_events)} total) ---\n")
        for t, y in result["grid"].clear_events:
            f.write(f"t={t:6.2f}s  cleared stale OCCUPIED cell at y={y:.2f}m\n")

    print(f"Saved {out_path}")


if __name__ == "__main__":
    os.makedirs("output", exist_ok=True)
    result = run_full_pipeline()
    make_trajectory_plot(result)
    make_run_log(result)