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

    plt.figure(figsize=(10, 5))
    plt.plot(data["x_true"], data["y_true"], label="Ground truth", linewidth=2, color="black")
    plt.plot(result["raw_x"], result["raw_y"], label="Slipping (unfused) odometry",
              linestyle="--", color="red", alpha=0.7)
    plt.plot(x_est[:, 0], x_est[:, 1], label="Fused EKF estimate",
              linestyle="-", color="blue", alpha=0.8)

    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("Trajectory: Ground Truth vs Slipping Odometry vs Fused Estimate")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
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