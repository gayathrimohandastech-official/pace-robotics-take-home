"""
scripts/run_pipeline.py
End-to-end integration run: plant -> noise -> scan info -> EKF -> diagnostics
-> mapping, all in one place. This is the sanity check before generating the
final video/plots/logs (Step 9).
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from sim.plant import simulate_ground_truth
from noise.inject import inject_odometry_slip, inject_imu_bias_drift
from estimation.scan_info import scan_information_matrix, generate_pseudo_measurements, smallest_eigenvalue_along_axis
from estimation.ekf import run_ekf
from estimation.diagnostics import detect_degeneracy, summarize_warnings
from estimation.mapping import GhostClearingGrid


def run_full_pipeline(seed=42):
    data = simulate_ground_truth()
    dt = data["t"][1] - data["t"][0]
    rng = np.random.default_rng(seed=seed)

    v_recorded, slip_s = inject_odometry_slip(data["v_true"], data["x_true"], dt, rng=rng)
    omega_measured, gyro_bias = inject_imu_bias_drift(data["omega_true"], dt, rng=rng)

    info = scan_information_matrix(data["x_true"])
    pseudo_meas = generate_pseudo_measurements(
        data["x_true"], data["y_true"], data["theta_true"], info, rng=rng
    )

    x_est, P_hist = run_ekf(data, v_recorded, omega_measured, pseudo_meas, info, dt)

    warnings, eig_min_series = detect_degeneracy(info, data["t"])

    grid = GhostClearingGrid()
    for i in range(len(data["t"])):
        grid.observe(data["t"][i], data["x_true"][i], data["wall_offset"][i])

    # Also compute the "raw" (unfused) odometry-integrated trajectory, for
    # comparison in the final plot (GT vs slipping odom vs fused estimate).
    raw_x = np.cumsum(v_recorded * dt)
    raw_x[0] = data["x_true"][0]
    raw_y = np.zeros_like(raw_x)  # straight-line assumption, theta stays ~0

    return {
        "data": data,
        "dt": dt,
        "v_recorded": v_recorded,
        "slip_s": slip_s,
        "omega_measured": omega_measured,
        "gyro_bias": gyro_bias,
        "info": info,
        "pseudo_meas": pseudo_meas,
        "x_est": x_est,
        "P_hist": P_hist,
        "warnings": warnings,
        "eig_min_series": eig_min_series,
        "raw_x": raw_x,
        "raw_y": raw_y,
        "grid": grid,
    }


if __name__ == "__main__":
    result = run_full_pipeline()
    data = result["data"]
    x_est = result["x_est"]

    print("=== Integration test: full pipeline ===")
    print(f"Steps simulated: {len(data['t'])}")
    print(f"Final GT position: ({data['x_true'][-1]:.3f}, {data['y_true'][-1]:.3f})")
    print(f"Final fused estimate: ({x_est[-1,0]:.3f}, {x_est[-1,1]:.3f})")
    print(f"Final raw (slipping) odom X: {result['raw_x'][-1]:.3f}")
    final_err = np.linalg.norm(x_est[-1, :2] - [data['x_true'][-1], data['y_true'][-1]])
    raw_err = abs(result['raw_x'][-1] - data['x_true'][-1])
    print(f"Fused estimate error: {final_err:.3f} m")
    print(f"Raw slipping-odom error: {raw_err:.3f} m  <-- fused should beat this")
    summarize_warnings(result["warnings"])
    print(f"Ghost-clear events: {len(result['grid'].clear_events)}")
    print("=== All stages ran without error ===")