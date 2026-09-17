"""
estimation/diagnostics.py
Localization degeneracy diagnostic: tracks the scan-matching information
matrix's smallest eigenvalue (per Task 2's spec: "calculate the Information
Matrix of scan matching to track axis degeneracy"), and logs a
LOCALIZATION_DEGENERACY_WARNING whenever it drops below threshold, meaning
that axis is weakly constrained by the environment.

The EKF's own fused pose covariance (P_hist) is reported separately as the
"running pose covariance matrix" deliverable -- it is NOT used for this
warning, because a Kalman filter naturally averages down its own covariance
over many measurements even when each individual measurement is weakly
informative, which would mask real degeneracy rather than reveal it.
"""

import numpy as np


def detect_degeneracy(info_matrices: np.ndarray, t: np.ndarray, threshold: float = 5.0):
    """
    info_matrices: (n, 3, 3) scan-matching information matrices.
    threshold: eigenvalue below which that axis is considered weakly
               constrained ("degenerate").

    Returns:
        warnings: list of (time, eigenvalue, weak_axis) tuples
        eig_min_series: per-step smallest eigenvalue (position sub-block),
                         useful for plotting
    """
    n = len(t)
    eig_min_series = np.zeros(n)
    warnings = []

    for i in range(n):
        pos_info = info_matrices[i][:2, :2]  # x,y sub-block
        eigvals, eigvecs = np.linalg.eigh(pos_info)
        min_eig = eigvals[0]
        weak_vec = eigvecs[:, 0]
        eig_min_series[i] = min_eig

        if min_eig < threshold:
            axis = "X" if abs(weak_vec[0]) > abs(weak_vec[1]) else "Y"
            warnings.append((t[i], min_eig, axis))

    return warnings, eig_min_series


def summarize_warnings(warnings):
    if not warnings:
        print("No degeneracy warnings triggered.")
        return
    print(f"LOCALIZATION_DEGENERACY_WARNING triggered {len(warnings)} times.")
    first_t, first_eig, first_axis = warnings[0]
    last_t, last_eig, last_axis = warnings[-1]
    print(f"  First: t={first_t:.2f}s, axis={first_axis}, eig={first_eig:.4f}")
    print(f"  Last:  t={last_t:.2f}s, axis={last_axis}, eig={last_eig:.4f}")
    weakest_eig = min(w[1] for w in warnings)
    print(f"  Weakest eigenvalue: {weakest_eig:.4f}")


if __name__ == "__main__":
    import sys, os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from sim.plant import simulate_ground_truth
    from noise.inject import inject_odometry_slip, inject_imu_bias_drift
    from estimation.scan_info import scan_information_matrix, generate_pseudo_measurements
    from estimation.ekf import run_ekf

    data = simulate_ground_truth()
    dt = data["t"][1] - data["t"][0]
    rng = np.random.default_rng(seed=42)

    v_recorded, s = inject_odometry_slip(data["v_true"], data["x_true"], dt, rng=rng)
    omega_measured, b = inject_imu_bias_drift(data["omega_true"], dt, rng=rng)
    info = scan_information_matrix(data["x_true"])
    pseudo_meas = generate_pseudo_measurements(data["x_true"], data["y_true"], data["theta_true"], info, rng=rng)
    x_est, P_hist = run_ekf(data, v_recorded, omega_measured, pseudo_meas, info, dt)

    warnings, eig_series = detect_degeneracy(info, data["t"])
    summarize_warnings(warnings)
    print(f"Eigenvalue range: {eig_series.min():.4f} to {eig_series.max():.4f}")
    print(f"Fraction of trajectory degenerate: {len(warnings)/len(data['t']):.2%}")
    print(f"EKF's own covariance trace range (separate output): "
          f"{min(np.trace(P[:2,:2]) for P in P_hist):.5f} to "
          f"{max(np.trace(P[:2,:2]) for P in P_hist):.5f}")