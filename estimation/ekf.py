"""
estimation/ekf.py
Extended Kalman Filter fusing:
  - process model: differential-drive kinematics driven by noisy odometry
    velocity (v_recorded) and noisy IMU heading rate (omega_measured)
  - measurement model: synthetic scan-matching pseudo-measurement (x, y, theta)
    with R = inverse of the per-step information matrix from scan_info.py

State vector: [x, y, theta]
"""

import numpy as np


class EKF:
    def __init__(self, x0, P0):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)

    def predict(self, v, omega, dt, Q):
        x, y, theta = self.x
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + omega * dt

        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0, 1]
        ])

        self.x = np.array([x_new, y_new, theta_new])
        self.P = F @ self.P @ F.T + Q

    def update(self, z, R):
        H = np.eye(3)
        residual = z - self.x
        residual[2] = np.arctan2(np.sin(residual[2]), np.cos(residual[2]))  # wrap angle
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ residual
        self.P = (np.eye(3) - K @ H) @ self.P


def run_ekf(data, v_recorded, omega_measured, pseudo_meas, info_matrices, dt):
    n = len(data["t"])
    ekf = EKF(
        x0=[data["x_true"][0], data["y_true"][0], data["theta_true"][0]],
        P0=np.eye(3) * 0.1
    )
    Q = np.diag([0.02, 0.02, 0.01]) * dt  # process noise

    x_est = np.zeros((n, 3))
    P_hist = np.zeros((n, 3, 3))

    for i in range(n):
        ekf.predict(v_recorded[i], omega_measured[i], dt, Q)
        R = np.linalg.inv(info_matrices[i] + np.eye(3) * 1e-6)
        ekf.update(pseudo_meas[i], R)
        x_est[i] = ekf.x
        P_hist[i] = ekf.P

    return x_est, P_hist


if __name__ == "__main__":
    import sys, os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from sim.plant import simulate_ground_truth
    from noise.inject import inject_odometry_slip, inject_imu_bias_drift
    from estimation.scan_info import scan_information_matrix, generate_pseudo_measurements

    data = simulate_ground_truth()
    dt = data["t"][1] - data["t"][0]
    rng = np.random.default_rng(seed=42)

    v_recorded, s = inject_odometry_slip(data["v_true"], data["x_true"], dt, rng=rng)
    omega_measured, b = inject_imu_bias_drift(data["omega_true"], dt, rng=rng)

    info = scan_information_matrix(data["x_true"])
    pseudo_meas = generate_pseudo_measurements(
        data["x_true"], data["y_true"], data["theta_true"], info, rng=rng
    )

    x_est, P_hist = run_ekf(data, v_recorded, omega_measured, pseudo_meas, info, dt)

    final_err = np.linalg.norm(x_est[-1, :2] - [data["x_true"][-1], data["y_true"][-1]])
    print(f"Final estimated position: ({x_est[-1,0]:.3f}, {x_est[-1,1]:.3f})")
    print(f"Final true position: ({data['x_true'][-1]:.3f}, {data['y_true'][-1]:.3f})")
    print(f"Final position error: {final_err:.3f} m")
    print(f"Mean position covariance trace: {np.mean([np.trace(P[:2,:2]) for P in P_hist]):.4f}")