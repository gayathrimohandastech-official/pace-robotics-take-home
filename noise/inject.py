"""
noise/inject.py
Implements the two noise/degradation models specified in the assignment:

A. Linear Odometry Slip Model (dynamic intermittent scale slip)
   v_recorded(t) = s(t) * v_true(t) + w_v(t)
   s(t) = 0.70 for 10 <= Position_X <= 15, else 1.00

B. IMU Sensor Deterioration Model (random walk + bias drift)
   omega_measured(t) = omega_true(t) + b(t) + eta_g(t)
   b_dot(t) = eta_b(t)
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class NoiseConfig:
    sigma_v: float = 0.01        # velocity measurement white noise std (m/s)
    sigma_g: float = 0.005       # gyro white noise std (rad/s) -- angle random walk
    sigma_b: float = 0.0008      # gyro bias random-walk std (rad/s per sqrt(s)) -- rate random walk
    slip_x_start: float = 10.0   # meters
    slip_x_end: float = 15.0     # meters
    slip_factor: float = 0.70


def slip_scale(x_true: np.ndarray, cfg: NoiseConfig) -> np.ndarray:
    """s(t) as a function of true X position: 0.70 inside the dusty patch, else 1.00."""
    s = np.ones_like(x_true)
    mask = (x_true >= cfg.slip_x_start) & (x_true <= cfg.slip_x_end)
    s[mask] = cfg.slip_factor
    return s


def inject_odometry_slip(v_true: np.ndarray, x_true: np.ndarray, dt: float,
                          cfg: NoiseConfig = NoiseConfig(), rng: np.random.Generator = None):
    """
    Returns v_recorded(t) = s(t) * v_true(t) + w_v(t)
    Also returns s(t) itself, useful for diagnostics/plots.
    """
    if rng is None:
        rng = np.random.default_rng()
    s = slip_scale(x_true, cfg)
    w_v = rng.normal(0, cfg.sigma_v, size=v_true.shape)
    v_recorded = s * v_true + w_v
    return v_recorded, s


def inject_imu_bias_drift(omega_true: np.ndarray, dt: float,
                           cfg: NoiseConfig = NoiseConfig(), rng: np.random.Generator = None):
    """
    Returns omega_measured(t) = omega_true(t) + b(t) + eta_g(t)
    where b(t) evolves as a random walk: b_dot(t) = eta_b(t)
    (integrated as b[i] = b[i-1] + eta_b * sqrt(dt) each step)
    """
    if rng is None:
        rng = np.random.default_rng()
    n = len(omega_true)
    b = np.zeros(n)
    eta_g = rng.normal(0, cfg.sigma_g, size=n)
    eta_b = rng.normal(0, cfg.sigma_b, size=n)

    for i in range(1, n):
        b[i] = b[i - 1] + eta_b[i] * np.sqrt(dt)

    omega_measured = omega_true + b + eta_g
    return omega_measured, b


if __name__ == "__main__":
    # Sanity check using the plant simulation
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from sim.plant import simulate_ground_truth

    data = simulate_ground_truth()
    rng = np.random.default_rng(seed=42)

    v_recorded, s = inject_odometry_slip(data["v_true"], data["x_true"], data["t"][1] - data["t"][0], rng=rng)
    omega_measured, b = inject_imu_bias_drift(data["omega_true"], data["t"][1] - data["t"][0], rng=rng)

    print(f"Slip factor range: {s.min():.2f} to {s.max():.2f}")
    print(f"Slip active for {np.sum(s < 1.0)} of {len(s)} steps")
    print(f"Mean recorded velocity: {v_recorded.mean():.4f} (true: {data['v_true'].mean():.4f})")
    print(f"Final gyro bias drift: {b[-1]:.6f} rad/s")
    print(f"Gyro measurement noise std check: {omega_measured.std():.6f}")