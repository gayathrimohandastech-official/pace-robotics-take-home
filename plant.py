@'
"""
sim/plant.py
Ground-truth kinematic simulation of a differential-drive robot traveling
down a 20m featureless corridor, with a wall panel that shifts sideways
by 1.5m at 15-second intervals.

This substitutes for a full Gazebo/Isaac Sim environment due to a fresh-
install time constraint on Windows (documented in README). All required
math (noise injection, EKF fusion, degeneracy diagnostics) is implemented
faithfully on top of this simulated ground truth.
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class PlantConfig:
    dt: float = 0.05          # 20 Hz simulation step
    v_true: float = 0.4       # constant forward velocity (m/s)
    corridor_length: float = 20.0   # meters
    wall_shift_interval: float = 15.0  # seconds
    wall_shift_magnitude: float = 1.5  # meters


def simulate_ground_truth(cfg: PlantConfig = PlantConfig()):
    duration = cfg.corridor_length / cfg.v_true
    t = np.arange(0, duration, cfg.dt)
    n = len(t)

    x_true = cfg.v_true * t
    y_true = np.zeros(n)
    theta_true = np.zeros(n)
    v_true = np.full(n, cfg.v_true)
    omega_true = np.zeros(n)

    wall_offset = np.zeros(n)
    direction = 1
    shift_count = 0
    for i, ti in enumerate(t):
        current_shift_number = int(ti // cfg.wall_shift_interval)
        if current_shift_number > shift_count:
            shift_count = current_shift_number
            direction *= -1
        wall_offset[i] = 0.0 if shift_count == 0 else direction * cfg.wall_shift_magnitude

    return {
        "t": t,
        "x_true": x_true,
        "y_true": y_true,
        "theta_true": theta_true,
        "v_true": v_true,
        "omega_true": omega_true,
        "wall_offset": wall_offset,
    }


if __name__ == "__main__":
    data = simulate_ground_truth()
    print(f"Simulated {len(data['t'])} steps over {data['t'][-1]:.1f}s")
    print(f"Final X position: {data['x_true'][-1]:.2f}m")
    print(f"Wall offset changes at: ", end="")
    prev = data["wall_offset"][0]
    for i, w in enumerate(data["wall_offset"]):
        if w != prev:
            print(f"t={data['t'][i]:.2f}s (->{w}m)", end="  ")
            prev = w
    print()
'@ | Set-Content -Path sim\plant.py -Encoding utf8