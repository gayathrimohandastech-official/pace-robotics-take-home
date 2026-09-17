"""
estimation/scan_info.py
Synthetic scan-matching Information Matrix proxy (Python-only fallback --
no real Gazebo/LiDAR available; documented substitution, see README).

Models the physical effect real scan-matching would show in this corridor:
flat parallel walls give strong Y/theta constraint but near-zero constraint
along the corridor's long axis (X) -- classic axial degeneracy. The wall
panel at the corridor midpoint locally restores an X constraint.
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class ScanInfoConfig:
    info_y: float = 400.0
    info_theta: float = 2500.0
    info_x_baseline: float = 0.5
    info_x_peak: float = 300.0
    panel_x: float = 10.0
    panel_influence_radius: float = 1.5


def scan_information_matrix(x_true: np.ndarray, cfg: ScanInfoConfig = ScanInfoConfig()):
    n = len(x_true)
    info = np.zeros((n, 3, 3))
    dist_to_panel = np.abs(x_true - cfg.panel_x)
    feature_gain = np.exp(-0.5 * (dist_to_panel / cfg.panel_influence_radius) ** 2)
    info_xx = cfg.info_x_baseline + feature_gain * (cfg.info_x_peak - cfg.info_x_baseline)
    for i in range(n):
        info[i, 0, 0] = info_xx[i]
        info[i, 1, 1] = cfg.info_y
        info[i, 2, 2] = cfg.info_theta
    return info


def smallest_eigenvalue_along_axis(info_matrices: np.ndarray):
    n = info_matrices.shape[0]
    eigvals_min = np.zeros(n)
    for i in range(n):
        eigvals_min[i] = np.linalg.eigvalsh(info_matrices[i]).min()
    return eigvals_min


def generate_pseudo_measurements(x_true, y_true, theta_true, info_matrices, rng=None):
    """Synthetic scan-matching output: true pose + noise scaled by 1/sqrt(information)."""
    if rng is None:
        rng = np.random.default_rng()
    n = len(x_true)
    z = np.zeros((n, 3))
    for i in range(n):
        sigma = 1.0 / np.sqrt(np.diag(info_matrices[i]) + 1e-9)
        noise = rng.normal(0, sigma)
        z[i] = [x_true[i] + noise[0], y_true[i] + noise[1], theta_true[i] + noise[2]]
    return z


if __name__ == "__main__":
    import sys, os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from sim.plant import simulate_ground_truth

    data = simulate_ground_truth()
    info = scan_information_matrix(data["x_true"])
    eig_min = smallest_eigenvalue_along_axis(info)

    print(f"Min eigenvalue overall: {eig_min.min():.3f}")
    print(f"Max eigenvalue overall: {eig_min.max():.3f}")
    frac_degenerate = np.mean(eig_min < 5.0)
    print(f"Fraction of trajectory flagged degenerate (eig < 5.0): {frac_degenerate:.2%}")