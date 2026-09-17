"""
estimation/mapping.py
Lifelong map update / ghost-obstacle clearing for the moving wall panel.

No real LiDAR/Gazebo available (Python-only fallback, see README), so this
simulates a column of range readings at the panel's X location, representing
what a real 3D LiDAR sweep would report about occupancy at each Y position
there. At each timestep, whichever Y-cells the panel currently occupies are
marked OCCUPIED, and every other in-range Y-cell is marked FREE -- which is
exactly what clears a "ghost obstacle" left behind after the panel teleports
to a new Y position.
"""

import numpy as np


class GhostClearingGrid:
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 1

    def __init__(self, y_min=-2.5, y_max=2.5, resolution=0.1, panel_x=10.0,
                 panel_half_width=0.4, sensor_range=5.0):
        self.resolution = resolution
        self.panel_x = panel_x
        self.panel_half_width = panel_half_width
        self.sensor_range = sensor_range
        self.y_bins = np.arange(y_min, y_max, resolution)
        self.grid = np.full(len(self.y_bins), self.UNKNOWN, dtype=int)
        self.clear_events = []  # (time, y_bin) log of ghost clears

    def observe(self, t, robot_x, panel_offset):
        """
        One simulated LiDAR sweep at time t. If the robot is within
        sensor_range of the panel's X location, update the grid: cells
        matching the panel's current footprint go OCCUPIED, all other
        in-range cells go FREE (clearing stale ghost cells).
        """
        if abs(robot_x - self.panel_x) > self.sensor_range:
            return  # panel not observable yet

        for i, y in enumerate(self.y_bins):
            in_panel_footprint = abs(y - panel_offset) <= self.panel_half_width
            new_state = self.OCCUPIED if in_panel_footprint else self.FREE

            prev_state = self.grid[i]
            if prev_state == self.OCCUPIED and new_state == self.FREE:
                # A previously-occupied cell is no longer observed as
                # occupied -- a "ghost obstacle" being cleared after the
                # panel moved away.
                self.clear_events.append((t, float(y)))

            self.grid[i] = new_state

    def occupied_y_positions(self):
        return self.y_bins[self.grid == self.OCCUPIED]


if __name__ == "__main__":
    import sys, os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from sim.plant import simulate_ground_truth

    data = simulate_ground_truth()
    grid = GhostClearingGrid()

    for i in range(len(data["t"])):
        grid.observe(data["t"][i], data["x_true"][i], data["wall_offset"][i])

    print(f"Total ghost-clear events: {len(grid.clear_events)}")
    for t, y in grid.clear_events[:10]:
        print(f"  t={t:.2f}s  cleared stale OCCUPIED cell at y={y:.2f}m")
    if len(grid.clear_events) > 10:
        print(f"  ... and {len(grid.clear_events) - 10} more")

    print(f"Final occupied Y positions: {grid.occupied_y_positions()}")