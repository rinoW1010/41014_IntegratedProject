"""
Shared settings and geometry for the TurtleBot project.
"""

import numpy as np

# Inner wall surfaces from the existing scene
BOUNDS = (-2.55, 3.15, -1.95, 1.95) # (x_min, x_max, y_min, y_max)

# Rectangles: xmin, xmax, ymin, ymax. Keep dock approach clear
OBSTACLES = (
    (-0.65, -0.15, -0.55, 0.25),
    (0.55, 1, 0.65, 1.10)
)

# Physical marker board
BOARD_RECT = (2.995, 3.005, -0.18, 0.18)

ROBOT_RADIUS = 0.12 
CLEARANCE = 0.10
GRID = 0.10
LIDAR_MAX = 6.0
LIDAR_SIGMA = 0.01
LIDAR_ANGLES = np.linspace(-np.pi, np.pi, 180, endpoint=False)
MAX_V = 0.12
MAX_W = 0.55

# Existing marker centre is at world x=2.993, y=0, z=0.22 with a thickness of 0.002mm
MARKER_POSITION = np.array([2.992, 0, 0.22])
PRE_DOCK = np.array([2.05, 0])
DOCK_POSE = np.array([2.55, 0, 0])
DOCK_POSITION_TOL = 0.04
DOCK_HEADING_TOL = np.deg2rad(5)
DOCK_HOLD = 1
TRIAL_LIMIT = 180

def position_is_free(xy, margin=ROBOT_RADIUS + CLEARANCE):
    """Check if a position is free of obstacles."""
    x, y = xy
    xmin, xmax, ymin, ymax = BOUNDS
    # Checks that position is clear of the outer walls
    if not (xmin + margin < x < xmax -margin and 
            ymin + margin < y < ymax - margin):
        return False
    # Checks that the position is clear of the inner obstacles and the marker board
    for x0, x1, y0, y1 in (*OBSTACLES, BOARD_RECT):
        if (x0 - margin <= x <= x1 + margin and 
            y0 - margin <= y <= y1 + margin):
            return False
    return True