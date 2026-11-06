import numpy as np
from .model import wrap
from .config import MAX_V, MAX_W, PRE_DOCK, DOCK_POSE

START = (0.3, 1.58, np.deg2rad(-30))
ROUTE = np.array([[1.3, 1.5], [1.65, 0.65], PRE_DOCK])
SPEED = 0.08

def safe_command(command, angles, ranges):
    """
    check 25 degrees either side of straight ahead. A reading under 0.23 m
    stops forward motion. The robot can still turn on the spot.
    """
    front = ranges[np.abs(angles) <= np.deg2rad(25)]
    blocked = not front.size or not np.isfinite(front).all() or front.min() < 0.23
    return (0, 0) if blocked and command[0] > 0 else command

def track(pose, target, tangent, speed, heading_gain=1.8):
    delta = target - pose[:2]
    c, s = np.cos(pose[2]), np.sin(pose[2])
    along = c*delta[0] + s*delta[1]
    across = -s*delta[0] + c*delta[1]
    heading = wrap(tangent-pose[2])
    v = np.clip(speed*np.cos(heading) + 0.9*along, 0, MAX_V)
    w = np.clip(heading_gain*heading + 2.8*across, -MAX_W, MAX_W)
    return v, w