"""Work out the differential-drive motion using radians and metres."""
import numpy as np
from .config import MARKER_POSITION

WHEEL_RADIUS = 0.033
TRACK_WIDTH = 0.160
CAMERA_FORWARD = 0.060
MARKER_XY = MARKER_POSITION[:2]
MARKER_SIDE = 0.24
DT = 0.05


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def motion(state, wheel_distances):
    left, right = wheel_distances
    distance = (left + right) / 2
    turn = (right - left) / TRACK_WIDTH
    mid = state[2] + turn / 2
    result = state + np.array([distance * np.cos(mid), distance * np.sin(mid), turn])
    result[2] = wrap(result[2])
    return result




def marker_measurement(state):
    """work out the range and bearing from the camera to the marker centre."""
    camera = state[:2] + CAMERA_FORWARD * np.array([np.cos(state[2]), np.sin(state[2])])
    delta = MARKER_XY - camera
    return np.array([np.linalg.norm(delta), wrap(np.arctan2(delta[1], delta[0]) - state[2])])




def wheel_rates(v, omega):
    rates = np.array([v - TRACK_WIDTH * omega / 2, v + TRACK_WIDTH * omega / 2]) / WHEEL_RADIUS
    # Scale both wheel speeds together so the turning radius stays the same.
    return rates / max(1.0, np.max(np.abs(rates)) / 6.5)
