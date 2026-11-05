"""find the ArUco marker in the image and work out its range and bearing."""
from dataclasses import dataclass
import cv2
import numpy as np
from .model import MARKER_SIDE, CAMERA_FORWARD, wrap
from .config import MARKER_POSITION

@dataclass
class Detection:
    measurement: np.ndarray
    covariance: np.ndarray
    corners: np.ndarray
    reprojection_error: float
    pose: np.ndarray
    pose_covariance: np.ndarray

def transform(rotation, translation):
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = np.asarray(translation).reshape(3)
    return T

def robot_pose_from_marker(rvec, tvec):
    R_CM, _ = cv2.Rodrigues(rvec)
    T_CM = transform(R_CM, tvec)

    # Columns: marker x, y, z axes expressed in world coords
    R_WM = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    T_WM = transform(R_WM, MARKER_POSITION)

    # OpenCV camera axes: right, down, forward
    # Express those axes in robot coordinates: -y, -z, +x
    R_BC = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    T_BC = transform(R_BC, [CAMERA_FORWARD, 0, 0.15])

    T_WB = T_WM @ np.linalg.inv(T_CM) @ np.linalg.inv(T_BC)
    return np.array([T_WB[0, 3], T_WB[1, 3], wrap(np.arctan2(T_WB[1, 0], T_WB[0, 0]))])


class MarkerCamera:
    def __init__(self, width=640, height=480, fovy=50.0):
        focal = height / (2 * np.tan(np.deg2rad(fovy) / 2))
        self.K = np.array([[focal, 0, (width-1)/2], [0, focal, (height-1)/2], [0, 0, 1.]])
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), parameters)
        s = MARKER_SIDE / 2
        self.points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)

    def detect(self, rgb):
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        if ids is None:
            return None
        matches = np.flatnonzero(ids.ravel() == 7)
        if not len(matches):
            return None
        pixels = corners[matches[0]].reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(self.points, pixels, self.K, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or not np.isfinite(tvec).all() or not np.isfinite(rvec).all() or tvec[2, 0] <= 0:
            return None
        projected, _ = cv2.projectPoints(self.points, rvec, tvec, self.K, np.zeros(5))
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels)**2, axis=1))))
        if not np.isfinite(error) or error > 2.5:
            return None
        tx, _, tz = tvec.ravel()
        distance = float(np.hypot(tx, tz))
        measurement = np.array([distance, np.arctan2(-tx, tz)])
        # start with a cautious noise estimate and check it against measurements before submission.
        covariance = np.diag([(0.008 + 0.006 * distance**2)**2, np.deg2rad(0.4 + 0.15*distance)**2])
        
        pose = robot_pose_from_marker(rvec, tvec)
        if not np.isfinite(pose).all():
            return None

        # Starting uncertainty assumptions. Replace using calibration results if available.
        position_std = 0.04 + 0.015 * distance**2
        heading_std = np.deg2rad(3 + distance)
        pose_covariance = np.diag([position_std**2, position_std**2, heading_std**2])

        return Detection(measurement, covariance, pixels, error, pose, pose_covariance)