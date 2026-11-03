"""find the ArUco marker in the image and work out its range and bearing."""
from dataclasses import dataclass
import cv2
import numpy as np
from .model import MARKER_SIDE


@dataclass
class Detection:
    measurement: np.ndarray
    covariance: np.ndarray
    corners: np.ndarray
    reprojection_error: float


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
        return Detection(measurement, covariance, pixels, error)
