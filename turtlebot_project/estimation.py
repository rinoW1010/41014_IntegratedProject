import numpy as np
from .model import motion, wrap

def jacobian(function, point, angle_row=2):
    """
    Compute the Jacobian of a function at a point using finite differences.
    """
    point = np.asarray(point, dtype=float)
    J = np.zeros((3, len(point)))
    epsilon = 1e-5
    for j in range(len(point)):
        change = np.zeros_like(point)
        change[j] = epsilon
        difference = function(point + change) - function(point - change)
        difference[angle_row] = wrap(difference[angle_row])
        J[:, j] = difference / (2 * epsilon)
    return J

class PoseEKF:
    def __init__(self):
        self.x = None
        self.P = None
        self.last_nis = np.nan
        self.accepted = False

    def initialise(self, pose, covariance):
        self.x = pose.copy()
        self.x[2] = wrap(self.x[2])
        self.P = covariance.copy()

    def predict(self, wheel_distances):
        if self.x is None:
            return
        previous = self.x.copy()
        F = jacobian(lambda state: motion(state, wheel_distances), previous)
        G = jacobian(lambda wheels: motion(previous, wheels), wheel_distances)

        # Independent wheel displacement noise. Encoder error + slip allowance

        std = 0.0005 + 0.05*np.abs(wheel_distances)
        Qw = np.diag(std**2)
        Qfloor = np.diag([1e-6, 1e-6, np.deg2rad(0.1)**2])
        self.x = motion(previous, wheel_distances)
        self.P = F @ self.P @ F.T + G @ Qw @ G.T + Qfloor
        self.P = (self.P + self.P.T) / 2  # Ensure symmetry

    def update(self, pose, covariance):
        self.accepted = False
        if self.x is None:
            self.initialise(pose, covariance)
            self.accepted = True
            return
        innovation = pose - self.x
        innovation[2] = wrap(innovation[2])
        S = self.P + covariance # H = identity for measure planar pose
        self.last_nis = float(innovation @ np.linalg.solve(S, innovation))
        if self.last_nis > 11.34:
            return False
        K = np.linalg.solve(S, self.P.T).T
        self.x += K @ innovation
        self.x[2] = wrap(self.x[2])
        A = np.eye(3) - K
        self.P = A @ self.P @ A.T + K @ covariance @ K.T
        self.P = (self.P + self.P.T) / 2  # Ensure symmetry
        self.accepted = True
        return True