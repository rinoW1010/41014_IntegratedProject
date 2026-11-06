from pathlib import Path
import numpy as np
from turtlebot_project.estimation import PoseEKF
from turtlebot_project.model import DT, motion, wrap
from turtlebot_project.simulation import Simulation
from turtlebot_project.vision import MarkerCamera

sim = Simulation(Path("outputs/check6"), start_pose=(1.85, -0.30, np.deg2rad(15)))
camera = MarkerCamera()
ekf = PoseEKF()
errors = []
updates = 0
try:
    first = camera.detect(sim.image())
    assert first is not None, "check that the camera can see the marker from the start"
    ekf.update(first.pose, first.pose_covariance)
    odom = ekf.x.copy()
    for k in range(60):
        wheels = sim.step((0.05, 0.))
        ekf.predict(wheels)
        odom = motion(odom, wheels)
        before = np.trace(ekf.P)
        if k == 15:
            dropout_start = before
        if k == 34:
            assert before > dropout_start, "uncertainty should grow when the camera updates stop"
        if not 15 <= k < 35:
            d = camera.detect(sim.image())
            if d is not None:
                ekf.update(d.pose, d.pose_covariance)
                if ekf.accepted:
                    updates += 1
                    assert np.trace(ekf.P) <= before + 1e-12
        assert np.allclose(ekf.P, ekf.P.T)
        assert np.linalg.eigvalsh(ekf.P).min() >= -1e-12
        truth = sim.evaluation_pose()
        errors.append([np.linalg.norm(est[:2]-truth[:2]) for est in (odom, ekf.x)])
    assert updates > 0 and not sim.collision
    print("accepted camera updates:", updates, "wheel-only / EKF position RMSE (m):",
          np.sqrt(np.mean(np.square(errors), axis=0)))
    probe = PoseEKF()
    probe.initialise(np.array([0., 0., np.deg2rad(179)]), np.eye(3)*0.01)
    probe.update(np.array([0., 0., np.deg2rad(-179)]), np.eye(3)*0.01)
    assert probe.accepted and abs(probe.x[2]) > np.deg2rad(175)
finally:
    sim.close()
