import numpy as np
from turtlebot_project.model import motion, wrap, wheel_rates


def test_wheel_commands_and_motion():
    assert np.all(wheel_rates(.1,0)>0)
    rates=wheel_rates(0,.3)
    assert rates[0]<0<rates[1]
    np.testing.assert_allclose(motion(np.zeros(3),np.array([.1,.1])),[.1,0,0])


def test_physics_encoder_agreement_and_real_rendered_detection(tmp_path):
    from turtlebot_project.simulation import Simulation
    from turtlebot_project.vision import MarkerCamera
    sim = Simulation(tmp_path, start_pose=(1.85, -0.30, np.deg2rad(15)))
    try:
        start = sim.evaluation_pose()
        detection = MarkerCamera().detect(sim.image())
        assert detection is not None
        error = detection.pose - start
        assert np.linalg.norm(error[:2]) < 0.05
        assert abs(wrap(error[2])) < np.deg2rad(3)
        odo = start.copy()
        for _ in range(40):
            odo = motion(odo, sim.step([0.12, 0]))
        true = sim.evaluation_pose()
        assert true[0]-start[0] > 0.15
        assert np.linalg.norm(true[:2]-odo[:2]) < 0.015
        for _ in range(40):
            sim.step([0, 0.2])
        assert wrap(sim.evaluation_pose()[2]-true[2]) > 0.15
        assert not sim.collision
    finally:
        sim.close()
