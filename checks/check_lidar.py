import numpy as np
from turtlebot_project import simulation
from turtlebot_project.control import safe_command
simulation.LIDAR_SIGMA = 0.
s = simulation.Simulation("outputs/block", start_pose=(0.775, 0.425, np.pi/2))
try:
    a, r = s.lidar()
    command = safe_command((0.08, 0.), a, r)
    print(r[np.argmin(abs(a))], command)
    assert command == (0., 0.)
finally:
    s.close()
