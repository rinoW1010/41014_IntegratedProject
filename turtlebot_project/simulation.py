"""
Run the physics and get the simulated sensor readings.

Keep the simulator state here and pass only encoder readings, camera images
to the rest of the code.
"""
from pathlib import Path
import cv2
import mujoco
import numpy as np
from .model import DT, WHEEL_RADIUS, wheel_rates
from .scene import create_scene
from .config import (BOUNDS, position_is_free, LIDAR_ANGLES, LIDAR_MAX, LIDAR_SIGMA)

class Simulation:
    def __init__(self, output, seed=1, encoder_noise=0.25, traction=0.9, start_y=-0.65, random_start=False):
        self._model = mujoco.MjModel.from_xml_path(str(create_scene(Path(output) / 'scene.xml').resolve()))
        self._data = mujoco.MjData(self._model)

        # Enable random starting position and heading, but keep the robot within the bounds and away from obstacles.
        spawn_seed, sensor_seed = np.random.SeedSequence(seed).spawn(2)
        spawn_rng = np.random.default_rng(spawn_seed)
        self._rng = np.random.default_rng(sensor_seed)
        self._data.qpos[1] = start_y
        if random_start:
            xmin, xmax, ymin, ymax = BOUNDS
            for _ in range (10000):
                xy = spawn_rng.uniform([xmin, ymin], [xmax, ymax])
                if position_is_free(xy):
                    break
            else:
                raise RuntimeError("Failed to find a free starting position for the robot.")
            yaw = spawn_rng.uniform(-np.pi, np.pi)
            self._data.qpos[:2] = xy
            # MuJoCo free joint quaternion is [w, x, y, z] with w = cos(theta/2) and z = sin(theta/2) for a rotation about the z axis.
            self._data.qpos[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]

        self._encoder_noise = encoder_noise
        self._base = self._model.body('base').id
        for name in ['left_tire', 'right_tire']:
            self._model.geom(name).friction[0] = traction
        self._renderer = mujoco.Renderer(self._model, height=480, width=640)
        self._step_count = round(DT / self._model.opt.timestep)
        mujoco.mj_forward(self._model, self._data)
        # Let the robot settle before taking the first encoder reading.
        for _ in range(250):
            mujoco.mj_step(self._model, self._data)
        mujoco.mj_forward(self._model, self._data)
        self._last_encoder = self._encoder_angles()
        self._time_origin = self._data.time
        self.collision = False

    @property
    def time(self):
        return self._data.time - self._time_origin

    def _encoder_angles(self):
        ticks = 4096
        angles = self._data.sensordata[:2].copy()
        return np.round(angles * ticks / (2*np.pi) + self._rng.normal(0, self._encoder_noise, 2)) * (2*np.pi/ticks)

    def step(self, command):
        self._data.ctrl[:] = wheel_rates(*command)
        for _ in range(self._step_count):
            mujoco.mj_step(self._model, self._data)
            for contact in self._data.contact:
                names = [self._model.geom(g).name for g in [contact.geom1, contact.geom2]]
                environment = any(n.startswith(('wall_', 'obstacle_')) or n == 'dock_board' for n in names)
                robot = any(n in ['chassis', 'left_tire', 'right_tire', 'caster', 'top', 'camera_shell'] for n in names)
                if environment and robot:
                    self.collision = True
        current = self._encoder_angles()
        increment = WHEEL_RADIUS * (current - self._last_encoder)
        self._last_encoder = current
        return increment

    def image(self):
        self._renderer.update_scene(self._data, camera='front')
        return self._renderer.render().copy()

    def lidar(self):
        """
        Return a simulated lidar scan, with noise and max range.
        
        The lidar is mounted on the robot, so it will not see the robot itself.
        The robots navigation will receive only angles and distances, not the full point cloud.
        Angles are relative to the robots heading, with 0 being straight ahead, and positive angles to the left.
        """
        site_id = self._model.site('lidar').id
        origin = self._data.site_xpos[site_id].copy()
        rotation = self._data.site_xmat[site_id].reshape(3,3)
        groups = np.array([1, 0, 0, 0, 0, 0])  # Ignore group 1 (the robot itself) and check for collisions with the rest of the environment
        hit = np.empty(1, dtype=np.int32)
        ranges = np.full(len(LIDAR_ANGLES), LIDAR_MAX)
        for i, angle in enumerate(LIDAR_ANGLES):
            direction = rotation @ np.array([np.cos(angle), np.sin(angle), 0])
            distance = mujoco.mj_ray(self._model, self._data, origin, direction, groups, True, -1, hit)
            if 0 <= distance < LIDAR_MAX:
                ranges[i] = np.clip(distance + self._rng.normal(0, LIDAR_SIGMA), 0, LIDAR_MAX)
        return LIDAR_ANGLES, ranges
            
    def evaluation_pose(self):
        """Use this only to log and check results, not to control the robot."""
        R = self._data.xmat[self._base].reshape(3, 3)
        return np.array([*self._data.xpos[self._base, :2], np.arctan2(R[1, 0], R[0, 0])])

    def save_overview(self, path):
        self._renderer.update_scene(self._data, camera='overview')
        cv2.imwrite(str(path), cv2.cvtColor(self._renderer.render(), cv2.COLOR_RGB2BGR))

    def close(self):
        self._renderer.close()
