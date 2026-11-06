"""Check the camera pose from a few known starts."""
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import mujoco
import numpy as np

from turtlebot_project.model import wrap
from turtlebot_project.simulation import Simulation
from turtlebot_project.vision import MarkerCamera


def main():
    camera = MarkerCamera()
    output = Path("outputs/stage5_checkpoint")
    output.mkdir(parents=True, exist_ok=True)
    cases = [(2.25, 0, 0), (1.85, -0.30, 15), (1.85, 0.30, -15),
             (2.25, -0.30, 20), (2.25, 0.30, -20)]
    passed = True
    with TemporaryDirectory(dir=output) as tmp:
        sim = Simulation(tmp)
        try:
            model, data = sim._model, sim._data  # use the simulator pose to check the camera reading.
            print("x and y are in metres. Yaw is in degrees.")
            for i, (x, y, deg) in enumerate(cases, 1):
                mujoco.mj_resetData(model, data)
                half = np.deg2rad(deg) / 2
                data.qpos[:2] = [x, y]
                data.qpos[3:7] = [np.cos(half), 0, 0, np.sin(half)]
                for _ in range(250):
                    mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                truth = sim.evaluation_pose()
                rgb = sim.image()
                cv2.imwrite(str(output / f"case_{i}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                detection = camera.detect(rgb)
                if detection is None:
                    print(f"case {i}: marker not found")
                    passed = False
                    continue
                pose = detection.pose
                error = pose - truth
                error[2] = wrap(error[2])
                scale = np.array([1, 1, 180 / np.pi])
                print(f"case {i}: truth {truth*scale}  measured {pose*scale}  error {error*scale}")
                passed &= np.linalg.norm(error[:2]) < 0.05 and abs(error[2]) < np.deg2rad(3)
        finally:
            sim.close()
    if not passed:
        raise SystemExit("check the failed cases and their saved images")
    print("all cases passed. Check if the errors lean the same way.")


if __name__ == "__main__":
    main()
