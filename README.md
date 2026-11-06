## TurtleBot Navigation and Docking

41014 Sensors and Control in Mechatronics Systems
Assessment 4: Integrated Project — Track A

Group 46: Thomas Kelsey and Ryan Cooper

[Project repository](https://github.com/rinoW1010/41014_IntegratedProject)
[Demonstration video](demo/TurtleBot_demo.mp4)

## Project Description
This project simulates a simplified TurtleBot3-sized robot in MuJoCo.
The robot follows a fixed route through a room with two obstacles.
It then lines up with an ArUco marker and stops at a charging position.
A green light shows that the docking conditions have been met.
Charging is only a simulated status. The robot does not touch the wall.

Wheel encoders measure movement and the camera detects the marker.
An Extended Kalman Filter (EKF) combines these readings to estimate the robot's pose.
The controller uses this estimate to follow the route.
LiDAR stops forward movement if an obstacle is too close.
Simulator ground truth is used to check results, not to control the robot.

## Installation
The supplied dependency versions are documented for Windows and Python 3.12.
A physical TurtleBot and ROS are not needed.
MuJoCo rendering requires a working graphics environment.

Download the project and open PowerShell in the folder containing `main.py`.
Create a virtual environment and install the dependencies:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
```

The main libraries are MuJoCo, NumPy, OpenCV-contrib and Matplotlib.
The full dependency list is in `requirements-lock.txt`.
The commands below use the virtual environment directly, so activation is optional.
Run all commands from the project folder.

## Running the Program
Run the normal docking trial:
```powershell
.\.venv\Scripts\python.exe main.py --output outputs/final_nominal
```

Run with a six-degree change in starting heading:
```powershell
.\.venv\Scripts\python.exe main.py --varied --output outputs/final_varied
```

Run the test that stops camera corrections during docking:
```powershell
.\.venv\Scripts\python.exe main.py --marker-loss --output outputs/final_loss
```

The marker-loss test should stop the robot and finish without charging.
A timeout is expected if the robot reaches the final approach safely.
This test suppresses camera corrections; it does not remove the marker image.

Other options are:
- `--frames`: save individual overview images as well as the video.
- `--heading-gain 1.5`: change the route-tracking heading gain from its default of 1.8.
- `--output PATH`: choose a folder for the results.
- `--help`: show the available options.

Without `--output`, results go to `outputs/nominal`, `outputs/varied` or `outputs/marker_loss`.
The program runs for up to 120 simulated seconds and prints a results summary.
There is no live viewer. Watch the saved video after the run.
Use separate output folders because another run can overwrite existing files.

## Expected Outputs
A completed trial saves:

- `scene.xml`: the generated room and robot model.
- `start.png`: an overview at the start.
- `start_camera.png`: the first camera image.
- `finish.png`: the final position and charging light.
- `demo.avi`: the simulation recording, saved at 5 frames per simulated second.
- `trial.csv`: recorded positions, commands, uncertainty and docking information.
- `metrics.json`: success, collision status and error measurements.
- `results.png`: paths and position errors.
- `uncertainty.png`: estimated and true states with uncertainty bands.
- `control.png`: commands, camera updates and docking hold time.

The optional `frames/` folder contains individual overview images.
The submission video is `demo/TurtleBot_demo.mp4`.
It is separate from the AVI produced by each run.
If initial marker detection fails, the full output set may not be created.

## How the Code Works
1. Build the room, obstacles, robot and marker in MuJoCo.
2. Detect the marker in the first image and initialise the EKF.
3. Choose a reference position along the fixed route.
4. Calculate movement commands from the estimated pose and reference.
5. Check the forward LiDAR readings before applying the commands.
6. Read wheel movement and use it to predict the next pose.
7. Correct the estimate when a usable camera measurement is available.
8. Align at the station, complete the docking hold and save the results.

The robot starts at `(0.30, 1.58)` metres with a heading of -30 degrees.
It follows waypoints `(1.30, 1.50)`, `(1.65, 0.65)` and `(2.05, 0.00)`.
The docking target is `(2.55, 0.00)` metres with a heading of zero degrees.
The reference moves at 0.08 m/s and pauses when forward motion is blocked.

## Movement Model and Estimation
The state contains world position `x`, `y` and heading `theta`.
The wheel radius is 0.033 m and the wheel separation is 0.160 m.
The control and estimation interval is 0.05 seconds.
The basic motion model is:

```text
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = angular_velocity
```

Wheel displacement gives the distance travelled and change in heading.
The prediction uses the heading halfway through each step.
The model assumes flat-floor differential-drive movement.
The simulated robot and its contact properties are simplified.

The camera estimates pose from a known 0.24 m ArUco marker, ID 7.
Images are 640 × 480 pixels with a 50-degree vertical field of view.
Camera and marker locations are known in the simulated setup.
The camera is 0.060 m forward and 0.15 m above the robot body origin.
Camera axes are right/down/forward; robot axes are forward/left/up.
The code converts marker measurements through these frames into world pose.

The EKF predicts from encoders and corrects using camera pose.
Wheel noise is set using `0.0005 + 0.05 * abs(wheel_displacement)` metres.
Camera position noise is set using `0.04 + 0.015 * range^2` metres.
Camera heading noise is set using `3 + range` degrees, with range in metres.
These are assumed standard deviations, not measured physical-sensor specifications.
The filter rejects measurements whose innovation test exceeds 11.34.

Full camera pose provides information about all three estimated states.
Encoders alone cannot recover an unknown global starting pose.
Without camera updates, the filter relies on prediction and uncertainty can grow.
For a straight moving reference, the local model has controllability rank three.
At standstill its rank is two; lateral movement requires turning and driving.

## Control and Docking
The state-feedback controller corrects position and heading errors.
The forward-position gain is 0.9 and the sideways-error turning gain is 2.8.
The default heading gain is 1.8. This controller is not LQR.
Commands are limited to 0.12 m/s and 0.55 rad/s.

LiDAR checks 25 degrees either side of forward.
A reading below 0.23 m, or unusable readings, stops forward commands.
It does not plan a different route around the blockage.

At the staging point, the robot turns towards the marker.
The final approach requires accepted camera corrections.
Charging requires estimated position within 0.04 m and heading within 5 degrees.
These conditions, an accepted correction and zero commands must hold for one second.
This checks zero commanded motion, not independently measured zero velocity.
Final success is checked separately against true pose and the collision flag.

## Checks and Results
Run the camera, estimator and LiDAR checks:
```powershell
.\.venv\Scripts\python.exe -m checks.check_camera
.\.venv\Scripts\python.exe -m checks.check_estimation
.\.venv\Scripts\python.exe -m checks.check_lidar
```

After creating the three `outputs/final_*` folders above, verify the saved trials:
```powershell
.\.venv\Scripts\python.exe -m checks.verify_runs
```

The checks cover camera accuracy, covariance behaviour, heading wrapping and stopping.
The saved nominal result reports successful docking without a collision in 43.45 seconds.
Final position error was 0.03450 m and heading error was 0.6383 degrees.
EKF position RMSE was 0.01797 m, compared with 0.02894 m for wheel-only odometry.
These are results from one saved trial, not a general success rate.

## File Guide
Most program files are inside `turtlebot_project/`:
- `config.py`: room geometry, sensor settings and docking tolerances.
- `model.py`: motion equations, angle wrapping and wheel commands.
- `scene.py`: creates the MuJoCo XML model.
- `simulation.py`: runs physics and provides sensor readings and rendered images.
- `vision.py`: detects the marker and calculates robot pose.
- `estimation.py`: EKF prediction and correction.
- `control.py`: route settings, feedback control and LiDAR stopping.
- `mission.py`: joins the components and manages docking and logging.
- `reporting.py`: makes the plots and video.
- `__init__.py`: identifies the Python package.

Other project files and folders:
- `main.py`: starts the program and reads command-line options.
- `requirements-lock.txt`: pinned dependency versions.
- `pyproject.toml`: package and dependency information.
- `checks/`: camera, estimation, LiDAR and saved-trial checks.
- `ARCHIVE/run.py` and `ARCHIVE/evaluation.py`: older development files, not current entry points.
- `demo/TurtleBot_demo.mp4`: the submission video.

## Limitations and Common Problems
The route and starting location are fixed, and the marker must be visible initially.
The final mission does not search for the marker or navigate from arbitrary starts.
Charging represents a stopping position, not a physical charging connection.
If a package cannot be imported, check that the virtual environment is being used.
If the marker is not found, inspect `start_camera.png` and the configured start.
If verification cannot find results, check the `final_*` output folder names.

## Contributions and Sources
Ryan Cooper carried out the initial setup, motion, odometry and camera
checks. Tom Kelsey's commits added the room configuration,
obstacles, random and repeatable starts, camera-pose EKF and feedback with
LiDAR safety.

The project uses the supplied tutorials for motion, frames, sensing,
estimation and feedback. robot geometry and the simulation and vision APIs
were checked against the project sources.
 [ROBOTIS model](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/main/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf) · 
 [MuJoCo docs](https://mujoco.readthedocs.io/en/stable/python.html) · 
 [OpenCV ArUco docs](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

## Gen AI Acknowledgement 
OpenAI Codex helped review the brief and tutorials, generate and debug the
initial simulation, organise the stages, implement and verify the final
fixed-route integration and assessment evidence from the supplied guide, and edit the final 5 min demo video.