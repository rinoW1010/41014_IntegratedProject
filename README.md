# TurtleBot project: steps 1-3

this is the first implementation milestone. get the robot moving, check the
wheel odometry and check the camera before connecting a feedback controller.

## What is in here

1. **movement:** a simplified Burger-sized robot in an empty MuJoCo room.
   Commands come from a fixed schedule for straight driving, turning or a square.
2. **wheel odometry:** calculate position and heading from encoder readings,
   then compare them against the simulator for checking.
3. **camera:** detect the ArUco marker from rendered images and check its range
   and bearing. Camera readings are not fused into the odometry yet.

The EKF, LQR, waypoints, LiDAR planning and docking are in the later-stage
folders beside this repo. Only this first stage is currently inside the repo.

## Setup

open PowerShell in this folder. If you are setting it up on a new computer:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
```

Use Python 3.11 or newer. If `python` opens the Microsoft store, use
`py -3.12 -m venv .venv` for the first command instead. The existing `.venv`
in the first-stage repo is already set up.

## Run it

```powershell
.\.venv\Scripts\python.exe run.py
```

this runs an 8-second straight drive. there is no live viewer. open
`outputs/steps_1_to_3/results.png` when it finishes.

```powershell
# check turning on the spot
.\.venv\Scripts\python.exe run.py --movement turn --duration 5 --output outputs/turn

# drive a square with a fixed command schedule
.\.venv\Scripts\python.exe run.py --movement square --duration 32 --output outputs/square

# Run the checks
.\.venv\Scripts\python.exe -m pytest -q
```

Each output folder has the plots, metrics, raw data, scene model, start and
finish images, and a marker detection image when the marker is visible.
using the same output folder again replaces the matching results.

## Files

| file in `turtlebot_project/` | what it does |
|---|---|
| `scene.py` | builds the empty room, robot and marker |
| `simulation.py` | runs the physics and reads encoders and camera images |
| `model.py` | wheel motion and camera range/bearing calculations |
| `vision.py` | detects the marker and estimates its range and bearing |
| `run.py` | runs the scheduled movement and records measurements |
| `evaluation.py` | compares readings against truth and saves results |

`run.py` at the repo root starts the run. `tests/` checks the movement, wheel
calculations and rendered camera detection. The original GitHub test and
`githubstuff/` files are kept.

## Before moving on

check the directions and units, compare straight and turning odometry against
truth, and look at marker errors at different ranges. This has a simulated
pinhole camera with known settings, not a calibrated physical camera.

The wheel radius is 0.033 m and separation is 0.160 m. The body, motor and
contact settings are simplified assumptions. Sensor axes and camera mounting
still need checking and explaining in the final report.

Ground truth is used for plots and checks only. The commands in this stage
come from time, not the true robot pose.

## Next stages

see `../IMPLEMENTATION_STAGES.md` for the layout. the next folder covers steps
3-6 for tomorrow afternoon, then the final folder covers steps 7-10.
previous full-navigation results are saved with the final stage, not reported
as first-stage results here. see `PROJECT_PLAN.md` and `RESULTS.md` for this stage.

## Sources and AI use

this code builds on the supplied tutorials, particularly weeks 1-3 for movement,
wheel odometry, frames and cameras. robot geometry was checked against the
[ROBOTIS model](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/main/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf).
See the [MuJoCo Python docs](https://mujoco.readthedocs.io/en/stable/python.html)
and [OpenCV marker docs](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

OpenAI Codex helped generate the initial code, debug it, prepare the tests and
split the stages. The team still needs to review, understand and validate the
parts used in the submission and keep the final AI declaration accurate.

`.venv`, `tmp` and generated outputs are ignored by git. Nothing has been
staged, committed or pushed by Codex.
