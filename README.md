# TurtleBot project

## Setup
Open PowerShell in this folder. If you are setting it up on a new computer:

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
