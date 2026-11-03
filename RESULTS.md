# Results for steps 1-3

these results check the first stage only. The earlier full navigation and
docking results are saved in the final-stage folder beside this repo.

The first-stage checks passed on 3 November 2026:

| check | result |
|---|---:|
| automated checks | 2 passed |
| straight drive duration | 8.00 s |
| collisions | none |
| wheel odometry position RMSE | 0.0000185 m |
| wheel odometry heading RMSE | 0.00982 deg |
| camera detections | 40 |
| camera range RMSE | 0.0585 m |
| camera bearing RMSE | 0.0213 deg |

run `run.py` and open `outputs/steps_1_to_3/results.png` to see the graphs.
The camera check compares detected range and bearing against simulator truth.
It does not feed camera measurements back into the odometry yet.

the very small odometry error is from a short, straight, nominal simulated run
with a known starting pose. it is not evidence of that accuracy on a real
robot, under slip, or after travelling a longer route. marker range errors are
larger at the distant starting position, which needs further calibration work.

The 5-second turn and 32-second square runs were also checked without
collisions. Their odometry position RMSE values were 0.00677 m and 0.0114 m.
Before moving on, check reversing, the other turn direction and more marker
positions. The next-stage folder has a generated EKF and waypoint
reference for review; those results are not first-stage results.
