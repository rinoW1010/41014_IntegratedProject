# Plan for steps 1-3

## 1. get the robot moving

- start with an empty room and the simplified robot.
- try forward movement, reversing and turning in both directions.
- check wheel directions, speed limits, contact settings and caster behaviour.
- save the commands and results so the checks can be repeated.

## 2. check the encoders and odometry

- use the measured wheel angles, 0.033 m radius and 0.160 m separation.
- calculate distance and heading change using the midpoint update.
- compare straight lines, turns and a square against simulator truth.
- explain what comes from the configured start, quantisation and slip.

## 3. check the camera

- detect the ArUco marker from actual rendered images.
- check range and bearing at different distances and viewing angles.
- confirm the marker size, camera settings and mounting transform.
- record when the marker disappears or a measurement is rejected.

The state is `[x, y, theta]` in metres and radians. Robot axes are x forward,
y left, z up; camera axes are x right, y down, z forward. The camera is at
`(0.06, 0, 0.15)` m relative to the robot, whose origin is about 0.07 m above
the floor. The marker is 0.24 m wide at `(3, 0, 0.22)` m.

## Moving to the next stage

bring the checked sensing files into the steps 3-6 folder, then review the EKF
and LQR additions there. Step 3 is repeated because the camera needs checking
again when its readings start correcting the estimated pose.

The two later folders sit beside this repo. they contain generated reference
code, not evidence that the team has already reviewed or implemented it.
keep the AI declaration and contribution statements accurate as work continues.
