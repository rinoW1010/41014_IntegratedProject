"""Check movement, wheel odometry and the camera before adding feedback control."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from ..turtlebot_project.model import DT, motion, marker_measurement
from ..turtlebot_project.simulation import Simulation
from ..turtlebot_project.vision import MarkerCamera
from .evaluation import save_results


def scheduled_command(time, duration, movement):
    if movement=='straight':
        return np.array([.12,0.])
    if movement=='turn':
        return np.array([0.,.3])
    phase=int(time/(duration/8))
    return np.array([.12,0.]) if phase%2==0 else np.array([0.,(np.pi/2)/(duration/8)])


def run(output='outputs/steps_1_to_3', duration=8., movement='straight', seed=1):
    if duration<DT or (movement=='square' and duration<16):
        raise ValueError('use at least 0.05 seconds, or 16 seconds for the square')
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    camera=MarkerCamera()
    simulation=Simulation(output, seed=seed, random_start=False)
    # this is the configured start, not a pose read from the simulator.
    odometry=np.array([-1.8,-.65,0.])
    records=[]
    try:
        simulation.save_overview(output/'start.png')
        for k in range(round(duration/DT)):
            # commands come from the schedule, not the true or estimated pose.
            command=scheduled_command(k*DT,duration,movement)
            odometry=motion(odometry,simulation.step(command))
            measured=np.full(2,np.nan)
            if k%4==0:
                rgb=simulation.image()
                detection=camera.detect(rgb)
                if detection is not None:
                    measured=detection.measurement
                    image=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
                    cv2.polylines(image,[detection.corners.astype(np.int32)],True,(0,200,0),2)
                    cv2.imwrite(str(output/'camera_detection.png'),image)
            # Use the true pose only to check the odometry and camera readings.
            truth=simulation.evaluation_pose()
            records.append(dict(time=simulation.time,truth=truth,pose=odometry.copy(),
                                odometry=odometry.copy(),camera=measured,
                                expected=marker_measurement(truth),covariance=np.full((3,3),np.nan)))
            if simulation.collision:
                break
        simulation.save_overview(output/'finish.png')
    finally:
        simulation.close()
    metrics=save_results(output,records,1,simulation.collision)
    print(json.dumps(metrics,indent=2))
    return metrics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='outputs/steps_1_to_3')
    parser.add_argument('--duration',type=float,default=8.)
    parser.add_argument('--movement',choices=['straight','turn','square'],default='straight')
    parser.add_argument('--seed',type=int,default=1)
    args=parser.parse_args()
    if args.duration<DT or (args.movement=='square' and args.duration<16):
        parser.error('use duration >= 0.05 seconds, or >= 16 for a square')
    run(args.output,args.duration,args.movement,args.seed)


if __name__=='__main__':
    main()
