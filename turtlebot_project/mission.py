"""
Follow the fixed route and use the camera to confirm docking.
"""
from pathlib import Path
import json
import numpy as np
import cv2

from .config import DOCK_POSE, DOCK_POSITION_TOL, DOCK_HEADING_TOL, DOCK_HOLD, position_is_free
from .estimation import PoseEKF
from .model import DT, motion, wrap
from .simulation import Simulation
from .vision import MarkerCamera
from .control import START, ROUTE, SPEED, safe_command, track
from .reporting import save_trial_plots, TrialVideo

def mission(varied=False, *, output=None, frames=False, marker_loss=False, heading_gain=1.8):
    name = "marker_loss" if marker_loss else "varied" if varied else "nominal"
    output = Path(output) if output is not None else Path("outputs") / name
    if frames:
        (output / "frames").mkdir(parents=True, exist_ok=True)
    start = (*START[:2], START[2] + np.deg2rad(6 if varied else 0))
    sim = Simulation(output, start_pose=start)
    video = TrialVideo(output)
    camera, ekf = MarkerCamera(), PoseEKF()
    records = []
    charging = False
    try:
        first_image = sim.image()
        cv2.imwrite(str(output / "start_camera.png"), cv2.cvtColor(first_image, cv2. COLOR_RGB2BGR))
        sim.save_overview(output / "start.png")
        first = camera.detect(first_image)
        if first is None:
            cv2.imwrite(str(output / "start_camera.png"), cv2.cvtColor(sim.image(), cv2.COLOR_RGB2BGR))
            raise RuntimeError(f"could not see the marker at the start. Check {output / 'start_camera.png'}")
        ekf.update(first.pose, first.pose_covariance)
        odom = ekf.x.copy()
        origin = ekf.x[:2].copy()
        index, progress, held = 0, 0., 0.
        aligning = False
        for _ in range(round(120 / DT)):
            docking = index == len(ROUTE)
            goal = DOCK_POSE[:2] if docking else ROUTE[index]
            segment = goal - origin
            length = np.linalg.norm(segment)
            tangent = np.arctan2(segment[1], segment[0])
            ref = origin + segment * min(progress / max(length, 1e-9), 1.)
            command = track(ekf.x, ref, tangent, SPEED if progress < length else 0., heading_gain=heading_gain)
            if docking and np.linalg.norm(goal-ekf.x[:2]) < DOCK_POSITION_TOL:
                heading_error = wrap(DOCK_POSE[2]-ekf.x[2])
                command = (0., 0. if abs(heading_error) < DOCK_HEADING_TOL else
                           np.clip(1.8*heading_error, -0.55, 0.55))
            if aligning:
                command = (0., np.clip(-1.8*wrap(ekf.x[2]), -0.55, 0.55))
                if abs(wrap(ekf.x[2])) < np.deg2rad(4) and ekf.accepted:
                    aligning = False
            if docking and not aligning and not ekf.accepted:
                command = (0., 0.)
            applied = safe_command(command, *sim.lidar())
            if not aligning and applied == command and (not docking or ekf.accepted):
                progress = min(length, progress + SPEED*DT)
            wheels = sim.step(applied)
            odom = motion(odom, wheels)
            ekf.predict(wheels)
            detection = camera.detect(sim.image())
            ekf.accepted = False
            if detection is not None and not (marker_loss and docking and not aligning):
                ekf.update(detection.pose, detection.pose_covariance)
            near = np.linalg.norm(goal-ekf.x[:2]) < (DOCK_POSITION_TOL if docking else 0.06)
            aligned = abs(wrap(DOCK_POSE[2]-ekf.x[2])) < DOCK_HEADING_TOL
            held = held + DT if docking and near and aligned and ekf.accepted and applied == (0., 0.) else 0.
            charging = held >= DOCK_HOLD
            truth = sim.evaluation_pose()  # use the true pose to check the run.
            records.append([sim.time, *ref, *ekf.x, *truth, *odom, ekf.P[0,0], 
                            int(ekf.accepted), *applied, int(docking), int(aligning), held,
                            ekf.P[1, 1], ekf.P[2, 2]])
            if len(records) % 4 == 0:
                overview = sim.overview_image()
                video.add(overview, ekf.x, truth, ekf.P, sim.time, 
                          "Docking" if docking else "Following route")
                if frames:
                    cv2.imwrite(str(output / "frames" / f"{len(records)//4:04d}.png"),
                    cv2.cvtColor(overview, cv2.COLOR_RGB2BGR))
            if sim.collision or charging:
                break
            if not docking and near and progress >= length:
                origin = goal.copy()
                index += 1
                aligning = index == len(ROUTE)
                progress = 0.
        sim.set_charging(charging)
        finish = sim.overview_image()
        cv2.imwrite(str(output / "finish.png"),
                    cv2.cvtColor(finish, cv2.COLOR_RGB2BGR))
        status = "Charging" if charging else "Collision" if sim.collision else "Timeout"
        video.add(finish, ekf.x, truth, ekf.P, sim.time, status)
        video.close()
        if frames:
            from shutil import copyfile
            copyfile(output / "finish.png",
                     output / "frames" / f"{len(records)//4+1:04d}.png")
        np.savetxt(output/"trial.csv", records, delimiter=",", comments="",
                   header="t, rx, ry, x, y, yaw, tx, ty, tyaw, ox, oy, oyaw, " \
                   "Pxx, accepted, v, w, docking, aligning, held, Pyy, Ptt")
        pos_error = np.linalg.norm(truth[:2]-DOCK_POSE[:2])
        yaw_error = abs(wrap(truth[2]-DOCK_POSE[2]))
        success = bool(charging and not sim.collision and pos_error <= DOCK_POSITION_TOL
                       and yaw_error <= DOCK_HEADING_TOL)
        rows = np.asarray(records)
        metrics = {
            "success": success, "charging": bool(charging), "collision": bool(sim.collision),
            "duration_s": float(sim.time), "dock_error_m": float(pos_error),
            "dock_heading_error_deg": float(np.rad2deg(yaw_error)), "heading_gain": heading_gain,
            "tracking_rmse_m": float(np.sqrt(np.mean(np.sum((rows[:, 1:3] - rows[:, 6:8])**2, axis=1)))),
            "ekf_position_rmse_m": float(np.sqrt(np.mean(np.sum((rows[:, 3:5] - rows[:, 6:8])**2, axis=1)))),
            "odometry_position_rmse_m": float(np.sqrt(np.mean(np.sum((rows[:, 9:11] - rows[:, 6:8])**2, axis=1)))),
            "driven_path_clear_with_margin": bool(all(position_is_free(row[6:8]) for row in rows)),
            "false_charging": bool(charging and not success)
        }
        metrics["termination"] = status.lower()
        metrics["varied"] = bool(varied)
        metrics["marker_loss"] = bool(marker_loss)
        metrics["camera_updates"] = int(rows[:, 13].sum())
        metrics["ekf_heading_rmse_deg"] = float(np.rad2deg(np.sqrt(
            np.mean(wrap(rows[:, 5] - rows[:, 8]) ** 2))))
        save_trial_plots(output)
        summary = json.dumps(metrics, indent=2)
        (output / "metrics.json").write_text(summary + "\n", encoding="utf-8")
        print(summary)
        return metrics
    finally:
        video.close()
        sim.close()