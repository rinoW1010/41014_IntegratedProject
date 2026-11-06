"""Save the sensor checks and compare the pose against the simulation."""
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).resolve().parents[1]/'tmp'/'matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from ..turtlebot_project.model import wrap


def save_results(output, records, stage, collision, completed=None):
    output=Path(output)
    time=np.array([r['time'] for r in records])
    truth=np.array([r['truth'] for r in records])
    pose=np.array([r['pose'] for r in records])
    odometry=np.array([r['odometry'] for r in records])
    camera=np.array([r['camera'] for r in records])
    expected=np.array([r['expected'] for r in records])
    error=truth-pose
    error[:,2]=wrap(error[:,2])
    visible=np.isfinite(camera).all(axis=1)
    camera_error=camera-expected
    camera_error[:,1]=wrap(camera_error[:,1])
    metrics=dict(stage=stage, duration_s=float(time[-1]), collision=bool(collision),
                 position_rmse_m=float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
                 heading_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(error[:,2]**2)))),
                 camera_detections=int(visible.sum()),
                 camera_range_rmse_m=float(np.sqrt(np.mean(camera_error[visible,0]**2))) if visible.any() else None,
                 camera_bearing_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(camera_error[visible,1]**2)))) if visible.any() else None)
    if completed is not None:
        metrics['waypoints_completed']=bool(completed)
        metrics['final_waypoint_error_m']=float(np.linalg.norm(truth[-1,:2]-[2.,0.]))
    np.savez_compressed(output/'run.npz',time=time,truth=truth,pose=pose,odometry=odometry,
                        camera=camera,expected_camera=expected,
                        covariance=np.array([r['covariance'] for r in records]))
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf8')
    fig,axes=plt.subplots(2,2,figsize=(11,7),constrained_layout=True)
    label='wheel odometry' if stage==1 else 'ekf estimate'
    axes[0,0].plot(truth[:,0],truth[:,1],label='True position for checking')
    axes[0,0].plot(pose[:,0],pose[:,1],'--',label=label)
    if stage==2:
        axes[0,0].plot(odometry[:,0],odometry[:,1],':',label='Wheel odometry')
    axes[0,0].set(xlabel='X [m]',ylabel='Y [m]',title='Path',aspect='equal')
    axes[0,1].plot(time,np.linalg.norm(error[:,:2],axis=1),label=label)
    axes[0,1].set(xlabel='Time [s]',ylabel='Position error [m]',title='Pose error')
    axes[1,0].plot(time[visible],camera_error[visible,0],'.',label='Camera range error')
    axes[1,0].set(xlabel='time [s]',ylabel='range error [m]',title='marker range check')
    axes[1,1].plot(time[visible],np.rad2deg(camera_error[visible,1]),'.',label='Camera bearing error')
    axes[1,1].set(xlabel='Time [s]',ylabel='Bearing error [deg]',title='Marker bearing check')
    for ax in axes.flat:
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.savefig(output/'results.png',dpi=150)
    plt.close(fig)
    return metrics
