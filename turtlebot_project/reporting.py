"""
Create simulation recording from saved mission
Create necessary plots from saved mission
"""

from pathlib import Path
import cv2
import matplotlib
matplotlib.use("Agg") # Saves figures without opening windows
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from .config import OBSTACLES
from .model import wrap, wheel_rates

def save_trial_plots(output):
    
    # Plot route and position errors
    r = np.atleast_1d(np.genfromtxt(output / "trial.csv", delimiter = ",", names = True))
    t = r["t"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout = True)
    for x0, x1, y0, y1 in OBSTACLES:
        axes[0].add_patch(Rectangle((x0, y0), x1-x0, y1-y0, color="0.8"))
    for x, y, label in [("rx", "ry", "Reference"), ("tx", "ty", "Truth"),
                        ("x", "y", "EKF"), ("ox", "oy", "Wheel odometry")]:
        axes[0].plot(r[x], r[y], label = label)
    axes[0].set(xlabel="World x (m)", ylabel="World y (m)", aspect="equal", title="Route tracking")

    for x, y, label in [("x", "y", "EKF localisation"), ("ox", "oy", "Wheel localisation"),
                        ("rx", "ry", "Reference tracking")]:
        axes[1].plot(t, np.hypot(r[x]-r["tx"], r[y]-r["ty"]), label = label)
    axes[1].set(xlabel = "Time (s)", ylabel = "Position error (m)", title = "Position errors")

    for ax in axes:
        ax.legend()
        ax.grid(alpha = .2)
    fig.savefig(output / "results.png", dpi = 150)
    plt.close(fig)

    # Plot uncertainty for all 3 states
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), constrained_layout=True)
    for ax, state, truth, variance, label in zip(
        axes, ("x","y","yaw"), ("tx","ty","tyaw"),
        ("Pxx","Pyy","Ptt"), ("x (m)", "y (m)", "Heading (degrees)")
    ):
        estimate = r[state].copy()
        actual = r[truth].copy()
        std = np.sqrt(np.maximum(r[variance], 0))
        if state == "yaw":
            estimate = np.unwrap(estimate)
            actual = estimate + wrap(actual-estimate)
            estimate, actual, std = [np.rad2deg(v) for v in (estimate, actual, std)]
        ax.plot(t, actual, label="Truth")
        ax.plot(t, estimate, label="EKF")
        ax.fill_between(t, estimate-2*std, estimate+2*std, alpha=.2, label="EKF Â±2 std")
        ax.set(xlabel="Time (s)", ylabel=label)
        ax.legend()
        ax.grid(alpha=.2)
    fig.savefig(output / "uncertainty.png", dpi=150)
    plt.close(fig)

    # Control and camera update plot
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), constrained_layout=True)
    axes[0].plot(t, r["v"], label="Linear command (m/s)")
    axes[0].plot(t, r["w"], label="Angular command (rad/s)")
    rates = np.array([wheel_rates(v, w) for v, w in zip(r["v"], r["w"])])
    axes[1].plot(t, rates[:, 0], label="Left wheel command")
    axes[1].plot(t, rates[:, 1], label="Right wheel command")
    axes[1].set_ylabel("rad/s")
    axes[2].step(t, r["accepted"], where="post", label="Camera correction accepted")
    axes[2].plot(t, r["held"], label="Dock hold (s)")
    for ax in axes:
        ax.set_xlabel("Time (s)")
        ax.legend()
        ax.grid(alpha=.2)
    fig.savefig(output / "control.png", dpi=150)
    plt.close(fig)

class TrialVideo:
    """
    Record overview frames with evaluation information at 5 frames/s.
    """
    def __init__(self, output):
        self.path = Path(output) / "demo.avi"
        self.writer = None

    def add(self, rgb, pose, truth, covariance, time, status):
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self.writer is None:
            height, width = frame.shape[:2]
            self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"MJPG"), 5, (width, height))
            if not self.writer.isOpened():
                raise RuntimeError("Could not open MJPG video writer")
        text = [
            f"{time:.1f}s  {status}",
            f"EKF x,y,yaw: {pose[0]:.2f}, {pose[1]:.2f}, {np.rad2deg(pose[2]):.1f} deg",
            f"Truth: {truth[0]:.2f}, {truth[1]:.2f}, {np.rad2deg(truth[2]):.1f} deg",
            f"EKF x +/-2 std: {2*np.sqrt(covariance[0,0]):.3f} m",
        ]
        for i, line in enumerate(text):
            cv2.putText(frame, line, (12, 22+22*i), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1, cv2.LINE_AA)
        self.writer.write(frame)

    def close(self):
        if self.writer is not None:
            self.writer.release()    
    
    
    
