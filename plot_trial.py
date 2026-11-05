from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np
from turtlebot_project.config import OBSTACLES
from matplotlib.patches import Rectangle

folder = Path(sys.argv[1])
r = np.genfromtxt(folder/"trial.csv", delimiter=",", names=True)
fig, (ax, bx) = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
for x0, x1, y0, y1 in OBSTACLES:
    ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, color="0.8"))
for a,b,label in [("rx","ry","reference"),("tx","ty","truth"),("x","y","EKF")]:
    ax.plot(r[a], r[b], label=label)
ax.set(xlabel="world x (m)", ylabel="world y (m)", aspect="equal", title="path around the obstacles")
sigma = np.sqrt(r["Pxx"])
bx.fill_between(r["t"], r["x"]-2*sigma, r["x"]+2*sigma, alpha=.25, label="EKF ±2 standard deviations")
bx.plot(r["t"], r["x"], label="EKF")
bx.plot(r["t"], r["tx"], "--", label="truth")
bx.set(xlabel="time (s)", ylabel="world x (m)", title="position and uncertainty")
for panel in (ax,bx):
    panel.legend()
fig.savefig(folder/"results.png", dpi=150)
for a,b,label in [("x","y","EKF position"),("ox","oy","odometry position"),("rx","ry","reference tracking")]:
    print(label, "RMSE (m)", np.sqrt(np.mean((r[a]-r["tx"])**2+(r[b]-r["ty"])**2)))
