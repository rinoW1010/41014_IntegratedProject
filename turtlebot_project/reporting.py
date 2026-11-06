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
    output = Path(output)
    r = np.atleast_1d(np.genfromtxt(output / "trial.csv", delimiter = ",", names = True))
    
