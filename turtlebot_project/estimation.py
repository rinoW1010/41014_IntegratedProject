import numpy as np
from .model import motion, wrap

def jacobian(function, point, angle_row=2):
    """
    Compute the Jacobian of a function at a point using finite differences.
    """