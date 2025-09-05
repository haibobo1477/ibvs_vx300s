import numpy as np
import modern_robotics as mr



def interaction_matrix_point(x, y, Z):
    return np.array([
        [-1/Z, 0, x/Z, x*y, -(1+x**2), y],
        [0, -1/Z, y/Z, 1+y**2, -x*y, -x]
    ])







