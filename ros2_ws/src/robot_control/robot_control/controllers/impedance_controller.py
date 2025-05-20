import numpy as np


class impedence_controller:
    def __init__(self):
        self.output = np.zeros(2)

    def compute_torque(self, x_e, x_d, x_dot_d, x_dot_e, x_ddot_d, x_ddot_e):
        x_error = x_d - x_e
        x_dot_error = x_dot_d - x_dot_e
        x_ddot_error = x_ddot_d - x_ddot_e

        
        return