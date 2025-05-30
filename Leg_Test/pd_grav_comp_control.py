import numpy as np
from scipy import constants
from math import sin

class pd_grav_comp_control:
    def __init__(self, l1 = 0.2, l2 = 0.2, lc1 = 0.1, lc2 = 0.1, m1 = 0.5, m2 = 0.1):
        self.l1 = l1
        self.l2 = l2
        self.lc1 = lc1
        self.lc2 = lc2
        self.m1 = m1
        self.m2 = m2

    # Call this function and get the control torques in return
    def calc_control_torque(self, q_des_rad, q_rad, qd_rad_s, Kp, Kd):
        q_error = q_des_rad - q_rad

        # PD control with gravity compensation
        grav_comp_torques = self.calc_grav_comp_torque(q_rad)
        u = grav_comp_torques + Kp @ q_error - Kd @ qd_rad_s

        return u

    # Expects a 1x2 np array of joint angles in radians
    def calc_grav_comp_torque(self, q_rad):
        q1, q2 = q_rad
        g = constants.g

        grav_torque = -np.array([
            self.m1 * g * self.lc1 * sin(q1) + self.m2 * g * (self.l1 * sin(q1) + self.lc2 * sin(q1 + q2)),
            self.m2 * g * self.lc2 * sin(q1 + q2)
        ])

        return grav_torque