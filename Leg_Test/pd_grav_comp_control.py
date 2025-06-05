import numpy as np
from scipy import constants
from math import sin, cos, acos, pi, sqrt, radians

class pd_grav_comp_control:
    def __init__(self, l1 = 0.125, l2 = 0.215, lc1 = 0.0613, lc2 = 0.11, m0 = 0.649, m1 = 0.478, m2 = 0.0633, th = 77.52):
        self.l1 = l1
        self.l2 = l2
        self.lc1 = lc1
        self.lc2 = lc2
        self.m0 = m0  # Mass of base (before joint 1)
        self.m1 = m1  # Mass of link 1 assembly
        self.m2 = m2  # Mass of link 2 assembly (including foot contact switch)
        self.th = th * pi / 180  # Offset angle of link 1 CoM

    # Call this function and get the control torques in return
    def calc_control_torque(self, q_des_rad, q_rad, qd_rad_s, Kp, Kd, phase = 'AERIAL'):
        q_error = q_des_rad - q_rad

        # PD control with gravity compensation
        grav_comp_torques = self.calc_grav_comp_torque(q_rad, phase)
        u = grav_comp_torques + Kp @ q_error - Kd @ qd_rad_s

        return u

    # Expects a 1x2 np array of joint angles in radians
    def calc_grav_comp_torque(self, q_rad, phase = 'AERIAL'):
        q1, q2 = q_rad
        q1 = q1 - pi / 2
        g = constants.g

        if phase == 'AERIAL':
            grav_torque = -np.array([
                self.m1 * g * self.lc1 * cos(q1 + self.th) + self.m2 * g * (self.l1 * cos(q1) + self.lc2 * cos(q1 + q2)),
                self.m2 * g * self.lc2 * cos(q1 + q2)
            ])
        elif phase == 'STANCE':
            # Ignoring the friction in +/- x direction experienced by the foot
            m0, m1, m2 = self.m0, self.m1, self.m2
            q1p = q2 + (q1 + pi)
            q2p = - q2
            lc2p = self.l2 - self.lc2
            lc1p = sqrt(self.l1**2 + self.lc1**2 - 2*self.l1*self.lc1*cos(self.th))
            thp = acos((self.l1**2 + lc1p**2 - self.lc1**2) / (2*self.l1*lc1p))

            tau1 = (m2*g * lc2p*cos(q1p) + 
                    m0*g * (self.l1*cos(q1p+q2p)+self.l2*cos(q1p)) + 
                    m1*g * (lc1p*cos(q1p+q2p-thp)+self.l2*cos(q1p)))
            tau2 = - (m0 + m1 + m2) * g * self.l2 * cos(q1 + q2) + m2 * g * self.lc2 * sin(q1 + q2)
            grav_torque = np.array([tau1, tau2]) 
            # print(f"grav_torque: {grav_torque}")
        else:
            grav_torque = np.array([0, 0])

        return grav_torque
    
    def calculate_FK(self, q):
        q1, q2 = q
        q1 = q1 - pi/2
        q2 = q2

        pe = np.array([self.l2*cos(q1 + q2) + self.l1*cos(q1), 
                       self.l2*sin(q1 + q2) + self.l1*sin(q1)])        
        return pe
    
    def calculate_IK(self, p):
        """
        Parameters:
        - p: [x, y] desired end-effector position

        Returns:
        - 1x2 numpy array. Elbow-up solution only. If unreachable, returns [np.nan, np.nan].
        """
        L1, L2 = self.l1, self.l2
        x, y = p
        r2 = x**2 + y**2

        # Check reachability
        max_reach = (L1 + L2)**2
        min_reach = (L1 - L2)**2
        if r2 > max_reach or r2 < min_reach:
            # return np.array([np.nan, np.nan],
            #                 [np.nan, np.nan]])
            return np.array([np.nan, np.nan])

        # Compute cos(q2) and clamp due to numerical error
        c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
        c2 = np.clip(c2, -1.0, 1.0)

        # Two possible q2 values
        q2a = np.arccos(c2)
        q2b = -q2a

        # Corresponding q1 values
        q1a = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q2a), L1 + L2 * np.cos(q2a))
        q1b = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q2b), L1 + L2 * np.cos(q2b))

        # return np.array([[q1a, q2a], [q1b, q2b]])
        return np.array([q1a + pi/2, q2a])  # Only need elbow-up.
    
