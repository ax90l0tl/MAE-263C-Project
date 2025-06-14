import numpy as np
from pybear import Manager

import numpy as np
from scipy import constants
from math import sin, cos, acos, pi, sqrt, radians
from conrad_functions import *

class ConradController:
    def __init__(self, l1 = 0.125, l2 = 0.215, l1x = 0.01416, l2x = 0.1171, l1y = 0.05920, l2y = -0.01922, m0 = 0.649, m1 = 0.478, m2 = 0.0633):
        self.l1 = l1
        self.l2 = l2
        self.l1x = l1x
        self.l2x = l2x
        self.l1y = l1y
        self.l2y = l2y
        self.m0 = m0  # Mass of base (before joint 1)
        self.m1 = m1  # Mass of link 1 assembly
        self.m2 = m2  # Mass of link 2 assembly (including foot contact switch)
        % the offset angles/zeros, add pi/2 to q1 to account for the BEAR's default orientation

    # Call this function and get the control torques in return
    def calc_control_torque(self, desired, q_rad, qd_rad_s, Kp, Kd, current_state,leg):
        # q_error = q_des_rad - q_rad
        q_des_rad = q_des_rad - [-pi/2, 0]  # Adjust q1 to BEAR's default orientation
        if current_state == 'AERIAL':
            # In AERIAL phase, we use the joint space PD control
            J = function_J14(q_rad, self.l1, self.l2)
            x_error = desired - function_p14(q_rad, self.l1, self.l2)
            x_dot = J @ qd_rad_s
            u = grav_comp_torques + friction_comp_torques + Kp @ x_error - Kd @ qd_rad_s
            # Task Space PD control with gravity and friction compensation
            grav_comp_torques = function_G(q_rad, constants.g, self.m1,self.m2, self.l1x, self.l1y, self.l2x,self.l2y,self.l1,self.l2)
            friction_comp_torques = self.calc_friction_torque(qd_rad_s)
            f = Kp @ x_error - Kd @ x_dot
            u = grav_comp_torques + friction_comp_torques + J.T @ f
        elif current_state == 'STANCE':
            # In STANCE phase, we use the task space PD control
            x_error = desired - function_p14(q_rad, self.l1, self.l2)
            J = function_J14(q_rad, self.l1, self.l2)
            x_dot = J @ qd_rad_s
            grav_comp_torques = function_G(q_rad, constants.g, self.m1,self.m2, self.l1x, self.l1y, self.l2x,self.l2y,self.l1,self.l2)
            f_ff = constants.g * np.array([0, (self.m0+self.m1+self.m2)])  # Static weight force feedforward
            friction_comp_torques = self.calc_friction_torque(qd_rad_s)
            f = Kp @ x_error - Kd @ x_dot
            u = grav_comp_torques + friction_comp_torques + J.T @ f
        return u

function [u,test] = controller_jump(t,X,p,test)
params = p.params;
q = X(1:3);
dq = X(4:6);
G = fcn_Ge(q,params);

% Control Gains in the Ground (Task Space)
Kp = p.Kp_stance;
Kd = p.Kd_stance;


% Feedback Torque using Controller to Stabilize System About Set Point in Task Space
x_des = p.foot_d_stance; % Desired Task Space Position of the Foot w.r.t Base
x = fcn_p14(q,params);
% Force Feedforward (Static Weight)
f_ff = [0;p.weight];
% Jacobian from Hip to Foot
J14 = fcn_J14(q,params);
x_dot = J14*dq;
% q_des = fcn_IK(x_des,params);
f_pd = Kp * (x_des - x) - Kd * x_dot;
u = (J14' * f_pd) -(J14' * f_ff) + G; % Gravity Compensation to Reduce SS Errors






    def calc_friction_torque(self, qd_rad_s):
        minimum_velocity = (1.08 + 3.32 + 0.86) / 3  # rad/s
        slope_near_zero = (0.192 + 0.113 + 0.243) / 3

        viscous_pos = (0.045 + 0.034 + 0.027) * 0.1 / 3
        viscous_neg = (0.55 + 0.032 + 0.024) * 0.1 / 3
        coulomb_pos = (0.139 + 0.258 + 0.255) * 0.1 / 3
        coulomb_neg = (-0.074 - 0.280 - 0.265) * 0.1 / 3

        u_fric = []
        for qd in qd_rad_s:
            if qd > minimum_velocity:
                val = viscous_pos * qd + coulomb_pos
                # print(f"Positive velocity: {qd:.6f} rad/s, u: {val:.6f} Nm")
                u_fric.append(val)
            elif qd < -minimum_velocity:
                val = viscous_neg * qd + coulomb_neg
                # print(f"Positive velocity: {qd:.6f} rad/s, u: {val:.6f} Nm")
                u_fric.append(val)
            else:
                u_fric.append(slope_near_zero * qd)
        return np.array(u_fric)
    
    
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
    
