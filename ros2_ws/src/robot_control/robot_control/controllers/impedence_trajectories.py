import numpy as np
from robot_control.robot_utilities import traj_generation

# BEAR Parameters
# max_acc = 50 # rad/s^2
f = 200 #hz
dt = 1 / f # seconds

# Idle mode
x_d_idle = np.array([0.0, -0.3])
xd_d_idle = np.zeros(2)
xdd_d_idle = np.zeros(2)
# Crouch mode
t = np.array([0.0, 0.5])  # seconds
x_d_crouch = np.array([0.0, -0.115])
(t_idle_to_crouch, 
 x_d_idle_to_crouch_traj, 
 xd_d_idle_to_crouch_traj, 
 xdd_d_idle_to_crouch_traj) = traj_generation.quintic_interp(
    t, np.vstack((x_d_idle, x_d_crouch)), 
    np.zeros((2, 2)), 
    np.zeros((2, 2)), 
    n=int(f*t[-1] - t[0])
)
# Jump mode
x_d_jump = np.array([0.0, -0.3])
t = np.array([0.0, 0.1])
(t_crouch_to_jump, 
 x_d_crouch_to_jump_traj, 
 xd_d_crouch_to_jump_traj, 
 xdd_d_crouch_to_jump_traj) = traj_generation.quintic_interp(
    t, np.vstack((x_d_crouch, x_d_jump)), 
    np.zeros((2, 2)), 
    np.zeros((2, 2)), 
    n=int(f*t[-1] - t[0])
)
# # In-air mode
# x_d_in_air_traj = np.array([])
# xd_d_in_air_traj = np.array([])
# xdd_d_in_air_traj = np.array([])
# # Landing mode
# x_d_landing_traj = np.array([])
# xd_d_landing_traj = np.array([])
# xdd_d_landing_traj = np.array([])