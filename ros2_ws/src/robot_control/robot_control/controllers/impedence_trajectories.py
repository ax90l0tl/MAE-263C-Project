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
t = np.array([0.0, 1.0])  # seconds
x_d_crouch = np.array([0.0, -0.15])
(t_idle_to_crouch, 
 x_d_idle_to_crouch_traj, 
 xd_d_idle_to_crouch_traj, 
 xdd_d_idle_to_crouch_traj) = traj_generation.quintic_interp(
    t, np.vstack((x_d_idle, x_d_crouch)), 
    np.zeros((2, 2)), 
    np.zeros((2, 2)), 
    f=f
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
    f=f
)
# # In-air mode
# x_d_in_air_traj = np.array([])
# xd_d_in_air_traj = np.array([])
# xdd_d_in_air_traj = np.array([])
# Landing mode
x_d_landing_traj = np.array([])
xd_d_landing_traj = np.array([])
xdd_d_landing_traj = np.array([])

x_d_jump = np.array([0.0, -0.2])
t = np.array([0.0, 1.0])

x_d_landing_pts = np.vstack((x_d_jump, x_d_crouch))
xd_d_landing_pts = np.array([[0.0, -1.0], [0.0, 0.0]])
xdd_d_landing_pts = np.array([[0.0, 0], [0.0, 0]])
(t_landing, 
 x_d_landing_traj, 
 xd_d_landing_traj, 
 xdd_d_landing_traj) = traj_generation.quintic_interp(
    t, x_d_landing_pts, 
    xd_d_landing_pts, 
    xdd_d_landing_pts, 
    f=f)

import matplotlib.pyplot as plt
plt.figure()
plt.plot(t_crouch_to_jump, x_d_crouch_to_jump_traj[:, 0], label='z position')
# plt.scatter(t, x_d_landing_pts[:, 1])
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid()

# plt.figure()
# plt.plot(t_landing, xd_d_landing_traj[:, 1], label='z velocity')
# plt.scatter(t, xd_d_landing_pts[:, 1])
# plt.xlabel('Time (s)')
# plt.ylabel('Velocity (m/s)')
# plt.legend()
# plt.grid()

# plt.figure()
# plt.plot(t_landing, xdd_d_landing_traj[:, 1], label='z acceleration')
# plt.scatter(t, xdd_d_landing_pts[:, 1])
# plt.xlabel('Time (s)')
# plt.ylabel('Acceleration (m/s^2)')
# plt.legend()
# plt.grid()
plt.show()