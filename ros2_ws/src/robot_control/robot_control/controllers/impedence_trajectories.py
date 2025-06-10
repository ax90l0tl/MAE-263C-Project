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
x_d_crouch = np.array([-0.0, -0.15])
t = np.array([0.0, 1.0])  # seconds
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
x_d_jump = np.array([0.0, -0.31])
t = np.array([0.0, 0.1])
(t_crouch_to_jump, 
 x_d_crouch_to_jump_traj, 
 xd_d_crouch_to_jump_traj, 
 xdd_d_crouch_to_jump_traj) = traj_generation.quintic_interp(
    t, np.vstack((x_d_crouch, x_d_jump)), 
    np.array([[0.0, 0.0], [0.0, 1.0]]), 
    np.array([[0.0, -9.8], [0.0, 0.0]]), 
    f=f
)

# # In-air mode
x_d_in_air = np.array([0.0, -0.22])
t = np.array([0.0, 0.04])
(t_in_air, 
 x_d_in_air_traj, 
 xd_d_in_air_traj, 
 xdd_d_in_air_traj) = traj_generation.quintic_interp(
    t, np.vstack((x_d_jump, x_d_in_air)), 
    np.zeros((2, 2)), 
    np.zeros((2, 2)), 
    f=f
)

# Landing mode
x_d_landing_pts = np.vstack((x_d_jump, x_d_crouch))
t = np.array([0.0, 1.0])
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

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(t_crouch_to_jump, x_d_crouch_to_jump_traj[:, 0], label='z position')
    plt.scatter(t, x_d_landing_pts[:, 1])
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t_landing, xd_d_landing_traj[:, 1], label='z velocity')
    plt.scatter(t, xd_d_landing_pts[:, 1])
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t_landing, xdd_d_landing_traj[:, 1], label='z acceleration')
    plt.scatter(t, xdd_d_landing_pts[:, 1])
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()
    plt.grid()
    plt.show()