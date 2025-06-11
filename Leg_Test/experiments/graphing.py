import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import sys, os
from datetime import datetime
from scipy.integrate import cumulative_trapezoid
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'controllers')))
from pd_grav_comp_controller import pdGravCompController

'''
Expected data format in CSV:
First row are headers for the columns:
Timestamp, foot_state, q1_des, q1_meas, q1d_des, q2d_meas, u1_des, u1_meas, q2_des, q2_meas, q2d_des, q2d_meas, u2_des, u2_meas
'''

def calc_z_velocity(q1, q1d, q2, q2d):
    l1, l2 = 0.125, 0.215
    sin_q1 = np.sin(q1)
    cos_q1 = np.cos(q1)
    sin_q1q2 = np.sin(q1 + q2)
    cos_q1q2 = np.cos(q1 + q2)

    # Jacobian entries
    J11 = -l1 * sin_q1 - l2 * sin_q1q2
    J12 = -l2 * sin_q1q2
    J21 =  l1 * cos_q1 + l2 * cos_q1q2
    J22 =  l2 * cos_q1q2

    # Compute xdot and ydot
    xdot = J11 * q1d + J12 * q2d
    ydot = J21 * q1d + J22 * q2d

    return np.stack([xdot, ydot], axis=1)  # shape (N, 2)
    # end_eff_vel = []
    # for q, q_dot in zip(joint_angles, joint_velocities):
    #     q1, q2 = q
    #     J = np.array([
    #         [-l1*np.sin(q1) - l2*np.sin(q1 + q2), -l2*np.sin(q1 + q2)],
    #         [ l1*np.cos(q1) + l2*np.cos(q1 + q2),  l2*np.cos(q1 + q2)]
    #     ])
    #     v = J @ q_dot  # matrix multiply
    #     end_eff_vel.append(v)
    # return np.array(end_eff_vel)

def plot_shaded_region(ax):
    for start, end in zip(starts, ends):
        ax.axvspan(start, end, color='tab:purple', alpha=0.2)


# Load data from CSV file and trim if needed
filepath = "pd_w_gravity_jump/pd_jump_20250610_5.csv"
front_trim = 750
back_trim = 300
if not os.path.isfile(filepath):
    raise FileNotFoundError(f"File not found: {filepath}")
recorded_data = np.loadtxt(filepath, delimiter=',', skiprows=1)
back_trim = len(recorded_data) - back_trim
recorded_data = recorded_data[front_trim:back_trim]

# Process foot sensor to graph shaded region
time = recorded_data[:, 0]
contact_raw = recorded_data[:, 1]
contact_clean = (contact_raw > 0.5).astype(int)  # Add debouncing here later if needed
contact_diff = np.diff(contact_clean, prepend=0)
starts = time[contact_diff == 1]
ends = time[contact_diff == -1]

# Assign columns to np arrays
q1_des = recorded_data[:, 2]
q1_meas = recorded_data[:, 3]
q1d_des = recorded_data[:, 4]
q1d_meas = recorded_data[:, 5]
u1_des = recorded_data[:, 6]
u1_meas = recorded_data[:, 7]
q2_des = recorded_data[:, 8]
q2_meas = recorded_data[:, 9]
q2d_des = recorded_data[:, 10]
q2d_meas = recorded_data[:, 11]
u2_des = recorded_data[:, 12]
u2_meas = recorded_data[:, 13]
imu_g = recorded_data[:, 14] - 1  # Minus 1g

# Integrate IMU reading to get operation-space velocity
imu_acc = imu_g * 9.81 
dt = np.diff(time, prepend=time[0])
# int_vel_z = np.cumsum(imu_acc * dt)
int_vel_z = cumulative_trapezoid(imu_acc, time, initial=0)

# Calculate EE velocity using jacobian
jac_vel = calc_z_velocity(q1_meas, q1d_meas, q2_meas, q2d_meas)

# Converts joint angles to operation space with FK
leg = pdGravCompController()
p_des, p_meas = [], []
for data_point in recorded_data:
    # print(data_point)
    p_des.append(leg.calculate_FK([data_point[2], data_point[8]]))
    p_meas.append(leg.calculate_FK([data_point[3], data_point[9]]))
p_des = np.array(p_des)
p_meas = np.array(p_meas)


######### PLOTTING #########
# 8 subplots for maximum readability
plt.rc('font', family='serif')
fig, axs = plt.subplots(4, 2, figsize=(10, 10))
fig.subplots_adjust(wspace=0.35)

# Plot shaded region as aerial phase
for i in range(4):
    for j in range(2):
        if i == 3 and j == 0:
            continue  # Skip axs[3][0] only
        plot_shaded_region(axs[i][j])

# Joint 1 Angle
axs[0][0].plot(time, q1_des, label='Desired', color='tab:blue', linestyle='--', alpha=0.5)
axs[0][0].plot(time, q1_meas, label='Measured', color='tab:blue')
axs[0][0].set_xlabel('Time (s)')
axs[0][0].set_ylabel('Hip Joint Angle (rad)')
axs[0][0].legend(fontsize=9)

# Joint 1 Velocity
axs[1][0].plot(time, q1d_des, label='Desired', color='tab:blue', linestyle='--', alpha=0.5)
axs[1][0].plot(time, q1d_meas, label='Measured', color='tab:blue')
axs[1][0].set_xlabel('Time (s)')
axs[1][0].set_ylabel('Hip Joint Velocity (rad/s)')

# Joint 1 Torque
axs[2][0].plot(time, u1_des, label='Desired', color='tab:red', linestyle='--', alpha=0.5)
axs[2][0].plot(time, u1_meas, label='Measured', color='tab:red')
axs[2][0].set_xlabel('Time (s)')
axs[2][0].set_ylabel('Hip Joint Torque (Nm)')
axs[2][0].legend(fontsize=9)

# Joint 2 Angle
axs[0][1].plot(time, q2_des, label='Desired', color='tab:orange', linestyle='--', alpha=0.5)
axs[0][1].plot(time, q2_meas, label='Measured', color='tab:orange')
axs[0][1].set_xlabel('Time (s)')
axs[0][1].set_ylabel('Knee Joint Angle (rad)')
axs[0][1].legend(fontsize=9)

# Joint 2 Velocity
axs[1][1].plot(time, q2d_des, label='Desired', color='tab:orange', linestyle='--', alpha=0.5)
axs[1][1].plot(time, q2d_meas, label='Measured', color='tab:orange')
axs[1][1].set_xlabel('Time (s)')
axs[1][1].set_ylabel('Knee Joint Velocity (rad/s)')

# Joint 2 Torque
axs[2][1].plot(time, u2_des, label='Desired', color='tab:red', linestyle='--', alpha=0.5)
axs[2][1].plot(time, u2_meas, label='Measured', color='tab:red')
axs[2][1].set_xlabel('Time (s)')
axs[2][1].set_ylabel('Knee Joint Torque (Nm)')

# Operation space desired vs actual
axs[3][0].plot(p_des[:, 0], p_des[:, 1], label='Desired', color='tab:green', linestyle='--', alpha=0.5)
axs[3][0].plot(p_meas[:, 0], p_meas[:, 1], label='Measured', color='tab:green')
axs[3][0].set_xlabel('X Position (m)')
axs[3][0].set_ylabel('Z Position (m)')
axs[3][0].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
axs[3][0].axis('equal')
axs[3][0].legend(fontsize=9)

# # IMU reading
# axs[3][1].plot(time, imu_g, label='IMU', color='tab:brown')
# axs[3][1].set_xlabel('Time (s)')
# axs[3][1].set_ylabel('Acceleration in Z (g)')

# Integrated IMU reading
axs[3][1].plot(time, int_vel_z, label='From IMU', color='tab:brown')
axs[3][1].plot(time, jac_vel[:, 1], label='From Jacobian', color='tab:olive')
# axs[3][1].plot(time, np.gradient(p_meas[:, 1], time), label='Velocity by Diff p_meas', color='tab:cyan')
axs[3][1].set_xlabel('Time (s)')
axs[3][1].set_ylabel('Z velocity (m/s)')
axs[3][1].legend(fontsize=9)

# Add subplot labels A) to H)
labels = ['A)', 'B)', 'C)', 'D)', 'E)', 'F)', 'G)', 'H)']
for idx, ax in enumerate(axs.flat):
    ax.text(-0.15, -0.1, labels[idx], transform=ax.transAxes, fontsize=10, va='top', ha='left', fontweight='bold')

plt.tight_layout()
plt.savefig(filepath.replace('.csv', '.png'), dpi=300)
plt.show()