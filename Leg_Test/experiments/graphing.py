import matplotlib.pyplot as plt
import sys, os
from datetime import datetime
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'controllers')))
from pd_grav_comp_controller import pdGravCompController

'''
Expected data format in CSV:
First row are headers for the columns:
Timestamp, foot_state, q1_des, q1_meas, q1d_des, q2d_meas, u1_des, u1_meas, q2_des, q2_meas, q2d_des, q2d_meas, u2_des, u2_meas
'''

def plot_shaded_region(ax):
    for start, end in zip(starts, ends):
        ax.axvspan(start, end, color='tab:purple', alpha=0.2)


# Load data from CSV file and trim if needed
filepath = "pd_w_gravity_jump/pd_jump_20250610_2.csv"
if not os.path.isfile(filepath):
    raise FileNotFoundError(f"File not found: {filepath}")
recorded_data = np.loadtxt(filepath, delimiter=',', skiprows=1)
# Manually trim data so that it starts and ends at stance phase.
front_trim = 400
back_trim = len(recorded_data) - 500
recorded_data = recorded_data[front_trim:back_trim]

# Process foot sensor to graph shaded region
time = recorded_data[:, 0]
contact_raw = recorded_data[:, 1]
contact_clean = (contact_raw > 0.5).astype(int)  # Add debouncing here later if needed
contact_diff = np.diff(contact_clean, prepend=0)
starts = time[contact_diff == 1]
ends = time[contact_diff == -1]

# Process desired and measured joint info


# Converts joint angles to operation space with FK
leg = pdGravCompController()
# TODO: Perform FK on desired and actual joint angles to get desired and actual
# positions in operation space
p_des, p_meas = [], []
for data_point in recorded_data:
    # print(data_point)
    p_des.append(leg.calculate_FK([data_point[2], data_point[8]]))
    p_meas.append(leg.calculate_FK([data_point[3], data_point[9]]))
p_des = np.array(p_des)
p_meas = np.array(p_meas)

# First plot contains 8 subplots for maximum readability
fig, axs = plt.subplots(4, 2, figsize=(10, 10))

# Plot shaded region as aerial phase
for i in range(4):
    for j in range(2):
        if i == 3 and j == 0:
            continue  # Skip axs[3][0] only
        plot_shaded_region(axs[i][j])

# Joint 1 Angle
axs[0][0].plot(time, recorded_data[:, 2], label='Desired', color='tab:blue', linestyle='--', alpha=0.5)
axs[0][0].plot(time, recorded_data[:, 3], label='Measured', color='tab:blue')
axs[0][0].set_xlabel('Time (s)')
axs[0][0].set_ylabel('Hip Joint Angle (rad)')
# axs[0][0].set_title('Joint Angle Desired vs Measured')
axs[0][0].legend()
# axs[0][0].grid(True)

# Joint 1 Velocity
axs[1][0].plot(time, recorded_data[:, 4], label='Desired', color='tab:blue', linestyle='--', alpha=0.5)
axs[1][0].plot(time, recorded_data[:, 5], label='Measured', color='tab:blue')
axs[1][0].set_xlabel('Time (s)')
axs[1][0].set_ylabel('Hip Joint Velocity (rad/s)')
# axs[1][0].legend()

# Joint 1 Torque
axs[2][0].plot(time, recorded_data[:, 6], label='Desired', color='tab:red', linestyle='--', alpha=0.5)
axs[2][0].plot(time, recorded_data[:, 7], label='Measured', color='tab:red')
axs[2][0].set_xlabel('Time (s)')
axs[2][0].set_ylabel('Hip Joint Torque (Nm)')
axs[2][0].legend()

# Joint 2 Angle
axs[0][1].plot(time, recorded_data[:, 8], label='Desired', color='tab:orange', linestyle='--', alpha=0.5)
axs[0][1].plot(time, recorded_data[:, 9], label='Measured', color='tab:orange')
axs[0][1].set_xlabel('Time (s)')
axs[0][1].set_ylabel('Knee Joint Angle (rad)')
axs[0][1].legend()

# Joint 2 Velocity
axs[1][1].plot(time, recorded_data[:, 10], label='Desired', color='tab:orange', linestyle='--', alpha=0.5)
axs[1][1].plot(time, recorded_data[:, 11], label='Measured', color='tab:orange')
axs[1][1].set_xlabel('Time (s)')
axs[1][1].set_ylabel('Knee Joint Velocity (rad/s)')

# Joint 2 Torque
axs[2][1].plot(time, recorded_data[:, 12], label='Desired', color='tab:red', linestyle='--', alpha=0.5)
axs[2][1].plot(time, recorded_data[:, 13], label='Measured', color='tab:red')
axs[2][1].set_xlabel('Time (s)')
axs[2][1].set_ylabel('Knee Joint Torque (Nm)')

# Operation space desired vs actual
axs[3][0].plot(p_des[:, 0], p_des[:, 1], label='Desired', color='tab:green', linestyle='--', alpha=0.5)
axs[3][0].plot(p_meas[:, 0], p_meas[:, 1], label='Measured', color='tab:green')
axs[3][0].set_xlabel('X Position (m)')
axs[3][0].set_ylabel('Y Position (m)')
axs[3][0].axis('equal')

# # Operation space desired vs actual
# axs[3][0].plot(time, p_des[:, 0], label='Desired', color='tab:green', linestyle='--', alpha = 0.5)
# axs[3][0].plot(time, p_meas[:, 0], label='Measured', color='tab:green')
# axs[3][0].set_xlabel('Time (s)')
# axs[3][0].set_ylabel('X Position (m)')

# # Operation space desired vs actual
# axs[3][1].plot(time, p_des[:, 1], label='Desired', color='tab:green', linestyle='--', alpha = 0.5)
# axs[3][1].plot(time, p_meas[:, 1], label='Measured', color='tab:green')
# axs[3][1].set_xlabel('Time (s)')
# axs[3][1].set_ylabel('Y Position (m)')


plt.tight_layout()
plt.show()