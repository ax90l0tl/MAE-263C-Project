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

def plot_shaded_region(ax):
    for start, end in zip(starts, ends):
        ax.axvspan(start, end, color='tab:purple', alpha=0.2)


# Load data from CSV file and trim if needed
filepaths = [
    "pd_w_gravity_jump/pd_jump_kp_8_12_kd_0.6_0.6.csv",
    "pd_w_gravity_jump/pd_jump_kp_10_15_kd_0.6_0.6.csv",
    "pd_w_gravity_jump/pd_jump_kp_10_18_kd_0.6_0.8.csv"
]
# Define custom trim values for each dataset
front_trims = [550, 500, 580]  # Example: adjust as needed per dataset
back_trims = [650, 700, 620]   # Example: adjust as needed per dataset

recorded_data_list = []
for i, filepath in enumerate(filepaths):
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")
    data = np.loadtxt(filepath, delimiter=',', skiprows=1)
    front_trim = front_trims[i]
    back_trim = back_trims[i]
    trimmed_data = data[front_trim:len(data)-back_trim]
    recorded_data_list.append(trimmed_data)
# Use the first set for the rest of the script for now
recorded_data = recorded_data_list[0]

# Process foot sensor to graph shaded region for each dataset
starts_list = []
ends_list = []
for idx, recorded_data in enumerate(recorded_data_list):
    time = recorded_data[:, 0]
    contact_raw = recorded_data[:, 1]
    contact_clean = (contact_raw > 0.5).astype(int)  # Add debouncing here later if needed
    contact_diff = np.diff(contact_clean, prepend=0)
    starts = time[contact_diff == 1]
    ends = time[contact_diff == -1]
    starts_list.append(starts)
    ends_list.append(ends)
    # print(f"Dataset {idx}: starts = {starts}, ends = {ends}")

# For plotting, use the first dataset's starts/ends as before
starts = starts_list[1]
ends = ends_list[1]
time = recorded_data_list[1][:, 0]

# Assign columns to np arrays
q1_des = [data[:, 2] for data in recorded_data_list]
q1_meas = [data[:, 3] for data in recorded_data_list]
q1d_des = [data[:, 4] for data in recorded_data_list]
q1d_meas = [data[:, 5] for data in recorded_data_list]
u1_des = [data[:, 6] for data in recorded_data_list]
u1_meas = [data[:, 7] for data in recorded_data_list]
q2_des = [data[:, 8] for data in recorded_data_list]
q2_meas = [data[:, 9] for data in recorded_data_list]
q2d_des = [data[:, 10] for data in recorded_data_list]
q2d_meas = [data[:, 11] for data in recorded_data_list]
u2_des = [data[:, 12] for data in recorded_data_list]
u2_meas = [data[:, 13] for data in recorded_data_list]

######### PLOTTING #########
# 2 subplots of hip and knee joints
plt.rc('font', family='serif')
fig, axs = plt.subplots(2, 1, figsize=(8, 10))
fig.subplots_adjust(wspace=0.35)

plot_shaded_region(axs[0])
plot_shaded_region(axs[1])

# Joint 1 Angle
axs[0].plot(time, np.rad2deg(q1_des[1]), label='Desired', color='tab:gray', linestyle='--', alpha=0.5)
axs[0].plot(time, np.rad2deg(q1_meas[0]), label='kp: [8,12] kd: [0.6,0.6]', color='tab:purple')
axs[0].plot(time, np.rad2deg(q1_meas[1]), label='kp: [10,15] kd: [0.6,0.6]', color='tab:brown')
axs[0].plot(time, np.rad2deg(q1_meas[2]), label='kp: [10,18] kd: [0.6,0.8]', color='tab:cyan')
axs[0].set_xlabel('Time (s)', fontsize=12)
axs[0].set_ylabel('Hip Joint Angle (deg)', fontsize=12)
axs[0].legend(fontsize=12)
axs[0].tick_params(axis='both', labelsize=12)
# Joint 2 Angle
axs[1].plot(time, np.rad2deg(q2_des[1]), label='Desired', color='tab:gray', linestyle='--', alpha=0.5)
axs[1].plot(time, np.rad2deg(q2_meas[0]), label='kp: [8,12] kd: [0.6,0.6]', color='tab:purple')
axs[1].plot(time, np.rad2deg(q2_meas[1]), label='kp: [10,15] kd: [0.6,0.6]', color='tab:brown')
axs[1].plot(time, np.rad2deg(q2_meas[2]), label='kp: [10,18] kd: [0.6,0.8]', color='tab:cyan')
axs[1].set_xlabel('Time (s)', fontsize=12)
axs[1].set_ylabel('Knee Joint Angle (deg)', fontsize=12)
axs[1].legend(fontsize=12)
axs[1].tick_params(axis='both', labelsize=12)

# Add subplot labels A) to H)
labels = ['A)', 'B)']
for idx, ax in enumerate(axs.flat):
    ax.text(-0.1, -0.1, labels[idx], transform=ax.transAxes, fontsize=12, va='top', ha='left', fontweight='bold')

plt.tight_layout()
plt.savefig('pd_w_gravity_jump/fig_results_gain_comparison.png', dpi=300)
plt.show()