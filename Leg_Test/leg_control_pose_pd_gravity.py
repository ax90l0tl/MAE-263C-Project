from leg_manager import legManager
from math import pi
from controllers.pd_grav_comp_controller import *
import matplotlib.pyplot as plt
import numpy as np
import sys
import time
import msvcrt  # For Windows keyboard input
import os
from datetime import datetime


# User-modifiable parameters
bear_port = '/dev/ttyUSB0'
sensor_port = '/dev/ttyACM0'
p_des_1 = [0, -0.20]
p_des_2 = [0, -0.30]
max_duration_s = 5  # cap recording time
Ts = 0.001  # sampling rate 500 Hz
num_samples = 2000

# Gains
Kp = np.diag([8, 12])
Kd = np.diag([0.5, 0.5])
# Kp = np.diag([0, 0])
# Kd = np.diag([0, 0])

# Initialize bear manager and desired controller
leg  = legManager(bear_port=bear_port, sensor_port=sensor_port)
controller = pdGravCompController()  # Leave blank if using the default values

# Define trajectory or desired states
q_des_1 = controller.calculate_IK(p_des_1)
q_des_2 = controller.calculate_IK(p_des_2)
q_des = q_des_2  # Start with q_des_2

# Prompt user to start control loop
input("Press Enter to start control loop and toggle target position. Press Esc anytime to exit.")
leg.enable_torque(1)

# For data collection
pos_desired = []
pos_measured = []
start_time = time.time()

while True:
    # Get feedback
    iq, q_rad, qd_rad_s = leg.get_feedback()

    # save data for plotting
    current_time = time.time() - start_time
    pos_desired.append([current_time, q_des[0], q_des[1]])
    pos_measured.append([current_time, q_rad[0], q_rad[1]])
    if len(pos_desired) > num_samples:
        print("Max duration reached, exiting data collection.")
        break

    # Check for key press
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'\r':  # Enter key
            if np.allclose(q_des, q_des_1):
                q_des = q_des_2
                print("Switched to q_des_2:", np.rad2deg(q_des_2))
            else:
                q_des = q_des_1
                print("Switched to q_des_1:", np.rad2deg(q_des_1))    
        elif key == b'\x1b':  # Esc key
            print("Esc pressed. Exiting control loop...")
            leg.enable_torque(0)
            break

    # Compute control action
    phase = 'AERIAL' if leg.get_foot_sensor_state() else 'STANCE'
    # print(phase)
    u = controller.calc_control_torque(q_des, q_rad, qd_rad_s, Kp, Kd, phase)
    # print(f"calculated control torque: {u}")
    leg.move_motors(u)

    # Delay based on sampling rate
    time.sleep(Ts)

final_time = time.time() - start_time
print(f"True sampling rate: {len(pos_desired) / final_time:.2f} Hz")

print(len(pos_desired), len(pos_measured))
pos_desired = np.array(pos_desired)
pos_measured = np.array(pos_measured)  

fig, axs = plt.subplots(1, 2, figsize=(14, 6))

# Joint 1
axs[0].plot(pos_desired[:, 0], pos_desired[:, 1], label='Desired Position', linestyle='--')
axs[0].plot(pos_measured[:, 0], pos_measured[:, 1], label='Measured Position')
# Add desired position ±0.01 as dashed lines
axs[0].plot(pos_desired[:, 0], pos_desired[:, 1] + 0.01, 'k--', alpha=0.5, linewidth=0.8, label='Desired +0.01 rad')
axs[0].plot(pos_desired[:, 0], pos_desired[:, 1] - 0.01, 'k--', alpha=0.5, linewidth=0.8, label='Desired -0.01 rad')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Joint 1 Position (rad)')
axs[0].set_title('Joint 1: Desired vs Measured')
axs[0].legend()
axs[0].grid(True)

# Joint 2
axs[1].plot(pos_desired[:, 0], pos_desired[:, 2], label='Desired Position', linestyle='--')
axs[1].plot(pos_measured[:, 0], pos_measured[:, 2], label='Measured Position')
# Add desired position ±0.01 as dashed lines
axs[1].plot(pos_desired[:, 0], pos_desired[:, 2] + 0.01, 'k--', alpha=0.5, linewidth=0.5, label='Desired +0.01 rad')
axs[1].plot(pos_desired[:, 0], pos_desired[:, 2] - 0.01, 'k--', alpha=0.5, linewidth=0.5, label='Desired -0.01 rad')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Joint 2 Position (rad)')
axs[1].set_title('Joint 2: Desired vs Measured')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()

# # Save logic
# csv_base = "pd_w_gravity"
# date_str = datetime.now().strftime("%Y%m%d")
# csv_base = f"pd_w_gravity_{date_str}"
# data = np.hstack([pos_desired, pos_measured[:, 1:]])
# csv_dir = "experiments/pd_w_gravity"
# os.makedirs(csv_dir, exist_ok=True)
# i = 0
# while True:
#     csv_path = os.path.join(csv_dir, f"{csv_base}_{i}.csv")
#     plot_path = os.path.join(csv_dir, f"{csv_base}_{i}.png")
#     if not os.path.exists(csv_path) and not os.path.exists(plot_path):
#         break
#     i += 1
# np.savetxt(csv_path, data, delimiter=",", header="time, q1_des, q2_des, q1_actual, q2_actual", comments='')
# fig.savefig(plot_path, dpi=300)
# print(f"Saved data to: {csv_path}")
# print(f"Saved plot to: {plot_path}")