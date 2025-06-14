from leg_manager import legManager
from math import pi
from controllers.conrad_controller import *
import matplotlib.pyplot as plt
import numpy as np
import sys
import time
import msvcrt  # For Windows keyboard input
import os
from datetime import datetime
import logging

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

# User-modifiable parameters
bear_port = 'COM3'
sensor_port = 'COM20'
land_delay_s = 2     # seconds to wait after landing before returning to IDLE
max_duration_s = 5   # cap recording time
Ts = 0.001           # add delay between loops
num_samples = 2000

# Desired positions in operation space
P_DES = {
    'IDLE': [0, -0.25],
    'AERIAL': [0 -0.2625],
    'STANCE': [-0.005 -0.275],
}

# Gains (Aerial and Stance phases in task space, Jump phase in joint space)
KP = {
    'IDLE': np.diag([3, 4]),
    'AERIAL': np.diag(250 * [1, 2]),
    'JUMP': np.diag(1.5 * [1, 0.75]),
    'STANCE': np.diag(150 * [1, 1.25]),
    }
KD = {
    'IDLE': np.diag([0.2, 0.2]),
    'AERIAL': np.diag(100 * [1, 0.875]),
    'JUMP': np.diag(0.0875 * [1, 1]),
    'STANCE': np.diag(15* [1, 1]),
}


# Initialize bear manager and desired controller
leg  = legManager(bear_port=bear_port, sensor_port=sensor_port)
controller = ConradController()  # Leave blank if using the default values

# Prompt user to start control loop
input("Press Enter to start control loop. Press Esc anytime to exit.")
leg.enable_torque(1)

# # For data collection
# pos_desired = []
# pos_measured = []
# start_time = time.time()
current_state = 'IDLE'  # Start in IDLE phase
q_des = controller.calculate_IK(P_DES[current_state])
input(f"Current state: {current_state}. Press Enter to continue to AERIAL state.")
# Finite state machine
current_state = 'AERIAL'  # Start in AERIAL phase
elapsed_time = None  # Initialize elapsed time for LAND state
leg.jump_flag = False  # Initialize jump flag
leg.previous_contact = False  # Initialize previous contact state
leg.stance_time = 0.75
# q_des = controller.calculate_IK(P_DES[current_state])

while True:
    # Get feedback
    iq, q_rad, qd_rad_s = leg.get_feedback()

    # # save data for plotting
    # current_time = time.time() - start_time
    # pos_desired.append([current_time, q_des[0], q_des[1]])
    # pos_measured.append([current_time, q_rad[0], q_rad[1]])
    # if len(pos_desired) > num_samples:
    #     print("Max duration reached, exiting data collection.")
    #     break

    # Get foot sensor state, see if on the ground or in the air
    # Determine the phase based on foot sensor state (HIGH means in the air, LOW means on the ground)
    if leg.get_foot_sensor_state():
        current_state = 'AERIAL'
        leg.previous_contact = False  # Update previous phase
    else:
        # Check a flag to determine if phase should be JUMP or STAND
        if hasattr(leg, 'stand_flag') and leg.jump_flag:
            current_state = 'JUMP'
            leg.previous_contact = True  # Update previous phase
        else:
            current_state = 'STAND'
            leg.previous_contact = True  # Update previous phase
    if leg.get_foot_sensor_state() == False and leg.previous_contact == False:
        # If the foot sensor was triggered with contact, and the previous phase was aerial, set contact time
        contact_time = time.time()  # Record the time of contact
        logger.info(f"Foot sensor triggered. Contact time recorded at {contact_time:.2f} seconds.")


    # State transitions
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'\r':  # Enter key
            if current_state == 'IDLE':
                current_state = 'AERIAL'
                logger.info("State changed to AERIAL")
            elif leg.jump_flag == True and current_state == 'AERIAL':  # Ensures that we don't switch to STAND if we are in JUMP phase
                leg.jump_flag = False  # Set jump flag to true
            else:
                leg.jump_flag = True  # Set jump flag to true
        elif key == b'\x1b':  # Esc key
            if current_state == 'STAND':
                logger.info("Esc pressed. Exiting control loop...")
                leg.enable_torque(0)
            else:
                logger.info("Wait until stance to exit control loop next time . :)")
                leg.enable_torque(0)
            break
    kp, kd = KP[current_state], KD[current_state]

    # Compute control action
    u = controller.calc_control_torque(q_des, q_rad, qd_rad_s, kp, kd,current_state,leg)
    # print(f"calculated control torque: {u}")
    leg.move_motors(u)

    # Delay based on sampling rate
    time.sleep(Ts)

# final_time = time.time() - start_time
# print(f"True sampling rate: {len(pos_desired) / final_time:.2f} Hz")

# print(len(pos_desired), len(pos_measured))
# pos_desired = np.array(pos_desired)
# pos_measured = np.array(pos_measured)  

# fig, axs = plt.subplots(1, 2, figsize=(14, 6))

# # Joint 1
# axs[0].plot(pos_desired[:, 0], pos_desired[:, 1], label='Desired Position', linestyle='--')
# axs[0].plot(pos_measured[:, 0], pos_measured[:, 1], label='Measured Position')
# # Add desired position ±0.01 as dashed lines
# axs[0].plot(pos_desired[:, 0], pos_desired[:, 1] + 0.01, 'k--', alpha=0.5, linewidth=0.8, label='Desired +0.01 rad')
# axs[0].plot(pos_desired[:, 0], pos_desired[:, 1] - 0.01, 'k--', alpha=0.5, linewidth=0.8, label='Desired -0.01 rad')
# axs[0].set_xlabel('Time (s)')
# axs[0].set_ylabel('Joint 1 Position (rad)')
# axs[0].set_title('Joint 1: Desired vs Measured')
# axs[0].legend()
# axs[0].grid(True)

# # Joint 2
# axs[1].plot(pos_desired[:, 0], pos_desired[:, 2], label='Desired Position', linestyle='--')
# axs[1].plot(pos_measured[:, 0], pos_measured[:, 2], label='Measured Position')
# # Add desired position ±0.01 as dashed lines
# axs[1].plot(pos_desired[:, 0], pos_desired[:, 2] + 0.01, 'k--', alpha=0.5, linewidth=0.5, label='Desired +0.01 rad')
# axs[1].plot(pos_desired[:, 0], pos_desired[:, 2] - 0.01, 'k--', alpha=0.5, linewidth=0.5, label='Desired -0.01 rad')
# axs[1].set_xlabel('Time (s)')
# axs[1].set_ylabel('Joint 2 Position (rad)')
# axs[1].set_title('Joint 2: Desired vs Measured')
# axs[1].legend()
# axs[1].grid(True)

# plt.tight_layout()
# plt.show()

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