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
import logging
import argparse


# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

# Argument parser setup
parser = argparse.ArgumentParser(description='Run PD control loop with optional data recording.')
parser.add_argument('--record', action='store_true', help='Enable data recording and plotting')
args = parser.parse_args()

# User-modifiable parameters
bear_port = 'COM9'
sensor_port = 'COM20'
land_delay_s = 2     # seconds to wait after landing before returning to IDLE
max_duration_s = 5   # cap recording time
Ts = 0.001           # add delay between loops
num_samples = 2000

# Desired positions in operation space
P_DES = {
    'IDLE': [0, -0.25],
    'PREP': [0, -0.18],
    'JUMP': [0, -0.32],
    'LAND': [0, -0.220],
}

# Gains
KP = {
    'IDLE': np.diag([3, 4]),
    'PREP': np.diag([1, 1.5]),
    'JUMP': np.diag([10, 18]),
    'LAND': np.diag([3, 4]),
    }
KD = {
    'IDLE': np.diag([0.2, 0.2]),
    'PREP': np.diag([0.1, 0.1]),
    'JUMP': np.diag([0.6, 0.8]),
    'LAND': np.diag([0.2, 0.2]),
}

# Initialize bear manager and desired controller
leg  = legManager(bear_port=bear_port, sensor_port=sensor_port)
controller = pdGravCompController()  # Leave blank if using the default values

# Prompt user to start control loop
input("Press Enter to start control loop and toggle target position. Press Esc anytime to exit.")
leg.enable_torque(1)

# For data collection
if args.record:
    logger.info("Data recording enabled.")
    recorded_data = []
    start_time = time.time()

# Finite state machine
current_state = 'IDLE'
q_des = controller.calculate_IK(P_DES[current_state])

while True:
    # Get feedback
    iq, q_rad, qd_rad_s = leg.get_feedback()
    foot_state = leg.get_foot_sensor_state()
    phase = 'AERIAL' if foot_state else 'STANCE'
    logger.debug(phase)

    # State transitions
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'\r':  # Enter key
            if current_state == 'IDLE':
                current_state = 'PREP'
                logger.info("State changed to PREP")
            elif current_state == 'PREP':
                if np.allclose(q_des, q_rad, atol = 0.2):
                    current_state = 'JUMP'
                    logger.info("State changed to JUMP") 
                else:
                    logger.warning("PREP pose not yet reached.")
                    print(f"Current pose: {np.rad2deg(q_rad)}, Desired pose: {np.rad2deg(q_des)}")
            q_des = controller.calculate_IK(P_DES[current_state])
        elif key == b'\x1b':  # Esc key
            logger.info("Esc pressed. Exiting control loop...")
            leg.enable_torque(0)
            break
    elif current_state == 'JUMP' and phase == 'AERIAL':
        current_state = 'LAND'
        q_des = controller.calculate_IK(P_DES[current_state])
        elapsed_time = None  # Reset elapsed time for LAND state
        logger.info("State changed to LAND")
    elif current_state == 'LAND' and phase == 'STANCE':
        if elapsed_time is None:
            elapsed_time = time.time()
            logger.info("Landed. Delaying before returning to IDLE...")
        else:
            if time.time() - elapsed_time >= land_delay_s:
                current_state = 'IDLE'
                q_des = controller.calculate_IK(P_DES[current_state])
                logger.info("State changed to IDLE")
    kp, kd = KP[current_state], KD[current_state]

    # Compute control action
    u = controller.calc_control_torque(q_des, q_rad, qd_rad_s, kp, kd, phase)
    logger.debug(f"calculated control torque: {u}")
    leg.move_motors(u)

    # save data for plotting
    if args.record:
        current_time = time.time() - start_time
        u_meas = leg.iq2torque(iq)
        recorded_data.append([current_time, foot_state,
                              q_des[0], q_rad[0], qd_rad_s[0], qd_rad_s[0], u[0], u_meas[0], 
                              q_des[1], q_rad[1], qd_rad_s[1], qd_rad_s[1], u[1], u_meas[1], 
                              ])
        if len(recorded_data) > num_samples:
            logger.info("Max duration reached, exiting data collection.")
            leg.enable_torque(0)
            break

    # Delay based on sampling rate
    time.sleep(Ts)

if args.record:
    final_time = time.time() - start_time
    recorded_data = np.array(recorded_data)
    logger.info(f"True sampling rate: {len(recorded_data) / final_time:.2f} Hz")
    logger.debug(f"Recorded data: {recorded_data}")

    # Save to CSV
    date_str = datetime.now().strftime("%Y%m%d")
    csv_base = f"pd_jump_{date_str}"
    csv_dir = "experiments/pd_w_gravity_jump"
    os.makedirs(csv_dir, exist_ok=True)
    i = 0
    while True:
        csv_path = os.path.join(csv_dir, f"{csv_base}_{i}.csv")
        plot_path = os.path.join(csv_dir, f"{csv_base}_{i}.png")
        if not os.path.exists(csv_path) and not os.path.exists(plot_path):
            break
        i += 1
    np.savetxt(csv_path, recorded_data, delimiter=",", 
               header="timestamp, foot_state, " \
               "q1_des, q1_meas, q1d_des, q2d_meas, u1_des, u1_meas, " \
               "q2_des, q2_meas, q2d_des, q2d_meas, u2_des, u2_meas", comments='')
    print(f"Saved data to: {csv_path}")

# # Plotting the recorded data
# fig, axs = plt.subplots(1, 2, figsize=(14, 6))

# # Joint 1
# axs[0].plot(recorded_data[:, 0], recorded_data[:, 1], label='Desired Position', linestyle='--')
# axs[0].plot(recorded_data[:, 0], recorded_data[:, 2], label='Measured Position')
# # Add desired position ±0.01 as dashed lines
# axs[0].plot(recorded_data[:, 0], recorded_data[:, 1] + 0.01, 'k--', alpha=0.5, linewidth=0.8, label='Desired +0.01 rad')
# axs[0].plot(recorded_data[:, 0], recorded_data[:, 1] - 0.01, 'k--', alpha=0.5, linewidth=0.8, label='Desired -0.01 rad')
# axs[0].set_xlabel('Time (s)')
# axs[0].set_ylabel('Joint 1 Position (rad)')
# axs[0].set_title('Joint 1: Desired vs Measured')
# axs[0].legend()
# axs[0].grid(True)

# # Joint 2
# axs[1].plot(recorded_data[:, 0], recorded_data[:, 5], label='Desired Position', linestyle='--')
# axs[1].plot(recorded_data[:, 0], recorded_data[:, 6], label='Measured Position')
# # Add desired position ±0.01 as dashed lines
# axs[1].plot(recorded_data[:, 0], recorded_data[:, 5] + 0.01, 'k--', alpha=0.5, linewidth=0.5, label='Desired +0.01 rad')
# axs[1].plot(recorded_data[:, 0], recorded_data[:, 5] - 0.01, 'k--', alpha=0.5, linewidth=0.5, label='Desired -0.01 rad')
# axs[1].set_xlabel('Time (s)')
# axs[1].set_ylabel('Joint 2 Position (rad)')
# axs[1].set_title('Joint 2: Desired vs Measured')
# axs[1].legend()
# axs[1].grid(True)

# fig.savefig(plot_path, dpi=300)
# print(f"Saved plot to: {plot_path}")
# plt.tight_layout()
# plt.show()