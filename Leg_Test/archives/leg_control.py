from pybear import Manager
from pybear.CONTROL_TABLE import *
from MSCL import mscl
from math import pi
from Leg_Test.controllers.pd_grav_comp_controller import *
import matplotlib.pyplot as plt
import numpy as np
import sys
import time
import msvcrt  # For Windows keyboard input
import os
from datetime import datetime

### SEARCH BEAR ###
def search_bear(bear):
    searched_list = []
    for i in range(0, 10):
        # print("Pinging BEAR with ID %d" % i)
        data = bear.ping(i)
        if data[0] != None:
            print("Found BEAR with ID %d." % i)
            searched_list.append(i)
    return searched_list

### CONVERT GOAL TORQUE TO GOAL IQ, Cap motor current to align with iq
def torque2iq(u):
    iq_max = 20
    u1, u2 = u
    iq1 = u1 / bear_kt
    iq2 = u2 / bear_kt
    iq1 = np.clip(iq1, -iq_max, iq_max)
    iq2 = np.clip(iq2, -iq_max, iq_max)
    return [iq1, iq2]

### CALCULATE FRICTION COMPENSATION TORQUE ###
def calc_friction_torque(qd_rad_s):
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

### READ FEEDBACK FROM BEAR ###
def get_feedback(bear, bear_ids):
    """
        |  Read multiple status registers from a single motor in one packet.
        |  Multiple target motors can be visited but goes through them one-by-one
        |  e.g.: get_status((ID, reg1, reg2), (ID, reg1, reg2, reg3))
    """
    results = bear.get_status((1, "present_position", "present_velocity"),
                              (2, "present_position", "present_velocity"))
    # print("Bulk read results: ", results) 
    q1_rad = results[0][0][0]
    q1d_rad_s = results[0][0][1]
    q2_rad = results[1][0][0]
    q2d_rad_s = results[1][0][1]
    return np.array([q1_rad, q2_rad]), np.array([q1d_rad_s, q2d_rad_s])

### MOVE MOTORS WITH IQ ###
def move_motors(bear, bear_ids, goal_iq):
    # print("Set iq to: ", goal_iq)
    bear.set_goal_iq((1, goal_iq[0]), (2, goal_iq[1]))
    # bear.bulk_write(bear_ids, [STAT_REG.GOAL_POS], [[x] for x in goal_iq])
    return

### INITIALIZE FOOT SENSOR ###
def init_foot_sensor(COM_PORT = "COM20", GPIO_PIN = 1):
    try:
        #create a Serial Connection with the specified COM Port, default baud rate of 921600
        connection = mscl.Connection.Serial(COM_PORT)
        #create an InertialNode with the connection
        node = mscl.InertialNode(connection)
    except mscl.Error as e:
        print("Error:", e)
        exit(1)
    # To use a GPIO pin, we first need to configure it
    try:
        new_config = mscl.GpioConfiguration()
        new_config.pin = GPIO_PIN  # GPIO pin number
        new_config.feature = new_config.GPIO_FEATURE  # Set pin as GPIO
        new_config.behavior = new_config.GPIO_INPUT  # Set GPIO pin as input
        node.setGpioConfig(new_config)  # Apply the configuration
    except mscl.Error as e:
        print("Error:", e)
        exit(1)
    return node



# User-modifiable parameters
bear_baudrate = 8000000
bear_port = 'COM3'
sensor_port = 'COM20'
bear_ids = []
bear_kt = 1.16  # Nm/A, from BEAR SDK. Koala: 0.35, Koala MB: 1.16. 
p_des_1 = [0, -0.20]
p_des_2 = [0, -0.30]
# q_des_1_deg = np.array([-24, 42])
# q_des_2_deg = np.array([-90, 120])
# q_des_1 = np.deg2rad(q_des_1_deg)
# q_des_2 = np.deg2rad(q_des_2_deg)
Kp = np.diag([8, 12])
Kd = np.diag([0.5, 0.5])
# Kp = np.diag([0, 0])
# Kd = np.diag([0, 0])
max_duration_s = 5  # cap recording time
Ts = 0.001  # sampling rate 500 Hz
num_samples = 2000


# Initialize bear manager
bear = Manager.BEAR(port=bear_port, baudrate=bear_baudrate)
# Initialize foot sensor
foot_sensor = init_foot_sensor(COM_PORT=sensor_port)

# Search for bear IDs if prompted
if '--search' in sys.argv or not bear_ids:
    bear_ids = search_bear(bear)
if not bear_ids:
    print("no bear actuators found")
    sys.exit(1)
# print(bear_ids)

# # First use direct fore mode to move to zero position (leg fully extended) 
# # Configure BEAR. NOTE: Check whether the iq gains are at default?
# for id in bear_ids:
#     bear.set_torque_enable((id, 0))
#     bear.set_mode((id, 3))
#     bear.set_p_gain_force((id,1.0))
#     bear.set_i_gain_force((id,0.0))
#     bear.set_d_gain_force((id,0.2))
#     time.sleep(0.1)
#     bear.set_torque_enable((id, 1))

# Disable torque, switch to torque mode, and enable torque.
for id in bear_ids:
    bear.set_torque_enable((id, 0))
    bear.set_mode((id, 0))
    bear.set_torque_enable((id, 1))

controller = pdGravCompController()  # Leave blank if using the default values
# controller = pd_grav_comp_control(l1, l2, lc1, lc2, m1, m2)
q_des_1 = controller.calculate_IK(p_des_1)
q_des_2 = controller.calculate_IK(p_des_2)
print(f"q_desired: {q_des_1}, {q_des_2}")
q_des = q_des_2  # Start with q_des_2

print("Current state: ", get_feedback(bear, bear_ids))
input("Press Enter to start control loop and toggle target position. Press Esc anytime to exit.")

friction_flag = True
gravity_flag = True

pos_desired = []
pos_measured = []

start_time = time.time()

while True:
    # Get feedback
    q_rad, qd_rad_s = get_feedback(bear, bear_ids)
    # print("GPIO State: ", foot_sensor.getGpioState(1))  # Read foot sensor state

    # save data for plotting
    current_time = time.time() - start_time
    pos_desired.append([current_time, q_des[0], q_des[1]])
    pos_measured.append([current_time, q_rad[0], q_rad[1]])
    if len(pos_desired) > num_samples:
        print("Max duration reached, stopping data collection.")
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
        if key == b'f':
            friction_flag = not friction_flag
            print(f"Friction compensation {'enabled' if friction_flag else 'disabled'}.")
        if key == b'g':
            gravity_flag = not gravity_flag
            print(f"Gravity compensation {'enabled' if gravity_flag else 'disabled'}.")
        elif key == b'\x1b':  # Esc key
            print("Esc pressed. Exiting control loop...")
            for id in bear_ids:
                bear.set_torque_enable((id, 0))
            break

    # Compute control action
    phase = 'AERIAL' if foot_sensor.getGpioState(1) else 'STANCE'
    # print(phase)
    u = controller.calc_control_torque(q_des, q_rad, qd_rad_s, Kp, Kd, phase)
    u_f = calc_friction_torque(qd_rad_s)  # Add friction compensation torque
    # u_f_formatted = np.array2string(u_f, formatter={'float_kind': lambda x: f"{x: .6f}"})
    # print(f"calculated friction compensation torque: {u_f_formatted}")

    # Command control actions
    if gravity_flag and friction_flag:
        u = u + u_f
    elif gravity_flag:
        u = u
    elif friction_flag:
        u = u_f
    else:
        u = np.zeros_like(u)
    # print(f"calculated control torque: {u}")
    move_motors(bear, bear_ids, torque2iq(u))

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