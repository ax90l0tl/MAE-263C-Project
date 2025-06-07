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
bear_port = 'COM3'
sensor_port = 'COM20'
Ts = 0.001  # sampling rate 500 Hz

# Initialize bear manager and desired controller
leg  = legManager(bear_port=bear_port, sensor_port=sensor_port)
controller = pdGravCompController()  # Leave blank if using the default values

input("Press Enter to start passive compenation demo." +
      "Use 'g' and 'f' to toggle gravity and friction compensation. " +
      "Press Esc anytime to exit.")
leg.enable_torque(1)

friction_flag = True
gravity_flag = True

pos_desired = []
pos_measured = []

start_time = time.time()

while True:
    # Get feedback
    iq, q_rad, qd_rad_s = leg.get_feedback()
    # print(f"iq: [{iq[0]:.6f}, {iq[1]:.6f}], q_rad: [{q_rad[0]:.6f}, {q_rad[1]:.6f}], qd_rad_s: [{qd_rad_s[0]:.6f}, {qd_rad_s[1]:.6f}]")
    # print(f"q_rad: [{q_rad[0]:.6f}, {q_rad[1]:.6f}]")

    # Check for key press
    if msvcrt.kbhit():
        key = msvcrt.getch() 
        if key == b'f':
            friction_flag = not friction_flag
            print(f"Friction compensation {'enabled' if friction_flag else 'disabled'}.")
        if key == b'g':
            gravity_flag = not gravity_flag
            print(f"Gravity compensation {'enabled' if gravity_flag else 'disabled'}.")
        elif key == b'\x1b':  # Esc key
            print("Esc pressed. Exiting control loop...")
            leg.enable_torque(0)
            break

    # Compute control actions
    phase = 'AERIAL' if leg.get_foot_sensor_state() else 'STANCE'
    # print(phase)
    u_g = controller.calc_grav_comp_torque(q_rad, phase)
    u_f = controller.calc_friction_torque(qd_rad_s)  # Add friction compensation torque
    u_g_formatted = np.array2string(u_g, formatter={'float_kind': lambda x: f"{x: .6f}"})
    u_f_formatted = np.array2string(u_f, formatter={'float_kind': lambda x: f"{x: .6f}"})
    # print(f"Compensation torque: friction: {u_f_formatted}, gravity: {u_g_formatted}")

    # Command control actions
    if gravity_flag and friction_flag:
        u = u_g + u_f
    elif gravity_flag:
        u = u_g
    elif friction_flag:
        u = u_f
    else:
        u = np.zeros_like(u)
    # print(f"calculated control torque: {u}")
    leg.move_motors(u)

    # Delay based on sampling rate
    time.sleep(Ts)