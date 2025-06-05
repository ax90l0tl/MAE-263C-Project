from pybear import Manager
from pybear.CONTROL_TABLE import *
from MSCL import mscl
from math import pi
from pd_grav_comp_control import *
import numpy as np
import sys
import time
import msvcrt  # For Windows keyboard input

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
p_des_1 = [0, -0.18]
p_des_2 = [0, -0.30]
# q_des_1_deg = np.array([-24, 42])
# q_des_2_deg = np.array([-90, 120])
# q_des_1 = np.deg2rad(q_des_1_deg)
# q_des_2 = np.deg2rad(q_des_2_deg)
Kp = np.diag([15, 20])
Kd = np.diag([1, 1])
max_duration_s = 10  # Hold position for 3 seconds
Ts = 0.002  # sampling rate 500 Hz



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

controller = pd_grav_comp_control()  # Leave blank if using the default values
# controller = pd_grav_comp_control(l1, l2, lc1, lc2, m1, m2)
q_des_1 = controller.calculate_IK(p_des_1)
q_des_2 = controller.calculate_IK(p_des_2)
print(f"q_desired: {q_des_1}, {q_des_2}")
q_des = q_des_2  # Start with q_des_2

print("Current state: ", get_feedback(bear, bear_ids))
input("Press Enter to start control loop and toggle target position. Press Esc anytime to exit.")

while True:
    # Get feedback
    q_rad, qd_rad_s = get_feedback(bear, bear_ids)
    # print("GPIO State: ", foot_sensor.getGpioState(1))  # Read foot sensor state

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
            for id in bear_ids:
                bear.set_torque_enable((id, 0))
            break

    # Compute control action
    phase = 'AERIAL' if foot_sensor.getGpioState(1) else 'STANCE'
    print(phase)
    u = controller.calc_control_torque(q_des, q_rad, qd_rad_s, Kp, Kd, phase)

    # Command control actions
    move_motors(bear, bear_ids, torque2iq(u))

    # Delay based on sampling rate
    time.sleep(Ts)


