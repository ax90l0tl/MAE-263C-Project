from pybear import Manager
from pybear.CONTROL_TABLE import *
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



# User-modifiable parameters
bear_baudrate = 8000000
devicename = 'COM3'
bear_ids = []
Ts = 0.002  # sampling rate 500 Hz



# Initialize bear manager
bear = Manager.BEAR(port=devicename, baudrate=bear_baudrate)

# Search for bear IDs if prompted
if '--search' in sys.argv or not bear_ids:
    bear_ids = search_bear(bear)
if not bear_ids:
    print("no bear actuators found")
    sys.exit(1)
# print(bear_ids)

# Configure BEAR. 
for id in bear_ids:
    bear.set_torque_enable((id, 0))
    bear.set_mode((id, 0))
    time.sleep(0.1)

controller = pd_grav_comp_control()  # Leave blank if using the default values
# controller = pd_grav_comp_control(l1, l2, lc1, lc2, m1, m2)

input("Press Enter to start listening to joint readings and calculate FK.")

deg = False
while True:
    # Get feedback
    q_rad, qd_rad_s = get_feedback(bear, bear_ids)
    q_deg, qd_deg_s = np.rad2deg(q_rad), np.rad2deg(qd_rad_s)
    if deg:
        print(f"joint positions: {np.round(q_deg, 2)}, unit: degrees. EE position: {np.round(controller.calculate_FK(q_rad), 2)}, unit: mm")
    else:
        print(f"joint positions: {np.round(q_rad, 2)}, unit: radins. EE position: {np.round(controller.calculate_FK(q_rad), 2)}, unit: mm")

    # Check for key press
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'd':  # Switch to degrees
            deg = True
        elif key == b'r':  # Switch to radians
            deg = False
        elif key == b'\x1b':  # Esc key
            print("Esc pressed. Exiting control loop...")
            for id in bear_ids:
                bear.set_torque_enable((id, 0))
            break

    # Delay based on sampling rate
    time.sleep(Ts)


