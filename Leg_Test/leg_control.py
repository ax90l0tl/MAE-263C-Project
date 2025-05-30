from pybear import Manager
from pybear.CONTROL_TABLE import *
from math import pi
from pd_grav_comp_control import *
import numpy as np
import sys
import time

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

### CONVERT GOAL TORQUE TO GOAL IQ
def torque2iq(u):
    u1, u2 = u
    iq1 = u1 / bear_kt
    iq2 = u2 / bear_kt
    return [iq1, iq2]

### READ FEEDBACK FROM BEAR ###
def get_feedback(bear, bear_ids):
    q1_rad = 0
    q2_rad = 0
    q1d_rad_s = 0
    q2d_rad_s = 0
    """
    bulk_read(m_ids, read_registers, error_mode = 0), with parameters:
    m_ids: list of IDs
    read_registers: list of regirsters to read
    error_mode: 0(default): return[None, -99] for BEAR with corrupted data;
                1: return None as long as there is any error
    """
    results = bear.bulk_read(bear_ids, [STAT_REG.PRESENT_POS, STAT_REG.PRESENT_VEL])
    print("Bulk read results: ", results) 
    return np.array([q1_rad, q2_rad]), np.array([q1d_rad_s, q2d_rad_s])

def move_motors(bear, bear_ids, goal_iq):
    """
    bulk_write(m_ids, write_registers, write_data), with parameters:
    m_ids: list of IDs
    write_registers: list of regirsters to write to
    write_data:
        list of data-list to write [[ID1-data1, ID1-data2 ...],
                                    [ID2-data1, ID2-data2 ...] ...]
    """
    bear.bulk_write(bear_ids, [STAT_REG.GOAL_POS], [[x] for x in goal_iq])
    return



# User-modifiable parameters
bear_baudrate = 8000000
devicename = 'COM3'
bear_ids = []
bear_kt = 1.16  # Nm/A, from BEAR SDK. Koala: 0.35, Koala MB: 1.16. 
q_des_1 = np.array([0, 0])
q_des_2 = np.array([-pi/4, pi/4])
Kp = np.asarray([0, 0])
Kd = np.asarray([0, 0])
max_duration_s = 3  # Hold position for 3 seconds
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

# First use direct fore mode to move to zero position (leg fully extended) 
# Configure BEAR. NOTE: Check whether the iq gains are at default?
for id in bear_ids:
    bear.set_torque_enable(id, 0)
    bear.set_mode(id, 3)
    bear.set_p_gain_force(id,1.0)
    bear.set_i_gain_force(id,0.0)
    bear.set_d_gain_force(id,0.2)
    time.sleep(0.1)
    bear.set_torque_enable(id, 1)

# Disable torque, switch to torque mode, and enable torque.
for id in bear_ids:
    bear.set_torque_enable(id, 0)
    bear.set_mode(id, 0)
    bear.set_torque_enable(id, 1)

input("Press Enter to start control loop. Ctrl-C to exit.")

q_des = q_des_2

controller = pd_grav_comp_control()  # Leave blank if using the default values
# controller = pd_grav_comp_control(l1, l2, lc1, lc2, m1, m2)

start_time = time.time()
while True:
    # Get feedback
    q_rad, qd_rad_s = get_feedback(bear, bear_ids)

    # (Optional) save joint position/velocity and timestamp
    # CODE TO SAVE DATAPOINTS

    # Check termination criterion
    if time.time - start_time > max_duration_s:
        print("Time limit reached. Exiting control loop...")
        break
    
    # Compute error term
    # Taken care of in controller class

    # Compute control action
    u = controller.calc_control_torque(q_des, q_rad, qd_rad_s, Kp, Kd)

    # Command control actions
    move_motors(bear, bear_ids, torque2iq(u))

    # Delay based on sampling rate
    time.sleep(Ts)



