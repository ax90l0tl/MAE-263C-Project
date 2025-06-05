from pybear import Manager
from pybear.CONTROL_TABLE import *
import sys
import numpy as np
import time
import os
import matplotlib.pyplot as plt

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
    iq = u / bear_kt
    if iq > iq_max:
        iq = iq_max
    elif iq < -iq_max:
        iq = -iq_max
    return iq

# User-modifiable parameters
bear_baudrate = 8000000
bear_port = 'COM3'
bear_ids = []
bear_kt = 1.16  # Nm/A, from BEAR SDK. Koala: 0.35, Koala MB: 1.16.
u_range = [-1, 1]
n_samples = 25  # Number of samples between 0 and positive/negative u max
delay = 2       # Delay between torque commands in seconds

# Initialize bear manager
bear = Manager.BEAR(port=bear_port, baudrate=bear_baudrate)

# Search for bear IDs if prompted
if '--search' in sys.argv or not bear_ids:
    bear_ids = search_bear(bear)
if not bear_ids:
    print("no bear actuators found")
    sys.exit()
print("Found BEAR: ", bear_ids)

# (First BEAR only) Disable torque, switch to torque mode, and enable torque.
bear.set_torque_enable((bear_ids[0], 0))
bear.set_mode((bear_ids[0], 0))
bear.set_torque_enable((bear_ids[0], 1))

u_positive = np.linspace(0, u_range[1], n_samples)
u_negative = np.linspace(0, u_range[0], n_samples)

data = []

for u in u_positive:
    print(f"Applying positive torque: {u:.2f} Nm")
    bear.set_goal_iq((bear_ids[0], torque2iq(u)))
    time.sleep(delay)
    result = bear.get_present_velocity((bear_ids[0]))
    vel = result[0][0][0]
    print(f"Resulting joint velocity: {vel}")
    data.append([vel, u])


for u in u_negative:
    print(f"Applying negative torque: {u:.2f} Nm")
    bear.set_goal_iq((bear_ids[0], torque2iq(u)))
    time.sleep(delay)
    result = bear.get_present_velocity((bear_ids[0]))
    vel = result[0][0][0]
    print(f"Resulting joint velocity: {vel}")
    data.append([vel, u])

# Disable torque and exit
bear.set_torque_enable((bear_ids[0], 0))

data = np.array(data)
data = data[data[:, 0].argsort()]  # Sort by velocity (first column)

# Save logic
csv_base = "friction_id_KM115"
csv_dir = "."  # or "output"
os.makedirs(csv_dir, exist_ok=True)
i = 0
while True:
    csv_path = os.path.join(csv_dir, f"{csv_base}_{i}.csv")
    if not os.path.exists(csv_path):
        break
    i += 1
np.savetxt(csv_path, data, delimiter=",", header="velocity,torque", comments='')

print(f"Saved to: {csv_path}")

# Plot the data
plt.figure()
plt.scatter(data[:, 0], data[:, 1])
plt.xlabel('Joint Velocity (rad/s)')
plt.ylabel('Applied Torque (Nm)')
plt.title('Friction Identification: Torque vs. Velocity')
plt.grid(True)
plt.show()



