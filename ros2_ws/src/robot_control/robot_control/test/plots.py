from robot_control.robot_utilities import robot_kinematics
from robot_control.robot_utilities import robot_parser
from robot_control.controllers import impedence_trajectories
import numpy as np
import os
import csv
import matplotlib.pyplot as plt

def load_csv_to_dict(file_path):
    with open(file_path, mode='r') as file:
        reader = csv.DictReader(file)
        data_dict = {key: [] for key in reader.fieldnames}  # Initialize dictionary with column names
        for row in reader:
            for key in row:
                try:
                    # Check if the value can be converted to a float
                    data_dict[key].append(float(row[key]))
                except ValueError:
                    # Handle empty or invalid data
                    data_dict[key].append(None)  # Append None for invalid data
    return data_dict

# Example usage
file_path = 'src/robot_control/robot_control/test/consecutive_jumps.csv'
data = load_csv_to_dict(file_path)
print(list(data.keys()))

time = np.array(data['__time'])
time -= time[0]  # Normalize time to start from 0

x_d_x = np.array(data['/x_d/pose/position/x'])
x_d_z = np.array(data['/x_d/pose/position/z'])

x_x = np.array(data['/x/pose/position/x'])
x_z = np.array(data['/x/pose/position/z'])

q_hip = np.array(data['/joint_states/hip_joint/position'])
q_knee = np.array(data['/joint_states/knee_joint/position'])
qd_hip = np.array(data['/joint_states/hip_joint/velocity'])
qd_knee = np.array(data['/joint_states/knee_joint/velocity'])

print(x_d_z)
plt.figure()
plt.plot(time, x_d_x, label='Desired X Position')
plt.show()

robot = robot_kinematics.Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
