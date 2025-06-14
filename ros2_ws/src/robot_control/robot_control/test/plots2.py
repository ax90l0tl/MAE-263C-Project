from pathlib import Path
from robot_control.robot_utilities import robot_kinematics
from robot_control.robot_utilities import robot_parser
from robot_control.controllers import impedence_trajectories
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

bagpath = Path('ros2bag/single_jump')
typestore = get_typestore(Stores.ROS2_JAZZY)
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == '/joint_states']
    q = []
    qd = []
    q_time = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        q_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        q.append(msg.position[1:])
        qd.append(msg.velocity[1:])
    connections = [x for x in reader.connections if x.topic == '/x_d']
    x_d = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        x_d.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.position.x, msg.pose.position.z])
    connections = [x for x in reader.connections if x.topic == '/x']
    x = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        x.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.position.x, msg.pose.position.z])

    connections = [x for x in reader.connections if x.topic == '/imu']
    imu = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        imu.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.linear_acceleration.z])

    connections = [x for x in reader.connections if x.topic == '/hip_FT']
    hip_T = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        hip_T.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.wrench.torque.z])
    connections = [x for x in reader.connections if x.topic == '/knee_FT']
    knee_T = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        knee_T.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.wrench.torque.z])

    connections = [x for x in reader.connections if x.topic == '/contact']
    contact = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        contact.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.contacts[0].wrenches[0].body_1_wrench.force.z])

    connections = [x for x in reader.connections if x.topic == '/tf']
    tf = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        # print(msg.transforms[2].transform.translation.z)
        tf.append([msg.transforms[2].header.stamp.sec + msg.transforms[2].header.stamp.nanosec * 1e-9, msg.transforms[2].transform.translation.z])
    

tf = np.array(tf)
imu = np.array(imu)
contact = np.array(contact)
x = np.array(x)
x_d = np.array(x_d)
hip_T = np.array(hip_T)
knee_T = np.array(knee_T)
q = np.array(q)
qd = np.array(qd)
q_time = np.array(q_time)
q_time -= q_time[0]
imu[:, 0] -= imu[0, 0]
tf[:, 0] -= tf[0, 0]
contact[:, 0] -= contact[0, 0]
x[:, 0] -= x[0, 0]
x_d[:, 0] -= x_d[0, 0]
hip_T[:, 0] -= hip_T[0, 0]
knee_T[:, 0] -= knee_T[0, 0]
print(q.shape, qd.shape, q_time.shape, x.shape, x_d.shape, imu.shape, contact.shape, hip_T.shape, knee_T.shape)
robot = robot_kinematics.Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
xd = np.zeros((q.shape[0], 2))
xdd = np.zeros((q.shape[0], 2))
for i in range(q.shape[0]):
    J = robot.jacobian(q[i])
    xd_temp = (J @ np.atleast_2d(qd[i]).T)
    xd[i] = xd_temp[[0, 2]].T




plt.rc('font', family='serif')
fig, axs = plt.subplots(4, 2, figsize=(10, 10))
fig.subplots_adjust(wspace=0.35)

start = 0
end = 0
mean = np.mean(contact[1:, 0] - contact[:-1, 0])
print("Mean contact time:", mean)

for i in range(len(contact) - 1):
    if contact[i + 1, 0] - contact[i, 0] > mean *2.0:
        print(contact[i + 1, 0], contact[i, 0])
        start = contact[i, 0]
        end = contact[i + 1, 0]
        print(start, end)
        break


def plot_shaded_region(ax):
    # for start, end in zip(starts, ends):
    ax.axvspan(start, end, color='tab:purple', alpha=0.2)

# Plot shaded region as aerial phase
for i in range(4):
    for j in range(2):
        if i == 3 and j == 0:
            continue  # Skip axs[3][0] only
        plot_shaded_region(axs[i][j])

# X Position
axs[0][0].plot(x_d[:, 0], x_d[:, 1], label='Desired', color='tab:blue', linestyle='--', alpha=0.5)
axs[0][0].plot(x[:, 0], x[:, 1], label='Measured', color='tab:blue')
axs[0][0].set_xlabel('Time (s)')
axs[0][0].set_ylabel('End Effector X Position (m)')
axs[0][0].legend(fontsize=9)
axs[0][0].set_xlim([3.0, 6.0])

# X Velocity
axs[1][0].plot(q_time, np.zeros(len(q_time)), label='Desired', color='tab:blue', linestyle='--', alpha=0.5)
axs[1][0].plot(q_time, xd[:, 0], label='Measured', color='tab:blue')
axs[1][0].set_xlabel('Time (s)')
axs[1][0].set_ylabel('End Effector X Velocity (m/s)')
axs[1][0].legend(fontsize=9)
axs[1][0].set_xlim([3.0, 6.0])

# Joint 1 Torque
# axs[2][0].plot(hip_T[:, 0], hip_T[:, 1], label='Desired', color='tab:red', linestyle='--', alpha=0.5)
axs[2][0].plot(hip_T[:, 0], hip_T[:, 1], label='Measured', color='tab:red')
axs[2][0].hlines([-20.1, 20.1], xmin=0, xmax=q_time[-1], color='black', linestyle='--', label='Max Torque')
axs[2][0].set_xlabel('Time (s)')
axs[2][0].set_ylabel('Hip Joint Torque (Nm)')
axs[2][0].legend(fontsize=9)
axs[2][0].set_xlim([3.0, 6.0])

# Z Position
axs[0][1].plot(x_d[:, 0], x_d[:, 2], label='Desired', color='tab:orange', linestyle='--', alpha=0.5)
axs[0][1].plot(x[:, 0], x[:, 2], label='Measured', color='tab:orange')
axs[0][1].set_xlabel('Time (s)')
axs[0][1].set_ylabel('End Effector Z Position (m)')
axs[0][1].legend(fontsize=9)
axs[0][1].set_xlim([3.0, 6.0])

# Z Velocity
idx = 0
for i in range(len(q_time)):
    if xd[i, 1] > xd[0, 1] + 0.001:
        idx = i
        break

axs[1][1].plot(impedence_trajectories.t_all + q_time[idx], impedence_trajectories.xd_d_all[:, 1], label='Desired', color='tab:orange', linestyle='--', alpha=0.5)
axs[1][1].plot(q_time, xd[:, 1], label='Measured', color='tab:orange')
axs[1][1].set_xlabel('Time (s)')
axs[1][1].set_ylabel('End Effector Z Velocity (m/s)')
axs[1][1].set_ylim([-2.0, 2.0])
axs[1][1].legend(fontsize=9)
axs[1][1].set_xlim([3.0, 6.0])

# Joint 2 Torque
# axs[2][1].plot(knee_T[:, 0], knee_T[:, 1], label='Desired', color='tab:red', linestyle='--', alpha=0.5)
axs[2][1].plot(knee_T[:, 0], knee_T[:, 1], label='Measured', color='tab:red')
# axs[2][1].plot(q_time, qd[:, 1], label='Measured', color='tab:green')
axs[2][1].hlines([-20.1, 20.1], xmin=0, xmax=q_time[-1], color='black', linestyle='--', label='Max Torque')
axs[2][1].set_xlabel('Time (s)')
axs[2][1].set_ylabel('Knee Joint Torque (Nm)')
axs[2][1].legend(fontsize=9)
axs[2][1].set_xlim([3.0, 6.0])

# # Operation space desired vs actual
axs[3][0].plot(x_d[:, 1], x_d[:, 2], label='Desired', color='tab:green', linestyle='--', alpha=0.5)
axs[3][0].plot(x[:, 1], x[:, 2], label='Measured', color='tab:green')
axs[3][0].set_xlabel('X Position (m)')
axs[3][0].set_ylabel('Z Position (m)')
axs[3][0].axis('equal')
axs[3][0].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
axs[3][0].legend(fontsize=9)

# # # IMU reading
# axs[3][1].plot(imu[:, 0], imu[:, 1], label='IMU', color='tab:brown')
# axs[3][1].set_xlabel('Time (s)')
# axs[3][1].set_ylabel('Acceleration in Z (g)')
# axs[3][1].set_ylim([-20, 100])

axs[3][1].plot(tf[:, 0], tf[:, 1], label='Base Frame Z', color='tab:brown')
axs[3][1].set_xlabel('Time (s)')
axs[3][1].set_ylabel('Base Frame Z Position (m)')
axs[3][1].legend(fontsize=9)
# axs[3][1].set_ylim([-20, 100])
axs[3][1].set_xlim([3.0, 6.0])

# Add subplot labels A) to H)
labels = ['A)', 'B)', 'C)', 'D)', 'E)', 'F)', 'G)', 'H)']
for idx, ax in enumerate(axs.flat):
    ax.text(-0.15, -0.1, labels[idx], transform=ax.transAxes, fontsize=10, va='top', ha='left', fontweight='bold')

plt.tight_layout()
plt.savefig('impedence_plots.png', dpi=1200)
plt.show()