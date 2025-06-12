from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import numpy as np

bagpath = Path('ros2bag/consecutive_jumps')
typestore = get_typestore(Stores.ROS2_JAZZY)
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == '/joint_states']
    q = []
    qd = []
    q_time = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        q_time.append(msg.header.stamp)
        q.append(msg.position)
        qd.append(msg.velocity)
    connections = [x for x in reader.connections if x.topic == '/x_d']
    x_d = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        x_d.append([msg.header.stamp, msg.pose.position.x, msg.pose.position.z])
    connections = [x for x in reader.connections if x.topic == '/x']
    x = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        x.append([msg.header.stamp, msg.pose.position.x, msg.pose.position.z])

    connections = [x for x in reader.connections if x.topic == '/imu']
    imu = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        imu.append([msg.header.stamp, msg.linear_acceleration.z])

    connections = [x for x in reader.connections if x.topic == '/hip_FT']
    hip_T = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        hip_T.append([msg.header.stamp, msg.wrench.torque.z])
    connections = [x for x in reader.connections if x.topic == '/knee_FT']
    knee_T = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        knee_T.append([msg.header.stamp, msg.wrench.torque.z])

    connections = [x for x in reader.connections if x.topic == '/contact']
    contact = []
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        contact.append([msg.header.stamp, msg.contacts[0].wrenches[0].body_1_wrench.force.z])
    

imu = np.array(imu)
contact = np.array(contact)
x = np.array(x)
x_d = np.array(x_d)
hip_T = np.array(hip_T)
knee_T = np.array(knee_T)
q = np.array(q)
qd = np.array(qd)
q_time = np.array(q_time)
print(q.shape, qd.shape, q_time.shape, x.shape, x_d.shape, imu.shape, contact.shape, hip_T.shape, knee_T.shape)


    