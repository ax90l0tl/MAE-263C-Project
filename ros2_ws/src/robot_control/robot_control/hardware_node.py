import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Contact, JointWrench
from std_msgs.msg import Float64
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from rcl_interfaces.msg import SetParametersResult

import numpy as np

from robot_control.robot_utilities.robot_kinematics import Robot
from robot_control.controllers.impedence_control import *
from robot_control.controllers.pid_control import pid_control
from robot_control.controllers.impedence_trajectories import *
from robot_interfaces.srv import Jump
from scipy import constants

from Leg_Test.leg_manager import legManager

class hardware_node(Node):
    def __init__(self):
        super().__init__('hardware_node')
        self.leg_manager = legManager(bear_port='/dev/ttyUSB0', sensor_port='/dev/ttyACM0')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        
        self.hip_cmd = self.create_subscription(Float64, '/hip_joint/cmd_torque', self.hip_callback, qos_profile=qos_profile)
        self.knee_cmd = self.create_subscription(Float64, '/knee_joint/cmd_torque', self.knee_callback, qos_profile=qos_profile)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.contact_pub = self.create_publisher(Contacts, '/contact', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)


        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.u = np.zeros(2)
        self.leg_manager.enable_torque(1)
        

    def timer_callback(self):
        self.leg_manager.move_motors(self.u)
        iq, q, qd = self.leg_manager.get_feedback()
        q[1] += np.deg2rad(21.75) # knee offset due to linkage
        print(self.u, q)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['hip_joint', 'knee_joint']
        joint_state_msg.position = np.hstack((0, q))
        joint_state_msg.velocity = np.hstack((0, qd))
        joint_state_msg.effort = np.hstack((0, self.leg_manager.iq_to_torque(iq)))
        self.joint_state_pub.publish(joint_state_msg)
        if not self.leg_manager.get_foot_sensor_state():
            contact_msg = Contacts()
            contact_msg.contacts.append(Contact())
            contact_msg.header.stamp = self.get_clock().now().to_msg()
            contact_msg.contacts[0].wrenches.append(JointWrench())
            contact_msg.contacts[0].wrenches[0].body_1_wrench.force.x = 0.0
            contact_msg.contacts[0].wrenches[0].body_1_wrench.force.y = 0.0
            contact_msg.contacts[0].wrenches[0].body_1_wrench.force.z = 0.0
            contact_msg.contacts[0].wrenches[0].body_1_wrench.torque.x = 0.0
            contact_msg.contacts[0].wrenches[0].body_1_wrench.torque.y = 0.0
            contact_msg.contacts[0].wrenches[0].body_1_wrench.torque.z = 0.0
            self.contact_pub.publish(contact_msg)


        # imu_msg = Imu()
        # imu_msg.header.stamp = self.get_clock().now().to_msg()
        # imu_msg.header.frame_id = 'imu'
        # imu_msg.orientation.x = 0.0
        # imu_msg.orientation.y = 0.0
        # imu_msg.orientation.z = 0.0
        # imu_msg.orientation.w = 1.0
        # imu_msg.angular_velocity.x = 0.0
        # imu_msg.angular_velocity.y = 0.0
        # imu_msg.angular_velocity.z = 0.0
        # imu_msg.linear_acceleration.x = 0.0
        # imu_msg.linear_acceleration.y = 0.0
        # imu_msg.linear_acceleration.z = self.leg_manager.imu.
        # imu_msg.orientation_covariance = [0.0] * 9
        # imu_msg.angular_velocity_covariance = [0.0] * 9
        # imu_msg.linear_acceleration_covariance = [0.0] * 9




    def hip_callback(self, msg):
        self.u[0] = msg.data

    def knee_callback(self, msg):
        self.u[1] = msg.data
    

def main(args=None):
    rclpy.init(args=args)
    node = hardware_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()