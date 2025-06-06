import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rcl_interfaces.msg import SetParametersResult

import numpy as np

from robot_control.robot_utilities.robot_kinematics import Robot
from robot_control.controllers.impedence_control import *
from robot_control.controllers.pid_control import pid_control
from robot_control.controllers.impedence_trajectories import *
from robot_interfaces.srv import Jump
from scipy import constants

states = {
    'IDLE': 0,
    'CROUCH': 1,
    'JUMP': 2,
    'IN_AIR': 3,
    'FALLING': 4,
    'LAND': 5,
}

state_names = {v: k for k, v in states.items()}


class impedence_control_node(Node):
    def __init__(self):
        super().__init__('impedence_control_node')
        self.robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        
        self.contact_sub = self.create_subscription(Contacts, '/contact', self.contacts_callback, qos_profile=qos_profile)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile=qos_profile)
        self.hip_FT_sub = self.create_subscription(WrenchStamped, '/hip_FT', self.hip_callback, qos_profile=qos_profile)
        self.knee_FT_sub = self.create_subscription(WrenchStamped, '/knee_FT', self.knee_callback, qos_profile=qos_profile)
        self.jump_server = self.create_service(Jump, '/jump', self.jump_callback) 
        
        self.hip_pub = self.create_publisher(Float64, '/hip_joint/cmd_torque', 10)
        self.knee_pub = self.create_publisher(Float64, '/knee_joint/cmd_torque', 10)
        self.desired_pose_pub = self.create_publisher(PoseStamped, '/x_d', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/x', 10)

        self.declare_parameter('Kp', [5000.0, 5000.0])
        self.declare_parameter('Kd', [100.0, 100.0])
        self.declare_parameter('Md', [1.0, 1.0])
        self.add_on_set_parameters_callback(self.parameter_callback)

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.on_ground = False
        self.on_ground_time = 0.0
        self.jump = False
        self.state = states['IDLE']
        
        self.default_pose = np.array([0.0, -0.3])
        self.x_d = self.default_pose
        self.q = np.zeros(2)
        self.qd = np.zeros(2)
        self.x = np.zeros(2)
        self.u = np.zeros(2)
        self.q_d = self.robot.inverse_kinematics(self.x_d)[0]
        self.T_d = self.robot.forward_kinematics(self.q_d)
        self.error = np.zeros(2)
        self.hip_FT = 0
        self.knee_FT = 0
        self.contact_force = np.zeros(3)
        self.Kp = np.diag(self.get_parameter('Kp').get_parameter_value().double_array_value)
        self.Kd = np.diag(self.get_parameter('Kd').get_parameter_value().double_array_value)
        self.Md = np.diag(self.get_parameter('Md').get_parameter_value().double_array_value)
        self.x_d_traj_points = np.zeros(2)
        self.xd_d_traj_points = np.zeros(2)
        self.xdd_d_traj_points = np.zeros(2)
        self.t_points = np.zeros(0)

        self.reset_time = self.get_clock().now().nanoseconds
        self.start_time = self.reset_time
        self.last_sim_time = self.reset_time
        self.landing_time = self.reset_time


    def timer_callback(self):
        # Use PID to get to starting position when the node starts
        current_time = self.get_clock().now().nanoseconds
        if (current_time - self.start_time)/(1e9) < 3.0 or (current_time - self.reset_time)/(1e9) < 3.0 or (current_time - self.last_sim_time)/(1e9) > 2.0:
            Kp=np.diag([100.0, 100.0])
            Kd=np.diag([1.0, 1.0])
            Ki=np.diag([0.001, 0.001])
            self.u, self.error = pid_control(self.q, np.array([-1.5, 2.62]), self.qd, Kp, Kd, Ki, error_prev=self.error, dt=0.005)
        else:
            if self.jump:
                if self.state == states['IDLE']:
                    self.state = states['CROUCH']
                    self.x_d_traj_points = x_d_idle_to_crouch_traj.copy()
                    self.xd_d_traj_points = xd_d_idle_to_crouch_traj.copy()
                    self.xdd_d_traj_points = xdd_d_idle_to_crouch_traj.copy()
                    self.t_points = t_idle_to_crouch.copy()
                elif self.state == states['CROUCH']:
                    y = compute_y(self.x_d_traj_points[0], self.xd_d_traj_points[0], 
                                    self.xdd_d_traj_points[0], self.x, self.q, self.qd, self.robot, self.Md, self.Kp, self.Kd)
                    self.u = compute_impedence_output(y, self.q, self.qd, self.robot)
                    if len(self.x_d_traj_points) - 1  > 1:
                        self.x_d_traj_points = self.x_d_traj_points[1:]
                        self.xd_d_traj_points = self.xd_d_traj_points[1:]
                        self.xdd_d_traj_points = self.xdd_d_traj_points[1:]
                    else:
                        print("max_error: ", np.max(np.abs(self.x - self.x_d_traj_points)))
                        if np.max(np.abs(self.x - self.x_d_traj_points)) < 0.01:
                            self.state = states['JUMP']
                            self.x_d_traj_points = x_d_crouch_to_jump_traj.copy()
                            self.xd_d_traj_points = xd_d_crouch_to_jump_traj.copy()
                            self.xdd_d_traj_points = xdd_d_crouch_to_jump_traj.copy()
                            self.t_points = t_crouch_to_jump.copy()
                elif self.state == states['JUMP']:
                    y = compute_y(self.x_d_traj_points[0], self.xd_d_traj_points[0], 
                                    self.xdd_d_traj_points[0], self.x, self.q, self.qd, self.robot, self.Md, self.Kp, self.Kd)
                    self.u = compute_impedence_output(y, self.q, self.qd, self.robot)
                    if len(self.x_d_traj_points) - 1  > 1:
                        self.x_d_traj_points = self.x_d_traj_points[1:]
                        self.xd_d_traj_points = self.xd_d_traj_points[1:]
                        self.xdd_d_traj_points = self.xdd_d_traj_points[1:]
                    else:   
                        # self.jump = False
                        print('finished jump')
                        self.state = states['IN_AIR']
                        self.landing_time = self.get_clock().now().nanoseconds

                elif self.state == states['IN_AIR']:
                    if (self.get_clock().now().nanoseconds - self.on_ground_time)/(1e9) > 0.005:
                        print("on_ground: ", (self.get_clock().now().nanoseconds - self.on_ground_time)/(1e9))
                    self.x_d = x_d_idle
                    Md = np.diag([1.0, 2.0])
                    Kp = np.diag([5000.0, 1000.0])
                    Kd = np.diag([200.0, 100.0])
                    y = compute_y(x_d_idle, xd_d_idle, xdd_d_idle, self.x, self.q, self.qd, self.robot, Md, Kp, Kd)
                    self.u = compute_impedence_output(y, self.q, self.qd, self.robot)
                    if (self.get_clock().now().nanoseconds - self.landing_time)/(1e9) > 2.0:
                        self.state = states['IDLE']
                        self.jump = False
                        
                self.x_d = self.x_d_traj_points[0]
                    
            else:
                # Idle state
                self.state = states['IDLE']
                self.x_d = x_d_idle
                y = compute_y(x_d_idle, xd_d_idle, xdd_d_idle, self.x, self.q, self.qd, self.robot, self.Md, self.Kp, self.Kd)
                self.u = compute_impedence_output(y, self.q, self.qd, self.robot)
                # print("u", self.u)


        # print("state: ", state_names[self.state])
        # print("u", self.u)
        hip_msg = Float64()
        knee_msg = Float64()
        hip_msg.data = float(self.u[0])
        knee_msg.data = float(self.u[1])
        self.hip_pub.publish(hip_msg)
        self.knee_pub.publish(knee_msg)

        x_d_msg = PoseStamped()
        x_msg = PoseStamped()
        x_msg.header.stamp = self.get_clock().now().to_msg()
        x_d_msg.header.stamp = self.get_clock().now().to_msg()
        x_d_msg.pose.position.x = self.x_d[0]
        x_d_msg.pose.position.z = self.x_d[1]
        x_msg.pose.position.x = self.x[0]
        x_msg.pose.position.z = self.x[1]
        self.desired_pose_pub.publish(x_d_msg)
        self.pose_pub.publish(x_msg)

        self.on_ground = False


    def reset(self):
        self.error = np.zeros(2)
        self.state = states['IDLE']
        self.reset_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Resetting")

    #CALLBACK FUNCTION

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'Kp':
                if param.type_ == param.Type.DOUBLE_ARRAY:
                    self.Kp = np.diag(param.value)
                    self.get_logger().info(f"Updated Kp: {self.Kp}")
            elif param.name == 'Kd':
                if param.type_ == param.Type.DOUBLE_ARRAY:
                    self.Kd = np.diag(param.value)
                    self.get_logger().info(f"Updated Kd: {self.Kd}")
            elif param.name == 'Md':
                if param.type_ == param.Type.DOUBLE_ARRAY:
                    self.Md = np.diag(param.value)
                    self.get_logger().info(f"Updated Md: {self.Md}")
        self.reset()
        print("w_nx: ", np.sqrt(self.Kp[0, 0] / self.Md[0, 0]))
        print("zeta_x", self.Kd[0, 0] / (2* np.sqrt(self.Kp[0, 0] * self.Md[0, 0])))
        print("w_nz: ", np.sqrt(self.Kp[1, 1] / self.Md[1, 1]))
        print("zeta_z", self.Kd[1, 1] / (2 * np.sqrt(self.Kp[1, 1] * self.Md[1, 1])))

        return SetParametersResult(successful=True)

    def contacts_callback(self, msg):
        self.on_ground = True
        self.on_ground_time = self.get_clock().now().nanoseconds
        contact = msg.contacts[0]
        self.contact_force = np.array([contact.wrenches[0].body_1_wrench.force.x,
                                       contact.wrenches[0].body_1_wrench.force.y,
                                      contact.wrenches[0].body_1_wrench.force.z])

    def joint_callback(self, msg):
        self.last_sim_time = self.get_clock().now().nanoseconds
        self.q = np.array(msg.position)[1:]
        fk = self.robot.forward_kinematics(self.q)
        self.x = np.array([fk[0, -1], fk[2, -1]])
        self.qd = np.array(msg.velocity)[1:]
    
    def hip_callback(self, msg):
        self.hip_FT = msg.wrench.force.z

    def knee_callback(self, msg):
        self.knee_FT = msg.wrench.force.z

    def jump_callback(self, request, response):
        print("Jump request received")
        if self.state == states['IDLE']:
            self.jump = True
            response.success = True
        else:
            print("Can't jump rn")
            response.success = False
        response.time = self.get_clock().now().nanoseconds
        return response

def main(args=None):
    rclpy.init(args=args)
    node = impedence_control_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()