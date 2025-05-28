import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy
from robot_control.robot_utilities.robot_kinematics import Robot
import numpy as np
from robot_interfaces.srv import Jump
from sensor_msgs.msg import JointState
from scipy import constants

from robot_control.controllers.active_compliance_control import active_compliance_control

states = {
    'IDLE': 0,
    'CROUCH': 1,
    'JUMP': 2,
    'IN_AIR': 3,
    'FALLING': 4,
    'LAND': 5,
}

state_names = {v: k for k, v in states.items()}


class active_compliance_control_node(Node):
    def __init__(self):
        super().__init__('active_compliance_control_node')
        self.robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        
        self.contact_sub = self.create_subscription(Contacts, '/contact', self.contacts_callback, qos_profile=qos_profile)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile=qos_profile)
        self.jump_server = self.create_service(Jump, '/jump', self.jump_callback) 
        
        self.hip_pub = self.create_publisher(Float64, '/hip_joint/cmd_torque', 10)
        self.knee_pub = self.create_publisher(Float64, '/knee_joint/cmd_torque', 10)

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.on_ground = False
        self.on_ground_time = 0.0
        self.jump = False
        self.state = states['IDLE']
        
        self.default_pose = np.array([0.0, -0.4])
        self.x_d = self.default_pose
        self.q = np.array([0.0, -0.4])
        self.qd = np.zeros(2)
        self.x = np.zeros(2)
        self.u = np.zeros(2)
        self.q_d = self.robot.inverse_kinematics(self.x_d)[0]
        self.T_d = self.robot.forward_kinematics(self.q_d)

        self.start_time = self.get_clock().now().nanoseconds
        self.last_sim_time = self.get_clock().now().nanoseconds

    def timer_callback(self):
        if (self.get_clock().now().nanoseconds - self.start_time)/(1e9) < 2.0 or (self.get_clock().now().nanoseconds - self.last_sim_time)/(1e9) > 2.0:
            self.u = np.array([-100000, 100000.0])
        else:
            # if self.on_ground:
            #     if self.jump:
            #         if self.state != states['CROUCH']:
            #             self.state = states['CROUCH']
            #             # x = self.robot.inverse_kinematics(np.array([-0.1, -0.1]))
            #             # print(self.x)
            #             # self.x_d = x[0]
            #         # else:
            #             # print(np.max(np.abs(self.x - self.x_d)))
            #             # if np.max(np.abs(self.x - self.x_d)) < 0.1:
            #             #     self.state = states['JUMP']
            #             #     self.x_d = np.array([0, 0])
            #             #     self.jump = False
            #     else:
            self.state = states['IDLE']
            self.x_d = self.default_pose
            x_error = np.atleast_2d(self.x_d - self.x).T
            print("x_error: ", x_error)
            Kp = np.diag([100.0, 200.0])
            Kd = np.diag([10.0, 40.0])
            # COM in urdf is defined at the base of the link TODO need to change
            jacobian =self.robot.jacobian(self.q)
            a1 = self.robot.dh_params[2]['a']
            a2 = self.robot.dh_params[3]['a']
            m1 = self.robot.inertial_properties[1]['mass']
            m2 = self.robot.inertial_properties[2]['mass']
            self.u = np.squeeze(active_compliance_control(self.x_d, self.x, jacobian, Kp, Kd, self.q, self.qd, a1, a2, m1, m2, self.q_d, self.T_d)).T
            # hip torque should be positive, knee torque should be negative
            print("u", self.u)

            # else:
                # if (self.get_clock().now().nanoseconds - self.on_ground_time)/(1e9) > 0.01 and self.state == states['JUMP']:
                    # self.state = states['IN_AIR']
                    # x = self.robot.inverse_kinematics(np.array([-0.1, -0.1]))
                    # self.x_d = x[0]
                    # self.on_ground = False

            # print("State: ", state_names[self.state])
        hip_msg = Float64()
        knee_msg = Float64()
        hip_msg.data = float(self.u[0])
        knee_msg.data = float(self.u[1])
        self.hip_pub.publish(hip_msg)
        self.knee_pub.publish(knee_msg)
        self.on_ground = False


    def contacts_callback(self, msg):
        self.on_ground = True
        self.on_ground_time = self.get_clock().now().nanoseconds

    def joint_callback(self, msg):
        self.last_sim_time = self.get_clock().now().nanoseconds
        self.q = np.array(msg.position)
        fk = self.robot.forward_kinematics(self.q)
        self.x = np.array([fk[0, -1], fk[2, -1]])
        self.qd = np.array(msg.velocity)

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
    node = active_compliance_control_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()