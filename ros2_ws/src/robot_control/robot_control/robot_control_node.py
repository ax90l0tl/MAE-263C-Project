import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy
from robot_control.robot_utilities.robot_kinematics import Robot
import numpy as np
from robot_interfaces.srv import Jump
from sensor_msgs.msg import JointState

states = {
    'IDLE': 0,
    'CROUCH': 1,
    'JUMP': 2,
    'IN_AIR': 3,
    'LAND': 4,
}

state_names = {v: k for k, v in states.items()}


class robot_control_node(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        self.contact_sub = self.create_subscription(Contacts, '/contact', self.contacts_callback, qos_profile=qos_profile)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile=qos_profile)
        self.jump_server = self.create_service(Jump, '/jump', self.jump_callback) 
        
        self.declare_parameter('use_torque', False)

        if self.get_parameter('use_torque').value == True:
            hip_topic = '/hip_joint/cmd_torque'
            knee_topic = '/knee_joint/cmd_torque'
        else:
            hip_topic = '/hip_joint/cmd_pos'
            knee_topic = '/knee_joint/cmd_pos'  
        self.hip_pub = self.create_publisher(Float64, hip_topic, 10)
        self.knee_pub = self.create_publisher(Float64, knee_topic, 10)

        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.on_ground = False
        self.on_ground_time = 0.0
        self.jump = False
        self.state = states['IDLE']
        self.joint_angles_d = np.zeros(2)
        self.joint_angles = np.zeros(2)

    def timer_callback(self):
        # print((self.get_clock().now().nanoseconds - self.on_ground_time)/(1e9))
        if self.on_ground:
            if self.jump:
                if self.state != states['CROUCH']:
                    self.state = states['CROUCH']
                    joint_angles = self.robot.inverse_kinematics(np.array([-0.1, -0.01]))
                    self.joint_angles_d = joint_angles[0]
                else:
                    print(np.max(np.abs(self.joint_angles - self.joint_angles_d)))
                    if np.max(np.abs(self.joint_angles - self.joint_angles_d)) < 0.5:
                        self.state = states['JUMP']
                        self.joint_angles_d = np.array([0, 0])
                        self.jump = False
            else:
                self.state = states['IDLE']
                self.joint_angles_d = np.array([-0.3, 0.6])
        else:
            if (self.get_clock().now().nanoseconds - self.on_ground_time)/(1e9) > 0.01 and self.state == states['JUMP']:
                self.state = states['IN_AIR']
                joint_angles = self.robot.inverse_kinematics(np.array([-0.1, -0.01]))
                self.joint_angles_d = joint_angles[0]
                self.on_ground = False

        print("State: ", state_names[self.state])
        hip_msg = Float64()
        knee_msg = Float64()

        hip_msg.data = float(self.joint_angles_d[0])
        knee_msg.data = float(self.joint_angles_d[1])

        self.hip_pub.publish(hip_msg)
        self.knee_pub.publish(knee_msg)
        self.on_ground = False


    def contacts_callback(self, msg):
        self.on_ground = True
        self.on_ground_time = self.get_clock().now().nanoseconds

    def joint_callback(self, msg):
        self.joint_angles = np.array(msg.position)

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
    node = robot_control_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()