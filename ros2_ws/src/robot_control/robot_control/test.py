class JointEffortPublisher(Node):
    def __init__(self):
        super().__init__('joint_effort_publisher')
        # Publishers for the two actuators
        self.joint1_pub = self.create_publisher(Float64, '/joint1/effort', 10)
        self.joint2_pub = self.create_publisher(Float64, '/joint2/effort', 10)
        # Timer to periodically publish efforts
        self.timer = self.create_timer(0.1, self.publish_effort)  # 10 Hz
        self.get_logger().info("Joint Effort Publisher Node has been started.")

    def publish_effort(self):
        # Define efforts for the two actuators
        joint1_effort = Float64()
        joint1_effort.data = 5.0  # Example effort value for joint1

        joint2_effort = Float64()
        joint2_effort.data = -3.0  # Example effort value for joint2

        # Publish efforts
        self.joint1_pub.publish(joint1_effort)
        self.joint2_pub.publish(joint2_effort)

        self.get_logger().info(f"Published efforts: joint1={joint1_effort.data}, joint2={joint2_effort.data}")

def main(args=None):
    rclpy.init(args=args)
    node = JointEffortPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

