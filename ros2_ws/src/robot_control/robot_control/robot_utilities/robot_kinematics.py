from robot_control.robot_utilities.robot_parser import URDFParser
import numpy as np
import matplotlib.pyplot as plt


class Robot:
    def __init__(self, package_name='robot_description', urdf_file='robot.urdf'):
        self.urdf_parser = URDFParser(package_name, urdf_file)
        self.inertial_properties = self.urdf_parser.get_inertial_prop()
        self.joint_properties = self.urdf_parser.get_joint_properties()
        self.dh_params = self.urdf_parser.get_DH_params()


    def forward_kinematics(self, joint_angles):
        """
        Calculate the forward kinematics of the robot.
        :param joint_angles: List of joint angles.
        :return: Transformation matrix of the end effector (4x4 numpy).
        """
        T = np.eye(4)
        for i, joint in enumerate(self.dh_params):
            if joint['type'] == 'fixed':
                theta = joint['theta']
            elif joint['type'] == 'revolute':
                theta = joint_angles[i - 1] - joint['offset']
            elif joint['type'] == 'prismatic':
                continue
            alpha = joint['alpha']
            a = joint['a']
            d = joint['d']
            T_i = np.array([
                [np.cos(theta), -np.sin(theta), 0, a],
                [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
                [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
                [0, 0, 0, 1]
            ])
            T = np.dot(T, T_i)

        print(np.round(T, 10))
        return T
    
    def inverse_kinematics(self, target_position):
        """
        Calculate the inverse kinematics of the robot.
        :param target_position: Target position of the end effector in homogenous matrix.
        :return: List of joint angles (2x2 numpy).
        """

        x = target_position[0]
        z = target_position[1]
        l1 = np.abs(self.dh_params[2]['a'])
        l2 = np.abs(self.dh_params[3]['a'])
        dist_sqrd = x**2 + z**2
        dist = np.sqrt(dist_sqrd)
        theta = np.arctan2(z, x)
        if dist >= l1 + l2 or dist <= np.abs(l1 - l2):
            if dist == l1 + l2:
                return np.array([[theta, 0], [theta, 0]])
            if dist == np.abs(l1 - l2):
                return np.array([[theta, np.pi], [theta, np.pi]])
            print("Target position is unreachable.")
            return np.array([None, None])

        # use this instead of law of cosines bc of roundoff error at small angles
        sin_t2_2 = (dist_sqrd - (l1-l2)**2) /(4*l1*l2)
        cos_t2 = 1 - 2*sin_t2_2
        t2 = np.pi - np.atan2(np.sqrt(1 - cos_t2**2), cos_t2)
        t2_alt = -t2

        cos_t1 = (dist_sqrd + l1**2 - l2**2)/(2*l1*dist)
        a = np.atan2(np.sqrt(1 - cos_t1**2), cos_t1)
        
        t1 = theta - self.dh_params[1]['offset'] - a
        t1_alt = theta - self.dh_params[1]['offset'] + a

        joint_angles = np.array([[t1, t2],
                                [t1_alt, t2_alt]])
        return joint_angles
    
    def jacobian(self, joint_angles):
        """
        Calculate the Jacobian matrix of the robot.
        :param joint_angles: List of joint angles.
        :return: Jacobian matrix.
        """

        # Everything in XZ frame
        # X forward, Z up
        # Default position of the leg is pointing down
        # Positive angle points the leg forward
        a1 = np.abs(self.dh_params[2]['a'])
        a2 = np.abs(self.dh_params[3]['a'])
        q1 = joint_angles[0]
        q2 = joint_angles[1]

        J = np.zeros((6, 2))
        J[0, 0] = a2*np.cos(q1 + q2) + a1*np.cos(q1)
        J[0, 1] = a2*np.cos(q1 + q2)
        J[2, 0] = a2*np.sin(q1 + q2) + a1*np.sin(q1)
        J[2, 1] = a2*np.sin(q1 + q2)
        J[4, 0] = -1
        J[4, 1] = -1
        return J
    
def plot_robot_arm(joint_angles, target_position=[0, 0]):
    """
    Plot the robot arm given joint angles.
    :param joint_angles: List of joint angles.
    """
    x_coords = [0]
    z_coords = [0]
    T = np.eye(4)

    for i, joint in enumerate(robot.dh_params):
        if joint['type'] == 'fixed':
            theta = joint['theta']
        elif joint['type'] == 'revolute':
            theta = joint_angles[i - 1] - joint['offset']
        elif joint['type'] == 'prismatic':
            continue
        alpha = joint['alpha']
        a = joint['a']
        d = joint['d']
        T_i = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
            [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
            [0, 0, 0, 1]
        ])
        T = np.dot(T, T_i)
        x_coords.append(T[0, 3])
        z_coords.append(T[2, 3])

    plt.figure()
    plt.plot(x_coords, z_coords, marker='o', linestyle='-', color='b')
    plt.scatter(target_position[0], target_position[1], color='r')
    plt.title("Robot Arm Configuration")
    plt.xlabel("X-axis")
    plt.ylabel("Z-axis")
    plt.xlim(-0.55, 0.55)
    plt.ylim(-0.55, 0.55)
    plt.grid()
    plt.show()

if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    # print(robot.dh_params)

    pose = np.array([0.1, -0.01])
    j = robot.inverse_kinematics(pose)

    print(np.rad2deg(j))
    print(robot.jacobian([np.pi/2, 0]))
    
    plot_robot_arm(j[1], pose)
    plot_robot_arm(j[0], pose)
    robot.forward_kinematics(j[0])
    robot.forward_kinematics(j[1])