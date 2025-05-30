from robot_control.robot_utilities.robot_parser import URDFParser
import numpy as np
import matplotlib.pyplot as plt
from scipy import constants
from typing import *
from numpy.typing import NDArray


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

        # print(np.round(T, 10))
        return T
    
    def inverse_kinematics(self, target_position: NDArray) -> NDArray:
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
        t2 = np.pi - np.arctan2(np.sqrt(1 - cos_t2**2), cos_t2)
        t2_alt = -t2

        cos_t1 = (dist_sqrd + l1**2 - l2**2)/(2*l1*dist)
        a = np.arctan2(np.sqrt(1 - cos_t1**2), cos_t1)
        
        t1 = theta + self.dh_params[1]['offset'] - a
        t1_alt = theta + self.dh_params[1]['offset'] + a

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
        a1 = self.dh_params[2]['a']
        a2 = self.dh_params[3]['a']
        q1 = joint_angles[0]
        q2 = joint_angles[1]

        J = np.zeros((6, 2))
        J[0, 0] = -a2*np.cos(q1 + q2) - a1*np.cos(q1)
        J[0, 1] = -a2*np.cos(q1 + q2)
        J[2, 0] = a2*np.sin(q1 + q2) + a1*np.sin(q1)
        J[2, 1] = a2*np.sin(q1 + q2)
        J[4, 0] = 1
        J[4, 1] = 1
        return J
    
    def jacobian_square(self, joint_angles):
        J = self.jacobian(joint_angles)
        J = J[[0, 2], :]
        return (J)
    
    def trajectory(self, start, end, steps=10, ):
        """
        Generate a trajectory from start to end position.
        :param start: Start position.
        :param end: End position.
        :param steps: Number of steps in the trajectory.
        :return: List of positions in the trajectory.
        """
        x = np.linspace(start[0], end[0], steps)
        z = np.linspace(start[1], end[1], steps)
        return np.array([x, z]).T

    def dynamics(self, joint_angles, joint_velocity):

        """
        Calculate the dynamics of the robot.
        :param joint_angles: List of joint angles.
        :return: Mass Matrix, Coriolis Matrix, Gravity Matrix.
        """
        m1 = self.inertial_properties[1]['mass']
        m2 = self.inertial_properties[2]['mass']
        I1 = self.inertial_properties[1]['inertia']
        I2 = self.inertial_properties[2]['inertia']
        a1 = self.dh_params[2]['a']
        a2 = self.dh_params[3]['a']
        
        # TODO: finalize COM and change coefficients
        M = np.zeros((2, 2))
        M[0, 0] = I1[2, 2] + I2[2, 2] + 0.25*(a1**2)*m1 + (a1**2)*m2 + 0.25*(a2**2)*m2 + a1*a2*m2*np.cos(joint_angles[1])
        M[1, 0] = I2[2, 2] + 0.25*(a2**2)*m2 + 0.5*a1*a2*m2*np.cos(joint_angles[1])
        M[0, 1] = I2[2, 2] + 0.25*(a2**2)*m2 + 0.5*a1*a2*m2*np.cos(joint_angles[1])
        M[1, 1] = I2[2, 2] + 0.25*(a2**2)*m2


        V = np.zeros((2, 1))
        V[0] = -0.5*a1*a2*m2*joint_velocity[1]*np.sin(joint_angles[1])*(joint_velocity[1] + 2*joint_velocity[0])
        V[1] = 0.5*a1*a2*m2*(joint_velocity[0]**2)*np.sin(joint_angles[1])

        G = np.zeros((2, 1))
        G[0] = -constants.g * m2 * (0.5*a2*np.sin(joint_angles[0] + joint_angles[1]) + 
                                       a1*np.sin(joint_angles[0])) - 0.5*a1*constants.g*m1*np.sin(joint_angles[0])
        G[1] = -0.5*a2*constants.g*m2*np.sin(joint_angles[0] + joint_angles[1])

        return M, V, G
    
    def draw_workspace(self, density=200, plot=False):
        # print(self.joint_properties[1]['lower_limit'], self.joint_properties[1]['upper_limit'])
        # print(self.joint_properties[2]['lower_limit'], self.joint_properties[2]['upper_limit'])
        
        j1_range = np.linspace(self.joint_properties[1]['lower_limit'], self.joint_properties[1]['upper_limit'], density)
        j2_range = np.linspace(self.joint_properties[2]['lower_limit'], self.joint_properties[2]['upper_limit'], density)
        x_coords = []
        z_coords = []
        for i in range(density):
            for j in range(density):
                j1 = j1_range[i]
                j2 = j2_range[j]
                T = self.forward_kinematics([j1, j2])
                x = T[0, -1]
                z = T[2, -1]
                x_coords.append(x)
                z_coords.append(z)
        
        if plot:
            plt.figure()
            plt.scatter(x_coords, z_coords, s=0.5, color='blue')
            plt.title("Robot Workspace")
            plt.xlabel("X-axis")
            plt.ylabel("Z-axis")
            plt.xlim(-0.55, 0.55)
            plt.ylim(-0.55, 0.55)
            plt.grid()
            plt.show()
        

    def plot_robot_arm(self, joint_angles, target_position=[0, 0], fig=None, ax=None, label=""):
        """
        Plot the robot arm given joint angles.
        :param joint_angles: List of joint angles.
        """
        x_coords = [0]
        z_coords = [0]
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
            x_coords.append(T[0, 3])
            z_coords.append(T[2, 3])

        # plt.figure()
        fig.plot(x_coords, z_coords, marker='o', linestyle='-', color='b', label=label)
        fig.scatter(target_position[0], target_position[1], color='r')
        fig.set_xlabel("X-axis")
        fig.set_ylabel("Z-axis")
        fig.set_xlim(-0.55, 0.55)
        fig.set_ylim(-0.55, 0.55)
        # plt.grid()
        # plt.show()

if __name__ == "__main__":
    robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
    # print(robot.dh_params)

    # pose = np.array([-0.1, -0.1])
    # j = robot.inverse_kinematics(pose)

    # print((j))
    # print(robot.jacobian([np.pi/2, 0]))
    
    # robot.plot_robot_arm(j[1], pose)
    # robot.plot_robot_arm(j[0], pose)
    # robot.forward_kinematics(j[0])
    # robot.forward_kinematics(j[1])
    robot.draw_workspace(plot=True, density=200)
    T = robot.forward_kinematics([-2.80, 2.62])
    print(T)
    print(robot.inverse_kinematics([T[0, 3], T[2, 3]]))
    # print(robot.trajectory([0, 0], [0, -0.01], steps=10))