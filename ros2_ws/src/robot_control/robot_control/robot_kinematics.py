from robot_parser import URDFParser
import numpy as np
import os

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
        :return: Transformation matrix of the end effector.
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
        :param target_position: Target position of the end effector.
        :return: List of joint angles.
        """ 
        joint_angles = [0] * len(self.joint_properties)
        return joint_angles
    
    def jacobian(self, joint_angles):
        """
        Calculate the Jacobian matrix of the robot.
        :param joint_angles: List of joint angles.
        :return: Jacobian matrix.
        """
        J = np.zeros((6, len(self.joint_properties)))
        return J
    
robot = Robot(package_name='robot_description', urdf_file='robot.urdf.xacro')
robot.forward_kinematics([np.pi/2, 0])