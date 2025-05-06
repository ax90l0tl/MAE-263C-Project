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
        :param target_position: Target position of the end effector in homogenous matrix.
        :return: List of joint angles.
        """

        x = target_position[0, -1]
        z = target_position[2, -1]
        l1 = np.abs(self.dh_params[2]['a'])
        l2 = np.abs(self.dh_params[3]['a'])

        theta = np.arctan2(z, x)

        dist_sqrd = (x**2 + z**2) 
        cos_t2 = l1**2 + l2**2 - dist_sqrd / (2 * l1 * l2)
        # Clip cos_t2 to ensure it's between -1 and 1
        cos_t2 = np.clip(cos_t2, -1.0, 1.0)
        print(cos_t2)
        t2 = np.arccos(cos_t2)
        t2_alt = np.arccos(-cos_t2)
        
        t1 = theta - np.atan2(l2*np.sin(t2), l1+l2*np.cos(t2)) + self.dh_params[1]['offset']
        t1_alt = theta - np.atan2(l2*np.sin(t2_alt), l1+l2*np.cos(t2_alt)) + self.dh_params[1]['offset']
        joint_angles = np.array([[t1, t2],
                                [t1_alt, t2_alt]])
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
j = robot.inverse_kinematics(np.array([[0, 0, 0, 0.0],
                                       [0, 0, 0, 0],
                                       [0, 0, 0, 0.50],
                                       [0, 0, 0, 1]]))
print(j)
robot.forward_kinematics(j[1])