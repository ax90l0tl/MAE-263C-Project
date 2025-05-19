import numpy as np
from urdf_parser_py import urdf
import os
import subprocess
from ament_index_python.packages import get_package_share_directory


# loads the URDF file and parses it to extract inertial properties, joint properties, and DH parameters
# doesn't require ROS2 but you will need to have a ros2 workspace setup to find packages and stuff

class URDFParser():
    def __init__(self, package_name='robot_description', urdf_file='robot.urdf'):

        # Get path to the URDF file
        self.package_path = get_package_share_directory(package_name)
        self.urdf_path = os.path.join(self.package_path, 'description', urdf_file)
        
        print(f"Loading URDF from: {self.urdf_path}")
        
        # Process XACRO if needed
        if urdf_file.endswith('.xacro'):
            urdf_content = self.process_xacro(self.urdf_path)
        else:
            # Parse the URDF file
            with open(self.urdf_path, 'r') as file:
                urdf_content = file.read()
                
        self.robot = urdf.Robot.from_xml_string(urdf_content)

    def process_xacro(self, xacro_path):
        """Process XACRO file to URDF string"""
        try:
            cmd = ['ros2', 'run', 'xacro', 'xacro', xacro_path]
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"Error processing xacro file: {e}")
            print(f"Error output: {e.stderr}")
            raise
    
    '''Returns a list of dictionaries containing the inertial properties of each link in the robot model.
    Each dictionary contains the link name, mass, inertia matrix, and center of mass (com) of the link.
    The inertia matrix is a 3x3 numpy array, and the com is a 3-element numpy array.'''
    def get_inertial_prop(self):
        inertial_properties = []
        for link in self.robot.links:
            if hasattr(link, 'inertial') and link.inertial is not None:
                origin = link.inertial.origin
                com = np.array([origin.position[0], origin.position[1], origin.position[2]]) if origin and hasattr(origin, 'position') else np.zeros(3)
                mass = link.inertial.mass
                inertia = np.array([[link.inertial.inertia.ixx, link.inertial.inertia.ixy, link.inertial.inertia.ixz],
                                    [link.inertial.inertia.ixy, link.inertial.inertia.iyy, link.inertial.inertia.iyz],
                                    [link.inertial.inertia.ixz, link.inertial.inertia.iyz, link.inertial.inertia.izz]])
                inertial_properties.append({
                    'link_name': link.name,
                    'mass': mass,
                    'inertia': inertia,
                    'com': com
                })
        return inertial_properties
    
    '''Returns a list of dictionaries containing the joint properties of each joint in the robot model.
    Each dictionary contains the joint name, lower limit, upper limit, effort, velocity, damping, and friction of the joint.'''
    
    def get_joint_properties(self):
        joint_properties = []
        for joint in self.robot.joints:
            if hasattr(joint, 'limit') and joint.limit is not None:
                joint_properties.append({
                    'joint_name': joint.name,
                    'lower_limit': joint.limit.lower,
                    'upper_limit': joint.limit.upper,
                    'effort': joint.limit.effort,
                    'velocity': joint.limit.velocity,
                    'damping': joint.dynamics.damping,
                    'friction': joint.dynamics.friction,
                })
        return joint_properties

    '''Returns a list of dictionaries containing the Denavit-Hartenberg parameters for each joint in the robot model.
    Each dictionary contains the joint name, type, alpha, a, theta, d, and offset of the joint.
    '''
    def get_DH_params(self):
        dh_params = []
        for joint in self.robot.joints:
            if hasattr(joint, 'origin') and joint.origin is not None:
                alpha = joint.origin.rpy[0]
                a = joint.origin.position[0]
                theta = joint.origin.rpy[2]
                d = joint.origin.position[2]
                T = np.array([
                    [np.cos(theta), -np.sin(theta), 0, a],
                    [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
                    [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
                    [0, 0, 0, 1]
                ])
                T = np.round(T, 15)
                # print(T)
                dh_params.append({
                    'name': joint.name,
                    'type': joint.type,
                    'alpha': alpha,
                    'a': a,
                    'theta': theta,
                    'd': d,
                    'offset': joint.origin.rpy[1],
                    # 'T': T,
                })
        return dh_params

if __name__ == "__main__":
    URDFParserNode = URDFParser(package_name='robot_description', urdf_file='robot.urdf.xacro')
    print(URDFParserNode.get_DH_params())
    print(URDFParserNode.get_inertial_prop())
    print(URDFParserNode.get_joint_properties())