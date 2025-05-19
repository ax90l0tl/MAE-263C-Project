import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    torque_control = LaunchConfiguration('torque_control')
    position_control = LaunchConfiguration('position_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    
    # Create a robot_state_publisher node
    robot_description_config = Command(['xacro ', xacro_file, 
                                        ' use_sim_time:=', use_sim_time,
                                        ' torque_control:=', torque_control,
                                        ' position_control:=', position_control,
                                        ' use_sim_time:=', use_sim_time])
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Need joint state publisher for continuous and revolute joints
    joint_robot_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'torque_control',
            default_value='false',
            description='Use torque control in sim'),
        DeclareLaunchArgument(
            'position_control',
            default_value='false',
            description='Use position control in sim'),

        node_robot_state_publisher,
        # joint_robot_state_publisher
    ])