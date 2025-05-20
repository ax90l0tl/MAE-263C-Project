import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    
    # Need joint state publisher for continuous and revolute joints
    controller = Node(
        package='robot_control',
        executable='robot_control_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        controller,
        # joint_robot_state_publisher
    ])