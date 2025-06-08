import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    config = os.path.join(get_package_share_directory('robot_control'), 'config', 'sim_gains.yaml')
    
    # Need joint state publisher for continuous and revolute joints
    controller = Node(
        package='robot_control',
        executable='impedence_control_node',
        output='screen',
        parameters=[config],
    )

    # Launch!
    return LaunchDescription([
        controller,
    ])