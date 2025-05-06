import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, GroupAction
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace



def generate_launch_description():
    package_name='robot_description'

    default_world = os.path.join(
        get_package_share_directory('robot_description'),
        'worlds',
        'world.sdf'
    )

    namespace = LaunchConfiguration('name')
    namespace_arg = DeclareLaunchArgument(
        'name',
        default_value='',
        description='robot namespace'
    )
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('robot_description'),'meshes')
            )
    
    
    robot_desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','demo.launch.py'
                )]), launch_arguments={'use_sim_time': 'true',
                                       'torque_control': 'false',
                                       'position_control': 'true'}.items()
    )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v0 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v0 '}.items()
    )

    bridge_params = os.path.join(
        get_package_share_directory('robot_description'),
        'config',
        'ros_gz_bridge.yaml'
        )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-x', '0.0', 
                                   '-y', '0.0',
                                   '-z', '0.0',
                                   ],
                        output='screen')


    ld = LaunchDescription()

    # Launch them all!
    ld.add_action(start_gazebo_ros_bridge_cmd)
    # ld.add_action(spawn_entity)
    ld.add_action(robot_desc)
    ld.add_action(namespace_arg)
    ld.add_action(set_env_vars_resources)
    ld.add_action(world_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    return ld