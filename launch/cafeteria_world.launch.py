import os
import random
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(get_package_share_directory(
        'tinman'), 'worlds', 'cafeteria.world')

    bin_model_dir = os.path.join(get_package_share_directory(
        'tinman'), 'models', 'green_can', 'model.sdf')

    if not os.path.exists(world):

        raise FileNotFoundError(f"World file not found: {world}")
    if not os.path.exists(bin_model_dir):
        print(bin_model_dir)
        raise FileNotFoundError(f"Bin model file not found: {bin_model_dir}")

    model_dir = os.path.join(get_package_share_directory('tinman'), 'models')

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0', 'z_pose': '0.0',
                          'model': TURTLEBOT3_MODEL, 'use_sim_time': use_sim_time}.items()
    )

    gzservercmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    gzclientcmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')))

    spawn_turtlebot3 = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')))

    spawn_bin = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_green_bin_',
        arguments=['-file', bin_model_dir, '-entity',
                   'green_can', '-x', '3.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(gzservercmd)
    ld.add_action(gzclientcmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot3)
    ld.add_action(spawn_bin)
    return ld
