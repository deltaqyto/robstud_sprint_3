from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch file for Gazebo
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Set the path to the world file
    world_file_path = os.path.expanduser('~/office_1.world')
    
    # Launch Gazebo with the specified world file
    gazebo_with_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # Include the TurtleBot3 launch file
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'),
        ),
        launch_arguments={'model': 'waffle'}.items()
    )

    return LaunchDescription([
        gazebo_with_world,
        turtlebot3_spawn,
    ])
