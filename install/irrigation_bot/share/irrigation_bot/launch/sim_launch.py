from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='irrigation_bot').find('irrigation_bot')
    
    # Set paths for URDF and world files
    urdf_file = os.path.join(pkg_share, 'urdf', 'irrigation_bot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'irrigation_world.sdf')
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros').find('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'irrigation_bot', '-topic', 'robot_description'],
        output='screen'
    )
    
    # RViz configuration
    rviz_config_file = os.path.join(pkg_share, 'config', 'irrigation_bot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Irrigation bot controller node
    irrigation_bot_node = Node(
        package='irrigation_bot',
        executable='irrigation_bot',
        name='irrigation_bot',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz,
        irrigation_bot_node
    ])
