# Copyright (c) 2024
# Launch file to spawn ackermann robot in house world

import os
import xacro
import yaml
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.
    """
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})

    return robot_description.toxml()


def generate_launch_description():
    # Setup model paths for br2_gazebo_worlds
    model_path = ''
    pkg_path = get_package_share_directory('br2_gazebo_worlds')
    model_path += os.path.join(pkg_path, 'models')

    if 'GZ_SIM_MODEL_PATH' in environ:
        model_path += pathsep + environ['GZ_SIM_MODEL_PATH']

    # Get ackermann_v2 package path
    ackermann_pkg = get_package_share_directory('ackermann_v2')

    # Set paths for robot description
    robot_description_path = os.path.join(ackermann_pkg, 'model', 'vehicle.xacro')
    vehicle_params_path = os.path.join(ackermann_pkg, 'config', 'parameters.yaml')
    gz_bridge_params_path = os.path.join(ackermann_pkg, 'config', 'ros_gz_bridge.yaml')
    rviz_config_file = os.path.join(ackermann_pkg, 'rviz', 'rviz.rviz')

    # House world path
    world_file = os.path.join(pkg_path, 'worlds', 'house.world')

    # Load robot URDF
    robot_description = load_robot_description(robot_description_path, vehicle_params_path)

    # Launch arguments for robot spawn position
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    robot_name = "ackermann_steering_vehicle"

    # Launch Gazebo (single process with server + GUI)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Node to spawn robot model in Gazebo
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-string', robot_description,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # ROS-Gazebo bridge
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Set environment variables for model paths
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_MODEL_PATH', model_path))

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0',
                                         description='Initial x position'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0',
                                         description='Initial y position'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.1',
                                         description='Initial z position'))
    ld.add_action(DeclareLaunchArgument('R', default_value='0.0',
                                         description='Initial roll'))
    ld.add_action(DeclareLaunchArgument('P', default_value='0.0',
                                         description='Initial pitch'))
    ld.add_action(DeclareLaunchArgument('Y', default_value='0.0',
                                         description='Initial yaw'))

    # Add actions
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_model_gazebo_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_bridge_node)
    ld.add_action(rviz_node)

    return ld
