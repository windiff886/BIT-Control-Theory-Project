import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument) # 移除 RegisterEventHandler, ExecuteProcess

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.
    """
    with open(vehicle_params_path, 'r') as file:
        # Load parameters from YAML file
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})

    return robot_description.toxml()

def generate_launch_description():
    # Define launch arguments (保持不变)
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)')

    # ... (x, y, z, R, P, Y arguments remain unchanged)
    # 为简洁起见，省略了 x, y, z, R, P, Y 的 DeclareLaunchArgument

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Define the robot's and package name (保持不变)
    package_name = "ackermann_v2"
    package_path = get_package_share_directory(package_name)

    # Set paths to Xacro model and configuration files (保持不变)
    robot_description_path = os.path.join(package_path, 'model',
                                          'vehicle.xacro')

    gz_bridge_params_path = os.path.join(package_path, 'config',
                                         'ros_gz_bridge.yaml')

    vehicle_params_path = os.path.join(package_path, 'config',
                                       'parameters.yaml')
    
    rviz_config_file = os.path.join(package_path, 'rviz', 
                                    'rviz.rviz')

    # Load URDF (保持不变)
    robot_description = load_robot_description(robot_description_path,
                                               vehicle_params_path)

    # Prepare and include the Gazebo launch file (保持不变)
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'),
                     'launch',
                     'gz_sim.launch.py'))

    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    robot_name = "ackermann_steering_vehicle"
    
    # Create node to spawn robot model (保持不变)
    spawn_model_gazebo_node = Node(package='ros_gz_sim',
                                   executable='create',
                                   arguments=['-name', robot_name,
                                              '-string', robot_description,
                                              '-x', x,
                                              '-y', y,
                                              '-z', z,
                                              '-R', roll,
                                              '-P', pitch,
                                              '-Y', yaw,
                                              '-allow_renaming', 'false'],
                                   output='screen')

    # Create node to publish the robot state (保持不变)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description,
                                          'use_sim_time': True}],
                                      output='screen')

    # Create a node for the ROS-Gazebo bridge (保持不变)
    gz_bridge_node = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args', '-p',
                                     f'config_file:={gz_bridge_params_path}'],
                          output='screen')

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='screen',
                     arguments=['-d', rviz_config_file])
  
    # Create the launch description (移除所有事件处理器)
    launch_description = LaunchDescription([
        # ⚠️ 移除: 所有 RegisterEventHandler (OnProcessExit)
        
        world_arg,
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        
        gazebo_launch,
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        gz_bridge_node,
        rviz_node
    ])

    return launch_description
