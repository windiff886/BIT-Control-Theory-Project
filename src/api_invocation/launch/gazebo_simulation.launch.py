# Copyright (c) 2026
# Launch file for Gazebo simulation with robot + odom_to_tf

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取 br2_gazebo_worlds 包路径
    br2_gazebo_worlds_dir = get_package_share_directory('br2_gazebo_worlds')
    
    # 包含 house_with_robot.launch.py
    house_with_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(br2_gazebo_worlds_dir, 'launch', 'house_with_robot.launch.py')
        )
    )

    return LaunchDescription([
        # 启动 Gazebo 仿真环境 + 机器人
        house_with_robot_launch,
        
        # odom_to_tf 节点：将 /odom 话题转换为 TF
        Node(
            package='api_invocation',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
        ),
    ])
