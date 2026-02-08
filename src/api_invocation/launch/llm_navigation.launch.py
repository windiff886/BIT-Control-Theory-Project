# Copyright (c) 2026
# Launch file for LLM navigation system (llm_analyze + advanced_executor + model_state_publisher)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取参数文件路径
    api_invocation_dir = get_package_share_directory('api_invocation')
    params_file = os.path.join(api_invocation_dir, 'config', 'llm_params.yaml')

    return LaunchDescription([
        # 模型状态发布器：从Gazebo获取模型位置，发布到 /gazebo/model_states
        Node(
            package='api_invocation',
            executable='model_state_publisher',
            name='model_state_publisher',
            output='screen',
        ),
        
        # LLM 分析节点：将自然语言转换为 JSON 任务
        Node(
            package='api_invocation',
            executable='llm_analyze',
            name='llm_analyzer',
            output='screen',
            parameters=[params_file],
        ),
        
        # 高级执行器：执行导航任务
        Node(
            package='api_invocation',
            executable='advanced_executor',
            name='advanced_executor',
            output='screen',
            parameters=[params_file],
        ),
    ])
