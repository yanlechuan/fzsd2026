# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    rl_real_my_dog_node = Node(
        package='rl_sar',
        executable='rl_real_my_dog',
        name='rl_real_my_dog',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        rl_real_my_dog_node,
    ])
