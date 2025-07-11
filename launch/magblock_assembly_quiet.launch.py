#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set environment variable to suppress warnings
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'aidf', 'magblock_assembly_executor'
            ],
            output='screen',
            env={
                'RCUTILS_LOGGING_SEVERITY_THRESHOLD': '30',  # ERROR level
                'RCUTILS_COLORIZED_OUTPUT': '1'
            }
        ),
    ])
