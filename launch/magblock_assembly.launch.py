#!/usr/bin/env python3

"""
Launch file for MagBlock assembly using three Kinova Gen3 arms with MoveIt2.
This launch file starts the necessary nodes for magblock assembly simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
    ]

    # Launch the three-arm MoveIt2 configuration
    three_arm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kinova_three_arm_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': launch_rviz,
        }.items()
    )

    # MagBlock assembly planner node
    magblock_planner_node = Node(
        package='aidf',
        executable='plan_magblock',
        name='magblock_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/skillgraph.json',
            '/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/I.json',
            '/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_I.json'
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            three_arm_moveit_launch,
            magblock_planner_node,
        ]
    )
