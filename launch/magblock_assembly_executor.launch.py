from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Get launch arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Build moveit config using MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder("gen3_dual_arm", package_name="kinova_three_arm_moveit_config")
    moveit_config = moveit_config.robot_description(mappings={
        'use_fake_hardware': "true",  # Use fake hardware for simulation
        'left_robot_ip': "xxx.yyy.zzz.www",
        'right_robot_ip': "xxx.yyy.zzz.www",
        'center_robot_ip': "xxx.yyy.zzz.www",
    })
    moveit_config = moveit_config.to_moveit_configs()

    # Static TF publishers for virtual joints
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )
    
    # ros2_control node
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("kinova_three_arm_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path, {"use_sim_time": use_sim_time}],
        output="both",
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Arm controllers
    arm_controllers = []
    controller_names = ["left_arm_controller", "right_arm_controller", "center_arm_controller"]
    for controller_name in controller_names:
        arm_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name, "--controller-manager", "/controller_manager"],
            )
        )
    
    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # MagBlock Assembly Executor node
    magblock_executor_node = Node(
        package="aidf",
        executable="magblock_assembly_executor",
        name="magblock_assembly_executor",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        arguments=[
            "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/I.json",
            "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_I.json"
        ]
    )

    # RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kinova_three_arm_moveit_config"), "config", "moveit.rviz"]
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    nodes_to_launch = [
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        *arm_controllers,
        move_group_node,
        magblock_executor_node,
        rviz_node,
    ]
    
    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 for visualization",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
