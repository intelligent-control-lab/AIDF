#!/usr/bin/env python3

"""
MagBlock Assembly Simulation Script
This script provides a complete simulation environment for magnetic block assembly
using the skillgraph framework and MoveIt trajectory planning.
"""

import rclpy
from rclpy.node import Node
import sys
import json
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

class MagBlockAssemblyController(Node):
    def __init__(self):
        super().__init__('magblock_assembly_controller')
        
        # Robot transformation configurations (from previous implementation)
        self.robot_configs = {
            "left_arm": {
                "x_origin_blocks": 16.0,
                "y_origin_blocks": -4.4,
                "default_orientation_deg": [0, -180, 90],
                "block_to_robot_matrix": np.array([[0, 1], [1, 0]])
            },
            "right_arm": {
                "x_origin_blocks": 35.15,
                "y_origin_blocks": 20.3,
                "default_orientation_deg": [0, -180, 90],
                "block_to_robot_matrix": np.array([[-1, 0], [0, 1]])
            },
            "center_arm": {
                "x_origin_blocks": 25.0,
                "y_origin_blocks": 8.0,
                "default_orientation_deg": [0, -180, 90],
                "block_to_robot_matrix": np.array([[0, -1], [-1, 0]])
            }
        }
        
        self.block_size = 0.025  # 2.5cm block size
        
        self.get_logger().info("MagBlock Assembly Controller initialized")
    
    def table_to_robot_frame(self, robot_name, x_table_blocks, y_table_blocks, z_table_blocks,
                             thetax_deg=None, thetay_deg=None, thetaz_deg=None):
        """
        Transform coordinates from block frame to robot frame.
        Mirrors the previous implementation structure.
        """
        if robot_name not in self.robot_configs:
            raise ValueError(f"Robot {robot_name} is not registered")

        config = self.robot_configs[robot_name]
        x0 = config["x_origin_blocks"]
        y0 = config["y_origin_blocks"]
        M = config["block_to_robot_matrix"]

        delta = np.array([x_table_blocks - x0, y_table_blocks - y0])
        x_robot_block_units, y_robot_block_units = M @ delta

        x_robot_m = x_robot_block_units * self.block_size + (self.block_size / 2)
        y_robot_m = y_robot_block_units * self.block_size
        z_robot_m = z_table_blocks * self.block_size

        if thetax_deg is None or thetay_deg is None or thetaz_deg is None:
            thetax_deg, thetay_deg, thetaz_deg = config["default_orientation_deg"]

        return x_robot_m, y_robot_m, z_robot_m, thetax_deg, thetay_deg, thetaz_deg
    
    def load_assembly_task(self, task_file, env_setup_file):
        """Load assembly task and environment setup from JSON files."""
        try:
            with open(task_file, 'r') as f:
                assembly_tasks = json.load(f)
            
            with open(env_setup_file, 'r') as f:
                env_setup = json.load(f)
            
            self.get_logger().info(f"Loaded {len(assembly_tasks)} assembly tasks")
            self.get_logger().info(f"Loaded {len(env_setup)} initial block positions")
            
            return assembly_tasks, env_setup
        
        except Exception as e:
            self.get_logger().error(f"Failed to load assembly files: {e}")
            return None, None
    
    def determine_robot_for_location(self, x, y, z):
        """Determine which robot should handle a given location."""
        # Simple spatial partitioning for three arms
        if x < -0.3:
            return 0, "left_arm"
        elif x > 0.3:
            return 2, "right_arm"
        else:
            return 1, "center_arm"
    
    def create_collision_object(self, block_id, x, y, z, size_x=0.025, size_y=0.025, size_z=0.025):
        """Create a collision object for the planning scene."""
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.frame_id = "world"
        collision_object.id = block_id

        # Define the block shape
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]

        # Define the block pose
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        return collision_object
    
    def simulate_assembly_sequence(self, assembly_tasks, env_setup):
        """Simulate the complete assembly sequence."""
        self.get_logger().info("Starting assembly sequence simulation")
        
        # Process each assembly task
        for step_id, step_data in assembly_tasks.items():
            self.get_logger().info(f"Processing assembly step: {step_id}")
            
            # Extract assembly parameters
            x_target = step_data["x"]
            y_target = step_data["y"]
            z_target = step_data["z"]
            robot_id = step_data.get("robot_id", 1)  # Default to center arm
            
            # Determine robot name
            robot_name = ["left_arm", "center_arm", "right_arm"][robot_id]
            
            # Transform to robot coordinates
            try:
                x_robot, y_robot, z_robot, rx, ry, rz = self.table_to_robot_frame(
                    robot_name, x_target, y_target, z_target
                )
                
                self.get_logger().info(
                    f"Robot {robot_name} target: ({x_robot:.3f}, {y_robot:.3f}, {z_robot:.3f})"
                )
                
                # Here you would call MoveIt planning and execution
                # For simulation, we just log the planned movement
                self.simulate_pick_and_place(robot_name, x_robot, y_robot, z_robot, step_id)
                
            except Exception as e:
                self.get_logger().error(f"Failed to transform coordinates for step {step_id}: {e}")
                continue
        
        self.get_logger().info("Assembly sequence simulation completed")
    
    def simulate_pick_and_place(self, robot_name, x, y, z, block_id):
        """Simulate pick and place operation."""
        self.get_logger().info(f"Simulating {robot_name} pick and place for {block_id}")
        
        # Approach position (above target)
        approach_pose = {
            'x': x, 'y': y, 'z': z + 0.1,
            'roll': 0, 'pitch': 0, 'yaw': 0
        }
        
        # Place position
        place_pose = {
            'x': x, 'y': y, 'z': z + 0.025,
            'roll': 0, 'pitch': 0, 'yaw': 0
        }
        
        self.get_logger().info(f"  Approach: ({approach_pose['x']:.3f}, {approach_pose['y']:.3f}, {approach_pose['z']:.3f})")
        self.get_logger().info(f"  Place: ({place_pose['x']:.3f}, {place_pose['y']:.3f}, {place_pose['z']:.3f})")
        
        # In a real implementation, this would:
        # 1. Plan trajectory to approach pose
        # 2. Execute approach movement
        # 3. Close gripper
        # 4. Plan trajectory to place pose
        # 5. Execute place movement
        # 6. Open gripper
        # 7. Plan retreat movement


def main():
    rclpy.init()
    
    # Default paths
    task_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/simple_stack.json"
    env_setup_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_simple_stack.json"
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        task_file = sys.argv[1]
    if len(sys.argv) > 2:
        env_setup_file = sys.argv[2]
    
    # Create controller
    controller = MagBlockAssemblyController()
    
    # Load assembly task
    assembly_tasks, env_setup = controller.load_assembly_task(task_file, env_setup_file)
    
    if assembly_tasks is None or env_setup is None:
        controller.get_logger().error("Failed to load assembly files")
        return 1
    
    # Simulate the assembly sequence
    controller.simulate_assembly_sequence(assembly_tasks, env_setup)
    
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
