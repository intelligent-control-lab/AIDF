#!/usr/bin/env python3
"""
Simple test script to verify that MoveIt2 can plan and execute trajectories in fake hardware mode.
This will help debug why the magblock planner might not be working.
"""

import rclpy
from rclpy.node import Node
from moveit import MoveItPy
import time

class MoveItTestNode(Node):
    def __init__(self):
        super().__init__('moveit_test_node')
        self.get_logger().info("Initializing MoveIt test node...")
        
        # Give MoveIt2 time to initialize
        time.sleep(3)
        
        try:
            # Initialize MoveIt2 Python interface
            self.get_logger().info("Creating MoveItPy instance...")
            self.moveit = MoveItPy(node_name="moveit_test_node")
            self.get_logger().info("MoveItPy instance created successfully!")
            
            # Get planning scene and robot model
            self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
            self.robot_model = self.planning_scene_monitor.robot_model
            
            self.get_logger().info(f"Robot model name: {self.robot_model.name}")
            self.get_logger().info(f"Available planning groups: {self.moveit.get_group_names()}")
            
            # Test planning with individual arms
            self.test_arm_movement()
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt: {e}")
    
    def test_arm_movement(self):
        """Test moving each arm individually"""
        group_names = ["left_arm", "right_arm", "center_arm"]
        
        for group_name in group_names:
            if group_name in self.moveit.get_group_names():
                self.get_logger().info(f"Testing {group_name}...")
                try:
                    # Get the planning group
                    planning_component = self.moveit.get_planning_component(group_name)
                    
                    # Set random target
                    planning_component.set_start_state_to_current_state()
                    planning_component.set_goal_state(configuration_name="ready")
                    
                    # Plan
                    self.get_logger().info(f"Planning for {group_name}...")
                    plan_result = planning_component.plan()
                    
                    if plan_result.error_code.val == plan_result.error_code.SUCCESS:
                        self.get_logger().info(f"Successfully planned for {group_name}")
                        
                        # Execute the plan
                        self.get_logger().info(f"Executing trajectory for {group_name}...")
                        robot_trajectory = plan_result.trajectory
                        
                        # Get robot executor
                        robot = self.moveit.get_planning_scene_monitor().robot_model
                        self.moveit.execute(robot_trajectory, controllers=[])
                        
                        self.get_logger().info(f"Executed trajectory for {group_name}")
                        time.sleep(2)  # Wait between movements
                        
                    else:
                        self.get_logger().warn(f"Planning failed for {group_name}: {plan_result.error_code}")
                        
                except Exception as e:
                    self.get_logger().error(f"Error testing {group_name}: {e}")
            else:
                self.get_logger().warn(f"Group {group_name} not found in available groups")

def main():
    rclpy.init()
    
    node = MoveItTestNode()
    
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
        time.sleep(5)  # Give time for test execution
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
