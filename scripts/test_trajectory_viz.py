#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import time

class TrajectoryTester(Node):
    def __init__(self):
        super().__init__('trajectory_tester')
        
        # Create publisher for trajectory display
        self.trajectory_pub = self.create_publisher(
            DisplayTrajectory, 
            '/display_planned_path', 
            10
        )
        
        # Wait a moment for connections
        time.sleep(2)
        
        # Create a simple test trajectory
        self.publish_test_trajectory()
        
    def publish_test_trajectory(self):
        # Create display trajectory message
        display_traj = DisplayTrajectory()
        display_traj.model_id = "three_arm_robot"
        
        # Create a simple trajectory
        robot_traj = RobotTrajectory()
        
        # Create joint trajectory for left arm
        joint_traj = JointTrajectory()
        joint_traj.header = Header()
        joint_traj.header.stamp = self.get_clock().now().to_msg()
        joint_traj.header.frame_id = "world"
        
        # Add joint names (left arm joints)
        joint_traj.joint_names = [
            "left_joint_1", "left_joint_2", "left_joint_3", 
            "left_joint_4", "left_joint_5", "left_joint_6", "left_joint_7"
        ]
        
        # Add trajectory points
        for i in range(5):
            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point.positions[0] = i * 0.1  # Move first joint
            point.time_from_start.sec = i
            joint_traj.points.append(point)
        
        robot_traj.joint_trajectory = joint_traj
        display_traj.trajectory.append(robot_traj)
        
        # Publish the trajectory
        self.trajectory_pub.publish(display_traj)
        self.get_logger().info("Published test trajectory to /display_planned_path")

def main(args=None):
    rclpy.init(args=args)
    
    tester = TrajectoryTester()
    
    # Keep the node alive for a few seconds
    rclpy.spin_once(tester, timeout_sec=5.0)
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
