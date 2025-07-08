#!/usr/bin/env python3
"""
Test script to verify magblock assembly execution in simulation.
This script will:
1. Parse the magblock JSON files (env_setup_I.json, simple_stack.json)
2. Transform block positions from block frame to robot frame
3. Execute pick/place operations using MoveIt2 in simulation
4. Visualize the robot arm movements in RViz
"""

import rclpy
from rclpy.node import Node
import json
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPlanningScene
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose

# Try to import moveit_commander
try:
    import moveit_commander
    MOVEIT_COMMANDER_AVAILABLE = True
except ImportError:
    print("Warning: moveit_commander not available. Using alternative approach...")
    MOVEIT_COMMANDER_AVAILABLE = False

class MagBlockAssemblyTest(Node):
    
    def __init__(self):
        super().__init__('magblock_assembly_test')
        
        # Initialize MoveIt if available
        moveit_available = MOVEIT_COMMANDER_AVAILABLE
        if moveit_available:
            try:
                moveit_commander.roscpp_initialize([])
                self.robot = moveit_commander.RobotCommander()
                self.scene = moveit_commander.PlanningSceneInterface()
                
                # Create move groups for each arm
                self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
                self.right_arm = moveit_commander.MoveGroupCommander("right_arm") 
                self.center_arm = moveit_commander.MoveGroupCommander("center_arm")
                
                self.get_logger().info("MoveIt Commander initialized successfully")
                
            except Exception as e:
                self.get_logger().error(f"Failed to initialize MoveIt Commander: {e}")
                moveit_available = False
        
        # Store moveit availability
        self.moveit_available = moveit_available
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers for visualization
        self.collision_object_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        self.get_logger().info("MagBlock Assembly Test Node initialized")
        
    def load_env_setup(self, filename):
        """Load environment setup JSON file with initial block positions."""
        try:
            with open(filename, 'r') as f:
                env_data = json.load(f)
            self.get_logger().info(f"Loaded environment setup from {filename}")
            return env_data
        except Exception as e:
            self.get_logger().error(f"Failed to load environment setup: {e}")
            return None
            
    def load_assembly_task(self, filename):
        """Load assembly task JSON file with target positions.""" 
        try:
            with open(filename, 'r') as f:
                task_data = json.load(f)
            self.get_logger().info(f"Loaded assembly task from {filename}")
            return task_data
        except Exception as e:
            self.get_logger().error(f"Failed to load assembly task: {e}")
            return None
            
    def transform_block_position(self, block_pose, from_frame="block_frame", to_frame="world"):
        """Transform block position from block frame to robot frame."""
        try:
            # Create PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = from_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Set pose from block data
            pose_stamped.pose.position.x = block_pose.get('x', 0.0)
            pose_stamped.pose.position.y = block_pose.get('y', 0.0) 
            pose_stamped.pose.position.z = block_pose.get('z', 0.0)
            
            # Set orientation (assume identity quaternion if not specified)
            pose_stamped.pose.orientation.x = block_pose.get('qx', 0.0)
            pose_stamped.pose.orientation.y = block_pose.get('qy', 0.0)
            pose_stamped.pose.orientation.z = block_pose.get('qz', 0.0)
            pose_stamped.pose.orientation.w = block_pose.get('qw', 1.0)
            
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                
            # Apply transform
            transformed_pose = do_transform_pose(pose_stamped, transform)
            
            self.get_logger().info(f"Transformed block from {from_frame} to {to_frame}")
            self.get_logger().info(f"Original: [{block_pose.get('x', 0):.3f}, {block_pose.get('y', 0):.3f}, {block_pose.get('z', 0):.3f}]")
            self.get_logger().info(f"Transformed: [{transformed_pose.pose.position.x:.3f}, {transformed_pose.pose.position.y:.3f}, {transformed_pose.pose.position.z:.3f}]")
            
            return transformed_pose.pose
            
        except Exception as e:
            self.get_logger().warn(f"Transform failed, using original coordinates: {e}")
            # Return original pose if transform fails
            pose = Pose()
            pose.position.x = block_pose.get('x', 0.0)
            pose.position.y = block_pose.get('y', 0.0)
            pose.position.z = block_pose.get('z', 0.0)
            pose.orientation.x = block_pose.get('qx', 0.0)
            pose.orientation.y = block_pose.get('qy', 0.0)
            pose.orientation.z = block_pose.get('qz', 0.0)
            pose.orientation.w = block_pose.get('qw', 1.0)
            return pose
            
    def add_collision_object(self, name, pose, size=[0.05, 0.05, 0.05]):
        """Add a collision object (block) to the planning scene."""
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # Create a box primitive
        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        primitive.dimensions = size
        
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        collision_object.operation = collision_object.ADD
        
        # Publish collision object
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info(f"Added collision object '{name}' at [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]")
        
    def determine_robot_for_position(self, x, y, z):
        """Determine which robot arm should handle this position based on workspace."""
        # Simple spatial partitioning for three arms
        if x < -0.3:
            return "left_arm", self.left_arm if self.moveit_available else None
        elif x > 0.3:
            return "right_arm", self.right_arm if self.moveit_available else None
        else:
            return "center_arm", self.center_arm if self.moveit_available else None
            
    def plan_pick_motion(self, arm_group, target_pose):
        """Plan a pick motion to the target pose."""
        if not self.moveit_available or arm_group is None:
            self.get_logger().warn("MoveIt Commander not available, simulating pick motion")
            return None
            
        try:
            # Set the target pose
            arm_group.set_pose_target(target_pose)
            
            # Plan the motion
            plan = arm_group.plan()
            
            if plan[0]:  # plan[0] is success boolean in newer versions
                self.get_logger().info("Pick motion planned successfully")
                return plan[1]  # plan[1] is the trajectory
            else:
                self.get_logger().warn("Failed to plan pick motion")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error planning pick motion: {e}")
            return None
            
    def execute_pick_place_sequence(self, env_data, task_data):
        """Execute the full pick and place sequence."""
        self.get_logger().info("=== Starting MagBlock Assembly Execution ===")
        
        # Parse environment setup to get initial block positions
        # The JSON structure has blocks directly as keys (b1, b2, etc.)
        for block_name, block_pose in env_data.items():
            self.get_logger().info(f"Processing block: {block_name}")
            
            # Convert coordinates from block frame (grid units) to robot frame (meters)
            # Assuming a scale factor and offset for conversion
            world_x = (block_pose.get('x', 0) - 12) * 0.05  # Convert grid to meters, center around 0
            world_y = (block_pose.get('y', 0) - 12) * 0.05  
            world_z = (block_pose.get('z', 0)) * 0.05 + 0.8  # Table height offset
            
            self.get_logger().info(f"Block {block_name}: Grid[{block_pose.get('x')}, {block_pose.get('y')}, {block_pose.get('z')}] -> World[{world_x:.3f}, {world_y:.3f}, {world_z:.3f}]")
            
            # Create pose for the block in world coordinates
            block_world_pose = Pose()
            block_world_pose.position.x = world_x
            block_world_pose.position.y = world_y
            block_world_pose.position.z = world_z
            block_world_pose.orientation.w = 1.0  # Identity quaternion
            
            # Add block as collision object for visualization
            self.add_collision_object(block_name, block_world_pose)
            
            # Determine which robot should handle this block
            robot_name, robot_group = self.determine_robot_for_position(world_x, world_y, world_z)
            
            self.get_logger().info(f"Assigned {block_name} to {robot_name}")
            
            # Plan pick motion
            pick_trajectory = self.plan_pick_motion(robot_group, block_world_pose)
            
            if pick_trajectory:
                self.get_logger().info(f"Successfully planned pick motion for {block_name}")
                
                # Execute pick motion
                if self.moveit_available and robot_group:
                    try:
                        success = robot_group.execute(pick_trajectory, wait=True)
                        if success:
                            self.get_logger().info(f"✅ Executed pick motion for {block_name}")
                        else:
                            self.get_logger().warn(f"❌ Failed to execute pick motion for {block_name}")
                    except Exception as e:
                        self.get_logger().error(f"Error executing pick motion: {e}")
                else:
                    self.get_logger().info(f"🎯 Simulated pick motion for {block_name}")
                    
            else:
                self.get_logger().warn(f"Could not plan pick motion for {block_name}")
        
        # Parse assembly task for place positions  
        # The JSON structure has numbered steps as keys
        for step_id, step_data in task_data.items():
            if step_data.get('type') == 'primitive':
                target_x = step_data.get('x', 0.0)
                target_y = step_data.get('y', 0.0) 
                target_z = step_data.get('z', 0.0)
                robot_id = step_data.get('robot_id', 0)
                
                self.get_logger().info(f"Step {step_id}: Place at [{target_x:.3f}, {target_y:.3f}, {target_z:.3f}] using robot {robot_id}")
                
                # Create target pose
                target_pose = Pose()
                target_pose.position.x = target_x
                target_pose.position.y = target_y
                target_pose.position.z = target_z + 0.8  # Add table height
                target_pose.orientation.w = 1.0
                
                # Determine robot from robot_id
                if robot_id == 0:
                    robot_name = "left_arm"
                    robot_group = self.left_arm if self.moveit_available else None
                elif robot_id == 1:
                    robot_name = "right_arm" 
                    robot_group = self.right_arm if self.moveit_available else None
                elif robot_id == 2:
                    robot_name = "center_arm"
                    robot_group = self.center_arm if self.moveit_available else None
                else:
                    self.get_logger().warn(f"Unknown robot_id: {robot_id}")
                    continue
                
                self.get_logger().info(f"Using {robot_name} for step {step_id}")
                
                if robot_group:
                    place_trajectory = self.plan_pick_motion(robot_group, target_pose)
                    if place_trajectory:
                        self.get_logger().info(f"✅ Planned place motion for step {step_id}")
                        
                        # Execute place motion
                        if self.moveit_available:
                            try:
                                success = robot_group.execute(place_trajectory, wait=True)
                                if success:
                                    self.get_logger().info(f"✅ Executed place motion for step {step_id}")
                                else:
                                    self.get_logger().warn(f"❌ Failed to execute place motion for step {step_id}")
                            except Exception as e:
                                self.get_logger().error(f"Error executing place motion: {e}")
                        else:
                            self.get_logger().info(f"🎯 Simulated place motion for step {step_id}")
                    else:
                        self.get_logger().warn(f"❌ Failed to plan place motion for step {step_id}")
                else:
                    self.get_logger().info(f"🎯 Simulated place motion for step {step_id}")
                        
        self.get_logger().info("=== MagBlock Assembly Execution Complete ===")

def main():
    rclpy.init()
    
    # Create test node
    test_node = MagBlockAssemblyTest()
    
    # Wait a bit for everything to initialize
    rclpy.spin_once(test_node, timeout_sec=2.0)
    
    # Load JSON files
    env_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_I.json"
    task_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/simple_stack.json"
    
    env_data = test_node.load_env_setup(env_file)
    task_data = test_node.load_assembly_task(task_file)
    
    if env_data and task_data:
        test_node.get_logger().info("✅ Successfully loaded JSON files")
        test_node.get_logger().info(f"Environment blocks: {list(env_data.keys())}")
        test_node.get_logger().info(f"Assembly task steps: {list(task_data.keys())}")
        
        # Execute the assembly sequence
        test_node.execute_pick_place_sequence(env_data, task_data)
    else:
        test_node.get_logger().error("❌ Failed to load JSON files")
    
    # Keep node running to see results
    test_node.get_logger().info("Test complete. Spinning to keep node alive...")
    rclpy.spin(test_node)
    
    if MOVEIT_COMMANDER_AVAILABLE:
        moveit_commander.roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
