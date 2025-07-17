/**
 * @brief Simple MagBlock Trajectory Test
 * 
 * This test demonstrates the restructured MagBlock trajectory planning
 * functionality using the MoveitInstance backend directly.
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>

#include "moveit_backend.hpp"
#include "Utils/Logger.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace skillgraph;

class MagBlockTrajectoryTest {
public:
    MagBlockTrajectoryTest() : node_(rclcpp::Node::make_shared("magblock_trajectory_test")) {
        log("Starting MagBlock Trajectory Test", LogLevel::INFO);
        
        // Initialize MoveIt backend for simulation
        initializeMoveitBackend();
        
        log("MagBlock Trajectory Test initialized successfully", LogLevel::INFO);
    }
    
    void initializeMoveitBackend() {
        try {
            // Initialize MoveIt backend for right arm
            moveit_backend_ = std::make_shared<MoveitInstance>("right_arm", "kortex_description");
            
            log("MoveIt backend initialized successfully", LogLevel::INFO);
        } catch (const std::exception& e) {
            log("Failed to initialize MoveIt backend: " + std::string(e.what()), LogLevel::ERROR);
            throw;
        }
    }
    
    geometry_msgs::msg::Pose createPose(double x, double y, double z, double rx, double ry, double rz) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        
        // Convert degrees to radians
        double roll = rx * M_PI / 180.0;
        double pitch = ry * M_PI / 180.0;
        double yaw = rz * M_PI / 180.0;
        
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        
        return pose;
    }
    
    void simulatePickPlace() {
        log("Simulating pick and place operation", LogLevel::INFO);
        
        // Create sample poses for pick and place
        auto pick_pose = createPose(0.4, -0.2, 0.25, 0.0, 180.0, 0.0);
        auto place_pose = createPose(0.3, -0.1, 0.30, 0.0, 180.0, 0.0);
        
        // Create robot states for the poses
        skillgraph::RobotState pick_state, place_state;
        pick_state.robot_name = "right_arm";
        place_state.robot_name = "right_arm";
        
        // Set some sample joint values (in practice, these would come from IK)
        pick_state.joint_values = {0.0, 0.2, 0.0, -1.5, 0.0, 1.0, 0.0};
        place_state.joint_values = {0.1, 0.3, 0.1, -1.4, 0.1, 1.1, 0.1};
        
        // Robot ID for right arm
        int robot_id = 2;
        
        // Simulate pick trajectory
        log("Moving to pick position", LogLevel::INFO);
        moveit_backend_->moveRobot(robot_id, pick_state);
        moveit_backend_->updateScene();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        log("*** PICKED UP BLOCK ***", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Simulate place trajectory
        log("Moving to place position", LogLevel::INFO);
        moveit_backend_->moveRobot(robot_id, place_state);
        moveit_backend_->updateScene();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        log("*** PLACED BLOCK ***", LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        log("Pick and place simulation completed", LogLevel::INFO);
    }
    
    void runTest() {
        log("Running MagBlock Trajectory Test", LogLevel::INFO);
        
        try {
            // Test basic trajectory simulation
            simulatePickPlace();
            
            log("MagBlock Trajectory Test PASSED", LogLevel::INFO);
            
        } catch (const std::exception& e) {
            log("MagBlock Trajectory Test FAILED: " + std::string(e.what()), LogLevel::ERROR);
        }
        
        log("Test completed", LogLevel::INFO);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MoveitInstance> moveit_backend_;
};

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    try {
        // Create and run the test
        MagBlockTrajectoryTest test;
        test.runTest();
        
        // Keep the node alive for visualization
        log("Test completed. Press Ctrl+C to exit.", LogLevel::INFO);
        rclcpp::spin(std::make_shared<rclcpp::Node>("magblock_trajectory_test_node"));
        
    } catch (const std::exception& e) {
        log("Test failed with exception: " + std::string(e.what()), LogLevel::ERROR);
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
