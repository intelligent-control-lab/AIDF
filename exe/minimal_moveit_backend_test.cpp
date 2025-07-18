/**
 * @brief Minimal test and visualization for MoveitInstance IK and joint interpolation
 *
 * This test demonstrates:
 *  - Solving IK for a hardcoded start and goal pose
 *  - Interpolating joint trajectories between start and goal
 *  - Visualizing the trajectory in RViz using MoveIt
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_backend.hpp>
#include <Utils/Logger.hpp>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

using namespace skillgraph;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_moveit_backend_test");

    // Initialize MoveitInstance for right_arm
    auto moveit_backend = std::make_shared<MoveitInstance>(node, "right_arm", "kortex_description");

    rclcpp::sleep_for(std::chrono::seconds(3));
    
    try {
        // Hardcoded start and goal poses
        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = 0.4;
        start_pose.position.y = -0.2;
        start_pose.position.z = 0.2;
        start_pose.orientation.w = 1.0;

        geometry_msgs::msg::Pose goal_pose = start_pose;
        goal_pose.position.z -= 0.1; // Move 10cm down

        // Get current joint values for right_arm
        std::vector<double> current_joints;
        if (!moveit_backend->getCurrentJointValues("right_arm", current_joints)) {
            log("Failed to get current joint values for right_arm", LogLevel::ERROR);
            return 1;
        }

        // Solve IK for start and goal pose
        std::vector<double> start_joints, goal_joints;
        if (!moveit_backend->solveIK("right_arm", start_pose, current_joints, start_joints)) {
            log("Failed to solve IK for start pose", LogLevel::ERROR);
            return 1;
        }
        if (!moveit_backend->solveIK("right_arm", goal_pose, start_joints, goal_joints)) {
            log("Failed to solve IK for goal pose", LogLevel::ERROR);
            return 1;
        }

        // Interpolate joint trajectory
        int num_steps = 20;
        auto trajectory = moveit_backend->interpolateJointTrajectory(start_joints, goal_joints, num_steps);

        // Visualize trajectory in RViz by updating robot state in planning scene
        double step_duration = 0.1; // 100ms per step
        moveit_backend->executeJointTrajectory("right_arm", trajectory, step_duration);

        log("Minimal MoveitInstance test completed. Trajectory visualized in RViz.", LogLevel::INFO);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during MoveitInstance test: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
