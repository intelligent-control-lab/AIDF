#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/srv/execute_known_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>

#include <vector>
#include <mutex>
#include <memory>

#include "backend.hpp"
#include "robots.hpp"
#include "tasks.hpp"
#include "algorithms.hpp"

namespace skillgraph {

// Forward declaration for signal handling
class MoveitInstance;

// Concrete implementation using MoveIt for ROS 2
class MoveitInstance : public PlanInstance {
public:
    MoveitInstance(moveit::core::RobotStatePtr kinematic_state,
                   const std::string &joint_group_name,
                   planning_scene::PlanningScenePtr planning_scene);
    MoveitInstance(const std::string &move_group_name, const std::string &moveit_pkg_name);
    MoveitInstance(rclcpp::Node::SharedPtr node, const std::string &move_group_name, const std::string &moveit_pkg_name);
    ~MoveitInstance();

    // Static cleanup function for signal handling
    static void emergencyCleanup();
    
    // Static signal handler
    static void signalHandler(int signal);
    virtual bool checkCollision(const std::vector<skillgraph::RobotState> &poses, bool self, bool debug=false) override;
    virtual double computeDistance(const skillgraph::RobotState& a, const skillgraph::RobotState &b) const override;
    virtual double computeDistance(const skillgraph::RobotState& a, const skillgraph::RobotState &b, int dof) const override;
    virtual bool connect(const skillgraph::RobotState& a, const skillgraph::RobotState& b, double col_step_size = 0.1, bool debug=false) override;
    virtual bool steer(const skillgraph::RobotState& a, const skillgraph::RobotState& b, double max_dist, skillgraph::RobotState& result, double col_step_size = 0.1) override;
    virtual bool sample(skillgraph::RobotState &pose) override;
    virtual skillgraph::RobotState interpolate(const skillgraph::RobotState &a, const skillgraph::RobotState&b, double t) const override;
    virtual double interpolate(const skillgraph::RobotState &a, const skillgraph::RobotState&b, double t, int dof) const override;
    // Implementation of abstract methods using MoveIt functionalities
    virtual void addMoveableObject(const skillgraph::Object& obj) override;
    virtual void moveObject(const skillgraph::Object& obj) override;
    virtual void removeObject(const std::string& name) override;
    virtual void moveRobot(int robot_id, const skillgraph::RobotState& pose) override;
    virtual void attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const skillgraph::RobotState &pose) override;
    virtual void detachObjectFromRobot(const std::string& name, const skillgraph::RobotState &pose) override;
    virtual void setObjectColor(const std::string &name, double r, double g, double b, double a);
    virtual moveit_msgs::msg::PlanningScene getPlanningSceneDiff() const {
        return planning_scene_diff_;
    }
    virtual void setPlanningSceneDiffClient(rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr client) {
        planning_scene_diff_client_ = client;
    }
    virtual planning_scene::PlanningScenePtr getPlanningScene() const {
        return planning_scene_;
    }
    virtual void updateScene() override;
    virtual void resetScene(bool reset_sim) override;
    virtual void setPadding(double padding) override;

    virtual void computeRelativeTransform(Object &obj, const RobotState &robot_state) override;
    virtual void computeWorldTransform(Object &obj, const RobotState &robot_state) override;
    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) override;
    virtual void printKnownObjects() const override;
    virtual void setState(const State &state) override;

    // IK solver and joint-space planning functions
    bool solveIK(const std::string& robot_name, 
                 const geometry_msgs::msg::Pose& target_pose,
                 const std::vector<double>& seed_joints,
                 std::vector<double>& solution_joints);
    
    bool getCurrentJointValues(const std::string& robot_name, 
                              std::vector<double>& current_joints);
    
    std::vector<std::vector<double>> interpolateJointTrajectory(
        const std::vector<double>& start_joints,
        const std::vector<double>& end_joints,
        int num_steps);
    
    void executeJointTrajectory(const std::string& robot_name,
                               const std::vector<std::vector<double>>& joint_trajectory,
                               double step_duration = 0.1);

    // Friend class declaration
    friend class MoveitControl;

private:
    // ROS 2 node
    rclcpp::Node::SharedPtr node_;

    // Static tracking for emergency cleanup
    static std::vector<MoveitInstance*> active_instances_;
    static std::mutex instances_mutex_;
    
    // Instance cleanup method
    void cleanupProcesses();

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::core::RobotModelPtr robot_model_;
    std::string joint_group_name_;
    moveit::core::RobotStatePtr kinematic_state_;
    planning_scene::PlanningScenePtr planning_scene_;
    moveit_msgs::msg::PlanningScene original_scene_;

    /* store the planning scene diff temporarily*/
    moveit_msgs::msg::PlanningScene planning_scene_diff_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // random number generator
    std::mt19937 rng_;

};

class MoveitControl : public ControlAlgorithm {
public: 
    MoveitControl(std::shared_ptr<MoveitInstance> instance, bool fake_move);

    bool move(TaskParamPtr post_condition, const RobotTrajectory &trajectory);

private:
    std::shared_ptr<MoveitInstance> instance_;
    bool fake_move_;
};

}
