#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>

#include <ros/ros.h>

#include "backend.hpp"
#include "robots.hpp"

namespace skillgraph {

// Concrete implementation using MoveIt
class MoveitInstance : public PlanInstance {
public:
    MoveitInstance(robot_state::RobotStatePtr kinematic_state,
                   const std::string &joint_group_name,
                   planning_scene::PlanningScenePtr planning_scene);
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
    virtual moveit_msgs::PlanningScene getPlanningSceneDiff() const {
        return planning_scene_diff_;
    }
    virtual void setPlanningSceneDiffClient(ros::ServiceClient &client) {
        planning_scene_diff_client_ = client;
    }
    virtual void updateScene() override;
    virtual void resetScene(bool reset_sim) override;
    virtual void setPadding(double padding) override;

    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) override;
    virtual void printKnownObjects() const override;

private:
    // moveit move_group and planning_scene_interface pointers
    std::string joint_group_name_;
    robot_state::RobotStatePtr kinematic_state_;
    planning_scene::PlanningScenePtr planning_scene_;
    moveit_msgs::PlanningScene original_scene_;

    /* store the planning scene diff temporarily*/
    moveit_msgs::PlanningScene planning_scene_diff_;
    ros::ServiceClient planning_scene_diff_client_;

    // random number generator
    std::mt19937 rng_;

};

}