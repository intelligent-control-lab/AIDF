#pragma once

#include "skills.hpp"
#include "moveit_backend.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <thread>
#include <chrono>

namespace skillgraph {

/**
 * @brief MagBlock-specific skill executor.
 * 
 * Inherits from SkillExecutor and provides execution logic for MagBlock tasks,
 * mirroring the LEGO implementation approach with MoveIt2 integration.
 */
class MagBlockSkillExecutor : public SkillExecutor {
public:
    /**
     * @brief Constructor for MagBlockSkillExecutor.
     * @param type The skill type.
     * @param backend Shared pointer to the PlanInstance backend.
     */
    MagBlockSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend);

    /**
     * @brief Execute the skill on the current state.
     * @param current_state The current state to execute on.
     * @return True if execution was successful.
     */
    virtual bool execute(State &current_state) override;

private:
    /**
     * @brief Backend plan instance.
     */
    std::shared_ptr<PlanInstance> backend_;
    
    /**
     * @brief Moveit controller for robot motion.
     */
    std::shared_ptr<MoveitControl> controller_;
    
    /**
     * @brief Skill type
     */
    Skill::Type skill_type_;

    /**
     * @brief Execute pick skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @param constraints JSON constraints for the skill.
     * @return True if successful.
     */
    bool execute_pick_skill(State &current_state);
    
    /**
     * @brief Execute place skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @return True if successful.
     */
    bool execute_place_skill(State &current_state);
    
    /**
     * @brief Execute transit skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @return True if successful.
     */
    bool execute_transit_skill(State &current_state);
    
    /**
     * @brief Execute pick and place skill for magnetic blocks.
     * @param current_state Current robot/environment state.
     * @return True if successful.
     */
    bool execute_pick_and_place_skill(State &current_state);
    
    /**
     * @brief MoveIt-specific functions for trajectory planning and execution
     */
    
    // Helper functions for MoveIt trajectory planning
    geometry_msgs::msg::Pose createPoseWithOrientation(double x, double y, double z, 
                                                      double thetax_deg, double thetay_deg, double thetaz_deg);
    geometry_msgs::msg::Pose createPlacePose(double x, double y, double z, 
                                            int press_face, int gripper_ori, double pick_thetaz);
    geometry_msgs::msg::Pose createPlaceApproachPose(const geometry_msgs::msg::Pose& place_pose, 
                                                    int press_face, double approach_distance = 0.15);
    double findOptimalThetaZ(int press_face);
    std::tuple<double, double, double> getPlaceApproachOffset(int press_face, double approach_distance = 0.15);
    std::pair<double, double> getPlaceOrientation(int gripper_ori);
    void transformSkillgraphToRobot(double x_sg, double y_sg, double z_sg, 
                                   double& x_robot, double& y_robot, double& z_robot, 
                                   double& rx, double& ry, double& rz);
    
    // MoveIt planning and execution functions
    bool planPickPlaceTrajectory(const std::string& robot_name,
                                const geometry_msgs::msg::Pose& pick_pose,
                                const geometry_msgs::msg::Pose& place_pose,
                                const std::string& object_name,
                                int press_face,
                                std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories);
    
    bool executeTrajectories(const std::string& robot_name,
                            const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
                            const std::string& object_name);
    
    // Robot state management
    std::vector<double> getCurrentJointState(int robot_id);

private:
    // MoveIt interfaces accessed through backend
    std::shared_ptr<MoveitInstance> getMoveitInstance() {
        return std::dynamic_pointer_cast<MoveitInstance>(backend_);
    }
};

/**
 * @brief Shared pointer type for MagBlockSkillExecutor.
 */
typedef std::shared_ptr<MagBlockSkillExecutor> MagBlockSkillExecutorPtr;

} // namespace skillgraph
