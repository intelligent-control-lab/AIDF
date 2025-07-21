#pragma once
#include "algorithms.hpp"
#include "magblock_objects.hpp"
#include "moveit_backend.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <jsoncpp/json/json.h>
#include <vector>
#include <tuple>
#include <set>
#include <memory>

namespace skillgraph {
    /**
     * @brief Configuration for the magnetic block assembly policy.
     */
    struct MagBlockPolicyCfg {
        // Approach distance for picking and placing
        double approach_distance = 0.1;
        // Grip width for the gripper
        double grip_width = 0.08;
        // Speed for robot motions
        double execution_speed = 0.5;
    };

    // ===============================
    // MagBlock Algorithm Functions
    // ===============================

    /**
     * @brief Load robot properties configuration from JSON file
     */
    void loadRobotProperties();

    /**
     * @brief Load environment setup from JSON file
     */
    Json::Value loadEnvironmentSetup();

    /**
     * @brief Load assembly instructions from JSON file
     */
    Json::Value loadAssemblyInstructions();

    /**
     * @brief Create pose with orientation from Euler angles
     */
    geometry_msgs::msg::Pose createPoseWithOrientation(double x, double y, double z, 
                                                        double thetax_deg, double thetay_deg, double thetaz_deg);

    /**
     * @brief Transform coordinates from skillgraph frame to robot frame
     */
    void transformSkillgraphToRobot(double x_sg, double y_sg, double z_sg, 
                                   double& x_robot, double& y_robot, double& z_robot, 
                                   double& rx, double& ry, double& rz);

    /**
     * @brief Transform coordinates from skillgraph frame to robot frame for specific robot
     */
    void transformSkillgraphToRobot(const std::string& robot_name, 
                                   double x_sg, double y_sg, double z_sg, 
                                   double& x_robot, double& y_robot, double& z_robot, 
                                   double& rx, double& ry, double& rz);

    /**
     * @brief Find optimal theta-Z angle that doesn't block the required press face
     */
    double findOptimalThetaZ(int press_face);

    /**
     * @brief Get blocked faces for a given thetaz orientation
     */
    std::set<int> getBlockedFaces(double thetaz_deg);

    /**
     * @brief Get approach direction offset based on press_face for place operation
     */
    std::tuple<double, double, double> getPlaceApproachOffset(int press_face, double approach_distance = 0.15);

    /**
     * @brief Get place orientation based on gripper_ori parameter
     */
    std::pair<double, double> getPlaceOrientation(int gripper_ori);

    /**
     * @brief Create place pose with proper orientation
     */
    geometry_msgs::msg::Pose createPlacePose(double x, double y, double z, 
                                              int press_face, int gripper_ori, double pick_thetaz);

    /**
     * @brief Create approach pose for place operation
     */
    geometry_msgs::msg::Pose createPlaceApproachPose(const geometry_msgs::msg::Pose& place_pose, 
                                                      int press_face, double approach_distance);

    /**
     * @brief Get block pick position from environment setup
     */
    bool getBlockPickPosition(const std::string& block_name, double& x, double& y, double& z);

    /**
     * @brief Get block place position from assembly instructions
     */
    bool getBlockPlacePosition(const std::string& block_name, double& x, double& y, double& z, 
                               int& press_face, int& gripper_ori);

    /**
     * @brief Plan pick-place trajectory using MoveIt planning and execution
     */
    bool planPickPlaceTrajectory(std::shared_ptr<MoveitInstance> moveit_instance,
                                 const std::string& robot_name,
                                 const geometry_msgs::msg::Pose& pick_pose,
                                 const geometry_msgs::msg::Pose& place_pose,
                                 const std::string& object_name,
                                 int press_face,
                                 std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories);
    
    bool planTransit(std::shared_ptr<MoveitInstance> moveit_instance,
                             const std::string& robot_name,
                             const geometry_msgs::msg::Pose& goal_pose,
                             std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories);

    /**
     * @brief Get the current joint state for a robot to ensure trajectory continuity
     */
    std::vector<double> getCurrentJointState(std::shared_ptr<MoveitInstance> moveit_instance, int robot_id);

    /**
     * @brief Interpolate between two joint configurations and move robot for visualization
     */
    void interpolateAndMoveRobot(std::shared_ptr<MoveitInstance> moveit_instance,
                                 int robot_id,
                                 const std::vector<double>& start_joints,
                                 const std::vector<double>& end_joints,
                                 int steps,
                                 int delay_ms);

    // ===============================
    // Planning Algorithm Classes
    // ===============================

    /**
     * @brief Generator for magnetic block grasp poses.
     */
    class MagBlockGraspGenerator {
    public:
        MagBlockGraspGenerator(std::shared_ptr<PlanInstance> backend,
                             const MagBlockPolicyCfg& config,
                             RobotPtr robot,
                             ObjPtr object)
            : backend_(backend), config_(config), robot_(robot), object_(object) {}

        /**
         * @brief Generate grasp poses for a given skill.
         * @param constraints JSON constraints for the skill.
         * @param type Type of the skill.
         * @param seq_id Sequence ID of the skill.
         * @param state Target state to update.
         * @return True if generation was successful.
         */
        bool generate(const Json::Value& constraints, Skill::Type type, int seq_id, State& state);

    protected:
        std::shared_ptr<PlanInstance> backend_;
        MagBlockPolicyCfg config_;
        RobotPtr robot_;
        ObjPtr object_;
    };

    /**
     * @brief Get current robot end-effector pose using forward kinematics.
     */
    bool getCurrentRobotPose(std::shared_ptr<PlanInstance> backend,
                           const std::string& robot_name, 
                           geometry_msgs::msg::Pose& current_pose);

    /**
     * @brief Calculate the exact approach pose for a task, consistent with pick/place operations.
     */
    bool calculateApproachPose(std::shared_ptr<PlanInstance> backend,
                             const Json::Value& task_constraints,
                             const std::string& robot_name, 
                             geometry_msgs::msg::Pose& approach_pose);

    /**
     * @brief Planning algorithms for magnetic block assembly.
     */
    class MagBlockPlan : public PlanningAlgorithm {
    public:
        MagBlockPlan(std::shared_ptr<PlanInstance> backend,
                    const MagBlockPolicyCfg& config,
                    RobotPtr robot,
                    ObjPtr object)
            : backend_(backend), config_(config), robot_(robot), object_(object) {}

        /**
         * @brief Plan a skill execution.
         * @param current_state Current state.
         * @param task_param Task parameters.
         * @param type Skill type.
         * @param traj Output trajectory.
         * @return True if planning was successful.
         */
        bool plan_skill(const State& current_state,
                       const TaskParam& task_param,
                       Skill::Type type,
                       RobotTrajectory& traj);

    protected:
        /**
         * @brief Plan a pick operation.
         */
        bool plan_pick(const State& current_state,
                      const TaskParam& task_param,
                      RobotTrajectory& traj);

        /**
         * @brief Plan a place operation.
         */
        bool plan_place(const State& current_state,
                       const TaskParam& task_param,
                       RobotTrajectory& traj);

    protected:
        std::shared_ptr<PlanInstance> backend_;
        MagBlockPolicyCfg config_;
        RobotPtr robot_;
        ObjPtr object_;
    };
}
