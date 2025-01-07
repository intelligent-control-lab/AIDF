#include "robots_env.hpp"

namespace robot {

void PlanInstance::setNumberOfRobots(int num_robots) {
    num_robots_ = num_robots;
    start_poses_.resize(num_robots);
    goal_poses_.resize(num_robots);
    robot_dof_.resize(num_robots);
    hand_dof_.resize(num_robots, 0);
}

void PlanInstance::setStartPose(int robot_id, const std::vector<double> &pose) {
    start_poses_[robot_id].robot_id = robot_id;
    start_poses_[robot_id].robot_name = robot_names_[robot_id];
    start_poses_[robot_id].joint_values = pose;
}

void PlanInstance::setGoalPose(int robot_id, const std::vector<double> &pose) {
    goal_poses_[robot_id].robot_id = robot_id;
    goal_poses_[robot_id].robot_name = robot_names_[robot_id];
    goal_poses_[robot_id].joint_values = pose;
}

void PlanInstance::setRobotDOF(int robot_id, size_t dof) {
    if (robot_id >= robot_dof_.size()) {
        robot_dof_.resize(robot_id + 1);
    }
    robot_dof_[robot_id] = dof;
}

void PlanInstance::setHandDof(int robot_id, size_t dof) {
    if (robot_id >= hand_dof_.size()) {
        hand_dof_.resize(robot_id + 1);
    }
    hand_dof_[robot_id] = dof;
}


size_t PlanInstance::getRobotDOF(int robot_id) const {
    return robot_dof_[robot_id];
}

size_t PlanInstance::getHandDOF(int robot_id) const {
    return hand_dof_[robot_id];
}

RobotState PlanInstance::initRobotState(int robot_id) const {
    RobotState pose;
    pose.robot_id = robot_id;
    pose.robot_name = robot_names_[robot_id];
    pose.joint_values.resize(robot_dof_[robot_id]);
    pose.hand_values.resize(hand_dof_[robot_id], 0);
    return pose;
}

double PlanInstance::getVMax(int robot_id) {
    return v_max_;
}

void PlanInstance::setVmax(double vmax) {
    v_max_ = vmax;
}

int PlanInstance::numCollisionChecks() {
    int ans = num_collision_checks_;
    num_collision_checks_ = 0;
    return ans;
}

}