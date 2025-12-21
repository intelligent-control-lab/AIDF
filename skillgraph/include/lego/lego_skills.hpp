/*
**********************************************************************************************************************
Ontology and Skill Graph for Autonomous Multi-Robot Assembly
AI Data Foundry (AIDF) Project

Copyright (c) 2025
Carnegie Mellon University
ARM Institute – Advanced Robotics for Manufacturing

Authors:
    Philip Huang philiphuang@cmu.edu 
    Peiqi Yu peiqiy@andrew.cmu.edu 
    Chaitanya Chawla cchawla@cs.cmu.edu 
    Changliu Liu cliu6@andrew.cmu.edu 
    Jiaoyang Li jiaoyanl@andrew.cmu.edu 
    Guanya Shi guanyas@andrew.cmu.edu

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

#pragma once

#include "skills.hpp"
#include "moveit_backend.hpp"

#ifdef HAVE_YK_TASKS
#include "yk_msgs/ExecuteCartesianTrajectory.h"
#include "yk_msgs/GetPose.h"
#endif

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

namespace skillgraph {
/**
 * @brief Executor for Lego-specific skills.
 *
 * Inherits from SkillExecutor and provides execution logic for Lego tasks.
 */
class LegoSkillExecutor : public SkillExecutor {
public:
    /**
     * @brief Constructor for LegoSkillExecutor.
     * @param type The skill type.
     * @param backend Shared pointer to the PlanInstance backend.
     */
    LegoSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend);

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

#ifdef HAVE_YK_TASKS
    /**
     * @brief Action client for YK tasks, if available.
     */
    ros::ServiceClient yk_move_client;
    /**
     * @brief Service client for getting pose from YK.
     */
    ros::ServiceClient yk_get_pose_client;
#endif
};

/**
 * @brief Shared pointer type for LegoSkillExecutor.
 */
typedef std::shared_ptr<LegoSkillExecutor> LegoSkillExecutorPtr;

}