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
class LegoSkillExecutor : public SkillExecutor {
public:
    LegoSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend);

    virtual bool execute(State &current_state) override;

private:
    std::shared_ptr<PlanInstance> backend_;
    std::shared_ptr<MoveitControl> controller_;

#ifdef HAVE_YK_TASKS
    ros::ServiceClient yk_move_client; // Action client for YK tasks, if available
    // rosservice client
    ros::ServiceClient yk_get_pose_client; // Service client for getting pose from YK
#endif
};


typedef std::shared_ptr<LegoSkillExecutor> LegoSkillExecutorPtr;

}