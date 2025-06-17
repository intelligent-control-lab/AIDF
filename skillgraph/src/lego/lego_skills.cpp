#include "lego/lego_skills.hpp"
#include "Utils/Logger.hpp"

namespace skillgraph {

/**
 * @brief Construct a LegoSkillExecutor for a specific skill type and backend.
 * @param type The skill type.
 * @param backend Shared pointer to the PlanInstance backend.
 */
LegoSkillExecutor::LegoSkillExecutor(Skill::Type type, std::shared_ptr<PlanInstance> backend) 
    : SkillExecutor(type), backend_(backend) {
    
    auto moveit_backend = std::dynamic_pointer_cast<MoveitInstance>(backend_);
    controller_ = std::make_shared<MoveitControl>(moveit_backend, true);

#if HAVE_YK_TASKS
    if (type == Skill::Type::TranslateWithRotation) {
        ros::NodeHandle nh;
        yk_move_client = nh.serviceClient<yk_msgs::ExecuteCartesianTrajectory>("/yk_destroyer/yk_execute_cartesian_trajectory");
        yk_get_pose_client = nh.serviceClient<yk_msgs::GetPose>("/yk_destroyer/yk_get_pose");
    }

    
#endif
}

/**
 * @brief Execute the Lego skill on the current state.
 *
 * This method executes the skill logic, including MoveIt or YK tasks if available.
 * @param current_state The current state to execute on.
 * @return True if execution was successful, false otherwise.
 */
bool LegoSkillExecutor::execute(State &current_state) {
    // To be implemented
    if (post_condition == nullptr) {
        log("Post condition is null", LogLevel::ERROR);
        return false;
    }

    if (skill_type == Skill::Type::TranslateWithRotation) {
#ifdef HAVE_YK_TASKS
        // Use actionlib for TranslateWithRotation skill
        yk_msgs::ExecuteCartesianTrajectory goal;
        std::vector<geometry_msgs::Pose> poses;

        double translate_speed = post_condition->constraints_json["Translate"]["speed"].asDouble();
        double rotate_speed = post_condition->constraints_json["Rotate"]["speed"].asDouble();
        double rotate_angle = post_condition->constraints_json["Rotate"]["angle"].asDouble();
        std::vector<double> offset;

        log("Translate speed: " + std::to_string(translate_speed) + 
            ", Rotate speed: " + std::to_string(rotate_speed) + 
            ", Rotate angle: " + std::to_string(rotate_angle), LogLevel::INFO);
        
        for (const auto & val : post_condition->constraints_json["Translate"]["offset"]) {
            offset.push_back(val.asDouble());
            log("Offset: " + std::to_string(val.asDouble()), LogLevel::INFO);

        }

        log("Getting current pose from YK", LogLevel::INFO);

        // Get the current pose from YK
        yk_msgs::GetPose get_pose_srv;
        if (!yk_get_pose_client.call(get_pose_srv)) {
            log("Failed to call YK get pose service", LogLevel::ERROR);
            return false;
        }
        geometry_msgs::Pose current_pose = get_pose_srv.response.pose;
        log("Current pose: " + std::to_string(current_pose.position.x) + ", " +
            std::to_string(current_pose.position.y) + ", " +
            std::to_string(current_pose.position.z), LogLevel::INFO);
        
        // set the trajectory based on the current pose + some offset
        geometry_msgs::Pose target_pose = current_pose;
        if (offset.size() >= 3) {
            target_pose.position.x += offset[0];
            target_pose.position.y += offset[1];
            target_pose.position.z += offset[2];
            // for orientation, rotate the yaw by rotate angle, keep all other angles the same
            tf2::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, 
                              current_pose.orientation.z, current_pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            yaw += rotate_angle; // Rotate around the Z axis
            tf2::Quaternion new_q;
            new_q.setRPY(roll, pitch, yaw);
            target_pose.orientation.x = new_q.x();
            target_pose.orientation.y = new_q.y();
            target_pose.orientation.z = new_q.z();
            target_pose.orientation.w = new_q.w();

        } else {
            log("Offset vector does not contain enough elements", LogLevel::ERROR);
            return false;
        }
        log("Target pose: " + std::to_string(target_pose.position.x) + ", " +
            std::to_string(target_pose.position.y) + ", " +
            std::to_string(target_pose.position.z), LogLevel::INFO);
        // Add the target pose to the trajectory
        poses.push_back(target_pose);

        goal.request.poses = poses;

        // call the yk_move service
        if (!yk_move_client.call(goal)) {
            log("Failed to call YK move service", LogLevel::ERROR);
            return false;
        }
        log("YK action server finished successfully", LogLevel::INFO);
#endif // HAVE_YK_TASKS

    } else {
        // Use MoveitControl for other skills
        bool success = controller_->move(post_condition, planned_trajectory_);
        current_state.robot_states = post_condition->target_state.robot_states;
        current_state.env_state = post_condition->target_state.env_state;
        // sleep for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return success;
    }
}

}