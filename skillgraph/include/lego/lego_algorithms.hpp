#pragma once
#include "algorithms.hpp"
#include "tasks.hpp"
#include "lego/Lego.hpp"
#include "lego/lego_objects.hpp"
#include "moveit_backend.hpp"
#include <geometry_msgs/WrenchStamped.h>


#ifdef HAVE_YK_TASKS
#include "yk_tasks/GoToJointsAction.h"
#include "yk_tasks/GoToPoseAction.h"
#include "yk_msgs/GetPose.h"
#include "industrial_msgs/RobotStatus.h"
#endif


namespace skillgraph {

/**
 * @brief Configuration parameters for Lego policy and manipulation.
 */
struct LegoPolicyCfg {
    double twist_rad = 0.244346; ///< Degree for twisting the brick
    double twist_rad_handover = 0.314159; ///< Degree for twisting the brick during handover
    double z_force_threshold = -15; ///< For pick and drop
    double z_force_thresh_w_sup = -5; ///< For drop with support
    double x_force_threshold = 15; ///< For place up
    double handover_force_threshold = -10; ///< For handover
    double dt = 0.1; ///< Discretization threshold for trajectory
    double velocity = 1.0; ///< Maximum L1 velocity for one robot arm's trajectory
    double sup_force_tol = 0.03; ///< Force tolerance for touching the object to support
    double pre_support_z_offset = -0.001; ///< Z offset for pre-support pose
};

/**
 * @brief Grasp generator for Lego manipulation tasks.
 *
 * Inherits from PlanningAlgorithm and generates grasp strategies for Lego bricks.
 */
class LegoGraspGenerator : public PlanningAlgorithm {
public:
    /**
     * @brief Constructor for LegoGraspGenerator.
     * @param lego_ptr Shared pointer to Lego object.
     * @param instance Shared pointer to PlanInstance.
     * @param config Lego policy configuration.
     * @param robot Robot pointer.
     * @param object Object pointer.
     */
    LegoGraspGenerator(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr,
                    std::shared_ptr<skillgraph::PlanInstance> instance,
                    const LegoPolicyCfg &config,
                    RobotPtr robot, 
                    ObjPtr object);

    virtual bool generate(const Json::Value &constraint, Skill::Type type, int skill_seq, State &goal_state);

private:
    void calculateIKforLego(const Eigen::MatrixXd& T, const Eigen::MatrixXd & home_q,
        int robot_id, int fk_type, bool check_collision, lego_manipulation::math::VectorJd &joint_q, 
        RobotState &robot_state, bool &reachable);

    bool calculateHandoverPoses(int robot_id, std::vector<RobotState> &handover_goal, Eigen::MatrixXd &receive_q);

    std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
    std::shared_ptr<skillgraph::PlanInstance> instance_;
    LegoPolicyCfg config_;

    RobotPtr robot_;
    ObjPtr object_;
    
    // misc for grasp generation
    bool IK_status_;
};

class LegoPlan : public PlanningAlgorithm {
public:
    LegoPlan(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr,
                    std::shared_ptr<skillgraph::PlanInstance> instance,
                    const LegoPolicyCfg &config,
                    RobotPtr robot, 
                    ObjPtr object);

    virtual bool plan_skill(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, Skill::Type type, skillgraph::RobotTrajectory &traj);
    
    virtual bool plan_pick(const skillgraph::State &current_state,const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj);

    virtual bool plan_placedown(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj);

    virtual bool plan_placeup(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj);

    virtual bool plan_support(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj);

    virtual bool plan_pressdown(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj);

    virtual bool plan_handover(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj);


protected:
    std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
    std::shared_ptr<skillgraph::PlanInstance> instance_;
    LegoPolicyCfg config_;

    RobotPtr robot_;
    ObjPtr object_;

    //helper functions
    void calculateIKforLegoPlan(const Eigen::MatrixXd& T_target_pose, const Eigen::MatrixXd & seed_q_deg, 
                                 int robot_id, int fk_type_for_ik, bool check_collision, 
                                 lego_manipulation::math::VectorJd &joint_q_deg_out, 
                                 RobotState &robot_state_rad_out, bool &IK_status_out);

    void interpolate_segment(const RobotState& start_pose_rad, const RobotState& end_pose_rad, 
                               skillgraph::RobotTrajectory &traj);

};

class LegoPolicy : public ControlAlgorithm {
public:
    LegoPolicy(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr, 
                const std::vector<std::string> & group_names,
                const std::vector<std::vector<std::string>> &joint_names,
                const LegoPolicyCfg &config);
    virtual bool execute(const skillgraph::TaskParam &task_param);
    virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id);

#ifdef HAVE_YK_TASKS
    virtual void add_actionlib(const std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>> &action_clients);
    virtual void add_stop_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &stop_clients);
    virtual void add_enable_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &enable_clients);
    virtual void add_getpose_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &getpose_clients);

#endif

private:
    bool pickplace(const skillgraph::TaskParam &task_param);
    bool support(const skillgraph::TaskParam &task_param);
    bool placeup(const skillgraph::TaskParam &task_param);
    bool pressdown(const skillgraph::TaskParam &task_param);
    std::vector<double> convertQ(const lego_manipulation::math::VectorJd &q_deg);
    void setTrajectory(int robot_id, const std::vector<double>&q1, double t0,
        const std::vector<double> &q2, trajectory_msgs::JointTrajectory &joint_traj);
    void computeVelAcc(trajectory_msgs::JointTrajectory &joint_traj);

    void wrenchCallbackA(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void wrenchCallbackB(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    bool checkForce(int robot_id, const skillgraph::TaskParam &task_param, double &force_reading);
    bool stop(int robot_id);
    bool enable(int robot_id);

    std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
    std::vector<std::vector<std::string>> joint_names_;

    ros::NodeHandle nh;
    ros::Subscriber wrench_sub_a_, wrench_sub_b_;
    ros::Subscriber joint_sub_;
    geometry_msgs::WrenchStamped wrench_a_, wrench_b_;
    LegoPolicyCfg config_;
    std::vector<double> r1_joints_;
    std::vector<double> r2_joints_;
    double x_force_base_ = 0.0;
    double z_force_base_ = 0.0;
    double ts_ = 0.0;
    bool IK_status_;
    bool sup_req_ = false;


#ifdef HAVE_YK_TASKS
    void statusCallbackA(const industrial_msgs::RobotStatus::ConstPtr &msg);
    void statusCallbackB(const industrial_msgs::RobotStatus::ConstPtr &msg);
    //bool move_actionlib(int robot_id, const lego_manipulation::math::VectorJd &q_deg, Activity::Type task_type);
    //bool move_actionlib(int robot_id, const std::vector<double> &q, Activity::Type task_type);
    //bool move_actionlib(int robot_id, const yk_tasks::GoToPoseGoal &goal, Activity::Type task_type);
    double waitUntilInMotion(int robot_id);
    double waitUntilStopped(int robot_id);
    std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>> action_clients_;
    std::vector<std::shared_ptr<ros::ServiceClient>> getpose_clients_;
    std::vector<std::shared_ptr<ros::ServiceClient>> stop_clients_;
    std::vector<std::shared_ptr<ros::ServiceClient>> enable_clients_;
    ros::Subscriber status_sub_a_, status_sub_b_;
    bool moving_a, moving_b;
#endif
};


}
