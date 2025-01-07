#include "algorithms.hpp"
#include "adg.h"
#include "object.hpp"
#include <geometry_msgs/WrenchStamped.h>

namespace algo {

struct LegoPolicyCfg {
    double twist_rad = 0.244346;
    double twist_rad_handover = 0.314159;
    double z_force_threshold = -15; // for pick and drop
    double z_force_thresh_w_sup = -5; // for drop with support
    double x_force_threshold = 15; // for place up
    double handover_force_threshold = -10; // for handover
    double dt = 0.1;
    double velocity = 1.0;
    double sup_force_tol = 0.03;
};

class LegoGripperPlanner : public Algorithm {
public:
    LegoGripperPlanner(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr,
                    std::shared_ptr<robot::PlanInstance> instance,
                    const LegoPolicyCfg &config);
    
    virtual bool plan_pick(const object::LegoBrick &brick, std::vector<task_def::ActPtr> &sub_tasks);

    virtual bool plan_place(const object::LegoBrick &brick, std::vector<task_def::ActPtr> &sub_tasks);

    virtual bool plan_support(const object::LegoBrick &brick, std::vector<task_def::ActPtr> &sub_tasks);

protected:
    std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
    std::shared_ptr<robot::PlanInstance> instance_;
    LegoPolicyCfg config_;

};

class LegoPolicy : public Algorithm {
public:
    LegoPolicy(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr, 
                const std::vector<std::string> & group_names,
                const std::vector<std::vector<std::string>> &joint_names,
                std::shared_ptr<task_def::ActivityGraph> act_graph,
                const LegoPolicyCfg &config);
    virtual bool execute(const std::shared_ptr<tpg::Node> &start_node,
                        const std::shared_ptr<tpg::Node> &end_node,
                        task_def::Activity::Type type);
    virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id);

#ifdef HAVE_YK_TASKS
    virtual void add_actionlib(const std::vector<std::shared_ptr<actionlib::SimpleActionClient<yk_tasks::GoToJointsAction>>> &action_clients);
    virtual void add_stop_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &stop_clients);
    virtual void add_enable_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &enable_clients);
    virtual void add_getpose_clients(const std::vector<std::shared_ptr<ros::ServiceClient>> &getpose_clients);
#endif

private:
    bool pickplace(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node,
                    task_def::Activity::Type type);
    bool support(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node);
    bool placeup(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node);
    bool pressdown(const std::shared_ptr<tpg::Node> &start_node,
                    const std::shared_ptr<tpg::Node> &end_node);
    std::vector<double> convertQ(const lego_manipulation::math::VectorJd &q_deg);
    void setTrajectory(int robot_id, const std::vector<double>&q1, double t0,
        const std::vector<double> &q2, trajectory_msgs::JointTrajectory &joint_traj);
    void computeVelAcc(trajectory_msgs::JointTrajectory &joint_traj);

    void wrenchCallbackA(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void wrenchCallbackB(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    bool checkForce(int robot_id, task_def::Activity::Type task_type, double &force_reading);
    bool stop(int robot_id);
    bool enable(int robot_id);

    std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr_;
    std::vector<std::vector<std::string>> joint_names_;

    ros::NodeHandle nh;
    ros::Subscriber wrench_sub_a_, wrench_sub_b_;
    ros::Subscriber joint_sub_;
    geometry_msgs::WrenchStamped wrench_a_, wrench_b_;
    std::shared_ptr<task_def::ActivityGraph> act_graph_;
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
    bool move_actionlib(int robot_id, const lego_manipulation::math::VectorJd &q_deg, Activity::Type task_type);
    bool move_actionlib(int robot_id, const std::vector<double> &q, Activity::Type task_type);
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