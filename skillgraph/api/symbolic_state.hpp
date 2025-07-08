#pragma once

#include <memory>
// instructed by Phillip to delete ros dependency
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace skillgraph {

class pddl_state {
public:
    bool inhand_rob1 = false;  ///< True if robot 1 (destroyer) has object
    bool inhand_rob2 = false;  ///< True if robot 2 (architect) has object
    bool assembly = false;     ///< True if the assembly is complete
};

using pddl_state_ptr = std::shared_ptr<pddl_state>;

class PDDLStateUpdater {
public:
    PDDLStateUpdater(ros::NodeHandle& nh, pddl_state_ptr state);

private:
    void callbackRob1(const std_msgs::Bool::ConstPtr& msg);
    void callbackRob2(const std_msgs::Bool::ConstPtr& msg);
    void callbackAssemblyComplete(const std_msgs::Bool::ConstPtr& msg);  // 新增装配完成回调


    
    pddl_state_ptr state_;
    ros::Subscriber sub_rob1_;
    ros::Subscriber sub_rob2_;
    ros::Subscriber sub_assembly_complete_;  // 新增订阅者
};

}  // namespace skillgraph

