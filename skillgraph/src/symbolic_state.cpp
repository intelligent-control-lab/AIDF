#include "symbolic_state.hpp"

namespace skillgraph {

PDDLStateUpdater::PDDLStateUpdater(ros::NodeHandle& nh, pddl_state_ptr state)
    : state_(state) {
    sub_rob1_ = nh.subscribe("/yk_destroyer/pick_place_classify", 1, &PDDLStateUpdater::callbackRob1, this);
    sub_rob2_ = nh.subscribe("/yk_architect/pick_place_classify", 1, &PDDLStateUpdater::callbackRob2, this);
    sub_assembly_complete_ = nh.subscribe("/lego/assembly_complete", 1, &PDDLStateUpdater::callbackAssemblyComplete, this);  // 新增订阅

}

void PDDLStateUpdater::callbackRob1(const std_msgs::Bool::ConstPtr& msg) {
    state_->inhand_rob1 = msg->data;
}

void PDDLStateUpdater::callbackRob2(const std_msgs::Bool::ConstPtr& msg) {
    state_->inhand_rob2 = msg->data;
}


void PDDLStateUpdater::callbackAssemblyComplete(const std_msgs::Bool::ConstPtr& msg) {
    state_->assembly = msg->data;
    ROS_INFO_STREAM("Assembly complete updated: " << (msg->data ? "true" : "false"));
}

}
