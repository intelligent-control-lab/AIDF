#include "symbolic_state.hpp"
#include "pddl_state_ros.hpp"

namespace skillgraph {

LegoPDDLStateUpdater::LegoPDDLStateUpdater(ros::NodeHandle& nh, pddl_state_ptr state)
    : state_(state) {
    sub_rob1_ = nh.subscribe("/yk_destroyer/pick_place_classify", 1, &LegoPDDLStateUpdater::callbackRob1, this);
    sub_rob2_ = nh.subscribe("/yk_architect/pick_place_classify", 1, &LegoPDDLStateUpdater::callbackRob2, this);
    sub_assembly_complete_ = nh.subscribe("/lego/assembly_complete", 1, &LegoPDDLStateUpdater::callbackAssemblyComplete, this);  // 新增订阅

}

void LegoPDDLStateUpdater::callbackRob1(const std_msgs::Bool::ConstPtr& msg) {
    (*state_)["inhand_rob1"] = msg->data;
}

void LegoPDDLStateUpdater::callbackRob2(const std_msgs::Bool::ConstPtr& msg) {
    (*state_)["inhand_rob2"] = msg->data;
}


void LegoPDDLStateUpdater::callbackAssemblyComplete(const std_msgs::Bool::ConstPtr& msg) {
    (*state_)["assembly"] = msg->data;
    ROS_INFO_STREAM("Assembly complete updated: " << (msg->data ? "true" : "false"));
}

}
