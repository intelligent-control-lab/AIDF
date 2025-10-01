#pragma once

#include "symbolic_state.hpp"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
//pddl state 针对lego

        //inhand rob1
        //inhand rob2
        //assembly
        
        //taskparam pddl
        //state pddl    
        // condition check is not used for now
        //简单的pre post condition check 先看手上有没有lego brick
        //it will be changed later


namespace skillgraph {



class LegoPDDLStateUpdater {
public:
    LegoPDDLStateUpdater(ros::NodeHandle& nh, pddl_state_ptr state);

private:
    void callbackRob1(const std_msgs::Bool::ConstPtr& msg);
    void callbackRob2(const std_msgs::Bool::ConstPtr& msg);
    void callbackAssemblyComplete(const std_msgs::Bool::ConstPtr& msg);  // 新增装配完成回调


    
    pddl_state_ptr state_;
    ros::Subscriber sub_rob1_;
    ros::Subscriber sub_rob2_;
    ros::Subscriber sub_assembly_complete_;  // 新增订阅者
};

} // namespace skillgraph