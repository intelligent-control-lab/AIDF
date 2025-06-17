#include "ros_compat/time.hpp"

#include <chrono>
#include <thread>

// Conditional compilation based on ROS version
#ifdef ROS2_BUILD
    #include <rclcpp/rclcpp.hpp>
#else
    #include <ros/ros.h>
#endif

namespace skillgraph {
namespace ros_compat {

double Time::now() {
#ifdef ROS2_BUILD
    auto now = rclcpp::Clock().now();
    return now.seconds();
#else
    ros::Time now = ros::Time::now();
    return now.toSec();
#endif
}

void Time::sleep(double seconds) {
#ifdef ROS2_BUILD
    rclcpp::sleep_for(std::chrono::duration<double>(seconds));
#else
    ros::Duration(seconds).sleep();
#endif
}

auto Time::getTimeStamp() {
#ifdef ROS2_BUILD
    return rclcpp::Clock().now();
#else
    return ros::Time::now();
#endif
#endif
}

void Duration::sleep() const {
#ifdef ROS2_BUILD
    rclcpp::sleep_for(std::chrono::duration<double>(seconds_));
#else
    ros::Duration(seconds_).sleep();
#endif
}

} // namespace ros_compat
} // namespace skillgraph
