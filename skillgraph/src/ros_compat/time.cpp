#include "ros_compat/time.hpp"

#include <chrono>
#include <thread>

// Conditional compilation based on ROS version
#ifdef ROS_VERSION_2
    #include <rclcpp/rclcpp.hpp>
#elif defined(ROS_VERSION_1)  
    #include <ros/ros.h>
#endif

namespace skillgraph {
namespace ros_compat {

double Time::now() {
#ifdef ROS_VERSION_2
    auto now = rclcpp::Clock().now();
    return now.seconds();
#elif defined(ROS_VERSION_1)
    ros::Time now = ros::Time::now();
    return now.toSec();
#else
    // Fallback to system time
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration);
    return seconds.count();
#endif
}

void Time::sleep(double seconds) {
#ifdef ROS_VERSION_2
    rclcpp::sleep_for(std::chrono::duration<double>(seconds));
#elif defined(ROS_VERSION_1)
    ros::Duration(seconds).sleep();
#else
    // Fallback to standard sleep
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
#endif
}

auto Time::getTimeStamp() {
#ifdef ROS_VERSION_2
    return rclcpp::Clock().now();
#elif defined(ROS_VERSION_1)
    return ros::Time::now();
#else
    // Return a simple timestamp for non-ROS builds
    return std::chrono::system_clock::now();
#endif
}

void Duration::sleep() const {
#ifdef ROS_VERSION_2
    rclcpp::sleep_for(std::chrono::duration<double>(seconds_));
#elif defined(ROS_VERSION_1)
    ros::Duration(seconds_).sleep();
#else
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds_));
#endif
}

} // namespace ros_compat
} // namespace skillgraph
