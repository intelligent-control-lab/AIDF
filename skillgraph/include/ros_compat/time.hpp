#pragma once

/**
 * @file time.hpp
 * @brief ROS-agnostic time abstraction layer
 * 
 * Provides unified time interface that works with both ROS1 and ROS2
 */

namespace skillgraph {
namespace ros_compat {

/**
 * @brief ROS-agnostic time interface
 */
class Time {
public:
    /**
     * @brief Get current time as seconds since epoch
     * @return Current time in seconds
     */
    static double now();
    
    /**
     * @brief Sleep for specified duration
     * @param seconds Duration to sleep in seconds
     */
    static void sleep(double seconds);
    
    /**
     * @brief Create a time stamp for message headers
     * @return Platform-specific time stamp
     */
    static auto getTimeStamp();
};

/**
 * @brief ROS-agnostic duration class
 */
class Duration {
public:
    Duration(double seconds = 0.0) : seconds_(seconds) {}
    
    void sleep() const;
    double toSec() const { return seconds_; }
    
private:
    double seconds_;
};

} // namespace ros_compat
} // namespace skillgraph
