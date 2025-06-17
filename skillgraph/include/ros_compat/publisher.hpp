#pragma once

#include "node.hpp"
#include <memory>
#include <string>

/**
 * @file publisher.hpp
 * @brief ROS-agnostic publisher implementations
 * 
 * Provides concrete implementations for ROS1 and ROS2 publishers
 */

namespace skillgraph {
namespace ros_compat {

#ifdef ROS2_BUILD
#include <rclcpp/rclcpp.hpp>

/**
 * @brief ROS2 publisher implementation
 */
template<typename MessageType>
class ROS2Publisher : public Publisher<MessageType> {
public:
    ROS2Publisher(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name, int queue_size)
        : publisher_(node->create_publisher<MessageType>(topic_name, queue_size)) {}
    
    void publish(const MessageType& message) override {
        publisher_->publish(message);
    }
    
    int getNumSubscribers() const override {
        return static_cast<int>(publisher_->get_subscription_count());
    }

private:
    typename rclcpp::Publisher<MessageType>::SharedPtr publisher_;
};

#else // ROS1_BUILD

#include <ros/ros.h>

/**
 * @brief ROS1 publisher implementation
 */
template<typename MessageType>
class ROS1Publisher : public Publisher<MessageType> {
public:
    ROS1Publisher(ros::NodeHandle& nh, const std::string& topic_name, int queue_size)
        : publisher_(nh.advertise<MessageType>(topic_name, queue_size)) {}
    
    void publish(const MessageType& message) override {
        publisher_.publish(message);
    }
    
    int getNumSubscribers() const override {
        return static_cast<int>(publisher_.getNumSubscribers());
    }

private:
    ros::Publisher publisher_;
};

#endif

} // namespace ros_compat
} // namespace skillgraph
