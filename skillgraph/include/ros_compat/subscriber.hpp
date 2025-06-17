#pragma once

#include "node.hpp"
#include <memory>
#include <string>
#include <functional>

/**
 * @file subscriber.hpp
 * @brief ROS-agnostic subscriber implementations
 * 
 * Provides concrete implementations for ROS1 and ROS2 subscribers
 */

namespace skillgraph {
namespace ros_compat {

#ifdef ROS2_BUILD
#include <rclcpp/rclcpp.hpp>

/**
 * @brief ROS2 subscriber implementation
 */
template<typename MessageType>
class ROS2Subscriber : public Subscriber<MessageType> {
public:
    ROS2Subscriber(std::shared_ptr<rclcpp::Node> node, 
                   const std::string& topic_name, 
                   int queue_size,
                   std::function<void(const std::shared_ptr<MessageType>&)> callback)
        : callback_(callback) {
        
        // ROS2 uses QoS instead of simple queue_size
        rclcpp::QoS qos(queue_size);
        
        subscriber_ = node->create_subscription<MessageType>(
            topic_name, 
            qos,
            [this](const std::shared_ptr<MessageType> msg) {
                this->callback_(msg);
            });
    }
    
    int getNumPublishers() const override {
        return static_cast<int>(subscriber_->get_publisher_count());
    }

private:
    typename rclcpp::Subscription<MessageType>::SharedPtr subscriber_;
    std::function<void(const std::shared_ptr<MessageType>&)> callback_;
};

#else // ROS1_BUILD

#include <ros/ros.h>

/**
 * @brief ROS1 subscriber implementation
 */
template<typename MessageType>
class ROS1Subscriber : public Subscriber<MessageType> {
public:
    ROS1Subscriber(ros::NodeHandle& nh, 
                   const std::string& topic_name, 
                   int queue_size,
                   std::function<void(const std::shared_ptr<MessageType>&)> callback)
        : callback_(callback) {
        
        subscriber_ = nh.subscribe<MessageType>(
            topic_name, 
            queue_size, 
            [this](const typename MessageType::ConstPtr& msg) {
                // Convert ROS1 ConstPtr to shared_ptr for compatibility
                auto shared_msg = std::make_shared<MessageType>(*msg);
                this->callback_(shared_msg);
            });
    }
    
    int getNumPublishers() const override {
        return static_cast<int>(subscriber_.getNumPublishers());
    }

private:
    ros::Subscriber subscriber_;
    std::function<void(const std::shared_ptr<MessageType>&)> callback_;
};

#endif

} // namespace ros_compat
} // namespace skillgraph
