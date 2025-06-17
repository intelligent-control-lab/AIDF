#pragma once

#include "node.hpp"
#include <memory>
#include <string>

/**
 * @file service_client.hpp
 * @brief ROS-agnostic service client implementations
 * 
 * Provides concrete implementations for ROS1 and ROS2 service clients
 */

namespace skillgraph {
namespace ros_compat {

#ifdef ROS2_BUILD
#include <rclcpp/rclcpp.hpp>

/**
 * @brief ROS2 service client implementation
 */
template<typename ServiceType>
class ROS2ServiceClient : public ServiceClient<ServiceType> {
public:
    ROS2ServiceClient(std::shared_ptr<rclcpp::Node> node, const std::string& service_name)
        : client_(node->create_client<ServiceType>(service_name)) {}
    
    bool call(typename ServiceType::Request& request, 
              typename ServiceType::Response& response) override {
        auto request_ptr = std::make_shared<typename ServiceType::Request>(request);
        
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            return false;
        }
        
        auto future = client_->async_send_request(request_ptr);
        
        // Wait for response
        auto status = future.wait_for(std::chrono::seconds(10));
        if (status == std::future_status::ready) {
            auto response_ptr = future.get();
            response = *response_ptr;
            return true;
        }
        
        return false;
    }
    
    bool waitForExistence(double timeout_sec = -1.0) override {
        if (timeout_sec < 0) {
            return client_->wait_for_service();
        } else {
            return client_->wait_for_service(std::chrono::duration<double>(timeout_sec));
        }
    }

private:
    typename rclcpp::Client<ServiceType>::SharedPtr client_;
};

#else // ROS1_BUILD

#include <ros/ros.h>

/**
 * @brief ROS1 service client implementation
 */
template<typename ServiceType>
class ROS1ServiceClient : public ServiceClient<ServiceType> {
public:
    ROS1ServiceClient(ros::NodeHandle& nh, const std::string& service_name)
        : client_(nh.serviceClient<ServiceType>(service_name)) {}
    
    bool call(typename ServiceType::Request& request, 
              typename ServiceType::Response& response) override {
        ServiceType srv;
        srv.request = request;
        
        bool success = client_.call(srv);
        if (success) {
            response = srv.response;
        }
        
        return success;
    }
    
    bool waitForExistence(double timeout_sec = -1.0) override {
        if (timeout_sec < 0) {
            return client_.waitForExistence();
        } else {
            return client_.waitForExistence(ros::Duration(timeout_sec));
        }
    }

private:
    ros::ServiceClient client_;
};

#endif

} // namespace ros_compat
} // namespace skillgraph
