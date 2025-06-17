#pragma once

#include "ros_compat/node.hpp"

#ifdef ROS_VERSION_1
#include <ros/ros.h>
#include <memory>

namespace skillgraph {
namespace ros_compat {

/**
 * @brief ROS1 implementation of NodeHandle
 */
class NodeHandleROS1 : public NodeHandle {
public:
    NodeHandleROS1(const std::string& node_name);
    virtual ~NodeHandleROS1();
    
    void shutdown() override;
    
protected:
    void* createServiceClientImpl(const std::string& service_name, const std::string& service_type) override;
    void* createPublisherImpl(const std::string& topic_name, const std::string& message_type, int queue_size) override;
    void* createSubscriberImpl(const std::string& topic_name, const std::string& message_type, int queue_size, void* callback) override;
    
private:
    std::shared_ptr<ros::NodeHandle> nh_;
};

/**
 * @brief ROS1 service client implementation
 */
template<typename ServiceType>
class ServiceClientROS1 : public ServiceClient<ServiceType> {
public:
    ServiceClientROS1(ros::ServiceClient client) : client_(client) {}
    
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

/**
 * @brief ROS1 publisher implementation
 */
template<typename MessageType>
class PublisherROS1 : public Publisher<MessageType> {
public:
    PublisherROS1(ros::Publisher pub) : pub_(pub) {}
    
    void publish(const MessageType& message) override {
        pub_.publish(message);
    }
    
    int getNumSubscribers() const override {
        return pub_.getNumSubscribers();
    }
    
private:
    ros::Publisher pub_;
};

/**
 * @brief ROS1 subscriber implementation
 */
template<typename MessageType>
class SubscriberROS1 : public Subscriber<MessageType> {
public:
    SubscriberROS1(ros::Subscriber sub) : sub_(sub) {}
    
    int getNumPublishers() const override {
        return sub_.getNumPublishers();
    }
    
private:
    ros::Subscriber sub_;
};

} // namespace ros_compat
} // namespace skillgraph

#endif // ROS_VERSION_1
