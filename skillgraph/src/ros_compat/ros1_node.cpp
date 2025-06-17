#include "../include/ros_compat/node.hpp"
#include "../include/ros_compat/service_client.hpp"
#include "../include/ros_compat/publisher.hpp"
#include "../include/ros_compat/subscriber.hpp"

#ifndef ROS2_BUILD
// ROS1 implementation

#include <ros/ros.h>
#include <memory>

namespace skillgraph {
namespace ros_compat {

/**
 * @brief ROS1 NodeHandle implementation
 */
class ROS1NodeHandle : public NodeHandle {
public:
    ROS1NodeHandle(const std::string& node_name) 
        : nh_(node_name) {}
    
    ROS1NodeHandle() = default;
    
    void shutdown() override {
        ros::shutdown();
    }

protected:
    void* createServiceClientImpl(const std::string& service_name, const std::string& service_type) override {
        // This is a generic implementation - specific service types will be handled by templates
        return nullptr;
    }
    
    void* createPublisherImpl(const std::string& topic_name, const std::string& message_type, int queue_size) override {
        // This is a generic implementation - specific message types will be handled by templates  
        return nullptr;
    }
    
    void* createSubscriberImpl(const std::string& topic_name, const std::string& message_type, int queue_size, void* callback) override {
        // This is a generic implementation - specific message types will be handled by templates
        return nullptr;
    }

public:
    ros::NodeHandle& getNodeHandle() { return nh_; }

private:
    ros::NodeHandle nh_;
};

// Template specializations for NodeHandle methods
template<typename ServiceType>
std::shared_ptr<ServiceClient<ServiceType>> NodeHandle::serviceClient(const std::string& service_name) {
    auto* ros1_nh = dynamic_cast<ROS1NodeHandle*>(this);
    if (ros1_nh) {
        return std::make_shared<ROS1ServiceClient<ServiceType>>(ros1_nh->getNodeHandle(), service_name);
    }
    return nullptr;
}

template<typename MessageType>
std::shared_ptr<Publisher<MessageType>> NodeHandle::advertise(const std::string& topic_name, int queue_size) {
    auto* ros1_nh = dynamic_cast<ROS1NodeHandle*>(this);
    if (ros1_nh) {
        return std::make_shared<ROS1Publisher<MessageType>>(ros1_nh->getNodeHandle(), topic_name, queue_size);
    }
    return nullptr;
}

template<typename MessageType>
std::shared_ptr<Subscriber<MessageType>> NodeHandle::subscribe(
    const std::string& topic_name,
    int queue_size,
    std::function<void(const std::shared_ptr<MessageType>&)> callback) {
    
    auto* ros1_nh = dynamic_cast<ROS1NodeHandle*>(this);
    if (ros1_nh) {
        return std::make_shared<ROS1Subscriber<MessageType>>(ros1_nh->getNodeHandle(), topic_name, queue_size, callback);
    }
    return nullptr;
}

// NodeFactory implementation for ROS1
std::shared_ptr<NodeHandle> NodeFactory::createNode(const std::string& node_name) {
    return std::make_shared<ROS1NodeHandle>(node_name);
}

void NodeFactory::init(int argc, char** argv, const std::string& node_name) {
    ros::init(argc, argv, node_name);
}

bool NodeFactory::ok() {
    return ros::ok();
}

int NodeFactory::detectRosVersion() {
    return 1; // ROS1
}

} // namespace ros_compat
} // namespace skillgraph

#endif // !ROS2_BUILD
