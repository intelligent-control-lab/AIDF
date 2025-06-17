#pragma once

#include <functional>
#include <memory>
#include <string>

/**
 * @file node.hpp  
 * @brief ROS-agnostic node interface abstraction
 * 
 * Provides unified node interface that works with both ROS1 and ROS2
 */

namespace skillgraph {
namespace ros_compat {

// Forward declarations
template<typename T> class ServiceClient;
template<typename T> class Publisher;
template<typename T> class Subscriber;

/**
 * @brief ROS-agnostic node handle interface
 */
class NodeHandle {
public:
    virtual ~NodeHandle() = default;
    
    /**
     * @brief Create a service client
     * @tparam ServiceType The service message type
     * @param service_name Name of the service
     * @return Service client instance
     */
    template<typename ServiceType>
    std::shared_ptr<ServiceClient<ServiceType>> serviceClient(const std::string& service_name);
    
    /**
     * @brief Create a publisher
     * @tparam MessageType The message type to publish
     * @param topic_name Name of the topic
     * @param queue_size Size of the message queue
     * @return Publisher instance
     */
    template<typename MessageType>
    std::shared_ptr<Publisher<MessageType>> advertise(const std::string& topic_name, int queue_size = 10);
    
    /**
     * @brief Create a subscriber
     * @tparam MessageType The message type to subscribe to
     * @param topic_name Name of the topic
     * @param queue_size Size of the message queue
     * @param callback Callback function for received messages
     * @return Subscriber instance
     */
    template<typename MessageType>
    std::shared_ptr<Subscriber<MessageType>> subscribe(
        const std::string& topic_name,
        int queue_size,
        std::function<void(const std::shared_ptr<MessageType>&)> callback);
        
    /**
     * @brief Shutdown the node handle
     */
    virtual void shutdown() = 0;
    
protected:
    // Platform-specific implementations will override these
    virtual void* createServiceClientImpl(const std::string& service_name, const std::string& service_type) = 0;
    virtual void* createPublisherImpl(const std::string& topic_name, const std::string& message_type, int queue_size) = 0;
    virtual void* createSubscriberImpl(const std::string& topic_name, const std::string& message_type, int queue_size, void* callback) = 0;
};

/**
 * @brief Factory for creating ROS-compatible node handles
 */
class NodeFactory {
public:
    /**
     * @brief Create a node handle based on the detected ROS version
     * @param node_name Name of the node
     * @return Node handle instance
     */
    static std::shared_ptr<NodeHandle> createNode(const std::string& node_name = "aidf_node");
    
    /**
     * @brief Initialize ROS system
     * @param argc Command line argument count
     * @param argv Command line arguments
     * @param node_name Name of the node
     */
    static void init(int argc, char** argv, const std::string& node_name);
    
    /**
     * @brief Check if ROS is running
     * @return True if ROS is active
     */
    static bool ok();
    
private:
    static int detectRosVersion();
};

/**
 * @brief ROS-agnostic service client interface
 */
template<typename ServiceType>
class ServiceClient {
public:
    virtual ~ServiceClient() = default;
    
    /**
     * @brief Call the service
     * @param request Service request
     * @param response Service response
     * @return True if call was successful
     */
    virtual bool call(typename ServiceType::Request& request, 
                     typename ServiceType::Response& response) = 0;
    
    /**
     * @brief Wait for service to become available
     * @param timeout_sec Timeout in seconds (-1 for infinite)
     * @return True if service is available
     */
    virtual bool waitForExistence(double timeout_sec = -1.0) = 0;
};

/**
 * @brief ROS-agnostic publisher interface
 */
template<typename MessageType>
class Publisher {
public:
    virtual ~Publisher() = default;
    
    /**
     * @brief Publish a message
     * @param message Message to publish
     */
    virtual void publish(const MessageType& message) = 0;
    
    /**
     * @brief Get number of subscribers
     * @return Number of subscribers
     */
    virtual int getNumSubscribers() const = 0;
};

/**
 * @brief ROS-agnostic subscriber interface
 */
template<typename MessageType>
class Subscriber {
public:
    virtual ~Subscriber() = default;
    
    /**
     * @brief Get number of publishers
     * @return Number of publishers
     */
    virtual int getNumPublishers() const = 0;
};

} // namespace ros_compat
} // namespace skillgraph
