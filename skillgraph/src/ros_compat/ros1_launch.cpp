#include "../include/ros_compat/launch.hpp"

#ifndef ROS2_BUILD
// ROS1 implementation

#include <boost/process.hpp>
#include <boost/asio.hpp>
#include <cstdlib>
#include <chrono>
#include <thread>

namespace skillgraph {
namespace ros_compat {

namespace bp = boost::process;

/**
 * @brief ROS1 process handle implementation
 */
class ROS1LaunchProcess : public LaunchProcess {
public:
    ROS1LaunchProcess(bp::child process) : process_(std::move(process)) {}
    
    bool running() const override {
        return process_.running();
    }
    
    void terminate() override {
        if (process_.running()) {
            process_.terminate();
        }
    }
    
    bool wait_for(double timeout_sec = -1.0) override {
        if (timeout_sec < 0) {
            process_.wait();
            return true;
        } else {
            return process_.wait_for(std::chrono::duration<double>(timeout_sec)) != std::future_status::timeout;
        }
    }
    
    void kill() override {
        if (process_.running()) {
            process_.terminate();
            
            // Give it a moment to terminate gracefully
            if (!process_.wait_for(std::chrono::seconds(2))) {
                // Force kill if it doesn't terminate
                system(("kill -9 " + std::to_string(process_.id())).c_str());
            }
        }
    }
    
    int exit_code() const override {
        return process_.exit_code();
    }

private:
    bp::child process_;
};

/**
 * @brief ROS1 launcher implementation
 */
class ROS1Launcher : public Launcher {
public:
    std::unique_ptr<LaunchProcess> launch(
        const std::string& package_name,
        const std::string& launch_file,
        const std::vector<std::string>& args = {}) override {
        
        std::vector<std::string> launch_args = {package_name, launch_file};
        launch_args.insert(launch_args.end(), args.begin(), args.end());
        
        boost::asio::io_context io;
        bp::environment env = boost::this_process::environment();
        env["LIBGL_ALWAYS_SOFTWARE"] = "1";  // Software rendering for compatibility
        
        std::string roslaunch_cmd = getRosLaunchPath();
        
        bp::child process(roslaunch_cmd,
                         bp::args(launch_args),
                         env,
                         bp::start_dir("/tmp"),
                         io);
        
        io.run();
        
        return std::make_unique<ROS1LaunchProcess>(std::move(process));
    }
    
    std::unique_ptr<LaunchProcess> launch_node(
        const std::string& package_name,
        const std::string& executable_name,
        const std::string& node_name = "",
        const std::vector<std::string>& args = {}) override {
        
        std::vector<std::string> run_args = {package_name, executable_name};
        
        if (!node_name.empty()) {
            run_args.push_back("__name:=" + node_name);
        }
        
        run_args.insert(run_args.end(), args.begin(), args.end());
        
        boost::asio::io_context io;
        bp::environment env = boost::this_process::environment();
        
        std::string rosrun_cmd = getRosRunPath();
        
        bp::child process(rosrun_cmd,
                         bp::args(run_args),
                         env,
                         bp::start_dir("/tmp"),
                         io);
        
        io.run();
        
        return std::make_unique<ROS1LaunchProcess>(std::move(process));
    }

private:
    std::string getRosLaunchPath() {
        // Try to find roslaunch in common locations
        const char* ros_root = std::getenv("ROS_ROOT");
        if (ros_root) {
            return std::string(ros_root) + "/bin/roslaunch";
        }
        
        std::string distro = getRosDistro();
        if (!distro.empty()) {
            return "/opt/ros/" + distro + "/bin/roslaunch";
        }
        
        return "roslaunch";  // Fallback to PATH
    }
    
    std::string getRosRunPath() {
        const char* ros_root = std::getenv("ROS_ROOT");
        if (ros_root) {
            return std::string(ros_root) + "/bin/rosrun";
        }
        
        std::string distro = getRosDistro();
        if (!distro.empty()) {
            return "/opt/ros/" + distro + "/bin/rosrun";
        }
        
        return "rosrun";  // Fallback to PATH
    }
    
    std::string getRosDistro() {
        const char* distro = std::getenv("ROS_DISTRO");
        return distro ? std::string(distro) : "noetic";  // Default to noetic
    }
};

// Factory implementation for ROS1
std::unique_ptr<Launcher> LauncherFactory::createLauncher() {
    return std::make_unique<ROS1Launcher>();
}

int LauncherFactory::detectRosVersion() {
    return 1; // ROS1
}

std::string LauncherFactory::getRosDistro() {
    const char* distro = std::getenv("ROS_DISTRO");
    return distro ? std::string(distro) : "noetic";
}

} // namespace ros_compat
} // namespace skillgraph

#endif // !ROS2_BUILD
