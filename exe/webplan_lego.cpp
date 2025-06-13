#include "skillgraph.hpp"
#include "lego/lego_skillgraph.hpp"
#include "Utils/Logger.hpp"
#include "planner.h"

#include <cstdlib> // For std::system
#include <csignal> // For signal handling
#include <atomic>

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <sys/stat.h>
#include <functional>

using namespace skillgraph;

// Use an atomic flag to ensure signal handler and main thread don't race.
std::atomic<bool> program_running(true);
std::atomic<bool> segfault_occurred(false); // New flag for segfault

// Signal handler
void signal_handler(int signal) {
    if (program_running) {
        program_running = false;

        if (signal == SIGSEGV) {
            segfault_occurred = true; // Set the segfault flag
            std::cerr << "\nCaught SIGSEGV (Segmentation fault).  Attempting graceful shutdown...\n";
             // Attempt a graceful shutdown using pkill -TERM
            int ret = std::system("pkill -TERM -f ros");
            // DO NOT call exit() or std::system("pkill -KILL ...") here.

            // Re-raise the signal to terminate the program with a core dump.
            std::signal(SIGSEGV, SIG_DFL); // Restore default handler
            std::raise(SIGSEGV);          // Re-raise the signal
        } else if (signal == SIGINT) {
            std::cout << "\nCaught SIGINT (Ctrl+C).  Killing ROS processes..." << std::endl;
            int term_result = std::system("pkill -TERM -f ros");
            if (term_result != 0) {
                std::cerr << "pkill -TERM -f roslaunch failed.  Attempting pkill -KILL..." << std::endl;
                int ret = std::system("pkill -KILL -f ros"); // Safe in SIGINT handler
            }
        } else { //For other signals
             std::cout << "\nSignal " << signal << " received. Killing ROS processes..." << std::endl;
            int term_result = std::system("pkill -TERM -f ros");
            if (term_result != 0) {
                std::cerr << "pkill -TERM -f roslaunch failed.  Attempting pkill -KILL..." << std::endl;
                int ret = std::system("pkill -KILL -f ros"); // Safe in SIGINT handler
            }
        }

    }
}

class FileMonitor {
    public:
        FileMonitor(const std::string& filepath, std::function<void()> callback, 
                   int poll_interval_ms = 100) 
            : filepath_(filepath), callback_(callback), 
              poll_interval_ms_(poll_interval_ms), running_(false) {}
        
        void start() {
            running_ = true;
            monitor_thread_ = std::thread([this]() { this->monitorFile(); });
        }
        
        void stop() {
            running_ = false;
            if (monitor_thread_.joinable()) {
                monitor_thread_.join();
            }
        }
        
        ~FileMonitor() {
            stop();
        }
        
    private:
        void monitorFile() {
            struct stat prev_stat = {0};
            struct stat current_stat = {0};
            
            // Get initial file info
            if (stat(filepath_.c_str(), &prev_stat) != 0) {
                // File doesn't exist yet, wait for creation
                prev_stat.st_mtime = 0;
            }
            
            while (running_) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(poll_interval_ms_));
                
                if (stat(filepath_.c_str(), &current_stat) == 0) {
                    // Check if modification time changed
                    if (current_stat.st_mtime > prev_stat.st_mtime) {
                        callback_();
                        prev_stat = current_stat;
                    }
                }
            }
        }
        
        std::string filepath_;
        std::function<void()> callback_;
        int poll_interval_ms_;
        std::atomic<bool> running_;
        std::thread monitor_thread_;
    };


int main(int argc, char* argv[]) {
    try {
        // Register signal handlers.
        std::signal(SIGINT, signal_handler);
        std::signal(SIGSEGV, signal_handler);

        // Default path
        std::string config_path = "/home/mfi/repos/ros1_ws/src/philip/aidf/config/lego_tasks/skillgraph.json";
        std::string web_msg_json_path = "/home/mfi/repos/ros1_ws/src/philip/aidf/config/web_message.json";

        // Parse command-line arguments
        if (argc > 1) {
            config_path = argv[1];
        } else {
            log("Using default config path: " + config_path, LogLevel::INFO);
        }

        // Main Logic
        auto sg = std::make_shared<skillgraph::LegoSkillGraph>(config_path);
        sg->initialize();
        log("Lego Skill Graph Initialized", LogLevel::INFO);
        sg->print_skillgraph();

        State state = sg->get_initial_state();

        // Define a callback function to process the file when it changes
        auto processJsonFile = [&sg, &state, web_msg_json_path]() {
            std::ifstream file(web_msg_json_path);
            if (!file.is_open()) {
                std::cerr << "Failed to open data.json" << std::endl;
                return;
            }
            
            Json::Value web_json;
            file >> web_json;

            // Process the JSON data and use it with your skill graph
            // You might need a JSON library like nlohmann/json
            std::cout << "Processing new data from " << web_msg_json_path << std::endl;
            // web_json is something like 
            // {
            //     "skill": "PickAndPlace",
            //     "object": "b2_1",
            //     "robot": "1",
            //     "target_location": {
            //         "x": "1",
            //         "y": "2",
            //         "z": "3",
            //         "ori": ""
            //     },
            //     "command_id": 1741971091512
            // }
            
            skillgraph::SkillPtr skill;
            bool success = sg->is_feasible(state, web_json, skill);
            if (success) {
                std::cout << "Skill is feasible: " << skill->to_string() << std::endl;
                // Execute the skill
                success = skill->executor->execute(state);
                if (!success) {
                    std::string msg;
                    getPastLog(msg);
                    std::cerr << "error: not feasible, id: " << web_json["command_id"].asUInt64() << ", reason: " << "execution failed" << std::endl;
                }
            } else {
                std::string msg;
                getPastLog(msg);
                std::cerr << "error: not feasible, id: " << web_json["command_id"].asUInt64() << ", reason: " << msg << std::endl;
            }

        };

        // Setup file monitoring
        FileMonitor monitor(web_msg_json_path, processJsonFile);
        monitor.start();

        // Keep program running until any stop signal is received
        while (program_running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        monitor.stop();

        // Shutdown Logic (for natural exit)
        std::cout << "Main logic finished.  Shutting down..." << std::endl;

        // Kill any remaining ROS processes.
        int term_result = std::system("pkill -TERM -f ros");
        if (term_result != 0) {
            std::cerr << "pkill -TERM -f roslaunch failed. Attempting pkill -KILL..." << std::endl;
            int kill_result = std::system("pkill -KILL -f ros");
                if(kill_result != 0){
                    std::cerr << "pkill -KILL failed" << std::endl;
            }
        }


    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        //Consider setting program_running = false; here to initiate shutdown.
        program_running = false;
    }

     // If a segfault occurred, the program will have already terminated
    // due to the re-raised signal.  This code will only be reached
    // for a normal exit or a handled signal (like SIGINT).
    return 0;
}