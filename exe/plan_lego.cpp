#include "skillgraph.hpp"
#include "lego/lego_skillgraph.hpp"
#include "Utils/Logger.hpp"

#include <cstdlib> // For std::system
#include <csignal> // For signal handling
#include <atomic>

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

int main() {
    try {
        // Register signal handlers.
        std::signal(SIGINT, signal_handler);
        std::signal(SIGSEGV, signal_handler);

        // Main Logic
        auto sg = std::make_shared<skillgraph::LegoSkillGraph>("/home/philip/catkin_ws/src/AIDF/config/lego_tasks/skillgraph.json");
        sg->initialize();
        log("Lego Skill Graph Initialized", LogLevel::INFO);
        sg->print_skillgraph();


        // Shutdown Logic (for natural exit)
        if (program_running) { // Only set program_running = false if no signal.
          program_running = false;
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