#include "skillgraph.hpp"
#include "lego/lego_skillgraph.hpp"
#include "Utils/Logger.hpp"

int main() {

    try {
        
        auto sg = std::make_shared<skillgraph::LegoSkillGraph>("/home/philip/catkin_ws/src/AIDF/config/lego_tasks/skillgraph.json");
        
        sg->initialize();
        log("Lego Skill Graph Initialized", LogLevel::INFO);
        sg->print_skillgraph();
        
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;

}