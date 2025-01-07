#include "skillgraph.hpp"

int main() {

    try {
        skillgraph::SkillGraph sg("~/catkin_ws/src/AIDF/config/lego_tasks/skillgraph.json");
        sg.print_skillgraph();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;

}