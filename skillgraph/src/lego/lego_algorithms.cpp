#include "lego/lego_algorithms.hpp"
#include "tasks.hpp"
#include "Utils/Math.hpp"
#include "Utils/Logger.hpp"
#include "adg.h"


namespace skillgraph {

LegoGraspGenerator::LegoGraspGenerator(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr,
    std::shared_ptr<skillgraph::PlanInstance> instance,
    const LegoPolicyCfg &config) : lego_ptr_(lego_ptr), instance_(instance), config_(config) {
}

bool LegoGraspGenerator::generate(const skillgraph::TaskParam &task_param, Skill::Type type) {

    return true;
}

}