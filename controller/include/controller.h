#pragma once

#include "skillgraph.hpp"
#include "Utils/Math.hpp"

namespace controller {

class AssemblyController {
public:
    AssemblyController(std::shared_ptr<skillgraph::SkillGraph> skillgraph);

};

}
