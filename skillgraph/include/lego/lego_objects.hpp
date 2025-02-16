#pragma once
#include "objects.hpp"
#include "lego/Lego.hpp"

namespace skillgraph {
struct LegoBrick : public Object {
    LegoBrick(lego_manipulation::lego::Lego::Ptr lego_ptr, const Json::Value &node, const std::string &brick_seq);
    virtual bool sameType(const LegoBrick &other) const {
        return this->brick_id == other.brick_id;
    }

    int brick_id;
    lego_manipulation::lego::lego_brick brick;
};

}