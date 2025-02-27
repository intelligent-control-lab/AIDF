#pragma once
#include "objects.hpp"
#include "lego/Lego.hpp"

namespace skillgraph {
struct LegoBrick : public Object {
    LegoBrick() = default;

    LegoBrick(lego_manipulation::lego::Lego::Ptr lego_ptr, const Json::Value &node, const std::string &brick_seq);
    virtual bool sameType(const LegoBrick &other) const {
        return this->brick_id == other.brick_id;
    }

    bool operator==(const Object &other) const override {
        const LegoBrick* other_brick = dynamic_cast<const LegoBrick*>(&other);
        if (!other_brick) {
            return false;
        }

        return Object::operator==(other) && this->brick_id == other_brick->brick_id
            && in_storage == other_brick->in_storage;
    }

    virtual std::string to_string() const override {
        std::string str = "LegoBrick: ";
        str += "Brick ID: " + std::to_string(brick_id) + " ";
        str += "In Storage: " + std::to_string(in_storage) + " ";
        str += Object::to_string();
        return str;
    }


    int brick_id;
    bool in_storage = true;
    lego_manipulation::lego::lego_brick brick;
};
typedef std::shared_ptr<LegoBrick> LegoBrickPtr;

} // namespace skillgraph

namespace std {
    template <>
    struct hash<skillgraph::LegoBrick> {
        std::size_t operator()(const skillgraph::LegoBrick& obj) const {
            // Start with base class hash
            std::size_t seed = std::hash<skillgraph::Object>{}(obj);
            
            // Add LegoBrick specific members
            std_hash_combine(seed, obj.brick_id);
            std_hash_combine(seed, obj.in_storage);
            // Add other fields as needed
            
            return seed;
        }
    };
} // namespace std