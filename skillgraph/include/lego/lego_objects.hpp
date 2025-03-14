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

    virtual std::shared_ptr<Object> clone() const override {
        auto obj =  std::make_shared<LegoBrick>(*this);
        obj->brick_id = this->brick_id;
        obj->in_storage = this->in_storage;
        obj->brick = this->brick;
        obj->length = this->length;
        obj->width = this->width;
        obj->height = this->height;
        obj->x = this->x;
        obj->y = this->y;
        obj->z = this->z;
        obj->qx = this->qx;
        obj->qy = this->qy;
        obj->qz = this->qz;
        obj->qw = this->qw;
        obj->state = this->state;
        obj->name = this->name;
        obj->parent_link = this->parent_link;
        return obj;
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