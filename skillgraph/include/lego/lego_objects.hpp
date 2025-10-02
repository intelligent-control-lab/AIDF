#pragma once
#include "objects.hpp"
#include "lego/Lego.hpp"

namespace skillgraph {
/**
 * @brief Represents a Lego brick object in the skillgraph system.
 * 
 * Inherits from Object and adds Lego-specific properties and methods.
 */
struct LegoBrick : public Object {
    /**
     * @brief Default constructor for LegoBrick.
     */
    LegoBrick() = default;

    /**
     * @brief Construct a LegoBrick from a Lego pointer, JSON node, and brick sequence.
     * @param lego_ptr Pointer to the Lego object.
     * @param node JSON node containing brick data.
     * @param brick_seq Sequence identifier for the brick.
     */
    LegoBrick(lego_manipulation::lego::Lego::Ptr lego_ptr, const Json::Value &node, const std::string &brick_seq);

    /**
     * @brief Check if another LegoBrick is of the same type.
     * @param other The other LegoBrick to compare.
     * @return True if the bricks are of the same type.
     */
    virtual bool sameType(const LegoBrick &other) const {
        return this->brick_id == other.brick_id;
    }

    /**
     * @brief Equality operator override for LegoBrick.
     * @param other The Object to compare.
     * @return True if the objects are equal.
     */
    bool operator==(const Object &other) const override {
        const LegoBrick* other_brick = dynamic_cast<const LegoBrick*>(&other);
        if (!other_brick) {
            return false;
        }

        return Object::operator==(other) && this->brick_id == other_brick->brick_id
            && in_storage == other_brick->in_storage;
    }

    /**
     * @brief String representation of the LegoBrick.
     * @return String describing the brick.
     */
    virtual std::string to_string() const override {
        std::string str = "LegoBrick: ";
        str += "Brick ID: " + std::to_string(brick_id) + " ";
        str += "In Storage: " + std::to_string(in_storage) + " ";
        str += "Fixed: " + std::string(fixed ? "yes" : "no") + " ";
        str += Object::to_string();
        return str;
    }

    /**
     * @brief Clone this LegoBrick.
     * @return Shared pointer to the cloned LegoBrick.
     */
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
    
    /**
     * @brief Unique identifier for the brick type.
     */
    int brick_id;
    /**
     * @brief Indicates if the brick is in storage.
     */
    bool in_storage = true;
    /**
     * @brief Indicates if the brick is fixed.
     */
    bool fixed = false;
    /**
     * @brief The underlying lego_brick data structure.
     */
    lego_manipulation::lego::lego_brick brick;
};

/**
 * @brief Shared pointer type for LegoBrick.
 */
typedef std::shared_ptr<LegoBrick> LegoBrickPtr;

} // namespace skillgraph

namespace std {
    /**
     * @brief Hash specialization for LegoBrick to allow use in hash containers.
     */
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