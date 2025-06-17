#include "lego/lego_objects.hpp"

namespace skillgraph {

/**
 * @brief Construct a LegoBrick from a Lego pointer, JSON node, and brick sequence.
 *
 * Initializes the LegoBrick's properties, pose, and dimensions using the provided Lego object and JSON node.
 * @param lego_ptr Pointer to the Lego object.
 * @param node JSON node containing brick data.
 * @param brick_seq Sequence identifier for the brick.
 */
LegoBrick::LegoBrick(lego_manipulation::lego::Lego::Ptr lego_ptr, const Json::Value &node, const std::string &brick_seq) {
        this->type = Object::Type::LegoBrick;
        this->brick_id = node["brick_id"].asInt();

        std::string brick_name = "b" + std::to_string(this->brick_id) + "_" + brick_seq;

        Eigen::Matrix4d brick_pose_mtx;
        lego_ptr->calc_bric_asssemble_pose(brick_name, node["x"].asInt(), node["y"].asInt(),
                 node["z"].asInt(), node["ori"].asInt(), brick_pose_mtx);
        

        
        this->name = brick_name;
        // define the object
        this->state = Object::State::Static;
        this->parent_link = "world";
        this->shape = Object::Shape::Box;
        lego_ptr->get_brick_sizes(brick_name, this->length, this->width, this->height);

        this->x = brick_pose_mtx(0, 3);
        this->y = brick_pose_mtx(1, 3);
        this->z = brick_pose_mtx(2, 3) - this->height/2;
        Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
        this->qx = quat.x();
        this->qy = quat.y();
        this->qz = quat.z();
        this->qw = quat.w();
    }

} // namespace skillgraph