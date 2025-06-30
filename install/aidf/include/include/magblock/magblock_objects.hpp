#pragma once
#include "objects.hpp"

namespace skillgraph {
    /**
     * @brief Represents a magnetic block in the assembly.
     */
    class MagBlock : public Object {
    public:
        MagBlock() = default;
        virtual ~MagBlock() = default;

        // Block ID in the assembly sequence
        int block_id;

        // Block dimensions
        double length;
        double width;
        double height;

        // Flag to indicate if block is in storage area
        bool in_storage = true;
    };

    typedef std::shared_ptr<MagBlock> MagBlockPtr;
}
