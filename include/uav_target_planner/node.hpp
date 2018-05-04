#ifndef UAV_TARGET_PLANNER_NODE_HPP_
#define UAV_TARGET_PLANNER_NODE_HPP_

#include <limits>
#include <memory>

#include "uav_target_planner/state.hpp"

namespace uav_target_planner {

struct Node
{
    std::shared_ptr<const State> state;
    double g = std::numeric_limits<double>::quiet_NaN();
    double v = std::numeric_limits<double>::quiet_NaN();
    double f = std::numeric_limits<double>::quiet_NaN();
    std::shared_ptr<const Node> backp;

    bool open;

    // This node is Dunzo.  Getting expanded?  Dunzo.  You wanna use me as
    // part of a path?  Dunzo.  Hey, node, you wanna get used for anything at
    // all?  No, I'm Dunzo.
    bool donezo;

    // Last search round this node was closed in (0 for never closed)
    int closed;
};

} // namespace uav_target_planner

#endif // include guard
