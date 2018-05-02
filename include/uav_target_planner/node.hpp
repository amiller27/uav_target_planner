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
    bool donezo;
    int closed;
};

} // namespace uav_target_planner

#endif // include guard
