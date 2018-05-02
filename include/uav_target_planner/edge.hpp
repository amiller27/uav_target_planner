#ifndef UAV_TARGET_PLANNER_EDGE_HPP_
#define UAV_TARGET_PLANNER_EDGE_HPP_

#include <memory>

#include "uav_target_planner/state.hpp"

namespace uav_target_planner {

struct Edge
{
    std::shared_ptr<const State> s1;
    std::shared_ptr<const State> s2;
    double cost;
};

}

#endif // include guard
