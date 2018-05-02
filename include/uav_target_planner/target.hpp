#ifndef UAV_TARGET_PLANNER_TARGET_HPP_
#define UAV_TARGET_PLANNER_TARGET_HPP_

#include <vector>

#include "uav_target_planner/waypoint.hpp"

namespace uav_target_planner {

struct TargetWaypoint
{
    double x;
    double y;
    double z;
};

struct Target
{
    std::vector<TargetWaypoint> trajectory;
    double dt;

    Waypoint get_waypoint(double t) const;
};

} // namespace uav_target_planner

#endif // include guard
