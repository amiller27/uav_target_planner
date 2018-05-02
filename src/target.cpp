#include "uav_target_planner/target.hpp"

namespace uav_target_planner {

Waypoint Target::get_waypoint(double t) const
{
    // TODO improve this?
    Waypoint result;
    result.t = t;

    double i_d = t / dt;
    size_t i = std::floor(i_d);

    TargetWaypoint p1;
    TargetWaypoint p2;
    double u;
    if (i >= trajectory.size() - 1) {
        p1 = trajectory[trajectory.size() - 2];
        p2 = trajectory.back();
        u = t / dt - (trajectory.size() - 2);
    } else {
        p1 = trajectory[i];
        p2 = trajectory[i + 1];
        u = t / dt - i;
    }

    result.px = u * p2.x + (1-u) * p1.x;
    result.py = u * p2.y + (1-u) * p1.y;
    result.pz = u * p2.z + (1-u) * p1.z;

    result.vx = (p2.x - p1.x) / dt;
    result.vy = (p2.y - p1.y) / dt;
    result.vz = (p2.z - p1.z) / dt;

    return result;
}

} // namespace uav_target_planner
