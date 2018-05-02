#ifndef UAV_TARGET_PLANNER_WAYPOINT_HPP_
#define UAV_TARGET_PLANNER_WAYPOINT_HPP_

#include <iostream>
#include <cmath>

namespace uav_target_planner {

struct Waypoint
{
    double px;
    double py;
    double pz;
    double vx;
    double vy;
    double vz;
    double t;
};

inline std::pair<double, double> dist(const Waypoint& w1, const Waypoint& w2)
{
    return std::make_pair<double, double>(
            std::sqrt(std::pow(w1.px - w2.px, 2)
                    + std::pow(w1.py - w2.py, 2)
                    + std::pow(w1.pz - w2.pz, 2)),
            std::sqrt(std::pow(w1.vx - w2.vx, 2)
                    + std::pow(w1.vy - w2.vy, 2)
                    + std::pow(w1.vz - w2.vz, 2)));
}

} // namespace uav_target_planner

inline std::ostream& operator<<(std::ostream& os, const uav_target_planner::Waypoint& w)
{
    os << "P: " << w.px << ",\t" << w.py << ",\t" << w.pz << std::endl
       << "V: " << w.vx << ",\t" << w.vy << ",\t" << w.vz << std::endl
       << "T: " << w.t << std::endl;
    return os;
}

#endif // include guard
