#ifndef UAV_TARGET_PLANNER_OCCUPANCY_GRID_HPP_
#define UAV_TARGET_PLANNER_OCCUPANCY_GRID_HPP_

#include <memory>
#include <vector>

#include <ros/ros.h>

#include "uav_target_planner/bounds.hpp"
#include "uav_target_planner/state.hpp"

namespace uav_target_planner {

class OccupancyGrid
{
  public:
    OccupancyGrid(const Bounds& bounds, double dp);

    bool collision_free(const std::shared_ptr<const State>& s) const;
  private:
    bool occupied(int i, int j, int k) const;
    void free(int i, int j, int k);
    void mark(int i, int j, int k);

    double inflation_radius_;
    std::vector<bool> grid_;

    const Bounds bounds_;
    const double dp_;

    const double sx_;
    const double sy_;
    const double sz_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

} // namespace uav_target_planner

#endif // include guard
