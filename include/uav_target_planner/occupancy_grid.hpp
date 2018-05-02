#ifndef UAV_TARGET_PLANNER_OCCUPANCY_GRID_HPP_
#define UAV_TARGET_PLANNER_OCCUPANCY_GRID_HPP_

#include <memory>
#include <vector>

#include "uav_target_planner/state.hpp"

namespace uav_target_planner {

class OccupancyGrid
{
  public:
    bool collision_free(const std::shared_ptr<const State>& s) const;
  private:
    std::vector<bool> grid_;
};

} // namespace uav_target_planner

#endif // include guard
