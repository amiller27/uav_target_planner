#include "uav_target_planner/occupancy_grid.hpp"

namespace uav_target_planner {

bool OccupancyGrid::collision_free(const std::shared_ptr<const State>& s) const
{
    (void)s;
    // FIXME
    return true;
}

} // namespace uav_target_planner
