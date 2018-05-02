#ifndef UAV_TARGET_PLANNER_ARASTAR_HPP_
#define UAV_TARGET_PLANNER_ARASTAR_HPP_

#include <memory>

#include "uav_target_planner/environment.hpp"
#include "uav_target_planner/node.hpp"
#include "uav_target_planner/state.hpp"

namespace uav_target_planner {

class AraStar
{
  public:
    AraStar(double eps,
            std::shared_ptr<Environment>& env,
            std::shared_ptr<State>& start);
    bool search();
    std::vector<Waypoint> get_path();
  private:
    double eps_;
    std::shared_ptr<Environment> env_;
    std::shared_ptr<State> start_state_;
    std::shared_ptr<Node> goal_node_;
};

} // namepace uav_target_planner

#endif // include guard
