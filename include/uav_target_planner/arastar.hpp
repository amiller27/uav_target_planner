#ifndef UAV_TARGET_PLANNER_ARASTAR_HPP_
#define UAV_TARGET_PLANNER_ARASTAR_HPP_

#include <chrono>
#include <memory>
#include <queue>
#include <unordered_map>

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
    bool replan(double eps, int round);

    double eps_;
    std::shared_ptr<Environment> env_;
    std::shared_ptr<State> start_state_;
    std::shared_ptr<Node> goal_node_;

    std::function<bool(const std::shared_ptr<const Node>&,
                       const std::shared_ptr<const Node>&)>
        comp_ = [](const std::shared_ptr<const Node>& a,
                   const std::shared_ptr<const Node>& b) { return a->f > b->f; };
    std::priority_queue<std::shared_ptr<Node>,
                        std::vector<std::shared_ptr<Node>>,
                        decltype(comp_)>
        open_;

    std::unordered_map<State, std::shared_ptr<Node>> nodes;
    std::unordered_map<State, std::shared_ptr<Node>> incons;

    std::chrono::time_point<std::chrono::high_resolution_clock> t_start_;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_stop_;
};

} // namepace uav_target_planner

#endif // include guard
