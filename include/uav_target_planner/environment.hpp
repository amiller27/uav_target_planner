#ifndef UAV_TARGET_PLANNER_ENVIRONMENT_HPP_
#define UAV_TARGET_PLANNER_ENVIRONMENT_HPP_

#include <vector>

#include "uav_target_planner/bounds.hpp"
#include "uav_target_planner/edge.hpp"
#include "uav_target_planner/utils.hpp"
#include "uav_target_planner/occupancy_grid.hpp"
#include "uav_target_planner/target.hpp"
#include "uav_target_planner/waypoint.hpp"

namespace uav_target_planner {

class Environment
{
  public:
    Environment(double p_tol,
                double v_tol,
                double dt,
                double max_a,
                int na,
                const Bounds& bounds,
                const std::shared_ptr<const Target>& target);
    std::vector<Edge> get_successors(const std::shared_ptr<const State>& s);
    double get_heuristic(const std::shared_ptr<const State>& s);
    Waypoint get_waypoint(const std::shared_ptr<const State>& s) const;
    bool is_goal(const std::shared_ptr<const State>& s);

    Logger logger;
  private:
    std::pair<double, double> dist_to_target(
            const std::shared_ptr<const State>& s);
    bool in_bounds(const Waypoint& w);
    bool collision_free(const std::shared_ptr<const State>& s);
    std::vector<Edge> apply_mprims(const std::shared_ptr<const State>& s);
    double get_inf_vel_heuristic_1d(double x0,
                                    double x1,
                                    double v0,
                                    double v1);
    double get_inf_vel_heuristic(const std::shared_ptr<const State>& s);

    const double p_tol_;
    const double v_tol_;

    const double dp_;
    const double dv_;
    const double dt_;

    const Bounds bounds_;

    const double max_a_;
    const int max_idx_a_;

    const double heur_dt_;

    const std::shared_ptr<const OccupancyGrid> ogrid_;
    const std::shared_ptr<const Target> target_;
};

} // namespace uav_target_planner

#endif // include guard
