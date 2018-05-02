#include <memory>

#include <ros/ros.h>

#include "uav_target_planner/arastar.hpp"
#include "uav_target_planner/environment.hpp"
#include "uav_target_planner/target.hpp"
#include "uav_target_planner/utils.hpp"

using namespace uav_target_planner;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "demo_node");

    Bounds bounds;

    bounds.min_px = -10.0;
    bounds.max_px = 10.0;
    bounds.min_py = -10.0;
    bounds.max_py = 10.0;
    bounds.min_pz = 0.0;
    bounds.max_pz = 5.0;

    bounds.max_vx = 10.0;
    bounds.max_vy = 10.0;
    bounds.max_vz = 10.0;

    bounds.max_t = 1000;

    double dt = 0.2;
    double max_a = 5.0;
    int na = 1;

    std::shared_ptr<Target> target (new Target());
    target->trajectory = {
        {3, 3, 0},
        {3, 3, 0}
    };
    target->dt = 10.0;

    std::shared_ptr<Environment> env = std::make_shared<Environment>(
            1.4, 3.4, dt, max_a, na, bounds, target);
    ros::Duration(2.0).sleep();

    // dp should be 0.5 * 5.0 * 0.2^2 = .1
    // dv should be 1.0

    std::shared_ptr<State> s (new State());
    s->idx_px = 96;
    s->idx_py = 100;
    s->idx_pz = 0;
    s->idx_vx = 0;
    s->idx_vy = 0;
    s->idx_vz = 0;
    s->idx_t = 0;

    env->logger.publish_target(target, 10);

    ROS_WARN_STREAM("Planning...");
    AraStar planner (1.0, env, s);
    bool result = planner.search();
    ROS_WARN_STREAM("Result: " << result);

    if (result) {
        std::vector<Waypoint> traj = planner.get_path();
        env->logger.publish_path(traj);
    }

    env->logger.publish();

    ros::spin();
}