#ifndef UAV_TARGET_PLANNER_UTILS_HPP_
#define UAV_TARGET_PLANNER_UTILS_HPP_

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "uav_target_planner/state.hpp"
#include "uav_target_planner/target.hpp"
#include "uav_target_planner/waypoint.hpp"


namespace uav_target_planner {

class Environment;

class Logger
{
  public:
    Logger();

    void publish_start(const std::shared_ptr<const State>& s,
                       const std::shared_ptr<const Environment>& e);
    void publish_goal(const std::shared_ptr<const State>& s,
                      const std::shared_ptr<const Environment>& e);
    void publish_state(const std::shared_ptr<const State>& s,
                       const std::shared_ptr<const Environment>& e);
    void publish_target(const std::shared_ptr<const Target>& t,
                        double max_t);
    void publish_path(const std::vector<Waypoint>& path);
    void publish();
  private:
    ros::NodeHandle nh_;
    std::vector<geometry_msgs::Point> expansions_;
    std::vector<Waypoint> path_;
    ros::Publisher marker_pub_;

    std::vector<geometry_msgs::Point> target_traj_;

    Waypoint start_w_;
    Waypoint goal_w_;
    bool have_goal_;
};

} // namespace uav_target_planner

#endif // include guard
