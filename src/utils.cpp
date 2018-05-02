#include "uav_target_planner/utils.hpp"
#include "uav_target_planner/environment.hpp"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

namespace uav_target_planner {

Logger::Logger()
    : nh_(),
      marker_pub_(nh_.advertise<visualization_msgs::Marker>("markers", 10))
{
}

void Logger::publish_start(const std::shared_ptr<const State>& s,
                           const std::shared_ptr<const Environment>& e)
{
    start_w_ = e->get_waypoint(s);
}

void Logger::publish_goal(const std::shared_ptr<const State>& s,
                          const std::shared_ptr<const Environment>& e)
{
    goal_w_ = e->get_waypoint(s);
    have_goal_ = true;
}

void Logger::publish_state(const std::shared_ptr<const State>& s,
                           const std::shared_ptr<const Environment>& e)
{
    Waypoint w = e->get_waypoint(s);

    geometry_msgs::Point p;
    p.x = w.px;
    p.y = w.py;
    p.z = w.pz;

    expansions_.push_back(p);
}

void Logger::publish_target(const std::shared_ptr<const Target>& target,
                            double max_t)
{
    for (double t = 0; t < max_t; t += target->dt) {
        Waypoint w = target->get_waypoint(t);
        geometry_msgs::Point p;
        p.x = w.px;
        p.y = w.py;
        p.z = w.pz;
        target_traj_.push_back(p);
    }
}

void Logger::publish_path(const std::vector<Waypoint>& path)
{
    path_ = path;
}

void Logger::publish()
{
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time();
    m.header.frame_id = "map";
    m.ns = "expansions";
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;

    m.scale.x = 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.05;

    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;

    for (const auto p : expansions_) {
        m.points.push_back(p);
    }

    marker_pub_.publish(m);

    m.ns = "path";
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = 0.01;

    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    m.points.clear();
    for (const auto& w : path_) {
        geometry_msgs::Point p;
        p.x = w.px;
        p.y = w.py;
        p.z = w.pz;
        m.points.push_back(p);
    }
    marker_pub_.publish(m);

    m.ns = "target";
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.scale.x = 0.01;

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    marker_pub_.publish(m);

    m.ns = "start";
    m.type = visualization_msgs::Marker::SPHERE_LIST;

    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    geometry_msgs::Point p;
    p.x = start_w_.px;
    p.y = start_w_.py;
    p.z = start_w_.pz;
    m.points.clear();
    m.points.push_back(p);
    marker_pub_.publish(m);

    if (have_goal_) {
        m.ns = "goal";
        m.points.clear();
        p.x = goal_w_.px;
        p.y = goal_w_.py;
        p.z = goal_w_.pz;
        m.points.push_back(p);
        marker_pub_.publish(m);
    }
}

} // namespace uav_target_planner
