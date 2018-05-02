#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "uav_target_planner/occupancy_grid.hpp"

namespace uav_target_planner {

OccupancyGrid::OccupancyGrid(const Bounds& bounds, double dp)
    : inflation_radius_(0.4),
      grid_(),
      bounds_(bounds),
      dp_(dp),
      sx_((bounds.max_px - bounds.min_px) / dp + 1),
      sy_((bounds.max_py - bounds.min_py) / dp + 1),
      sz_((bounds.max_pz - bounds.min_pz) / dp + 1),
      nh_(),
      pub_(nh_.advertise<visualization_msgs::Marker>("voxel_grid", 10))
{
    grid_.resize(sx_ * sy_ * sz_);

    for (size_t i = 0; i < sx_; i++) {
        for (size_t j = 0; j < sy_; j++) {
            for (size_t k = 0; k < sz_; k++) {
                free(i, j, k);
            }
        }
    }

#define MARK(i1, i2, j1, j2, k1, k2) \
    for (size_t i = i1; i < i2; i++) { \
        for (size_t j = j1; j < j2; j++) { \
            for (size_t k = k1; k < k2; k++) { \
                mark(i, j, k); \
            } \
        } \
    }
#define FREE(i1, i2, j1, j2, k1, k2) \
    for (size_t i = i1; i < i2; i++) { \
        for (size_t j = j1; j < j2; j++) { \
            for (size_t k = k1; k < k2; k++) { \
                free(i, j, k); \
            } \
        } \
    }

    // Short wall
    MARK(115, 125, 80, 110, 0, 30);

    // Table
    MARK(80, 110, 105, 115, 3, 10);

    // Wall with door
    MARK(105, 125, 75, 80, 0, 40);
    MARK(40, 90, 75, 80, 0, 40);
    MARK(90, 105, 75, 80, 30, 40);
    FREE(65, 85, 75, 80, 15, 25);

    // Wall w/o door
    MARK(40, 125, 45, 50, 0, 40);

#undef MARK

    visualization_msgs::Marker m;
    m.header.stamp = ros::Time();
    m.header.frame_id = "map";
    m.ns = "occupied_voxels";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;

    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 1.0;

    for (size_t i = 0; i < sx_; i++) {
        for (size_t j = 0; j < sy_; j++) {
            for (size_t k = 0; k < sz_; k++) {
                if (grid_[i*sy_*sz_ + j*sz_ + k]) {
                    geometry_msgs::Point p;
                    p.x = bounds_.min_px + dp_*i;
                    p.y = bounds_.min_py + dp_*j;
                    p.z = bounds_.min_pz + dp_*k;
                    m.points.push_back(p);
                }
            }
        }
    }

    ros::Duration(2.0).sleep();
    pub_.publish(m);

    std::vector<std::array<int, 3>> sphere_points;
    for (int i = -inflation_radius_/dp_; i < inflation_radius_/dp_; i++) {
        for (int j = -inflation_radius_/dp_; j < inflation_radius_/dp_; j++) {
            for (int k = -inflation_radius_/dp_; k < inflation_radius_/dp_; k++) {
                if (std::sqrt(i*i + j*j + k*k) < inflation_radius_/dp_) {
                    std::array<int, 3> arr = {i, j, k};
                    sphere_points.push_back(arr);
                }
            }
        }
    }

    std::vector<std::array<int, 3>> points_to_mark;
    for (size_t i = 0; i < sx_; i++) {
        for (size_t j = 0; j < sy_; j++) {
            for (size_t k = 0; k < sz_; k++) {
                if (occupied(i, j, k)) {
                    for (const auto& arr : sphere_points) {
                        int i2 = i + arr[0];
                        int j2 = j + arr[1];
                        int k2 = k + arr[2];
                        if (i2 >= 0 && i2 < sx_
                         && j2 >= 0 && j2 < sy_
                         && k2 >= 0 && k2 < sz_) {
                            std::array<int, 3> arr  = {i2, j2, k2};
                            points_to_mark.push_back(arr);
                        }
                    }
                }
            }
        }
    }

    for (const auto& arr : points_to_mark) {
        mark(arr[0], arr[1], arr[2]);
    }

    m.points.clear();
    m.ns = "inflated_occupied_voxels";
    m.color.a = 0.2;
    m.color.r = 0.5;
    m.color.g = 0.2;
    m.color.b = 0.8;
    for (size_t i = 0; i < sx_; i++) {
        for (size_t j = 0; j < sy_; j++) {
            for (size_t k = 0; k < sz_; k++) {
                if (occupied(i, j, k)) {
                    geometry_msgs::Point p;
                    p.x = bounds_.min_px + dp_*i;
                    p.y = bounds_.min_py + dp_*j;
                    p.z = bounds_.min_pz + dp_*k;
                    m.points.push_back(p);
                }
            }
        }
    }

    pub_.publish(m);
}

bool OccupancyGrid::collision_free(const std::shared_ptr<const State>& s) const
{
    return !occupied(s->idx_px, s->idx_py, s->idx_pz);
}

bool OccupancyGrid::occupied(int i, int j, int k) const
{
    return grid_[i*sy_*sz_ + j*sz_ + k];
}

void OccupancyGrid::free(int i, int j, int k)
{
    grid_[i*sy_*sz_ + j*sz_ + k] = false;
}

void OccupancyGrid::mark(int i, int j, int k)
{
    grid_[i*sy_*sz_ + j*sz_ + k] = true;
}

} // namespace uav_target_planner
