#ifndef UAV_TARGET_PLANNER_BOUNDS_HPP_
#define UAV_TARGET_PLANNER_BOUNDS_HPP_

struct Bounds {
    double min_px;
    double max_px;
    double min_py;
    double max_py;
    double min_pz;
    double max_pz;
    double max_vx;
    double max_vy;
    double max_vz;
    double max_t;
};

#endif // include guard
