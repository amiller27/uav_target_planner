#ifndef UAV_TARGET_PLANNER_STATE_HPP_
#define UAV_TARGET_PLANNER_STATE_HPP_

#include <iostream>

namespace uav_target_planner {

struct State
{
    int idx_px;
    int idx_py;
    int idx_pz;
    int idx_vx;
    int idx_vy;
    int idx_vz;
    int idx_t;

    bool operator==(const State& b) const
    {
        return idx_px == b.idx_px
            && idx_py == b.idx_py
            && idx_pz == b.idx_pz
            && idx_vx == b.idx_vx
            && idx_vy == b.idx_vy
            && idx_vz == b.idx_vz
            && idx_t  == b.idx_t;
    }
};

} // namespace uav_target_planner

namespace std {

template<> struct hash<uav_target_planner::State>
{
    std::size_t operator()(const uav_target_planner::State& s) const
    {
        return std::hash<int>{}(s.idx_px)
            ^ (std::hash<int>{}(s.idx_py) << 1)
            ^ (std::hash<int>{}(s.idx_pz) << 2)
            ^ (std::hash<int>{}(s.idx_vx) << 3)
            ^ (std::hash<int>{}(s.idx_vy) << 4)
            ^ (std::hash<int>{}(s.idx_vz) << 5)
            ^ (std::hash<int>{}(s.idx_t)  << 6);
    }
};

}

inline std::ostream& operator<<(std::ostream& os,
                                const uav_target_planner::State& s)
{
    os << "P: " << s.idx_px << ",\t"
                << s.idx_py << ",\t"
                << s.idx_pz << std::endl
       << "V: " << s.idx_vx << ",\t"
                << s.idx_vy << ",\t"
                << s.idx_vz << std::endl
       << "T: " << s.idx_t << std::endl;
    return os;
}

#endif // include guard
