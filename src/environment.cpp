#include "uav_target_planner/environment.hpp"

#include <algorithm>
#include <iostream>
#include <limits>

namespace uav_target_planner {

// PRIVATE METHODS

std::pair<double, double> Environment::dist_to_target(const std::shared_ptr<const State>& s)
{
    Waypoint w = get_waypoint(s);
    return dist(w, target_->get_waypoint(w.t));
}

bool Environment::in_bounds(const Waypoint& w)
{
    return (w.px <= bounds_.max_px)
        && (w.px >= bounds_.min_px)
        && (w.py <= bounds_.max_py)
        && (w.py >= bounds_.min_py)
        && (w.pz <= bounds_.max_pz)
        && (w.pz >= bounds_.min_pz)
        && (w.vx <=  bounds_.max_vx)
        && (w.vx >= -bounds_.max_vx)
        && (w.vy <=  bounds_.max_vy)
        && (w.vy >= -bounds_.max_vy)
        && (w.vz <=  bounds_.max_vz)
        && (w.vz >= -bounds_.max_vz);
}

bool Environment::collision_free(const std::shared_ptr<const State>& s)
{
    return ogrid_->collision_free(s);
}

std::vector<Edge> Environment::apply_mprims(
        const std::shared_ptr<const State>& s)
{
    std::vector<Edge> results;
    for (int ax = -max_idx_a_; ax <= max_idx_a_; ax++) {
        for (int ay = -max_idx_a_; ay <= max_idx_a_; ay++) {
            for (int az = -max_idx_a_; az <= max_idx_a_; az++) {
                Edge e;
                e.s1 = s;
                e.cost = dt_;

                State* s2 = new State();
                s2->idx_px = s->idx_px + s->idx_vx * 2 + ax;
                s2->idx_py = s->idx_py + s->idx_vy * 2 + ay;
                s2->idx_pz = s->idx_pz + s->idx_vz * 2 + az;

                s2->idx_vx = s->idx_vx + ax;
                s2->idx_vy = s->idx_vy + ay;
                s2->idx_vz = s->idx_vz + az;

                s2->idx_t = s->idx_t + 1;

                e.s2.reset(s2);

                results.push_back(e);
            }
        }
    }

    return results;
}

double Environment::get_inf_vel_heuristic_1d(double x0,
                                             double x1,
                                             double v0,
                                             double v1)
{
    double dx = x1 - x0;
    double dv = v1 - v0;
    double a = max_a_;
    double D = std::pow(v0, 2) + v0*dv + 0.5 * std::pow(dv, 2) + a*dx;

    double t1 = (-2 * v0 - dv + 2 * std::sqrt(D)) / a;
    double t2 = (-2 * v0 - dv - 2 * std::sqrt(D)) / a;

    // Flip acceleration sign
    a = -max_a_;
    D = std::pow(v0, 2) + v0*dv + 0.5 * std::pow(dv, 2) + a*dx;

    double t3 = (-2 * v0 - dv + 2 * std::sqrt(D)) / a;
    double t4 = (-2 * v0 - dv - 2 * std::sqrt(D)) / a;

    if (std::isnan(t1) || t1 < 0) t1 = std::numeric_limits<double>::infinity();
    if (std::isnan(t2) || t2 < 0) t2 = std::numeric_limits<double>::infinity();
    if (std::isnan(t3) || t3 < 0) t3 = std::numeric_limits<double>::infinity();
    if (std::isnan(t4) || t4 < 0) t4 = std::numeric_limits<double>::infinity();

    double result = std::min({t1, t2, t3, t4});

    if (std::isinf(result)) {
        std::cout << x0 << " "
                  << x1 << " "
                  << v0 << " "
                  << v1 << " "
                  << t1 << " "
                  << t2 << " "
                  << t3 << " "
                  << t4 << " "
                  << result << " "
                  << a  << " "
                  << D << std::endl;
        throw std::runtime_error("BAD Poly solve");
    }

    return result;
}

double Environment::get_inf_vel_heuristic(const std::shared_ptr<const State>& s)
{
    Waypoint w = get_waypoint(s);
    for (double t = w.t; t < bounds_.max_t; t += heur_dt_) {
        Waypoint tw = target_->get_waypoint(t);
        double tx = get_inf_vel_heuristic_1d(w.px, tw.px, w.vx, tw.vx);
        double ty = get_inf_vel_heuristic_1d(w.py, tw.py, w.vy, tw.vy);
        double tz = get_inf_vel_heuristic_1d(w.pz, tw.pz, w.vz, tw.vz);
        double max_t = std::max({tx, ty, tz});
        if (max_t < (t - w.t)) return t - w.t;
    }

    throw std::runtime_error("Not enough time");
    return std::numeric_limits<double>::infinity();
}

// PUBLIC METHODS

Environment::Environment(double p_tol,
                         double v_tol,
                         double dt,
                         double max_a,
                         int na,
                         const Bounds& bounds,
                         const std::shared_ptr<const Target>& target)
    : p_tol_(p_tol),
      v_tol_(v_tol),
      dp_(0.5 * (max_a / na) * dt * dt),
      dv_((max_a / na)* dt),
      dt_(dt),
      bounds_(bounds),
      max_a_(max_a),
      max_idx_a_(na),
      heur_dt_(0.001),
      ogrid_(new OccupancyGrid()),
      target_(target)
{
}

std::vector<Edge> Environment::get_successors(
        const std::shared_ptr<const State>& s)
{
    std::vector<Edge> candidates = apply_mprims(s);
    std::vector<Edge> results;

    for (const auto& e : candidates) {
        if (in_bounds(get_waypoint(e.s2)) && collision_free(e.s2)) {
            results.push_back(e);
        }
    }

    return results;
}

double Environment::get_heuristic(const std::shared_ptr<const State>& s)
{
    return get_inf_vel_heuristic(s);
}

Waypoint Environment::get_waypoint(const std::shared_ptr<const State>& s) const
{
    Waypoint result;

    result.px = s->idx_px * dp_ + bounds_.min_px;
    result.py = s->idx_py * dp_ + bounds_.min_py;
    result.pz = s->idx_pz * dp_ + bounds_.min_pz;

    result.vx = s->idx_vx * dv_;
    result.vy = s->idx_vy * dv_;
    result.vz = s->idx_vz * dv_;

    result.t = s->idx_t * dt_;

    return result;
}

bool Environment::is_goal(const std::shared_ptr<const State>& s)
{
    std::pair<double, double> dist = dist_to_target(s);
    return (dist.first <= p_tol_ && dist.second <= v_tol_);
}

} // namespace uav_target_planner
