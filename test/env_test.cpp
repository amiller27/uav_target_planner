#include "uav_target_planner/environment.hpp"
#include "uav_target_planner/arastar.hpp"

#include "gtest/gtest.h"

namespace uav_target_planner {

TEST(EnvironmentTests, testSuccessors)
{
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

    double dt = 0.2;
    double max_a = 5.0;
    int na = 1;
    std::shared_ptr<Target> target = std::make_shared<Target>();
    Environment env (0, 0, dt, max_a, na, bounds, target);

    // dp should be 0.5 * 5.0 * 0.2^2 = .1
    // dv should be 1.0

    std::shared_ptr<State> s (new State());
    s->idx_px = 20;
    s->idx_py = 20;
    s->idx_pz = 20;
    s->idx_vx = 0;
    s->idx_vy = 0;
    s->idx_vz = 0;
    s->idx_t = 0;

    std::vector<Edge> edges = env.get_successors(s);
    EXPECT_EQ(edges.size(), 27);

    s->idx_px = 0;
    edges = env.get_successors(s);
    EXPECT_EQ(edges.size(), 18);

    s->idx_px = 200;
    edges = env.get_successors(s);
    EXPECT_EQ(edges.size(), 18);

    s->idx_px = 100;
    s->idx_vx = 9;
    edges = env.get_successors(s);
    EXPECT_EQ(edges.size(), 27);

    s->idx_vx = 10;
    edges = env.get_successors(s);
    EXPECT_EQ(edges.size(), 18);
}

TEST(EnvironmentTests, testTarget)
{
    Target target;
    target.dt = 1.0;
    target.trajectory = {{0, 0, 0}, {0, 0, 1}, {0, 0, 3}};

    Waypoint w = target.get_waypoint(0.5);
    EXPECT_LT(std::abs(w.px - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.py - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.pz - 0.5), 0.000001);
    EXPECT_LT(std::abs(w.vx - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vy - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vz - 1.0), 0.000001);

    w = target.get_waypoint(1.0);
    EXPECT_LT(std::abs(w.px - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.py - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.pz - 1.0), 0.000001);
    EXPECT_LT(std::abs(w.vx - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vy - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vz - 2.0), 0.000001);

    w = target.get_waypoint(3.0);
    EXPECT_LT(std::abs(w.px - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.py - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.pz - 5.0), 0.000001);
    EXPECT_LT(std::abs(w.vx - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vy - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vz - 2.0), 0.000001);

    w = target.get_waypoint(0.0);
    EXPECT_LT(std::abs(w.px - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.py - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.pz - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vx - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vy - 0.0), 0.000001);
    EXPECT_LT(std::abs(w.vz - 1.0), 0.000001);
}

TEST(EnvironmentTests, testSearchOneStep)
{
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
    target->trajectory = {{0, 0, 0}, {0, 0, 0}};
    target->dt = 100.0;

    std::shared_ptr<Environment> env = std::make_shared<Environment>(
            0.000001, 0.000001, dt, max_a, na, bounds, target);

    // dp should be 0.5 * 5.0 * 0.2^2 = .1
    // dv should be 1.0

    std::shared_ptr<State> s (new State());
    s->idx_px = 99;
    s->idx_py = 100;
    s->idx_pz = 0;
    s->idx_vx = 1;
    s->idx_vy = 0;
    s->idx_vz = 0;
    s->idx_t = 0;

    AraStar planner (3.0, env, s);
    bool result = planner.search();
    EXPECT_TRUE(result);
}

TEST(EnvironmentTests, testHeuristic)
{
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
    target->trajectory = {{1, 0, 0}, {1, 0, 0}};
    target->dt = 100.0;

    std::shared_ptr<Environment> env = std::make_shared<Environment>(
            0.000001, 0.000001, dt, max_a, na, bounds, target);

    // dp should be 0.5 * 5.0 * 0.2^2 = .1
    // dv should be 1.0

    std::shared_ptr<State> s (new State());
    s->idx_px = 100;
    s->idx_py = 100;
    s->idx_pz = 0;
    s->idx_vx = 0;
    s->idx_vy = 0;
    s->idx_vz = 0;
    s->idx_t = 0;

    double heur = env->get_heuristic(s);
    EXPECT_LT(std::abs(heur - std::sqrt(4./5)), 0.01);
}

TEST(EnvironmentTests, testSearchStationaryTargetClose)
{
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
    target->trajectory = {{0, 0, 0}, {0, 0, 0}};
    target->dt = 100.0;

    std::shared_ptr<Environment> env = std::make_shared<Environment>(
            0.000001, 0.000001, dt, max_a, na, bounds, target);

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

    AraStar planner (3.0, env, s);
    bool result = planner.search();
    EXPECT_TRUE(result);
}

} // namespace uav_target_planner

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "unit_tests");
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    ros::shutdown();
    return result;
}
