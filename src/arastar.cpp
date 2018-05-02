#include <algorithm>
#include <chrono>
#include <memory>
#include <queue>
#include <unordered_map>

#include <ros/ros.h>

#include "uav_target_planner/utils.hpp"
#include "uav_target_planner/node.hpp"
#include "uav_target_planner/arastar.hpp"

namespace uav_target_planner {

AraStar::AraStar(double eps,
                 std::shared_ptr<Environment>& env,
                 std::shared_ptr<State>& start)
    : eps_(eps),
      env_(env),
      start_state_(start)
{
}

bool AraStar::search()
{
    const auto comp = [](const std::shared_ptr<const Node>& a,
                         const std::shared_ptr<const Node>& b) { return a->f > b->f; };
    std::priority_queue<std::shared_ptr<Node>,
                        std::vector<std::shared_ptr<Node>>,
                        decltype(comp)>
        open (comp);

    std::unordered_map<State, std::shared_ptr<Node>> nodes;

    ROS_WARN("START");
    env_->logger.publish_start(start_state_, env_);

    const auto t_start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<Node> start_node (new Node());
    start_node->state = start_state_;
    start_node->g = 0;
    start_node->f = eps_ * env_->get_heuristic(start_node->state);
    start_node->open = true;
    open.push(std::move(start_node));
    nodes.insert({*start_state_, start_node});

    int expansions = 0;
    while (open.size() != 0) {
        std::shared_ptr<Node> s = open.top();
        open.pop();
        if (s->donezo) continue;

        s->open = false;
        s->closed = true;

        expansions++;
        ROS_WARN_THROTTLE(0.3, "%d %f %f", expansions, s->g, s->f);
        env_->logger.publish_state(s->state, env_);

        if (!std::isfinite(s->f)) {
            // Goal not reachable from here
            throw std::runtime_error("BAD");
            continue;
        }

        if (env_->is_goal(s->state)) {
            ROS_WARN("GOAL");
            const auto t_end = std::chrono::high_resolution_clock::now();
            ROS_WARN_STREAM("Planning time: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()
                    << "ms");
            ROS_WARN_STREAM("Expansions: " << expansions);
            ROS_WARN_STREAM("Cost (s): " << s->g);
            // Found goal!
            env_->logger.publish_goal(s->state, env_);
            goal_node_ = s;
            return true;
        }

        for (auto& e : env_->get_successors(s->state)) {
            bool already_created = (nodes.count(*e.s2) != 0);
            if (!already_created || !nodes[*e.s2]->closed) {
                double new_g = s->g + e.cost;
                if (!already_created || new_g < nodes[*e.s2]->g) {
                    std::shared_ptr<Node> node (new Node());
                    node->state = e.s2;
                    node->g = new_g;
                    node->f = new_g + eps_ * env_->get_heuristic(e.s2);
                    node->backp = s;
                    node->open = true;
                    open.push(node);

                    if (already_created) {
                        // Remove node from OPEN
                        nodes[*e.s2]->donezo = true;
                        nodes.erase(*e.s2);
                    }

                    nodes.insert({*e.s2, node});
                }
            }
        }
    }

    return false;
}

std::vector<Waypoint> AraStar::get_path()
{
    std::vector<Waypoint> result;

    if (goal_node_ == nullptr) {
        return result;
    }

    std::shared_ptr<const Node> curr = goal_node_;
    while (curr != nullptr) {
        Waypoint w = env_->get_waypoint(curr->state);
        result.push_back(w);
        curr = curr->backp;
    }

    std::reverse(result.begin(), result.end());
    return result;
}

} // namespace uav_target_planner
