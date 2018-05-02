#include <algorithm>
#include <chrono>
#include <memory>

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
      start_state_(start),
      open_(comp_)
{
    std::shared_ptr<Node> start_node (new Node());
    start_node->state = start_state_;
    start_node->g = 0;
    start_node->f = eps_ * env_->get_heuristic(start_node->state);
    start_node->open = true;
    start_node->closed = 0;
    open_.push(std::move(start_node));
    nodes.insert({*start_state_, start_node});
}

bool AraStar::search()
{
    ROS_WARN("START");
    env_->logger.publish_start(start_state_, env_);

    t_start_ = std::chrono::high_resolution_clock::now();
    t_stop_ = t_start_ + std::chrono::milliseconds(300);

    double eps = eps_;
    bool result = false;
    int round = 1;
    while (eps > 1.0) {
        eps -= 0.3;
        if (eps < 1.0) eps = 1.0;

        result = replan(eps, round) || result;

        if (std::chrono::high_resolution_clock::now() > t_stop_) break;
        round++;
    }

    if (result) {
        env_->logger.publish_goal(goal_node_->state, env_);
    }

    return result;
}

bool AraStar::replan(double eps, int round)
{

    {
        std::priority_queue<std::shared_ptr<Node>,
                            std::vector<std::shared_ptr<Node>>,
                            decltype(comp_)>
            new_open (comp_);
        while (!open_.empty()) {
            std::shared_ptr<Node> s = open_.top();
            open_.pop();
            if (s->donezo) continue;
            s->f = s->g + eps * env_->get_heuristic(s->state);
            new_open.push(s);
        }

        for (auto& p : incons) {
            std::shared_ptr<Node> s = p.second;
            s->f = s->g + eps * env_->get_heuristic(s->state);
            new_open.push(s);
        }
        incons.clear();

        open_.swap(new_open);
    }

    int expansions = 0;
    while (open_.size() != 0) {
        if (std::chrono::high_resolution_clock::now() > t_stop_) {
            ROS_WARN_STREAM("Expansions: " << expansions);
            ROS_ERROR("OUT OF TIME");
            return false;
        }

        if (goal_node_ != nullptr && goal_node_->g <= open_.top()->f) {
            // Done this round
            ROS_WARN_STREAM("Expansions: " << expansions);
            ROS_WARN("DONE WITH EPS=%f", eps);
            return true;
        }

        std::shared_ptr<Node> s = open_.top();
        open_.pop();
        if (s->donezo) continue;

        s->open = false;
        s->closed = round;
        s->v = s->g;

        expansions++;
        env_->logger.publish_state(s->state, env_);

        if (!std::isfinite(s->f)) {
            // Goal not reachable from here
            throw std::runtime_error("BAD");
            continue;
        }

        if (env_->is_goal(s->state) && (goal_node_ == nullptr || goal_node_->g > s->g)) {
            ROS_WARN("GOAL");
            const auto t_end = std::chrono::high_resolution_clock::now();
            ROS_WARN_STREAM("Planning time: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start_).count()
                    << "ms");
            ROS_WARN_STREAM("Expansions: " << expansions);
            ROS_WARN_STREAM("Cost (s): " << s->g);
            // Found goal!
            goal_node_ = s;
            return true;
        }

        for (auto& e : env_->get_successors(s->state)) {
            bool already_created = (nodes.count(*e.s2) != 0);
            double new_g = s->g + e.cost;
            if (!already_created || nodes[*e.s2]->closed < round) {
                if (!already_created || new_g < nodes[*e.s2]->g) {
                    std::shared_ptr<Node> node (new Node());
                    node->state = e.s2;
                    node->g = new_g;
                    node->f = new_g + eps * env_->get_heuristic(e.s2);
                    node->backp = s;
                    node->open = true;
                    node->closed = 0;
                    open_.push(node);

                    if (already_created) {
                        // Remove node from OPEN
                        nodes[*e.s2]->donezo = true;
                        nodes.erase(*e.s2);
                    }

                    nodes.insert({*e.s2, node});
                }
            } else {
                // already created and closed
                nodes[*e.s2]->g = new_g;
                if (incons.count(*e.s2) == 0) {
                    incons.insert({*e.s2, nodes[*e.s2]});
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
