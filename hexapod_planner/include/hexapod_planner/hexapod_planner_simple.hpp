#ifndef HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP
#define HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP

#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include <algorithm>
#include <limits>

namespace hexapod_planner {

    template <typename StateSimple, typename Graph>
    class HexapodPlannerSimple {
    public:
        HexapodPlannerSimple() {}
        HexapodPlannerSimple(const std::shared_ptr<Graph>& graph) : _graph(graph)
        {
        }

        StateSimple goal()
        {
            return _goal;
        }

        std::vector<StateSimple> plan(const StateSimple& start, const StateSimple& goal)
        {
            assert(_graph->_nodes.size() > 0);
            _goal = goal;

            std::vector<std::shared_ptr<StateSimple>> closed_set, open_set;
            std::shared_ptr<StateSimple> curr_best = nullptr;

            if (std::find_if(_graph->_nodes.begin(), _graph->_nodes.end(), typename StateSimple::PointerEqual(std::make_shared<StateSimple>(_goal))) == _graph->_nodes.end()) {
                std::cout << "Goal cannot be reached... Updating it to nearest state.." << std::endl;
                _update_goal();
                std::cout << "New goal: " << _goal << std::endl;
            }

            auto it = std::find_if(_graph->_nodes.begin(), _graph->_nodes.end(), typename StateSimple::PointerEqual(std::make_shared<StateSimple>(start)));

            if (it == _graph->_nodes.end()) {
                std::cout << "Start node not in graph!" << std::endl;
                return _trajectory(nullptr);
            }

            (*it)->f_score = *(*it) - _goal;
            (*it)->g_score = 0.0;
            open_set.push_back(*it);

            while (open_set.size() > 0) {
                std::sort(open_set.begin(), open_set.end(), typename StateSimple::PointerCompare());
                open_set.erase(std::unique(open_set.begin(), open_set.end(), [](std::shared_ptr<StateSimple> l, std::shared_ptr<StateSimple> r) { return *l == *r; }), open_set.end());

                // get best state
                curr_best = open_set.front();
                open_set.erase(open_set.begin());

                // if reached goal
                if (*curr_best == _goal || (*curr_best - _goal) < _distance_tolerance) {
                    std::cout << "Goal reached" << std::endl;
                    break;
                }

                closed_set.push_back(curr_best);

                for (size_t j = 0; j < curr_best->children.size(); j++) {
                    auto neighbor = curr_best->children[j];
                    if (std::find_if(closed_set.begin(), closed_set.end(), typename StateSimple::PointerEqual(neighbor)) != closed_set.end())
                        continue;

                    double tentative_cost = curr_best->g_score + neighbor->cost_from(*curr_best);
                    if (std::find_if(open_set.begin(), open_set.end(), typename StateSimple::PointerEqual(neighbor)) == open_set.end())
                        open_set.push_back(neighbor);
                    else if (tentative_cost >= neighbor->g_score)
                        continue;

                    neighbor->parent = curr_best;
                    neighbor->g_score = tentative_cost;
                    neighbor->f_score = tentative_cost + (*neighbor - _goal);
                }
            }

            // return best so far trajectory
            return _trajectory(curr_best);
        }

    protected:
        std::vector<StateSimple> _trajectory(const std::shared_ptr<StateSimple>& end)
        {
            std::vector<StateSimple> traj;
            auto tmp = end;
            while (tmp != nullptr) {
                auto a = *tmp;
                if (a.parent == nullptr)
                    break;
                auto it = std::find_if(a.parent->children.begin(), a.parent->children.end(), typename StateSimple::PointerEqual(tmp));
                if (it != a.parent->children.end())
                    a.id = a.parent->actions[std::distance(a.parent->children.begin(), it)];
                traj.push_back(a);
                tmp = tmp->parent;
            }
            std::reverse(traj.begin(), traj.end());
            return traj;
        }

        void _update_goal()
        {
            assert(_graph->_nodes.size() > 0);
            std::vector<std::shared_ptr<StateSimple>> states = _graph->_nodes;
            for (size_t i = 0; i < _graph->_nodes.size(); i++)
                states[i]->f_score = *_graph->_nodes[i] - _goal;
            std::sort(states.begin(), states.end(), typename StateSimple::PointerCompare());
            for (size_t i = 0; i < states.size(); i++)
                states[i]->f_score = std::numeric_limits<double>::infinity();
            _graph->_nodes.front()->f_score = 0.0;
            if (_goal - *states.front() < _distance_tolerance)
                return;
            _goal = *states.front();
        }

        StateSimple _goal;
        std::shared_ptr<Graph> _graph;
        static constexpr double _distance_tolerance = 0.01;
    };
}

#endif
