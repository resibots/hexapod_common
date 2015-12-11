#ifndef HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP
#define HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP

#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include <algorithm>

template <typename ActionSimple>
class HexapodPlannerSimple {
public:
    HexapodPlannerSimple() {}
    HexapodPlannerSimple(const std::vector<ActionSimple> actions, const ActionSimple& goal, size_t stop_iter = 100) : _actions(actions), _goal(goal), _stop_iter(stop_iter) {}

    void add_action(const ActionSimple& action)
    {
        _actions.push_back(action);
    }

    std::vector<ActionSimple> actions()
    {
        return _actions;
    }

    void set_goal(const ActionSimple& goal)
    {
        _goal = goal;
    }
    ActionSimple goal()
    {
        return _goal;
    }

    void set_stop_iter(size_t stop_iter)
    {
        _stop_iter = stop_iter;
    }
    size_t stop_iter()
    {
        return _stop_iter;
    }

    std::vector<std::string> plan(const ActionSimple& start)
    {
        assert(_stop_iter > 0);
        assert(_actions.size() > 0);

        // random shuffling of the actions
        std::random_shuffle(_actions.begin(), _actions.end());

        std::vector<std::shared_ptr<ActionSimple>> frontier;
        std::vector<std::shared_ptr<ActionSimple>> visited;

        frontier.push_back(std::make_shared<ActionSimple>(start));
        frontier.back()->distance = start - _goal;
        frontier.back()->cost = 0.0;
        frontier.back()->parent = nullptr;

        std::shared_ptr<ActionSimple> curr_best = nullptr;

        for (size_t i = 0; i < _stop_iter; i++) {
            if (frontier.size() == 0) {
                break;
            }

            std::sort(frontier.begin(), frontier.end(), ActionSimplePointerCompare());

            // get best state
            curr_best = frontier.front();
            frontier.erase(frontier.begin());

            // if reached goal or arrived to a state we shouldn't
            if (*curr_best == _goal || std::find_if(visited.begin(), visited.end(), ActionSimplePointerEqual(curr_best)) != visited.end()) {
                break;
            }

            // mark state as visited
            visited.push_back(curr_best);

            // expand frontier using available actions
            for (size_t j = 0; j < _actions.size(); j++) {
                auto tmp_state = std::make_shared<ActionSimple>(_actions[j] + *curr_best);
                tmp_state->distance = *tmp_state - _goal;
                tmp_state->parent = curr_best;

                // check if state is revisited before adding it to frontier
                if (std::find_if(visited.begin(), visited.end(), ActionSimplePointerEqual(tmp_state)) == visited.end())
                    frontier.push_back(tmp_state);
            }
        }
        return _trajectory(curr_best);
    }

protected:
    struct ActionSimplePointerCompare {
        bool operator()(const std::shared_ptr<ActionSimple>& l, const std::shared_ptr<ActionSimple>& r)
        {
            return *l < *r;
        }
    };

    struct ActionSimplePointerEqual {
        ActionSimplePointerEqual(const std::shared_ptr<ActionSimple>& ptr)
        {
            _ptr = ptr;
        }
        bool operator()(const std::shared_ptr<ActionSimple>& l)
        {
            return *l == *_ptr;
        }

        std::shared_ptr<ActionSimple> _ptr;
    };

    std::vector<std::string> _trajectory(const std::shared_ptr<ActionSimple>& end)
    {
        std::vector<std::string> traj;
        auto tmp = end;
        while (tmp != nullptr) {
            auto a = *tmp;
            if (a.parent == nullptr)
                break;
            traj.push_back(a.id);
            tmp = tmp->parent;
        }
        std::reverse(traj.begin(), traj.end());
        return traj;
    }

    std::vector<ActionSimple> _actions;
    ActionSimple _goal;
    size_t _stop_iter;
};

#endif