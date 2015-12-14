#ifndef HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP
#define HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP

#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include <algorithm>
#include <limits>

template <typename StateSimple, typename RobotDesc, typename EnvironmentDesc>
class HexapodPlannerSimple {
public:
    HexapodPlannerSimple() {}
    HexapodPlannerSimple(const std::vector<StateSimple> actions, const StateSimple& goal, const EnvironmentDesc& environment, size_t stop_iter = 2000) : _actions(actions),
                                                                                                                                                         _goal(goal),
                                                                                                                                                         _stop_iter(stop_iter),
                                                                                                                                                         _environment(std::make_shared<EnvironmentDesc>(environment))
    {
    }

    void add_action(const StateSimple& action)
    {
        _actions.push_back(action);
    }

    std::vector<StateSimple> actions()
    {
        return _actions;
    }

    void set_goal(const StateSimple& goal)
    {
        _goal = goal;
    }
    StateSimple goal()
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

    void expand_states(const StateSimple& start)
    {
        assert(_stop_iter > 0);
        assert(_actions.size() > 0);

        _states.clear();

        std::vector<std::shared_ptr<StateSimple>> frontier;
        std::shared_ptr<StateSimple> curr = nullptr;

        frontier.push_back(std::make_shared<StateSimple>(start));
        frontier.back()->f_score = start - _goal;
        frontier.back()->g_score = 0.0;
        frontier.back()->parent = nullptr;
        frontier.back()->children.clear();

        for (size_t i = 0; i < _stop_iter; i++) {
            if (frontier.size() == 0) {
                break;
            }

            frontier.erase(std::unique(frontier.begin(), frontier.end(), [](std::shared_ptr<StateSimple> l, std::shared_ptr<StateSimple> r) { return *l == *r; }), frontier.end());

            // get one state
            curr = frontier.front();
            frontier.erase(frontier.begin());

            if (std::find_if(_states.begin(), _states.end(), StateSimplePointerEqual(curr)) != _states.end()) {
                continue;
            }

            // record state
            _states.push_back(curr);
            curr->children.clear();

            // expand using available actions
            for (size_t j = 0; j < _actions.size(); j++) {
                auto tmp_state = std::make_shared<StateSimple>(*curr + _actions[j]);
                tmp_state->f_score = std::numeric_limits<double>::infinity();
                tmp_state->g_score = std::numeric_limits<double>::infinity();
                auto it = std::find_if(_states.begin(), _states.end(), StateSimplePointerEqual(tmp_state));
                if (it != _states.end())
                    tmp_state = *it;
                if (std::find_if(curr->children.begin(), curr->children.end(), StateSimplePointerEqual(tmp_state)) == curr->children.end()) {
                    curr->children.push_back(tmp_state);
                }
                else
                    continue;

                RobotDesc robot;
                robot.constructFromAction(*tmp_state);
                // check if state is revisited before adding it to frontier or if it collides with the environment
                if (std::find_if(frontier.begin(), frontier.end(), StateSimplePointerEqual(tmp_state)) == frontier.end() && !_environment->collides(robot.bounding)) {
                    frontier.push_back(tmp_state);
                }
            }
        }

        _states.erase(std::unique(_states.begin(), _states.end(), [](std::shared_ptr<StateSimple> l, std::shared_ptr<StateSimple> r) { return *l == *r; }), _states.end());
    }

    std::vector<StateSimple> plan(const StateSimple& start, bool re_expand = false)
    {
        assert(_stop_iter > 0);
        assert(_actions.size() > 0);

        if (_states.size() == 0 || re_expand)
            expand_states(start);

        assert(_states.size() > 0);

        std::cout << "Expanded " << _states.size() << " states!" << std::endl;

        std::vector<std::shared_ptr<StateSimple>> closed_set, open_set;
        std::shared_ptr<StateSimple> curr_best = nullptr;

        if (std::find_if(_states.begin(), _states.end(), StateSimplePointerEqual(std::make_shared<StateSimple>(_goal))) == _states.end()) {
            std::cout << "Goal cannot be reached... Updating it to nearest state.." << std::endl;
            _update_goal();
            std::cout << "New goal: " << _goal << std::endl;
        }

        open_set.push_back(_states[0]);

        while (open_set.size() > 0) {
            std::sort(open_set.begin(), open_set.end(), StateSimplePointerCompare());
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
                if (std::find_if(closed_set.begin(), closed_set.end(), StateSimplePointerEqual(neighbor)) != closed_set.end())
                    continue;

                double tentative_cost = curr_best->g_score + neighbor->cost_from(*curr_best);
                if (std::find_if(open_set.begin(), open_set.end(), StateSimplePointerEqual(neighbor)) == open_set.end())
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
    struct StateSimplePointerCompare {
        bool operator()(const std::shared_ptr<StateSimple>& l, const std::shared_ptr<StateSimple>& r)
        {
            return *l < *r;
        }
    };

    struct StateSimplePointerEqual {
        StateSimplePointerEqual(const std::shared_ptr<StateSimple>& ptr)
        {
            _ptr = ptr;
        }
        bool operator()(const std::shared_ptr<StateSimple>& l)
        {
            return *l == *_ptr;
        }

        std::shared_ptr<StateSimple> _ptr;
    };

    std::vector<StateSimple> _trajectory(const std::shared_ptr<StateSimple>& end)
    {
        std::vector<StateSimple> traj;
        auto tmp = end;
        while (tmp != nullptr) {
            auto a = *tmp;
            if (a.parent == nullptr)
                break;
            traj.push_back(a);
            tmp = tmp->parent;
        }
        std::reverse(traj.begin(), traj.end());
        return traj;
    }

    void _update_goal()
    {
        assert(_states.size() > 0);
        std::vector<std::shared_ptr<StateSimple>> states = _states;
        for (size_t i = 0; i < _states.size(); i++)
            states[i]->f_score = *_states[i] - _goal;
        std::sort(states.begin(), states.end(), StateSimplePointerCompare());
        _goal = *states.front();
        for (size_t i = 0; i < states.size(); i++)
            states[i]->f_score = std::numeric_limits<double>::infinity();
        _states.front()->f_score = 0.0;
    }

    std::vector<StateSimple> _actions;
    std::vector<std::shared_ptr<StateSimple>> _states;
    StateSimple _goal;
    size_t _stop_iter;
    std::shared_ptr<EnvironmentDesc> _environment;
    static constexpr double _distance_tolerance = 0.2;
};

#endif
