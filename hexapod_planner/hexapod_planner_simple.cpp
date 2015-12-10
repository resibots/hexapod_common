#include <hexapod_planner_simple.hpp>
#include <algorithm>

HexapodPlannerSimple::HexapodPlannerSimple(const std::vector<HexapodPlannerSimple::ActionSimple> actions, const ActionSimple& goal, size_t stop_iter) : _actions(actions), _goal(goal), _stop_iter(stop_iter)
{
}

void HexapodPlannerSimple::add_action(const HexapodPlannerSimple::ActionSimple& action)
{
    _actions.push_back(action);
}

std::vector<HexapodPlannerSimple::ActionSimple> HexapodPlannerSimple::actions()
{
    return _actions;
}

void HexapodPlannerSimple::set_goal(const HexapodPlannerSimple::ActionSimple& goal)
{
    _goal = goal;
}

HexapodPlannerSimple::ActionSimple HexapodPlannerSimple::goal()
{
    return _goal;
}

void HexapodPlannerSimple::set_stop_iter(size_t stop_iter)
{
    _stop_iter = stop_iter;
}

size_t HexapodPlannerSimple::stop_iter()
{
    return _stop_iter;
}

std::vector<HexapodPlannerSimple::ActionSimple> HexapodPlannerSimple::plan(const ActionSimple& start)
{
    assert(_stop_iter > 0);
    assert(_actions.size() > 0);

    // random shuffling of the actions
    std::random_shuffle(_actions.begin(), _actions.end());

    std::vector<std::shared_ptr<HexapodPlannerSimple::ActionSimple>> frontier;
    frontier.push_back(std::make_shared<HexapodPlannerSimple::ActionSimple>(start));
    frontier.back()->distance = _dist_from_goal(start);
    frontier.back()->parent = nullptr;

    std::shared_ptr<HexapodPlannerSimple::ActionSimple> curr_best = nullptr, prev_best = nullptr;

    std::vector<std::shared_ptr<HexapodPlannerSimple::ActionSimple>> visited;

    for (size_t i = 0; i < _stop_iter; i++) {
        if (frontier.size() == 0) {
            break;
        }

        std::sort(frontier.begin(), frontier.end(), ActionSimplePointerCompare());

        // get best state
        curr_best = frontier.front();
        frontier.erase(frontier.begin());

        if (_dist_from_goal(*curr_best) < 1e-2) {
            return _trajectory(curr_best);
        }

        // expand frontier using available actions
        for (size_t j = 0; j < _actions.size(); j++) {
            auto tmp_state = std::make_shared<HexapodPlannerSimple::ActionSimple>(_actions[j]);
            tmp_state->x += curr_best->x;
            tmp_state->y += curr_best->y;
            // TO-DO: angle needs fixing
            tmp_state->theta += curr_best->theta;
            tmp_state->distance = _dist_from_goal(*tmp_state);
            tmp_state->parent = curr_best;

            // check if state is revisited before adding it to frontier
            if (std::find_if(visited.begin(), visited.end(), ActionSimplePointerEqual(tmp_state)) == visited.end())
                frontier.push_back(tmp_state);
        }

        prev_best = curr_best;
    }
    return _trajectory(curr_best);
}

double HexapodPlannerSimple::_dist_from_goal(const ActionSimple& action)
{
    double diff_x = action.x - _goal.x;
    double diff_y = action.y - _goal.y;
    // distance squared is enough
    return diff_x * diff_x + diff_y * diff_y;
}

std::vector<HexapodPlannerSimple::ActionSimple> HexapodPlannerSimple::_trajectory(const std::shared_ptr<HexapodPlannerSimple::ActionSimple>& end)
{
    std::vector<HexapodPlannerSimple::ActionSimple> traj;
    auto tmp = end;
    while (tmp != nullptr) {
        auto a = *tmp;
        if (a.parent) {
            a.x -= a.parent->x;
            a.y -= a.parent->y;
            a.theta -= a.parent->theta;
        }
        else
            break;
        traj.push_back(a);
        tmp = tmp->parent;
    }
    std::reverse(traj.begin(), traj.end());
    return traj;
}