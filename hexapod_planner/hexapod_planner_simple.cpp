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

std::vector<std::string> HexapodPlannerSimple::plan(const ActionSimple& start)
{
    assert(_stop_iter > 0);
    assert(_actions.size() > 0);

    // random shuffling of the actions
    std::random_shuffle(_actions.begin(), _actions.end());

    std::vector<std::shared_ptr<HexapodPlannerSimple::ActionSimple>> frontier;
    std::vector<std::shared_ptr<HexapodPlannerSimple::ActionSimple>> visited;

    frontier.push_back(std::make_shared<HexapodPlannerSimple::ActionSimple>(start));
    frontier.back()->distance = _dist_from_goal(start);
    frontier.back()->cost = 0.0;
    frontier.back()->parent = nullptr;

    std::shared_ptr<HexapodPlannerSimple::ActionSimple> curr_best = nullptr;

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
            auto tmp_state = std::make_shared<HexapodPlannerSimple::ActionSimple>(_actions[j]);
            tmp_state->x += curr_best->x;
            tmp_state->y += curr_best->y;
            // TO-DO: check if this angle is correct
            tmp_state->theta += curr_best->theta;
            tmp_state->theta = std::atan2(std::sin(tmp_state->theta), std::cos(tmp_state->theta));
            tmp_state->distance = _dist_from_goal(*tmp_state);
            tmp_state->cost = curr_best->cost+tmp_state->cost_from(*curr_best);
            tmp_state->parent = curr_best;

            // check if state is revisited before adding it to frontier
            if (std::find_if(visited.begin(), visited.end(), ActionSimplePointerEqual(tmp_state)) == visited.end())
                frontier.push_back(tmp_state);
        }
    }
    return _trajectory(curr_best);
}

double HexapodPlannerSimple::_dist_from_goal(const ActionSimple& action)
{
    double diff_x = action.x - _goal.x;
    double diff_y = action.y - _goal.y;
    // maybe could use theta too?
    // distance squared is enough
    return diff_x * diff_x + diff_y * diff_y;
}

std::vector<std::string> HexapodPlannerSimple::_trajectory(const std::shared_ptr<HexapodPlannerSimple::ActionSimple>& end)
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