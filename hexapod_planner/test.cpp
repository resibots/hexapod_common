#include <iostream>
#include <hexapod_planner_simple.hpp>

typedef HexapodPlannerSimple::ActionSimple ActionSimple;

int main()
{
    std::vector<ActionSimple> actions;
    ActionSimple u, d, l, r;
    u.x = 0;
    u.y = 1;
    u.theta = 0;
    d.x = 0;
    d.y = -1;
    d.theta = 0;
    l.x = -1;
    l.y = 0;
    l.theta = 0;
    r.x = 1;
    r.y = 0;
    r.theta = 0;
    actions.push_back(u);
    actions.push_back(d);
    actions.push_back(l);
    actions.push_back(r);
    // ActionSimple diag1, diag2;
    // diag1.x = 1; diag1.y = 1; diag1.theta = 0;
    // diag2.x = -1; diag2.y = -1; diag2.theta = 0;
    // actions.push_back(diag1);
    // actions.push_back(diag2);

    ActionSimple goal;
    goal.x = 5;
    goal.y = 5;
    goal.theta = 0;

    ActionSimple start;
    start.x = 0;
    start.y = 0.25;
    start.theta = 0;

    HexapodPlannerSimple planner(actions, goal);

    auto traj = planner.plan(start);

    for (auto a : traj) {
        std::cout << a.x << " " << a.y << std::endl;
    }

    return 0;
}