#include <iostream>
#include <hexapod_planner_simple.hpp>
#include <action_simple.hpp>
#include <simple_env.hpp>

int main()
{
    std::vector<ActionSimple> actions;
    ActionSimple u, d, l, r, tcw, tccw;
    u.x = 0;
    u.y = 1;
    u.theta = 0;
    u.id = "up";
    d.x = 0;
    d.y = -1;
    d.theta = 0;
    d.id = "down";
    l.x = -1;
    l.y = 0;
    l.theta = 0;
    l.id = "left";
    r.x = 1;
    r.y = 0;
    r.theta = 0;
    r.id = "right";
    tcw.x = 0;
    tcw.y = 0;
    tcw.theta = 1.56;
    tcw.id = "turn clockwise";
    tccw.x = 0;
    tccw.y = 0;
    tccw.theta = -1.56;
    tccw.id = "turn counter-clockwise";
    actions.push_back(u);
    actions.push_back(d);
    actions.push_back(l);
    actions.push_back(r);
    actions.push_back(tcw);
    actions.push_back(tccw);
    // ActionSimple diag1, diag2;
    // diag1.x = 1;
    // diag1.y = 1;
    // diag1.theta = 0;
    // diag1.id = "diagonal up right";
    // diag2.x = -1;
    // diag2.y = -1;
    // diag2.theta = 0;
    // diag2.id = "diagonal down left";
    // actions.push_back(diag1);
    // actions.push_back(diag2);

    ActionSimple goal;
    goal.x = 5;
    goal.y = 5;
    goal.theta = 1.56;

    ActionSimple start;
    start.x = 0;
    start.y = -0.25;
    start.theta = 0;

    EnvironmentSimple<ObstacleSimple> env;
    ObstacleSimple obs1;
    obs1.bounding.x = 2;
    obs1.bounding.y = 2;
    obs1.bounding.radius = 1;
    env.add_obstacle(obs1);

    HexapodPlannerSimple<ActionSimple, RobotSimple, EnvironmentSimple<ObstacleSimple>> planner(actions, goal, env);

    auto traj = planner.plan(start);

    for (auto a : traj) {
        std::cout << a << std::endl;
    }

    return 0;
}