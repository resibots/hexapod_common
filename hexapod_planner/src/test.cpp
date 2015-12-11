#include <iostream>
#include <hexapod_planner_simple.hpp>
#include <action_simple.hpp>
#include <simple_env.hpp>
#include <SFML/Graphics.hpp>

#define WIDTH 640
#define HEIGHT 480
#define ENTITY_SIZE 10
// Uncomment to use rotations - they are not displaying correctly
// #define USE_ROTATION

void get_pos(double x, double y, double& out_x, double& out_y)
{
    out_x = x*ENTITY_SIZE+WIDTH/2.0;
    out_y = -y*ENTITY_SIZE+HEIGHT/2.0;
}

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

    ObstacleSimple obs2;
    obs2.bounding.x = -2;
    obs2.bounding.y = 2;
    obs2.bounding.radius = 0.5;
    env.add_obstacle(obs2);

    HexapodPlannerSimple<ActionSimple, RobotSimple, EnvironmentSimple<ObstacleSimple>> planner(actions, goal, env);

    auto traj = planner.plan(start);

    for (auto a : traj) {
        std::cout << a << std::endl;
    }

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Simple Action Planner");
    std::vector<sf::CircleShape> obstacles;
    double x,y;

    for (auto ob : env.obstacles())
    {
        sf::CircleShape obs(ENTITY_SIZE*ob->bounding.radius);
        obs.setFillColor(sf::Color::Red);
        obs.setOrigin(ENTITY_SIZE/2.0, ENTITY_SIZE/2.0);
        get_pos(ob->bounding.x, ob->bounding.y, x, y);
        obs.setPosition(x, y);
        obstacles.push_back(obs);
    }

#ifndef USE_ROTATION
    sf::CircleShape target(ENTITY_SIZE);
#else
    sf::CircleShape target(ENTITY_SIZE, 3);
#endif
    target.setFillColor(sf::Color::Green);
    target.setOrigin(ENTITY_SIZE/2.0, ENTITY_SIZE/2.0);
    get_pos(goal.x, goal.y, x, y);
    target.setPosition(x, y);
#ifdef USE_ROTATION
    target.rotate(goal.theta*57.2958);
#endif

#ifndef USE_ROTATION
    sf::CircleShape robot(ENTITY_SIZE);
#else
    sf::CircleShape robot(ENTITY_SIZE, 3);
#endif
    robot.setFillColor(sf::Color::Blue);
    robot.setOrigin(ENTITY_SIZE/2.0, ENTITY_SIZE/2.0);
    get_pos(start.y, start.y, x, y);
    robot.setPosition(x, y);
#ifdef USE_ROTATION
    robot.rotate(start.theta*57.2958);
#endif

    double curr_x = start.x, curr_y = start.y;
#ifdef USE_ROTATION
    double curr_th = start.theta;
#endif
    int i = 0;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);
        for(auto obs : obstacles)
            window.draw(obs);
        window.draw(target);
        window.draw(robot);
        window.display();
        while(!sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {}
        sleep(sf::seconds(0.2));
        if(i<traj.size())
        {
            auto a = traj[i];
            if(a == "up")
            {
                curr_x += u.x;
                curr_y += u.y;
                get_pos(curr_x, curr_y, x, y);
                robot.setPosition(x,y);
            }
            else if(a == "down")
            {
                curr_x += d.x;
                curr_y += d.y;
                get_pos(curr_x, curr_y, x, y);
                robot.setPosition(x,y);
            }
            else if(a == "right")
            {
                curr_x += r.x;
                curr_y += r.y;
                get_pos(curr_x, curr_y, x, y);
                robot.setPosition(x,y);
            }
            else if(a == "left")
            {
                curr_x += l.x;
                curr_y += l.y;
                get_pos(curr_x, curr_y, x, y);
                robot.setPosition(x,y);
            }
#ifdef USE_ROTATION
            else if(a == "turn clockwise")
            {
                double tmp = curr_th + tcw.theta;
                curr_th = std::atan2(std::sin(curr_th-tmp), std::cos(curr_th-tmp));
                robot.rotate(-curr_th*57.2958);
            }
            else if(a == "turn counter-clockwise")
            {
                double tmp = curr_th + tccw.theta;
                curr_th = std::atan2(std::sin(curr_th-tmp), std::cos(curr_th-tmp));
                robot.rotate(-curr_th*57.2958);
            }
#endif
            i++;
        }
    }


    return 0;
}