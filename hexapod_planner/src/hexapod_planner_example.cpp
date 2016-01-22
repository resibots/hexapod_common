#include <iostream>
#include <hexapod_planner/hexapod_planner_simple.hpp>
#include <hexapod_planner/state_simple.hpp>
#include <hexapod_planner/environment_simple.hpp>
#include <hexapod_planner/graph_simple.hpp>
#include <map>

#ifdef GRAPHIC
#include <SFML/Graphics.hpp>
#endif

using namespace hexapod_planner;

#define WIDTH 640
#define HEIGHT 480
#define ENTITY_SIZE 10
// Uncomment to use rotations - they are not displaying correctly
// #define USE_ROTATION
// Comment for hexapod data
#define SIMPLE_EXALPLE

#define OBSTACLES

void get_pos(double x, double y, double& out_x, double& out_y)
{
    out_x = x * ENTITY_SIZE + WIDTH / 2.0;
    out_y = -y * ENTITY_SIZE + HEIGHT / 2.0;
}

int main()
{
    std::vector<StateSimple> actions;
    std::map<std::string, StateSimple> acts;
#ifdef SIMPLE_EXALPLE
    StateSimple u, d, l, r, tcw, tccw;
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
    tcw.theta = -1.56;
    tcw.id = "turn-clockwise";
    tccw.x = 0;
    tccw.y = 0;
    tccw.theta = 1.56;
    tccw.id = "turn-counter-clockwise";
    actions.push_back(u);
    actions.push_back(d);
    actions.push_back(l);
    actions.push_back(r);
    actions.push_back(tcw);
    actions.push_back(tccw);
    // StateSimple diag1, diag2;
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
    acts["up"] = u;
    acts["down"] = d;
    acts["left"] = l;
    acts["right"] = r;
    acts["turn-clockwise"] = tcw;
    acts["turn-counter-clockwise"] = tccw;
#else
    StateSimple t;
    t.id = "fwd";
    t.x = 0.62;
    t.y = 0;
    t.theta = -10.87 * M_PI / 180.0;
    actions.push_back(t);
    acts["fwd"] = t;
    t.id = "back";
    t.x = -0.6;
    t.y = 0.07;
    t.theta = 0.03 * M_PI / 180.0;
    actions.push_back(t);
    acts["back"] = t;
    t.id = "turn-cw";
    t.x = 0.0;
    t.y = 0.02;
    t.theta = -83.01 * M_PI / 180.0;
    actions.push_back(t);
    acts["turn-cw"] = t;
    t.id = "turn-ccw";
    t.x = -0.02;
    t.y = 0.03;
    t.theta = 89.05 * M_PI / 180.0;
    actions.push_back(t);
    acts["turn-ccw"] = t;
#endif

    StateSimple goal;
#ifndef SIMPLE_EXALPLE
    goal.x = 1;
    goal.y = 2;
    goal.theta = 0;
#else
    goal.x = 5;
    goal.y = 5;
    goal.theta = -1.56;
#endif
    goal.transformation.setIdentity();
    Eigen::Vector2d vec = {goal.x, goal.y};
    goal.transformation.translate(vec);
    goal.transformation.rotate(goal.theta);
    goal.theta = std::atan2(goal.transformation.rotation()(1, 0), goal.transformation.rotation()(0, 0));

    std::cout << "goal: " << goal << std::endl;

    StateSimple start;
    start.x = 0;
    start.y = 0;
    start.theta = 0;
    start.transformation.setIdentity();
    vec = {start.x, start.y};
    start.transformation.translate(vec);
    start.transformation.rotate(start.theta);
    start.theta = std::atan2(start.transformation.rotation()(1, 0), start.transformation.rotation()(0, 0));

    std::cout << "start: " << start << std::endl;

    std::shared_ptr<EnvironmentSimple<ObstacleSimple>> env = std::make_shared<EnvironmentSimple<ObstacleSimple>>();
#ifdef OBSTACLES
    ObstacleSimple obs1;
#ifndef SIMPLE_EXALPLE
    obs1.bounding.x = 2;
    obs1.bounding.y = 0;
#else
    obs1.bounding.x = 2;
    obs1.bounding.y = 2;
#endif
    obs1.bounding.radius = 1;
    env->add_obstacle(obs1);

    ObstacleSimple obs2;
    obs2.bounding.x = -2;
    obs2.bounding.y = 2;
    obs2.bounding.radius = 1;
    env->add_obstacle(obs2);
#endif

    auto graph = std::make_shared<GraphSimple<StateSimple, RobotSimple, EnvironmentSimple<ObstacleSimple>>>();
    std::string graph_file = "example_graphs/test";
#ifndef SIMPLE_EXALPLE
    graph_file = "example_graphs/test_robot";
#endif

#ifndef OBSTACLES
    graph_file += "_no_obs";
#endif

    graph_file += ".dat";

    // graph->expand_nodes(env, actions, start, 5000);
    // graph->save_to_file(graph_file);
    graph->load_from_file(graph_file);
    std::cout << "Graph loaded: " << graph->_nodes.size() << "\n";

    HexapodPlannerSimple<StateSimple, GraphSimple<StateSimple, RobotSimple, EnvironmentSimple<ObstacleSimple>>> planner(graph);

    auto traj = planner.plan(start, goal);

    for (auto a : traj) {
        std::cout << a.id << " : " << a << std::endl;
    }

#ifdef GRAPHIC
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Simple Action Planner");
    std::vector<sf::CircleShape> obstacles;
    double x, y;

    for (auto ob : env->obstacles()) {
        sf::CircleShape obs(ENTITY_SIZE * ob->bounding.radius);
        obs.setFillColor(sf::Color::Red);
        obs.setOrigin(ENTITY_SIZE / 2.0, ENTITY_SIZE / 2.0);
        get_pos(ob->bounding.x, ob->bounding.y, x, y);
        obs.setPosition(x, y);
        obstacles.push_back(obs);
    }

    sf::Transform robot_tf = sf::Transform::Identity, target_tf = sf::Transform::Identity;

    sf::CircleShape target(ENTITY_SIZE, 3);
    target.setFillColor(sf::Color::Green);
    target.setOrigin(ENTITY_SIZE / 2.0, ENTITY_SIZE / 2.0);
    get_pos(goal.x, goal.y, x, y);
    target_tf.translate(x, y);
#ifdef USE_ROTATION
    target_tf.rotate(-goal.theta * 57.2958);
#endif

    sf::CircleShape robot(ENTITY_SIZE, 3);
    robot.setFillColor(sf::Color::Blue);
    robot.setOrigin(ENTITY_SIZE / 2.0, ENTITY_SIZE / 2.0);
    get_pos(start.y, start.y, x, y);
    robot_tf.translate(x, y);
#ifdef USE_ROTATION
    robot_tf.rotate(-start.theta * 57.2958);
#endif

    double curr_x = start.x, curr_y = start.y;
    double prev_x, prev_y;
    get_pos(start.y, start.y, prev_x, prev_y);

    double curr_th = start.theta;
    double prev_th = curr_th;

    size_t i = 0;
    bool first = true;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);
        for (auto obs : obstacles)
            window.draw(obs);
        window.draw(target, target_tf);
        window.draw(robot, robot_tf);
        window.display();
        if (first) {
            while (!sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
            }
            first = false;
        }

        if (i < traj.size()) {
            auto a = traj[i];
            double tmp = a.theta;
            tmp = std::atan2(std::sin(tmp - prev_th), std::cos(tmp - prev_th));
            curr_x = a.x;
            curr_y = a.y;
            get_pos(curr_x, curr_y, x, y);
            robot_tf.translate(x - prev_x, y - prev_y);
            prev_x = x;
            prev_y = y;
#ifdef USE_ROTATION
            robot_tf.rotate(-tmp * 57.2958);
#endif
            curr_th = a.theta;
            // std::cout<<curr_x<<" "<<curr_y<<" "<<curr_th<<std::endl;
            i++;
            sleep(sf::seconds(0.05));
        }
    }
#endif

    return 0;
}