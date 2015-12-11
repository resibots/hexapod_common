#ifndef HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP
#define HEXAPOD_PLANNER_HEXAPOD_PLANNER_SIMPLE_HPP

#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>

class HexapodPlannerSimple {
public:
    struct ActionSimple {
        double x, y, theta, distance, cost;
        std::string id;

        std::shared_ptr<ActionSimple> parent;

        bool operator<(const ActionSimple& other) const
        {
            double d1 = cost+distance;
            double d2 = cost+other.distance;
            return d1 < d2;
        }

        bool operator==(const ActionSimple& other) const
        {
            double dx = x - other.x;
            double dy = y - other.y;
            double dth = theta - other.theta;
            dth = std::atan2(std::sin(dth), std::cos(dth));
            double d = dx * dx + dy * dy + dth*dth;
            return (d < 1e-2);
        }

        double cost_from(const ActionSimple& other) const
        {
            // simple cost
            return 1.0;
        }
    };

    HexapodPlannerSimple() {}
    HexapodPlannerSimple(const std::vector<ActionSimple> actions, const ActionSimple& goal, size_t stop_iter = 100);

    void add_action(const ActionSimple& action);
    std::vector<ActionSimple> actions();

    void set_goal(const ActionSimple& goal);
    ActionSimple goal();

    void set_stop_iter(size_t stop_iter);
    size_t stop_iter();

    std::vector<std::string> plan(const ActionSimple& start);

protected:
    double _dist_from_goal(const ActionSimple& action);
    std::vector<std::string> _trajectory(const std::shared_ptr<ActionSimple>& end);
    std::vector<ActionSimple> _actions;
    ActionSimple _goal;
    size_t _stop_iter;
};

struct ActionSimplePointerCompare {
    bool operator()(const std::shared_ptr<HexapodPlannerSimple::ActionSimple>& l, const std::shared_ptr<HexapodPlannerSimple::ActionSimple>& r)
    {
        return *l < *r;
    }
};

struct ActionSimplePointerEqual {
    ActionSimplePointerEqual(const std::shared_ptr<HexapodPlannerSimple::ActionSimple>& ptr)
    {
        _ptr = ptr;
    }
    bool operator()(const std::shared_ptr<HexapodPlannerSimple::ActionSimple>& l)
    {
        return *l == *_ptr;
    }

    std::shared_ptr<HexapodPlannerSimple::ActionSimple> _ptr;
};

#endif