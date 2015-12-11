#ifndef HEXAPOD_PLANNER_ACTION_SIMPLE_HPP
#define HEXAPOD_PLANNER_ACTION_SIMPLE_HPP

struct ActionSimple {
    double x, y, theta, distance, cost;
    std::string id;

    std::shared_ptr<ActionSimple> parent;

    double operator-(const ActionSimple& other) const
    {
        double diff_x = x - other.x;
        double diff_y = y - other.y;
        // maybe could use theta too?
        return diff_x * diff_x + diff_y * diff_y;
    }

    ActionSimple operator+(const ActionSimple& other)
    {
        ActionSimple tmp_action = *this;
        tmp_action.x += other.x;
        tmp_action.y += other.y;
        // TO-DO: check if this angle is correct
        tmp_action.theta += other.theta;
        tmp_action.theta = std::atan2(std::sin(tmp_action.theta), std::cos(tmp_action.theta));
        // This would have been overrided
        // tmp_action.distance = distance + other.distance;
        tmp_action.cost = cost + tmp_action.cost_from(*this);
        // This would have been overrided
        // tmp_action->parent = std::make_shared<ActionSimple>(other);
        return tmp_action;
    }

    bool operator<(const ActionSimple& other) const
    {
        double d1 = cost + distance;
        double d2 = cost + other.distance;
        return d1 < d2;
    }

    bool operator==(const ActionSimple& other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        double dth = theta - other.theta;
        dth = std::atan2(std::sin(dth), std::cos(dth));
        double d = dx * dx + dy * dy + dth * dth;
        return (d < 1e-2);
    }

    double cost_from(const ActionSimple& other) const
    {
        // simple cost
        return 1.0;
    }
};

#endif
