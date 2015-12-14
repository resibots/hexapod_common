#ifndef HEXAPOD_PLANNER_ACTION_SIMPLE_HPP
#define HEXAPOD_PLANNER_ACTION_SIMPLE_HPP

#include <Eigen/Geometry>

struct ActionSimple {
    typedef Eigen::Transform<double, 2, Eigen::Affine> transform_t;
    double x, y, theta, distance, cost;
    std::string id;
    transform_t transformation;

    std::shared_ptr<ActionSimple> parent;

    double operator-(const ActionSimple& other) const
    {
        double diff_x = x - other.x;
        double diff_y = y - other.y;
        double diff_th = theta - other.theta;
        diff_th = std::atan2(std::sin(diff_th), std::cos(diff_th));
        return diff_x * diff_x + diff_y * diff_y + diff_th * diff_th;
    }

    ActionSimple operator+(const ActionSimple& other)
    {
        ActionSimple tmp_action = *this;
        tmp_action.id = other.id;
        tmp_action.cost = cost + tmp_action.cost_from(other);

        Eigen::Vector2d vec{other.x, other.y};
        tmp_action.transformation.translate(vec);
        tmp_action.transformation.rotate(other.theta);
        tmp_action.theta = std::atan2(tmp_action.transformation.rotation()(1, 0), tmp_action.transformation.rotation()(0, 0));
        tmp_action.x = tmp_action.transformation.translation()[0];
        tmp_action.y = tmp_action.transformation.translation()[1];
        // This will be overrided
        tmp_action.distance = distance + other.distance;
        // This would have been overrided
        // tmp_action->parent = std::make_shared<ActionSimple>(other);
        return tmp_action;
    }

    bool operator<(const ActionSimple& other) const
    {
        double d1 = cost + distance;
        double d2 = other.cost + other.distance;
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
