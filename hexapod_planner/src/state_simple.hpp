#ifndef HEXAPOD_PLANNER_STATE_SIMPLE_HPP
#define HEXAPOD_PLANNER_STATE_SIMPLE_HPP

#include <Eigen/Geometry>

struct StateSimple {
    typedef Eigen::Transform<double, 2, Eigen::Affine> transform_t;
    double x, y, theta, f_score, g_score;
    std::string id;
    transform_t transformation;

    std::vector<std::shared_ptr<StateSimple>> children;
    std::shared_ptr<StateSimple> parent;

    double operator-(const StateSimple& other) const
    {
        double diff_x = x - other.x;
        double diff_y = y - other.y;
        double diff_th = theta - other.theta;
        diff_th = std::atan2(std::sin(diff_th), std::cos(diff_th));
        return std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_th * diff_th);
    }

    StateSimple operator+(const StateSimple& other)
    {
        StateSimple tmp_action = *this;
        tmp_action.id = other.id;

        Eigen::Vector2d vec{other.x, other.y};
        tmp_action.transformation.translate(vec);
        tmp_action.transformation.rotate(other.theta);
        tmp_action.theta = std::atan2(tmp_action.transformation.rotation()(1, 0), tmp_action.transformation.rotation()(0, 0));
        tmp_action.x = tmp_action.transformation.translation()[0];
        tmp_action.y = tmp_action.transformation.translation()[1];
        return tmp_action;
    }

    bool operator<(const StateSimple& other) const
    {
        double d1 = f_score;
        double d2 = other.f_score;
        return d1 < d2;
    }

    bool operator==(const StateSimple& other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        double dth = theta - other.theta;
        dth = std::atan2(std::sin(dth), std::cos(dth));
        double d = dx * dx + dy * dy + dth * dth;
        return (d < 1e-2);
    }

    double cost_from(const StateSimple& other) const
    {
        // simple cost
        return 1.0;
    }
};

std::ostream& operator<<(std::ostream& os, const StateSimple& obj)
{
    os << obj.x << " " << obj.y << " " << obj.theta << " f: " << obj.f_score << " g: " << obj.g_score;
    return os;
}

#endif
