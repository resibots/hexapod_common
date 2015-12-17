#ifndef HEXAPOD_PLANNER_STATE_SIMPLE_HPP
#define HEXAPOD_PLANNER_STATE_SIMPLE_HPP

#include <iostream>
#include <string>
#include <algorithm>
#include <Eigen/Geometry>

namespace hexapod_planner {

    struct StateSimple {
        typedef Eigen::Transform<double, 2, Eigen::Affine> transform_t;
        std::string id;
        double x, y, theta, f_score, g_score;
        transform_t transformation;

        std::vector<std::shared_ptr<StateSimple>> children;
        std::vector<std::string> actions;
        std::shared_ptr<StateSimple> parent;

        struct PointerCompare {
            bool operator()(const std::shared_ptr<StateSimple>& l, const std::shared_ptr<StateSimple>& r)
            {
                return *l < *r;
            }
        };

        struct PointerEqual {
            PointerEqual(const std::shared_ptr<StateSimple>& ptr)
            {
                _ptr = ptr;
            }
            bool operator()(const std::shared_ptr<StateSimple>& l)
            {
                return *l == *_ptr;
            }

            std::shared_ptr<StateSimple> _ptr;
        };

        double operator-(const StateSimple& other) const
        {
            double diff_x = x - other.x;
            double diff_y = y - other.y;
            return std::sqrt(diff_x * diff_x + diff_y * diff_y);
        }

        StateSimple operator+(const StateSimple& other)
        {
            StateSimple tmp_action = *this;

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
        std::stringstream tr, rot;
        tr << obj.transformation.translation();
        std::string t, r;
        t = tr.str();
        std::replace(t.begin(), t.end(), '\n', ' ');
        rot << obj.transformation.rotation();
        r = rot.str();
        std::replace(r.begin(), r.end(), '\n', ' ');
        os << t << " " << r;
        return os;
    }

    std::istream& operator>>(std::istream& is, StateSimple& obj)
    {
        double x, y, r00, r01, r10, r11;
        is >> x >> y >> r00 >> r01 >> r10 >> r11;

        obj.transformation.setIdentity();
        obj.transformation.translation()[0] = x;
        obj.transformation.translation()[1] = y;
        obj.transformation.linear()(0, 0) = r00;
        obj.transformation.linear()(0, 1) = r01;
        obj.transformation.linear()(1, 0) = r10;
        obj.transformation.linear()(1, 1) = r11;

        obj.theta = std::atan2(obj.transformation.rotation()(1, 0), obj.transformation.rotation()(0, 0));
        obj.x = obj.transformation.translation()[0];
        obj.y = obj.transformation.translation()[1];

        return is;
    }
}

#endif
