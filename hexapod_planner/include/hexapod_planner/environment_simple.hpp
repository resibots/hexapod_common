#ifndef HEXAPOD_PLANNER_SIMPLE_ENV_HPP
#define HEXAPOD_PLANNER_SIMPLE_ENV_HPP

#include <vector>
#include <memory>

namespace hexapod_planner {

    struct BoundingSimple {
        double x, y, radius;

        template <typename BoundingOther>
        bool collides(const BoundingOther& other) const
        {
            double dx = x - other.x;
            double dy = y - other.y;
            double d = std::sqrt(dx * dx + dy * dy);
            double r = radius + other.radius;
            return d < r;
        }

        template <typename ActionSimple>
        void constructFromAction(const ActionSimple& action, double r)
        {
            x = action.x;
            y = action.y;
            radius = r;
        }
    };

    struct RobotSimple {
        BoundingSimple bounding;

        template <typename ActionSimple>
        void constructFromAction(const ActionSimple& action)
        {
            bounding.constructFromAction(action, 1.0);
        }
    };

    struct ObstacleSimple {
        BoundingSimple bounding;
    };

    template <typename Obstacle>
    class EnvironmentSimple {
    public:
        EnvironmentSimple() {}

        std::vector<std::shared_ptr<Obstacle>> obstacles()
        {
            return _obstacles;
        }
        void add_obstacle(const Obstacle& obs)
        {
            _obstacles.push_back(std::make_shared<Obstacle>(obs));
        }

        template <typename Bounding>
        bool collides(const Bounding& bound)
        {
            for (auto obs : _obstacles) {
                if (obs->bounding.collides(bound))
                    return true;
            }
            return false;
        }

    protected:
        std::vector<std::shared_ptr<Obstacle>> _obstacles;
    };
}

#endif