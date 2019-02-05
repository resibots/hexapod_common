#include <Eigen/Core>
#include <hexapod_controller/hexapod_controller_imu.hpp>
#include <iostream>

using namespace hexapod_controller;

int main()
{
    Eigen::VectorXd _target_positions(24);
    HexapodControllerImu controller({1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5}, {});
    Eigen::VectorXd joint(24);
    joint << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    for (double t = 0.0; t <= 5.0; t += 0.1) {
        auto angles = controller.pos(t);
        std::cout << "position calculated " << std::endl;
        for (size_t i = 0; i < angles.size(); i++) {
            std::cout << "for loop" << std::endl;
            _target_positions(i + 6) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];
        }
        std::cout << "target position" << std::endl;
        auto cmd = controller.computeErrors(0, 0, joint, _target_positions, 0.1);
        std::cout << "errors computed" << std::endl;
    }

    std::cout << "finish" << std::endl;
    return 0;
}
