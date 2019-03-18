#include <Eigen/Core>
#include <hexapod_controller/hexapod_controller_imu_position.hpp>
#include <iostream>

using namespace hexapod_controller;

int main()
{
    std::cout << "start" << std::endl;
    Eigen::VectorXd _target_positions;
    HexapodControllerImuPos controller({1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5}, {});
    std::cout << "controller created" << std::endl;

    for (double t = 0.0; t <= 5.0; t += 0.1) {
        controller.computeErrors(0, 0);
        std::cout << "compute errors" << std::endl;
        auto angles = controller.pos(t);
        std::cout << "calculate position" << std::endl;
    }

    std::cout << "finish" << std::endl;
    return 0;
}
