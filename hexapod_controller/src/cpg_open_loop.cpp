#include <hexapod_controller/cpg_open_loop.hpp>
#include <iostream>

using namespace hexapod_controller;

int main()
{
    CpgOpenLoop controller;
    controller.computeTrajectory(20);
    std::vector<double> angles;
    for (double t = 0.0; t <= 2; t += 0.1)
        angles = controller.pos(t);
    angles.clear();

    return 0;
}
