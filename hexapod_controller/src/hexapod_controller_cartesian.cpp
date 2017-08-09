#include <iostream>
#include <fstream>
#include <iomanip>
#include <hexapod_controller/hexapod_controller_cartesian.hpp>

using namespace hexapod_controller;

std::ostream& operator<<(std::ostream& in, const std::vector<double>& vector)
{
    using it_t = std::vector<double>::const_iterator;
    auto cend = --vector.cend();
    it_t it = vector.cbegin();
    for (; it != cend; ++it) {
        in << std::setw(10) << std::setprecision(5) << *it << "  ";
    }
    in << std::setw(10) << std::setprecision(5) << *it;

    return in;
}

int main()
{
    // std::array<std::array<double, 3>, 6> scaling = {{{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}};
    static std::array<std::array<double, 3>, 6> scaling = {{{{0.01, 0.01, 0.01}},
        {{0.01, 0.01, 0.01}},
        {{0.01, 0.01, 0.01}},
        {{0.01, 0.01, 0.01}},
        {{0.01, 0.01, 0.01}},
        {{0.01, 0.01, 0.01}}}};

    std::vector<double> control_params = {{1, 0, 0.5,
        1, 0.25, 0.5,
        1, 0.25, 0.9,
        0.25, 0.75, 0.5,
        1, 0, 0.5,
        0.25, 0.25, 0.5,
        1, 0, 0.5,
        0.25, 0.75, 0.5,
        1, 0.5, 0.5,
        0.25, 0.25, 0.5,
        1, 0, 0.5,
        0.25, 0.75, 0.5,
        0.5, 1, 0,
        0.5, 0.25, 0.75,
        0.5, 0.5, 1,
        0, 0.5, 0.25,
        0.75, 0.5, 0.5,
        1, 0, 0.5}};

    HexapodControllerCartesian controller(control_params, {}, scaling);

    std::ofstream trajectory_file("traj.csv");
    for (double t = 0.0; t <= 5.0; t += 0.01) {
        auto angles = controller.pos(t);
        trajectory_file << t << ", " << angles[0] << ", " << angles[1] << ", " << angles[2] << std::endl;
    }
    return 0;
}
