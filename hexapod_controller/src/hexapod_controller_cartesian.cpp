#include <iostream>
#include <fstream>
#include <iomanip>
#include <hexapod_controller/hexapod_controller_cartesian.hpp>

using namespace hexapod_controller;

template <typename T>
std::ostream& operator<<(std::ostream& in, const std::vector<T>& vector)
{
    using it_t = typename std::vector<T>::const_iterator;
    auto cend = --vector.cend();
    it_t it = vector.cbegin();
    for (; it != cend; ++it) {
        in << std::setw(10) << std::setprecision(5) << *it << " ";
    }
    in << std::setw(10) << std::setprecision(5) << *it;

    return in;
}

template <typename T, size_t N>
std::ostream& operator<<(std::ostream& in, const std::array<T, N>& array)
{
    using it_t = typename std::array<T, N>::const_iterator;
    auto cend = array.cend() - 1;
    it_t it = array.cbegin();
    for (; it != cend; ++it) {
        in << std::setw(10) << std::setprecision(5) << *it << ", ";
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
        0.25, 0.5, 0.25,
        0.75, 0.5, 0.5,
        1, 0, 0.5}};

    // Testing the broken leg parameter
    // std::vector<int> broken_legs = {{1, 2, 3, 4}};
    // HexapodControllerCartesian<6, 3> controller(control_params, broken_legs, scaling);

    HexapodControllerCartesian<6, 3> controller(control_params, {}, scaling);

    std::ofstream trajectory_file("traj.csv");
    for (double t = 0.0; t <= 5.0; t += 0.01) {
        auto angles = controller.pos(t);
        trajectory_file << t << ", " << angles << std::endl;
        // trajectory_file << t << ", " << angles[0][0] << ", " << angles[0][1] << ", " << angles[0][2] << std::endl;
    }
    return 0;
}
