#ifndef HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_SIMPLE_HPP
#define HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_SIMPLE_HPP

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#define ARRAY_DIM 100

class HexapodControllerSimple {
public:
    typedef std::array<double, ARRAY_DIM> array_t;

    HexapodControllerSimple() {}
    HexapodControllerSimple(const std::vector<double>& ctrl, std::vector<int> broken_legs);

    void set_parameters(const std::vector<double>& ctrl);
    std::vector<double> parameters();

    void set_broken(const std::vector<int> broken_legs);
    std::vector<int> broken_legs();

    std::vector<double> pos(double t);
protected:

    array_t _control_signal(double amplitude, double phase, double duty_cycle);

    std::vector<array_t> _legs0commands;
    std::vector<array_t> _legs1commands;
    std::vector<array_t> _legs2commands;
    std::vector<array_t> _legs3commands;
    std::vector<array_t> _legs4commands;
    std::vector<array_t> _legs5commands;

    std::vector<double> _controller;
    std::vector<int> _broken_legs;
};

#endif
