#ifndef HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_CARTESIAN_HPP
#define HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_CARTESIAN_HPP

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

namespace hexapod_controller {

    // SamplingFrequency represents the number of samples per second. It is also the number of point generated for the trajectory.
    template <uint8_t NLegs = 6, uint16_t SamplingFrequency = 100>
    class HexapodControllerCartesian {
    public:
        typedef std::array<double, SamplingFrequency> array_t;

        HexapodControllerCartesian() {}
        HexapodControllerCartesian(
            const std::vector<double>& ctrl,
            std::vector<int> broken_legs = std::vector<int>(),
            const std::array<std::array<double, 3>, 6>& scaling = std::array<std::array<double, 3>, 6>())

            : _scaling(scaling)
        {
            // _broken_legs.fill(false);
            // for (auto broken_leg : broken_legs) {
            //     _broken_legs[broken_leg] = true;
            // }

            set_parameters(ctrl);
        }

        /** Set the trajectory's parameters.

            This method is also called at contruction with the provided parameters
            (unles the empty contructor is used).

            @param ctrl parameters of the trajectory, nine parameter per leg;
                these nine parameters are actually three triplet, each
                representing an axis in cartesian space

                All parameters should be between 0 and 1.
            **/
        void set_parameters(const std::vector<double>& ctrl)
        {
            assert(ctrl.size() == NLegs * 3 * 3);

            _controller = ctrl;

            for (uint8_t leg = 0; leg < NLegs; ++leg) {
                for (uint8_t joint = 0; joint < 3; ++joint) {
                    auto offset = (leg * 3 + joint) * 3;
                    _leg_commands[leg][joint] = _control_signal(ctrl[offset], ctrl[offset + 1], ctrl[offset + 2]);
                }
            }
        }

        const std::vector<double>& parameters() const
        {
            return _controller;
        }

        // void set_broken(const std::vector<int> broken_legs)
        // {
        //     _broken_legs.fill(false);
        //     for (auto broken_leg : broken_legs) {
        //         _broken_legs[broken_leg] = true;
        //     }
        //     // _broken_legs = broken_legs;
        // }
        //
        // const std::vector<int> broken_legs() const
        // {
        //     std::vector<int> broken_legs;
        //     for (int i = 0; i < _broken_legs.size(); ++i) {
        //         if (_broken_legs[i])
        //             broken_legs.push_back(i);
        //     }
        //     return broken_legs;
        //     // return _broken_legs;
        // }

        void set_scaling(const std::array<std::array<double, 3>, 6>& scaling)
        {
            _scaling = scaling;
        }

        const std::array<std::array<double, 3>, 6>& scaling() const
        {
            return _scaling;
        }

        /** Get the state at time t.

            For a hexapod with 3 joints per leg, you'll get an array of 6 entries
            (one per leg), each being an array of 3 joint angles.

            @param t current time, in seconds

            @return the first dimension is for the leg and the second for the
                joint within the leg.
        **/
        std::array<std::array<double, 3>, NLegs> pos(const double t) const
        {
            std::array<std::array<double, 3>, NLegs> angles;

            // the signal being generated for a 1s window with SamplingFrequency
            // samples, move the absolute time input to that window and convert it to a sample number
            uint16_t time_index = (int(std::floor(t * SamplingFrequency))) % SamplingFrequency;

            // iteration over the legs
            for (uint8_t leg = 0; leg < NLegs; ++leg) {
                // // skip this leg if declared broken
                // if (_broken_legs[leg])
                //     continue;

                // iteration over the leg's joints
                for (uint8_t joint = 0; joint < 3; ++joint) {
                    angles[leg][joint]
                        = _scaling[leg][joint] * _leg_commands[leg][joint][time_index];
                }
            }
            return angles;
        }

    protected:
        /** Generate a trajectory shaped like a rounded square signal.

            All parameters should have a value between 0 and 1.
        **/
        array_t _control_signal(double amplitude, double phase, double duty_cycle) const
        {
            array_t temp;
            int up_time = SamplingFrequency * duty_cycle;
            for (int i = 0; i < up_time; i++)
                temp[i] = amplitude;
            for (int i = up_time; i < SamplingFrequency; i++)
                temp[i] = -amplitude;

            // filtering
            int kernel_size = SamplingFrequency / 10;

            array_t command;

            std::vector<double> kernel(2 * kernel_size + 1, 0);
            double sigma = kernel_size / 3;

            double sum = 0;
            for (int i = 0; i < int(kernel.size()); i++) {
                kernel[i] = exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma * sigma)) / (sigma * sqrt(M_PI));
                sum += kernel[i];
            }

            for (int i = 0; i < SamplingFrequency; i++) {
                command[i] = 0;
                for (int d = 1; d <= kernel_size; d++) {
                    if (i - d < 0)
                        command[i] += temp[SamplingFrequency + i - d] * kernel[kernel_size - d];
                    else
                        command[i] += temp[i - d] * kernel[kernel_size - d];
                }
                command[i] += temp[i] * kernel[kernel_size];
                for (int d = 1; d <= kernel_size; d++) {
                    if (i + d >= SamplingFrequency)
                        command[i] += temp[i + d - SamplingFrequency] * kernel[kernel_size + d];
                    else
                        command[i] += temp[i + d] * kernel[kernel_size + d];
                }

                command[i] /= sum;
            }

            // apply phase
            array_t final_command;
            int current = 0;
            int start = std::floor(SamplingFrequency * phase);
            for (int i = start; i < SamplingFrequency; i++) {
                final_command[current] = command[i];
                current++;
            }
            for (int i = 0; i < start; i++) {
                final_command[current] = command[i];
                current++;
            }

            return final_command;
        }

        std::array<std::array<array_t, 3>, NLegs> _leg_commands;

        std::vector<double> _controller;
        // std::array<bool, NLegs> _broken_legs;
        std::array<std::array<double, 3>, 6> _scaling;
    };
} // namespace hexapod_controller

#endif
