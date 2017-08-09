#ifndef HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_CARTESIAN_HPP
#define HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_CARTESIAN_HPP

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

// #define ARRAY_DIM 100

namespace hexapod_controller {

    class HexapodControllerCartesian {
    public:
        constexpr static int ARRAY_DIM = 100;
        typedef std::array<double, ARRAY_DIM> array_t;

        HexapodControllerCartesian() {}
        HexapodControllerCartesian(const std::vector<double>& ctrl, std::vector<int> broken_legs, const std::array<std::array<double, 3>, 6>& scaling = std::array<std::array<double, 3>, 6>())
            : _broken_legs(broken_legs),
              _scaling(scaling)
        {
            set_parameters(ctrl);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            assert(ctrl.size() == 54);

            _legs0commands.clear();
            _legs1commands.clear();
            _legs2commands.clear();
            _legs3commands.clear();
            _legs4commands.clear();
            _legs5commands.clear();

            _controller = ctrl;

            _legs0commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
            _legs0commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
            _legs0commands.push_back(_control_signal(ctrl[6], ctrl[7], ctrl[8]));

            _legs1commands.push_back(_control_signal(ctrl[9], ctrl[10], ctrl[11]));
            _legs1commands.push_back(_control_signal(ctrl[12], ctrl[13], ctrl[14]));
            _legs1commands.push_back(_control_signal(ctrl[15], ctrl[16], ctrl[17]));

            _legs2commands.push_back(_control_signal(ctrl[18], ctrl[19], ctrl[20]));
            _legs2commands.push_back(_control_signal(ctrl[21], ctrl[22], ctrl[23]));
            _legs2commands.push_back(_control_signal(ctrl[24], ctrl[25], ctrl[26]));

            _legs3commands.push_back(_control_signal(ctrl[27], ctrl[28], ctrl[29]));
            _legs3commands.push_back(_control_signal(ctrl[30], ctrl[31], ctrl[32]));
            _legs3commands.push_back(_control_signal(ctrl[33], ctrl[34], ctrl[35]));

            _legs4commands.push_back(_control_signal(ctrl[36], ctrl[37], ctrl[38]));
            _legs4commands.push_back(_control_signal(ctrl[39], ctrl[40], ctrl[41]));
            _legs4commands.push_back(_control_signal(ctrl[42], ctrl[43], ctrl[44]));

            _legs5commands.push_back(_control_signal(ctrl[45], ctrl[46], ctrl[47]));
            _legs5commands.push_back(_control_signal(ctrl[48], ctrl[49], ctrl[50]));
            _legs5commands.push_back(_control_signal(ctrl[51], ctrl[52], ctrl[53]));
        }

        const std::vector<double>& parameters() const
        {
            return _controller;
        }

        void set_broken(const std::vector<int> broken_legs)
        {
            _broken_legs = broken_legs;
        }

        const std::vector<int>& broken_legs() const
        {
            return _broken_legs;
        }

        void set_scaling(const std::array<std::array<double, 3>, 6>& scaling)
        {
            _scaling = scaling;
        }

        const std::array<std::array<double, 3>, 6>& scaling() const
        {
            return _scaling;
        }

        std::vector<double> pos(const double t) const
        {
            assert(_controller.size() == 54);

            std::vector<double> angles;
            int leg = 0;

            for (size_t i = 0; i < 18; i += 3) {
                for (size_t j = 0; j < _broken_legs.size(); j++) {
                    if (leg == _broken_legs[j]) {
                        leg++;
                        if (_broken_legs.size() > j + 1 && _broken_legs[j + 1] != leg)
                            break;
                    }
                }

                switch (leg) {
                case 0:
                    angles.push_back(_scaling[0][0] * _legs0commands[0][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[0][1] * _legs0commands[1][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[0][2] * _legs0commands[2][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    break;

                case 1:
                    angles.push_back(_scaling[1][0] * _legs1commands[0][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[1][1] * _legs1commands[1][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[1][2] * _legs1commands[2][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    break;

                case 2:
                    angles.push_back(_scaling[2][0] * _legs2commands[0][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[2][1] * _legs2commands[1][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[2][2] * _legs2commands[2][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    break;

                case 3:
                    angles.push_back(_scaling[3][0] * _legs3commands[0][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[3][1] * _legs3commands[1][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[3][2] * _legs3commands[2][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    break;

                case 4:
                    angles.push_back(_scaling[4][0] * _legs4commands[0][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[4][1] * _legs4commands[1][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[4][2] * _legs4commands[2][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    break;

                case 5:
                    angles.push_back(_scaling[5][0] * _legs5commands[0][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[5][1] * _legs5commands[1][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    angles.push_back(_scaling[5][2] * _legs5commands[2][((int)std::floor(t * ARRAY_DIM)) % ARRAY_DIM]);
                    break;
                }

                ++leg;
            }
            return angles;
        }

    protected:
        /**
            All parameters should have a value between 0 and 1.
        **/
        array_t _control_signal(double amplitude, double phase, double duty_cycle) const
        {
            array_t temp;
            int up_time = ARRAY_DIM * duty_cycle;
            for (int i = 0; i < up_time; i++)
                temp[i] = amplitude;
            for (int i = up_time; i < ARRAY_DIM; i++)
                temp[i] = -amplitude;

            // filtering
            int kernel_size = ARRAY_DIM / 10;

            array_t command;

            std::vector<double> kernel(2 * kernel_size + 1, 0);
            double sigma = kernel_size / 3;

            double sum = 0;
            for (int i = 0; i < (int)kernel.size(); i++) {
                kernel[i] = exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma * sigma)) / (sigma * sqrt(M_PI));
                sum += kernel[i];
            }

            for (int i = 0; i < ARRAY_DIM; i++) {
                command[i] = 0;
                for (int d = 1; d <= kernel_size; d++) {
                    if (i - d < 0)
                        command[i] += temp[ARRAY_DIM + i - d] * kernel[kernel_size - d];
                    else
                        command[i] += temp[i - d] * kernel[kernel_size - d];
                }
                command[i] += temp[i] * kernel[kernel_size];
                for (int d = 1; d <= kernel_size; d++) {
                    if (i + d >= ARRAY_DIM)
                        command[i] += temp[i + d - ARRAY_DIM] * kernel[kernel_size + d];
                    else
                        command[i] += temp[i + d] * kernel[kernel_size + d];
                }

                command[i] /= sum;
            }

            // apply phase
            array_t final_command;
            int current = 0;
            int start = std::floor(ARRAY_DIM * phase);
            for (int i = start; i < ARRAY_DIM; i++) {
                final_command[current] = command[i];
                current++;
            }
            for (int i = 0; i < start; i++) {
                final_command[current] = command[i];
                current++;
            }

            return final_command;
        }

        std::vector<array_t> _legs0commands;
        std::vector<array_t> _legs1commands;
        std::vector<array_t> _legs2commands;
        std::vector<array_t> _legs3commands;
        std::vector<array_t> _legs4commands;
        std::vector<array_t> _legs5commands;

        std::vector<double> _controller;
        std::vector<int> _broken_legs;
        std::array<std::array<double, 3>, 6> _scaling;
    };
} // namespace hexapod_controller

#endif
