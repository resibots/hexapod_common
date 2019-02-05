#ifndef HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_IMU_HPP
#define HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_IMU_HPP

// For M_PI constant
#define _USE_MATH_DEFINES
#include <Eigen/Core>
#include <array>
//#include <boost/filesystem.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

#define ARRAY_DIM 100

namespace hexapod_controller {

    class HexapodControllerImu {
    public:
        typedef std::array<double, ARRAY_DIM> array_t;

        HexapodControllerImu()
        {
            int legs_number = 6;
            _error.resize(legs_number * 3 + 6, 0.0);
            _error_derivated.resize(legs_number * 3 + 6, 0.0);
            _error_integrated.resize(legs_number * 3 + 6, 0.0);
            _error_prev.resize(legs_number * 3 + 6, 0.0);
            _kp = 3;
        }
        HexapodControllerImu(const std::vector<double>& ctrl, std::vector<int> broken_legs)
            : _broken_legs(broken_legs)
        {

            int legs_number = 6 - _broken_legs.size();
            _error.resize(legs_number * 3 + 6, 0.0);
            _error_derivated.resize(legs_number * 3 + 6, 0.0);
            _error_integrated.resize(legs_number * 3 + 6, 0.0);
            _error_prev.resize(legs_number * 3 + 6, 0.0);
            _kp = 3;
            set_parameters(ctrl);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            assert(ctrl.size() == 36);

            _legs0commands.clear();
            _legs1commands.clear();
            _legs2commands.clear();
            _legs3commands.clear();
            _legs4commands.clear();
            _legs5commands.clear();

            _controller = ctrl;

            _legs0commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
            _legs0commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
            _legs0commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));

            _legs1commands.push_back(_control_signal(ctrl[6], ctrl[7], ctrl[8]));
            _legs1commands.push_back(_control_signal(ctrl[9], ctrl[10], ctrl[11]));
            _legs1commands.push_back(_control_signal(ctrl[9], ctrl[10], ctrl[11]));

            _legs2commands.push_back(_control_signal(ctrl[12], ctrl[13], ctrl[14]));
            _legs2commands.push_back(_control_signal(ctrl[15], ctrl[16], ctrl[17]));
            _legs2commands.push_back(_control_signal(ctrl[15], ctrl[16], ctrl[17]));

            _legs3commands.push_back(_control_signal(ctrl[18], ctrl[19], ctrl[20]));
            _legs3commands.push_back(_control_signal(ctrl[21], ctrl[22], ctrl[23]));
            _legs3commands.push_back(_control_signal(ctrl[21], ctrl[22], ctrl[23]));

            _legs4commands.push_back(_control_signal(ctrl[24], ctrl[25], ctrl[26]));
            _legs4commands.push_back(_control_signal(ctrl[27], ctrl[28], ctrl[29]));
            _legs4commands.push_back(_control_signal(ctrl[27], ctrl[28], ctrl[29]));

            _legs5commands.push_back(_control_signal(ctrl[30], ctrl[31], ctrl[32]));
            _legs5commands.push_back(_control_signal(ctrl[33], ctrl[34], ctrl[35]));
            _legs5commands.push_back(_control_signal(ctrl[33], ctrl[34], ctrl[35]));
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

        std::vector<double> pos(double t) const
        {
            assert(_controller.size() == 36);

            int leg = 0;
            std::vector<double> angles;
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
                    angles.push_back(M_PI_4 / 2 * _legs0commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(M_PI_4 * _legs0commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(-M_PI_4 * _legs0commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 1:
                    angles.push_back(M_PI_4 / 2 * _legs1commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(M_PI_4 * _legs1commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(-M_PI_4 * _legs1commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 2:
                    angles.push_back(M_PI_4 / 2 * _legs2commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(M_PI_4 * _legs2commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(-M_PI_4 * _legs2commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 3:
                    angles.push_back(M_PI_4 / 2 * _legs3commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(M_PI_4 * _legs3commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(-M_PI_4 * _legs3commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 4:
                    angles.push_back(M_PI_4 / 2 * _legs4commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(M_PI_4 * _legs4commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(-M_PI_4 * _legs4commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 5:
                    angles.push_back(M_PI_4 / 2 * _legs5commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(M_PI_4 * _legs5commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(-M_PI_4 * _legs5commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;
                }

                ++leg;
            }

            return angles;
        }

        std::vector<float> computeErrors(float roll, float pitch, const Eigen::VectorXd& joint, const Eigen::VectorXd& angles, double loop_rate)
        {

            for (unsigned int i = 0; i < _error.size(); i++) {
                _error_prev[i] = _error[i];
            }
            std::vector<float> kpitch_ = {10, 0, -10, -10, 0, 10};
            //{1.4, 0, -1.4, -1.4, 0, 1.4};
            std::vector<float> kroll_ = {10, 10, 10, -10, -10, -10};
            int leg = 0;
            int index_leg = 0;
            //  assert(_controller.size() == 36);

            //bool is_broken;
            int i_broken = 0;
            std::vector<float> command_final;

            int num_leg = 6;

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

                    _error[index_leg * 3 + 6] = joint(index_leg * 3 + 6) - angles(index_leg * 3 + 6); // +6 offset CoM
                    command_final.push_back(-_kp * _error[index_leg * 3 + 6]);
                    _error[3 * index_leg + 7] = joint(3 * index_leg + 7) - angles(3 * index_leg + 7);
                    command_final.push_back(-_kp * _error[3 * index_leg + 7] + kpitch_[0] * pitch + kroll_[0] * roll);
                    _error[3 * index_leg + 8] = joint(3 * index_leg + 8) - angles(3 * index_leg + 8);
                    command_final.push_back(-_kp * _error[3 * index_leg + 8] + kpitch_[0] * pitch + kroll_[0] * roll);
                    index_leg++;
                    break;
                case 1:

                    _error[index_leg * 3 + 6] = joint(index_leg * 3 + 6) - angles(index_leg * 3 + 6); // +6 offset CoM
                    command_final.push_back(-_kp * _error[index_leg * 3 + 6]);
                    _error[3 * index_leg + 7] = joint(3 * index_leg + 7) - angles(3 * index_leg + 7);
                    command_final.push_back(-_kp * _error[3 * index_leg + 7] + kpitch_[1] * pitch + kroll_[1] * roll);
                    _error[3 * index_leg + 8] = joint(3 * index_leg + 8) - angles(3 * index_leg + 8);
                    command_final.push_back(-_kp * _error[3 * index_leg + 8] + kpitch_[1] * pitch + kroll_[1] * roll);
                    index_leg++;
                    break;
                case 2:

                    _error[index_leg * 3 + 6] = joint(index_leg * 3 + 6) - angles(index_leg * 3 + 6); // +6 offset CoM
                    command_final.push_back(-_kp * _error[index_leg * 3 + 6]);
                    _error[3 * index_leg + 7] = joint(3 * index_leg + 7) - angles(3 * index_leg + 7);
                    command_final.push_back(-_kp * _error[3 * index_leg + 7] + kpitch_[2] * pitch + kroll_[2] * roll);
                    _error[3 * index_leg + 8] = joint(3 * index_leg + 8) - angles(3 * index_leg + 8);
                    command_final.push_back(-_kp * _error[3 * index_leg + 8] + kpitch_[2] * pitch + kroll_[2] * roll);
                    index_leg++;
                    break;
                case 3:

                    _error[3 * index_leg + 6] = joint(3 * index_leg + 6) - angles(3 * index_leg + 6); // +6 offset CoM
                    command_final.push_back(-_kp * _error[3 * index_leg + 6]);
                    _error[3 * index_leg + 7] = joint(3 * index_leg + 7) - angles(3 * index_leg + 7);
                    command_final.push_back(-_kp * _error[3 * index_leg + 7] + kpitch_[3] * pitch + kroll_[3] * roll);
                    _error[3 * index_leg + 8] = joint(3 * index_leg + 8) - angles(3 * index_leg + 8);
                    command_final.push_back(-_kp * _error[3 * index_leg + 8] + kpitch_[3] * pitch + kroll_[3] * roll);
                    index_leg++;
                    break;
                case 4:

                    _error[index_leg * 3 + 6] = joint(index_leg * 3 + 6) - angles(index_leg * 3 + 6); // +6 offset CoM
                    command_final.push_back(-_kp * _error[index_leg * 3 + 6]);
                    _error[3 * index_leg + 7] = joint(3 * index_leg + 7) - angles(3 * index_leg + 7);
                    command_final.push_back(-_kp * _error[3 * index_leg + 7] + kpitch_[4] * pitch + kroll_[4] * roll);
                    _error[3 * index_leg + 8] = joint(3 * index_leg + 8) - angles(3 * index_leg + 8);
                    command_final.push_back(-_kp * _error[3 * index_leg + 8] + kpitch_[4] * pitch + kroll_[4] * roll);
                    index_leg++;
                    break;
                case 5:

                    _error[index_leg * 3 + 6] = joint(index_leg * 3 + 6) - angles(index_leg * 3 + 6); // +6 offset CoM
                    command_final.push_back(-_kp * _error[index_leg * 3 + 6]);
                    _error[3 * index_leg + 7] = joint(3 * index_leg + 7) - angles(3 * index_leg + 7);
                    command_final.push_back(-_kp * _error[3 * index_leg + 7] + kpitch_[5] * pitch + kroll_[5] * roll);
                    _error[3 * index_leg + 8] = joint(3 * index_leg + 8) - angles(3 * index_leg + 8);
                    command_final.push_back(-_kp * _error[3 * index_leg + 8] + kpitch_[5] * pitch + kroll_[5] * roll);
                    index_leg++;
                    break;
                }

                ++leg;
            }
            // for (int i = 0; i < num_leg; i++) {
            //
            //     _error[i * 3 + 6] = joint(i * 3 + 6) - angles(i * 3 + 6); // +6 offset CoM
            //     _error_derivated[i * 3 + 6] = (_error[i * 3 + 6] - _error_prev[i * 3 + 6]) / loop_rate;
            //     _error_integrated[i * 3 + 6] += _error[i * 3 + 6];
            //
            //     command_final.push_back(-_kp * _error[i * 3 + 6]);
            //
            //     _error[3 * i + 7] = joint(3 * i + 7) - angles(3 * i + 7);
            //     _error_derivated[3 * i + 7] = (_error[3 * i + 7] - _error_prev[3 * i + 7]) / loop_rate;
            //     _error_integrated[3 * i + 7] += _error[3 * i + 7];
            //
            //     command_final.push_back(-_kp * _error[3 * i + 7] + kpitch_[i] * pitch + kroll_[i] * roll);
            //
            //     _error[3 * i + 8] = joint(3 * i + 8) - angles(3 * i + 8);
            //     _error_derivated[3 * i + 8] = (_error[3 * i + 8] - _error_prev[3 * i + 8]) / loop_rate;
            //     _error_integrated[3 * i + 8] += _error[3 * i + 8];
            //
            //     command_final.push_back(-_kp * _error[3 * i + 8] + kpitch_[i] * pitch + kroll_[i] * roll);
            // }

            return command_final;
        }

    private:
        std::vector<float> _error;
        std::vector<float> _error_prev;
        std::vector<float> _error_derivated;
        std::vector<float> _error_integrated;

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
            for (int i = 0; i < int(kernel.size()); i++) {
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
            array_t _final_command;
            int current = 0;
            int start = std::floor(ARRAY_DIM * phase);
            for (int i = start; i < ARRAY_DIM; i++) {
                _final_command[current] = command[i];
                current++;
            }
            for (int i = 0; i < start; i++) {
                _final_command[current] = command[i];
                current++;
            }

            return _final_command;
        }

        std::vector<array_t> _legs0commands;
        std::vector<array_t> _legs1commands;
        std::vector<array_t> _legs2commands;
        std::vector<array_t> _legs3commands;
        std::vector<array_t> _legs4commands;
        std::vector<array_t> _legs5commands;

        std::vector<double> _controller;
        std::vector<int> _broken_legs;

        //  std::vector<float> _kpitch;
        //  std::vector<float> _kroll;
        float _kp;
        float _kd;
        float _t_prev;
        float _t;

        std::vector<double> _angles;
    };
} // namespace hexapod_controller

#endif
