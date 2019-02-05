#ifndef HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_IMU_POSITION_HPP
#define HEXAPOD_CONTROLLER_HEXAPOD_CONTROLLER_IMU_POSITION_HPP

// For M_PI constant
#define _USE_MATH_DEFINES
#include <Eigen/Core>
#include <array>
#include <boost/filesystem.hpp>
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

    class HexapodControllerImuPos {
    public:
        typedef std::array<double, ARRAY_DIM> array_t;

        /**
        * \brief create an object HexapodControllerImuPos
        */
        HexapodControllerImuPos()
        {
            int legs_number = 6;
            _error.resize(legs_number * 3 + 6, 0.0);
            _error_derivated.resize(legs_number * 3 + 6, 0.0);
            _error_integrated.resize(legs_number * 3 + 6, 0.0);
            _error_prev.resize(legs_number * 3 + 6, 0.0);
            _kp = 3;
        }

        /**
        * \brief create an object HexapodControllerImuPos
        * \param const std::vector<double>& ctrl a vector of size 36 here, which are the parameter extract from map-elites.
        * \param std::vector<int> broken_legs a vector containing the number of different broken legs
        */
        HexapodControllerImuPos(const std::vector<double>& ctrl, std::vector<int> broken_legs)
            : _broken_legs(broken_legs)
        {
            int legs_number = 6;
            _error.resize(legs_number * 3 + 6, 0.0);
            _error_derivated.resize(legs_number * 3 + 6, 0.0);
            _error_integrated.resize(legs_number * 3 + 6, 0.0);
            _error_prev.resize(legs_number * 3 + 6, 0.0);
            _kp = 3;
            set_parameters(ctrl);
        }

        /**
        * \brief set parameters for controller
        * \param const std::vector<double>& ctrl a vector of size 36 here, which are the parameter extract from map-elites.
        */
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

        /**
        * \brief get parameters
        * \return std::vector<double>& _controller which are controller parameters.
        */
        const std::vector<double>& parameters() const
        {
            return _controller;
        }

        /**
        * \brief set broken legs
        * \param std::vector<int> broken_legs a vector of different broken leg number
        */
        void set_broken(const std::vector<int> broken_legs)
        {
            _broken_legs = broken_legs;
        }

        /**
        * \brief get broken legs
        * \return std::vector<int> broken_legs a vector of different broken leg number
        */
        const std::vector<int>& broken_legs() const
        {
            return _broken_legs;
        }

        /**
        * \brief compute trajectory for each motors
        * \param double t which is  the current time
        * \return std::vector<double> angles, different angles to apply at time t.
        */
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
                    angles.push_back(_legs0center[0] * _legs0commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs0center[1] * _legs0commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs0center[2] * _legs0commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 1:
                    angles.push_back(_legs1center[0] * _legs1commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs1center[1] * _legs1commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs1center[2] * _legs1commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 2:
                    angles.push_back(_legs2center[0] * _legs2commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs2center[1] * _legs2commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs2center[2] * _legs2commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 3:
                    angles.push_back(_legs3center[0] * _legs3commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs3center[1] * _legs3commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs3center[2] * _legs3commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 4:
                    angles.push_back(_legs4center[0] * _legs4commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs4center[1] * _legs4commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs4center[2] * _legs4commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;

                case 5:
                    angles.push_back(_legs5center[0] * _legs5commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs5center[1] * _legs5commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    angles.push_back(_legs5center[2] * _legs5commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                    break;
                }

                ++leg;
            }

            return angles;
        }

        /**
        * \brief set different leg center from IMU sensors
        * \param float roll which is  the current roll
        * \param float pitch which is  the current pitch
        */
        void computeErrors(float roll, float pitch)
        {
            _legs0center.clear();
            _legs1center.clear();
            _legs2center.clear();
            _legs3center.clear();
            _legs4center.clear();
            _legs5center.clear();

            std::vector<float> Kpitch_ = {1, 0, -1, -1, 0, 1};
            //{1.4, 0, -1.4, -1.4, 0, 1.4};
            std::vector<float> Kroll_ = {1, 1, 1, -1, -1, -1};

            _legs0center.push_back(M_PI_4 / 2);
            _legs0center.push_back(M_PI_4 + Kpitch_[0] * pitch + Kroll_[0] * roll);
            _legs0center.push_back(-M_PI_4 + Kpitch_[0] * pitch + Kroll_[0] * roll);

            _legs1center.push_back(M_PI_4 / 2);
            _legs1center.push_back(M_PI_4 + Kpitch_[1] * pitch + Kroll_[1] * roll);
            _legs1center.push_back(-M_PI_4 + Kpitch_[1] * pitch + Kroll_[1] * roll);

            _legs2center.push_back(M_PI_4 / 2);
            _legs2center.push_back(M_PI_4 + Kpitch_[2] * pitch + Kroll_[2] * roll);
            _legs2center.push_back(-M_PI_4 + Kpitch_[2] * pitch + Kroll_[2] * roll);

            _legs3center.push_back(M_PI_4 / 2);
            _legs3center.push_back(M_PI_4 + Kpitch_[3] * pitch + Kroll_[3] * roll);
            _legs3center.push_back(-M_PI_4 + Kpitch_[3] * pitch + Kroll_[3] * roll);

            _legs4center.push_back(M_PI_4 / 2);
            _legs4center.push_back(M_PI_4 + Kpitch_[4] * pitch + Kroll_[4] * roll);
            _legs4center.push_back(-M_PI_4 + Kpitch_[4] * pitch + Kroll_[4] * roll);

            _legs5center.push_back(M_PI_4 / 2);
            _legs5center.push_back(M_PI_4 + Kpitch_[5] * pitch + Kroll_[5] * roll);
            _legs5center.push_back(-M_PI_4 + Kpitch_[5] * pitch + Kroll_[5] * roll);
        }

    private:
        /*!  current error  */
        std::vector<float> _error;
        /*!  previous error  */
        std::vector<float> _error_prev;
        /*!  derivated error  */
        std::vector<float> _error_derivated;
        /*!  integrated error  */
        std::vector<float> _error_integrated;

    protected:
        /**
            All parameters should have a value between 0 and 1.
        **/
        /**
        * \brief compute a control signal for each motors, here a sinusoidale square wave
        * \param double amplitude which is  the sinus amplitude
        * \param double phase which is the sinus phase
        * \param double duty_cycle which is the time where the command position saturate
        * \return array_t final_command, final command to apply (position)
        */
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
        /*!  legs commands  */
        std::vector<array_t> _legs0commands;
        std::vector<array_t> _legs1commands;
        std::vector<array_t> _legs2commands;
        std::vector<array_t> _legs3commands;
        std::vector<array_t> _legs4commands;
        std::vector<array_t> _legs5commands;
        /*!  legs centers  */
        std::vector<double> _legs0center;
        std::vector<double> _legs1center;
        std::vector<double> _legs2center;
        std::vector<double> _legs3center;
        std::vector<double> _legs4center;
        std::vector<double> _legs5center;

        /*!  controller parameters  */
        std::vector<double> _controller;
        /*!   number of broken legs  */
        std::vector<int> _broken_legs;

        /*!   different gains  */
        float _kp;
        float _kd;
        float _t_prev;
        float _t;

        /*!   final motors angles  */
        std::vector<double> _angles;
    };
} // namespace hexapod_controller

#endif
