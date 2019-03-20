#ifndef CPG_OPEN_LOOP_H_
#define CPG__OPEN_LOOP_H_
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

namespace hexapod_controller {

    class CpgOpenLoop {
    public:
        CpgOpenLoop(int legs_number, float w, float gammacpg, float lambdaa, float a, float b, int d,
            float euler_dt, float rk_dt, std::vector<float> cx0,
            std::vector<float> cy0, float kp, float kd);
        /**
   * \brief compute the derivative of X and Y, the joints angles. It uses Eq 4 of the paper.
   * \param std::vector<float> X a vector of size legs_number containing the x joints angle (in the
   axial plane)
   * \param std::vector<float> Y a vector of size legs_number containing the y joints angle (in the
   sagittal plane)
   * \param std::vector<float> cx  x center coordinate of cpgg limit cycle
   * \param std::vector<float> cy  y center coordinate of cpgg limit cycle
   * \return std::vector<std::pair<float, float>> XYdot a vector of size legs_number containing the
   pairs(xdot,ydot)
   */
        std::vector<std::pair<float, float>> computeXYdot(std::vector<float> X, std::vector<float> Y);
        std::vector<std::pair<float, float>> computeXYdot(std::vector<float> X, std::vector<float> Y, Eigen::Matrix<float, 1, 6> integrate_delta_thetax, Eigen::Matrix<float, 1, 6> integrate_delta_thetay, Eigen::Matrix<float, 3, 6> delta_theta_e);
        void computeTrajectory(double duration);
        std::vector<double> pos(double t);
        /**
   * \brief Uses Euler integration to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!! It
   diverges when euler_dt is too small !!!! <br> It would also be better to use Runge Kutta instead
   of Euler
   * \param float xprev : x angle at t-1
   * \param float yprev : y angle at t-1
   * \param std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot
   * \return std::pair<float, float> return (x,y)
   */
        std::pair<float, float> euler(float xprev, float yprev, std::pair<float, float> xydot);

        /**
   * \brief Uses Runge-Kutta 4 integration to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!!
   * \param float xprev : x angle at t-1 \param float yprev : y angle at t-1 \param
   * std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot \param
   * std::pair<float, float> return (x,y)
   */
        std::pair<float, float> RK4(float xprev, float yprev, std::pair<float, float> xydot);

        std::vector<float> computeErrors(float roll, float pitch, std::vector<float> joints);
        std::pair<std::vector<float>, std::vector<float>> computeCPGcmd();

        void set_parameters(const std::vector<double>& ctrl, std::vector<int> broken_legs);
        const std::vector<double>& parameters() const;

        std::vector<float> get_cx()
        {
            return cx_;
        };

        std::vector<float> get_cy()
        {
            return cy_;
        };

        void set_cy(std::vector<float> cy)
        {
            cy_ = cy;
        };

        void set_rk_dt(float rk_dt)
        {
            rk_dt_ = rk_dt;
        };

        std::vector<std::vector<float>> createK()
        {
            std::vector<std::vector<float>> K;
            K.push_back(std::vector<float>{0, -1, -1, 1, 1, -1});
            K.push_back(std::vector<float>{-1, 0, 1, -1, -1, 1});
            K.push_back(std::vector<float>{-1, 1, 0, -1, -1, 1});
            K.push_back(std::vector<float>{1, -1, -1, 0, 1, -1});
            K.push_back(std::vector<float>{1, -1, -1, 1, 0, -1});
            K.push_back(std::vector<float>{-1, 1, 1, -1, -1, 0});
            return K;
        };

    private:
        /*!   number of legs  */
        int legs_number_;
        /*!   number of broken legs  */
        std::vector<int> _broken_legs;
        /*!   angular frequency of the oscillations  */
        float w_;
        /*!   forcing to the limit cycle , a higher value will force to form the ellipse faster. A 0 value
   * is a spiral*/
        float gammacpg_;
        /*!   coupling strength, a higher value will increase the phase shift  */
        float lambda_;
        /*!   major  axes of the limir ellipse */
        float a_;
        /*!   minor axes of the limir ellipse */
        float b_;
        /*!  curvature of the ellipse*/
        int d_;
        /*!  time step in s for euler integration*/
        float euler_dt_;
        /*! time step in s for Runge Kutta 4 integration*/
        float rk_dt_;
        /*!   coupling matrix */
        std::vector<std::vector<float>> K_;
        /*!   center of cpgg limit cycle */
        // std::vector<float> cx0 = {M_PI / 4, -M_PI / 4, 0, 0, -M_PI / 4, M_PI / 4};
        std::vector<float> cx_;
        /*!   center of cpgg limit cycle*/
        std::vector<float> cy_;
        std::vector<float> cx0_;
        /*!   center of cpgg limit cycle*/
        std::vector<float> cy0_;
        // std::vector<float> cy0 = {0, 0, 0, 0, 0, 0};
        float safety_pos_thresh_;
        std::vector<float> Xcommand_;
        std::vector<float> Ycommand_;
        bool integration_has_diverged_;
        std::vector<int> sign_;
        std::vector<float> error_;
        std::vector<float> error_prev_;
        std::vector<float> error_derivated_;
        std::vector<float> error_integrated_;
        std::vector<int> leg_map_to_paper_;
        std::vector<float> kpitch_;
        std::vector<float> kroll_;
        float loop_rate_;
        float kp_;
        float kd_;
        std::vector<double> ctrl_;
        std::vector<std::vector<float>> trajectoryX_;
        std::vector<std::vector<float>> trajectoryY_;
        double step_;
    };

    // CPG::CPG(int legs_number = 6, float w = 5, float gammacpg = 7, float lambda = 14, float a = 0.2,
    //     float b = 0.5, int d = 2, float euler_dt = 0.001, float rk_dt = 0.01,
    //     std::vector<std::vector<float>> K = createK(), std::vector<float> cx0 = {0.01, 0.01, 0, 0, 0.01, 0.01},
    //     std::vector<float> cy0 = {-M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8} // namespace cpg
    //     ) // 10 ou 100

    CpgOpenLoop::CpgOpenLoop(int legs_number = 6, float w = 0.5, float gammacpg = 0.7, float lambda = 0.14, float a = 0.2,
        float b = 0.5, int d = 2, float euler_dt = 0.001, float rk_dt = 0.001,
        std::vector<float> cx0 = {0.01, 0.0, 0.0, 0.01, 0.01, 0},
        std::vector<float> cy0 = {-M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8}, float kp = 1, float kd = 0.0)
    {
        std::cout << "inside the constructor CpgOpenLoop" << std::endl;
        legs_number_ = legs_number;
        w_ = w;
        gammacpg_ = gammacpg;
        lambda_ = lambda;
        a_ = a;
        b_ = b;
        d_ = d;
        euler_dt_ = euler_dt;
        rk_dt_ = rk_dt;

        cx_ = cx0;
        cy_ = cy0;
        cx0_ = cx0;
        cy0_ = cy0;
        K_ = CpgOpenLoop::createK();
        kp_ = kp;
        kd_ = kd;

        safety_pos_thresh_ = 1;
        Xcommand_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Ycommand_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        integration_has_diverged_ = false;
        error_.resize(legs_number * 3, 0.0);
        error_derivated_.resize(legs_number * 3, 0.0);
        error_integrated_.resize(legs_number * 3, 0.0);
        error_prev_.resize(legs_number * 3, 0.0);

        leg_map_to_paper_ = {0, 2, 4, 5, 3, 1};
        sign_ = {1, 1, 1, -1, -1, -1};
        loop_rate_ = 0.001;
        step_ = 0.0001;
    }

    /**
 * \brief compute the derivative of X and Y, the joints angles. It uses Eq 4 of the paper.
 * \param std::vector<float> X a vector of size legs_number containing the x joints angle (in the
 axial plane)
 * \param std::vector<float> Y a vector of size legs_number containing the y joints angle (in the
 sagittal plane)
 * \param std::vector<float> cx  x center coordinate of cpgg limit cycle
 * \param std::vector<float> cy  y center coordinate of cpgg limit cycle
 * \return std::vector<std::pair<float, float>> XYdot a vector of size legs_number containing the
 pairs(xdot,ydot)
 */
    std::vector<std::pair<float, float>>
    CpgOpenLoop::computeXYdot(std::vector<float> X, std::vector<float> Y)
    {
        std::vector<std::pair<float, float>> XYdot;
        float x, y, Hcx, dHx, Hcy, dHy, Hc, xdot, ydot, Kterm;

        for (unsigned int i = 0; i < K_.size(); i++) {
            x = X[i];
            y = Y[i];
            Hcx = pow((x - cx_[i]) / a_, d_); /* x part of Hc*/
            dHx = d_ * pow((1 / a_), d_) * pow(x, d_ - 1); /* x derivative of Hcx*/

            Hcy = pow((y - cy_[i]) / b_, d_); /* y part of Hc*/
            dHy = d_ * pow((1 / b_), d_) * pow(y, d_ - 1); /* y derivative of Hcx*/

            Hc = Hcx + Hcy;

            xdot = -w_ * dHy + gammacpg_ * (1 - Hc) * dHx;
            // std::cout << xdot << std::endl;
            ydot = w_ * dHx + gammacpg_ * (1 - Hc) * dHy;

            Kterm = 0; /* coupling term which needs to be added to ydot*/
            for (unsigned int j = 0; j < K_[i].size(); j++) {
                Kterm += K_[i][j] * (Y[j] - cy_[j]);
                // std::cout << "K_[i][j] " << K_[i][j] << std::endl;
                // std::cout << " K_[i][j] * (Y[j] - cy_[j])] " << K_[i][j] * (Y[j] - cy_[j]) << std::endl;
            }
            // std::cout << " w * dHx    = " << w_ * dHx << '\n';
            // std::cout << "gammacpg_ * (1 - Hc) * dHy   = " << gammacpg_ * (1 - Hc) * dHy << '\n';
            ydot += lambda_ * Kterm;
            // std::cout << "lambda * Kterm    = " << lambda_ * Kterm << '\n';
            // std::cout << " ydot   = " << ydot << '\n';
            // std::cout << "xdot " << xdot << std::endl;
            XYdot.push_back(std::pair<float, float>(xdot, ydot));
        }
        return XYdot;
    }

    void CpgOpenLoop::computeTrajectory(double duration)
    {
        trajectoryX_.clear();
        trajectoryY_.clear();

        for (double t = 0.0; t < duration; t += step_) {
            std::vector<std::pair<float, float>> XYdot = CpgOpenLoop::computeXYdot(Xcommand_, Ycommand_);
            for (int i = 0; i < XYdot.size(); i++) {
                /*Integrate XYdot*/
                std::pair<float, float> xy = CpgOpenLoop::RK4(Xcommand_[i], Ycommand_[i], XYdot[i]);
                Xcommand_[i] = xy.first;
                // std::cout << "x " << xy.first << std::endl;
                Ycommand_[i] = xy.second;

                if (xy.first > safety_pos_thresh_) {
                    Xcommand_[i] = safety_pos_thresh_;
                }
                if (xy.first < -safety_pos_thresh_) {
                    Xcommand_[i] = -safety_pos_thresh_;
                }
                if (xy.second > safety_pos_thresh_) {
                    Ycommand_[i] = safety_pos_thresh_;
                }
                if (xy.second < -safety_pos_thresh_) {
                    Ycommand_[i] = -safety_pos_thresh_;
                }
                // std::cout << std::abs(xy.first) << " " << std::abs(xy.second) << std::endl;
                // if ((std::abs(xy.first) > cx0_[i] + 4 * std::abs(a_)) || (std::abs(xy.second) > cy0_[i] + 4 * std::abs(b_))) {
                //     std::cout << "INTEGRATION HAS DIVERGED 2 : reboot the node and use a bigger loop rate"
                //               << std::endl;
                //     integration_has_diverged_ = true;
                // }
                if (std::isnan(xy.first) || std::isnan(xy.second)) {
                    // std::cout << "INTEGRATION HAS DIVERGED : reboot the node and use a bigger loop rate"
                    // << std::endl;
                    integration_has_diverged_ = true;
                }
            }

            trajectoryX_.push_back(Xcommand_);
            trajectoryY_.push_back(Ycommand_);
        }
        //trajectory for X and Y calculated
        Xcommand_.clear();
        Ycommand_.clear();
        Xcommand_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Ycommand_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    std::vector<double>
    CpgOpenLoop::pos(double t)
    {
        std::vector<double> angles;
        int leg = 0;
        int elem = t / step_;
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
                angles.push_back(trajectoryX_[elem][leg_map_to_paper_[0]]);
                angles.push_back(trajectoryY_[elem][leg_map_to_paper_[0]]);
                angles.push_back(-trajectoryY_[elem][leg_map_to_paper_[0]]);
            case 1:
                angles.push_back(trajectoryX_[elem][leg_map_to_paper_[1]]);
                angles.push_back(trajectoryY_[elem][leg_map_to_paper_[1]]);
                angles.push_back(-trajectoryY_[elem][leg_map_to_paper_[1]]);
            case 2:
                angles.push_back(trajectoryX_[elem][leg_map_to_paper_[2]]);
                angles.push_back(trajectoryY_[elem][leg_map_to_paper_[2]]);
                angles.push_back(-trajectoryY_[elem][leg_map_to_paper_[2]]);
            case 3:
                angles.push_back(trajectoryX_[elem][leg_map_to_paper_[3]]);
                angles.push_back(trajectoryY_[elem][leg_map_to_paper_[3]]);
                angles.push_back(-trajectoryY_[elem][leg_map_to_paper_[3]]);
            case 4:
                angles.push_back(trajectoryX_[elem][leg_map_to_paper_[4]]);
                angles.push_back(trajectoryY_[elem][leg_map_to_paper_[4]]);
                angles.push_back(-trajectoryY_[elem][leg_map_to_paper_[4]]);
            case 5:
                angles.push_back(trajectoryX_[elem][leg_map_to_paper_[5]]);
                angles.push_back(trajectoryY_[elem][leg_map_to_paper_[5]]);
                angles.push_back(-trajectoryY_[elem][leg_map_to_paper_[5]]);
            }
            ++leg;
        }
    }
    /**
 * \brief Uses RK4 integation to obtain x,y from xdot, ydot, xprev, yprev. <br>!!!!! It diverges
 when euler_dt is too small !!!! <br> It would also be better to use Runge Kutta instead of Euler
 * \param float xprev : x angle at t-1
 * \param float yprev : y angle at t-1
 * \param std::pair<float, float> xydot : derivative of xprev, yprev obtained with computeXYdot
 * \return std::pair<float, float> return (x,y)
 */
    std::pair<float, float> CpgOpenLoop::RK4(float xprev, float yprev, std::pair<float, float> xydot)
    {
        float k1_x = xydot.first;
        float k2_x = xprev + k1_x * (rk_dt_ / 2);
        float k3_x = xprev + k2_x * (rk_dt_ / 2);
        float k4_x = xprev + k3_x * rk_dt_;
        float xcurr = xprev + (rk_dt_ / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);

        float k1_y = xydot.second;
        float k2_y = yprev + k1_y * (rk_dt_ / 2);
        float k3_y = yprev + k2_y * (rk_dt_ / 2);
        float k4_y = yprev + k3_y * rk_dt_;
        float ycurr = yprev + (rk_dt_ / 6) * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);

        return std::pair<float, float>(xcurr, ycurr);
    }

    std::pair<std::vector<float>, std::vector<float>>
    CpgOpenLoop::computeCPGcmd()
    {

        for (unsigned int i = 0; i < error_.size(); i++) {
            error_prev_[i] = error_[i];
        }

        /*compute X,Y derivatives*/
        std::vector<std::pair<float, float>> XYdot = CpgOpenLoop::computeXYdot(Xcommand_, Ycommand_);

        for (int i = 0; i < XYdot.size(); i++) {
            /*Integrate XYdot*/
            std::pair<float, float> xy = CpgOpenLoop::RK4(Xcommand_[i], Ycommand_[i], XYdot[i]);
            Xcommand_[i] = xy.first;
            // std::cout << "x " << xy.first << std::endl;
            Ycommand_[i] = xy.second;

            if (xy.first > safety_pos_thresh_) {
                Xcommand_[i] = safety_pos_thresh_;
            }
            if (xy.first < -safety_pos_thresh_) {
                Xcommand_[i] = -safety_pos_thresh_;
            }
            if (xy.second > safety_pos_thresh_) {
                Ycommand_[i] = safety_pos_thresh_;
            }
            if (xy.second < -safety_pos_thresh_) {
                Ycommand_[i] = -safety_pos_thresh_;
            }
            // std::cout << std::abs(xy.first) << " " << std::abs(xy.second) << std::endl;
            // if ((std::abs(xy.first) > cx0_[i] + 4 * std::abs(a_)) || (std::abs(xy.second) > cy0_[i] + 4 * std::abs(b_))) {
            //     std::cout << "INTEGRATION HAS DIVERGED 2 : reboot the node and use a bigger loop rate"
            //               << std::endl;
            //     integration_has_diverged_ = true;
            // }
            if (std::isnan(xy.first) || std::isnan(xy.second)) {
                // std::cout << "INTEGRATION HAS DIVERGED : reboot the node and use a bigger loop rate"
                // << std::endl;
                integration_has_diverged_ = true;
            }
        }

        return std::pair<std::vector<float>, std::vector<float>>(Xcommand_, Ycommand_);
    }

    std::vector<float>
    CpgOpenLoop::computeErrors(float roll, float pitch, std::vector<float> joints)
    {
        std::vector<float> command_final;
        if (integration_has_diverged_ == false) {
            for (unsigned int i = 0; i < legs_number_; i++) {

                error_[i] = (joints[i] - sign_[i] * Xcommand_[leg_map_to_paper_[i]]);
                error_derivated_[i] = (error_[i] - error_prev_[i]) / loop_rate_;
                error_integrated_[i] += error_[i];

                command_final.push_back(-kp_ * error_[i]); // - kd_ * error_derivated_[i]);

                error_[6 + i] = (joints[6 + i] - Ycommand_[leg_map_to_paper_[i]]);
                error_derivated_[6 + i] = (error_[6 + i] - error_prev_[6 + i]) / loop_rate_;
                error_integrated_[6 + i] += error_[6 + i];

                command_final.push_back(-kp_ * error_[6 + i]); // - kd_ * error_derivated_[6 + i] + kpitch_[i] * pitch + kroll_[i] * roll);

                error_[12 + i] = (joints[12 + i] - Ycommand_[leg_map_to_paper_[i]]);
                error_derivated_[12 + i] = (error_[12 + i] - error_prev_[12 + i]) / loop_rate_;
                error_integrated_[12 + i] += error_[12 + i];
                // std::cout << "error " << (-kp_ * error_[i]) << std::endl;
                command_final.push_back(-kp_ * error_[12 + i]); // - kd_ * error_derivated_[12 + i] + kpitch_[i] * pitch + kroll_[i] * roll);
            }
        }
        else {
            for (unsigned int i = 0; i < 3 * legs_number_; i++) {
                command_final.push_back(0);
            }
            command_final.push_back(1);
            // std::cout << "INTEGRATION HAS DIVERGED : sending 0 commands" << std::endl;
        }

        return command_final;
    }

    void CpgOpenLoop::set_parameters(const std::vector<double>& ctrl, std::vector<int> broken_legs)
    {
        _broken_legs = broken_legs;
        // std::cout << "aaaaa " << ctrl.size() << std::endl;
        if (ctrl.size() == 3) {
            w_ = ctrl[0];
            lambda_ = ctrl[1];
            gammacpg_ = ctrl[2];
        }
        else if (ctrl.size() == 5) {
            w_ = ctrl[0];
            lambda_ = ctrl[1];
            gammacpg_ = ctrl[2];
            a_ = ctrl[3];
            b_ = ctrl[4];
            // std::cout << b_ << std::endl;
        }
        else if (ctrl.size() == 6) {
            w_ = ctrl[0];
            lambda_ = ctrl[1];
            gammacpg_ = ctrl[2];
            a_ = ctrl[3];
            b_ = ctrl[4];
            d_ = ctrl[5];
        }
        else if (ctrl.size() == 36) {

            std::vector<std::vector<float>> Kk;
            std::vector<float> k;
            for (int i = 0; i < ctrl.size(); i++) {
                if (((i % 6) == 0) && i != 0) {
                    Kk.push_back(k);
                    k.clear();
                }
                k.push_back(ctrl[i]);
            }

            K_ = Kk;
            // K_ = createK();
        }
        else if (ctrl.size() == 41) {
            w_ = ctrl[0];
            lambda_ = ctrl[1];
            gammacpg_ = ctrl[2];
            a_ = ctrl[3];
            b_ = ctrl[4];

            std::vector<std::vector<float>> Kk;
            std::vector<float> k;
            for (int i = 5; i < ctrl.size(); i++) {
                if (((i % 6) == 0) && i != 0) {
                    Kk.push_back(k);
                    k.clear();
                }
                k.push_back(ctrl[i]);
            }

            K_ = Kk;
            // K_ = createK();
        }
    }

    const std::vector<double>& CpgOpenLoop::parameters() const
    {
        return ctrl_;
    };

} // namespace hexapod_controller
#endif