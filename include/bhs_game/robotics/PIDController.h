//
// Created by zartris on 4/11/23.
//

#ifndef BHS_GAME_PIDCONTROLLER_H
#define BHS_GAME_PIDCONTROLLER_H

# include "bhs_game/utils/tensor_alias.h"
# include "torch/torch.h"
# include "bhs_game/utils/global_device.h"
# include "Eigen/Dense"
# include "Eigen/Core"

namespace bhs_game {
    struct PIDValues {
        TScalarDouble kp;
        TScalarDouble ki;
        TScalarDouble kd;

        PIDValues &setDevice(torch::Device d) {
            kp.to(d);
            ki.to(d);
            kd.to(d);
            return *this;
        }
    };

    class PIDController {
    private:
        PIDValues vel_pid;
        PIDValues ang_pid;
        TScalarDouble v_error_intergral;
        TScalarDouble v_error_prev;
        TScalarDouble a_error_intergral;
        TScalarDouble a_error_prev;

    public:
        PIDController(PIDValues vel_pid, PIDValues angle_pid, int N = 1) : vel_pid(vel_pid), ang_pid(angle_pid) {
            v_error_intergral = torch::zeros({N}, TOptions(torch::kDouble, device));
            v_error_prev = torch::zeros({N}, TOptions(torch::kDouble, device));
            a_error_intergral = torch::zeros({N}, TOptions(torch::kDouble, device));
            a_error_prev = torch::zeros({N}, TOptions(torch::kDouble, device));
        }

        PIDController(double vkp, double vki, double vkd, double akp, double aki, double akd, int N = 1) {
            vel_pid = PIDValues{torch::scalar_tensor(vkp, TOptions(torch::kDouble, device)),
                                torch::scalar_tensor(vki, TOptions(torch::kDouble, device)),
                                torch::scalar_tensor(vkd, TOptions(torch::kDouble, device))
            };
            ang_pid = PIDValues{torch::scalar_tensor(akp, TOptions(torch::kDouble, device)),
                                torch::scalar_tensor(aki, TOptions(torch::kDouble, device)),
                                torch::scalar_tensor(akd, TOptions(torch::kDouble, device))
            };
            v_error_intergral = torch::zeros({N}, TOptions(torch::kDouble, device));
            v_error_prev = torch::zeros({N}, TOptions(torch::kDouble, device));
            a_error_intergral = torch::zeros({N}, TOptions(torch::kDouble, device));
            a_error_prev = torch::zeros({N}, TOptions(torch::kDouble, device));
        }


        Tensor2Double update(const Tensor3Double &currentState, const Tensor3Double &desiredState);

        TScalarDouble getPositionError(Tensor3Double currentState, Tensor3Double desiredState);

        TScalarDouble getAngleError(Tensor3Double currentState, Tensor3Double desiredState);

        void reset();

    };


    struct EPIDValues {
        double kp;
        double ki;
        double kd;
    };

    class EPIDController {
    private:
        EPIDValues vel_pid;
        EPIDValues ang_pid;
        Eigen::VectorXd v_error_integral;
        Eigen::VectorXd v_error_prev;
        Eigen::VectorXd a_error_integral;
        Eigen::VectorXd a_error_prev;

    public:
        EPIDController(EPIDValues vel_pid, EPIDValues angle_pid, int N = 1) : vel_pid(vel_pid), ang_pid(angle_pid) {
            v_error_integral = Eigen::VectorXd::Zero(N);
            v_error_prev = Eigen::VectorXd::Zero(N);
            a_error_integral = Eigen::VectorXd::Zero(N);
            a_error_prev = Eigen::VectorXd::Zero(N);
        }

        EPIDController(double vkp, double vki, double vkd, double akp, double aki, double akd, int N = 1) {
            vel_pid = EPIDValues{vkp, vki, vkd};
            ang_pid = EPIDValues{akp, aki, akd};
            v_error_integral = Eigen::VectorXd::Zero(N);
            v_error_prev = Eigen::VectorXd::Zero(N);
            a_error_integral = Eigen::VectorXd::Zero(N);
            a_error_prev = Eigen::VectorXd::Zero(N);
        }

        Eigen::MatrixXd update(const Eigen::Matrix<double, 3, Eigen::Dynamic> &currentState,
                               const Eigen::Matrix<double, 2, Eigen::Dynamic> &desiredState);

        double getPositionError(Eigen::Matrix<double, 3, Eigen::Dynamic> currentState,
                                Eigen::Matrix<double, 2, Eigen::Dynamic> desiredState);

        double getAngleError(Eigen::Matrix<double, 3, Eigen::Dynamic> currentState,
                             Eigen::Matrix<double, 2, Eigen::Dynamic> desiredState);

        void reset();

    };
} // namespace bhs_game

#endif //BHS_GAME_PIDCONTROLLER_H
