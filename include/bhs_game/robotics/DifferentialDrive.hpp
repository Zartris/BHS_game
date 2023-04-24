#ifndef BHS_GAME_DIFFERENTIALDRIVE_H
#define BHS_GAME_DIFFERENTIALDRIVE_H

#include "torch/torch.h"
#include "bhs_game/utils/tensor_alias.h"
#include "bhs_game/utils/global_device.h"
#include "Eigen/Core"

class DifferentialDrive {
public:
    /**
     * Calculate the wheel angular velocities for a differential drive robot.
     *
     * @param v             Linear velocity of the robot (m/s)
     * @param omega         Angular velocity of the robot (rad/s)
     * @param wheel_radius  Radius of the wheels (m)
     * @param wheel_distance Distance between the wheels (m)
     * @return torch::Tensor containing the left wheel angular velocity (rad/s) and right wheel angular velocity (rad/s)
     */
    static torch::Tensor
    differential_drive_inverse_kinematics(TScalarDouble v, TScalarDouble omega, TScalarDouble wheel_radius,
                                          TScalarDouble wheel_distance, TensorXDouble out) {
        torch::Tensor two = torch::scalar_tensor({2.0}, TOptions(torch::kDouble, device));
        torch::Tensor v_right = v + (omega * wheel_distance) / two;
        torch::Tensor v_left = v - (omega * wheel_distance) / two;

        torch::Tensor w_right = v_right / wheel_radius;
        torch::Tensor w_left = v_left / wheel_radius;
        out.index_put_({0}, w_left);
        out.index_put_({1}, w_right);
        return out;
    }

    /**
 * Calculate the wheel angular velocities for a differential drive robot.
 *
 * @param v             Linear velocity of the robot (m/s)
 * @param omega         Angular velocity of the robot (rad/s)
 * @param wheel_radius  Radius of the wheels (m)
 * @param wheel_distance Distance between the wheels (m)
 * @return Eigen::Vector2d containing the left wheel angular velocity (rad/s) and right wheel angular velocity (rad/s)
 */
    static Eigen::Vector2d
    differential_drive_inverse_kinematics(double v, double omega, double wheel_radius,
                                          double wheel_distance) {
        double v_right = v + (omega * wheel_distance) / 2.0;
        double v_left = v - (omega * wheel_distance) / 2.0;

        double w_right = v_right / wheel_radius;
        double w_left = v_left / wheel_radius;

        return Eigen::Vector2d(w_left, w_right);
    }

    /**
     * Calculate the linear and angular velocities of a differential drive robot.
     *
     * @param w_left        Left wheel angular velocity (rad/s)
     * @param w_right       Right wheel angular velocity (rad/s)
     * @param wheel_radius  Radius of the wheels (m)
     * @param wheel_distance Distance between the wheels (m)
     * @return torch::Tensor containing the linear velocity of the robot (m/s), 0, and angular velocity of the robot (rad/s)
     */
    static TensorXDouble
    differential_drive_forward_kinematics(TScalarDouble w_left, TScalarDouble w_right, TScalarDouble wheel_radius,
                                          TScalarDouble wheel_distance) {
        TScalarDouble v_right = w_right * wheel_radius;
        TScalarDouble v_left = w_left * wheel_radius;

        TScalarDouble v = (v_right + v_left) / 2.0;
        TScalarDouble omega = (v_right - v_left) / wheel_distance;

        // THIS WORKS
//        TensorXDouble out = torch::zeros({3, 1}, TOptions(torch::kDouble, device));
//
//        out.index_put_({0}, v).index_put_({2}, omega);


        TensorXDouble out = torch::stack({v, torch::tensor(0, TOptions(torch::kDouble, device)), omega}, 0).unsqueeze(
                1);

        return out;
    }

    /**
 * Calculate the linear and angular velocities of a differential drive robot.
 *
 * @param w_left        Left wheel angular velocity (rad/s)
 * @param w_right       Right wheel angular velocity (rad/s)
 * @param wheel_radius  Radius of the wheels (m)
 * @param wheel_distance Distance between the wheels (m)
 * @return Eigen::Vector2d containing the linear velocity of the robot (m/s) and angular velocity of the robot (rad/s)
 */
    static Eigen::Vector3d
    differential_drive_forward_kinematics(double w_left, double w_right, double wheel_radius,
                                          double wheel_distance) {
        double v_right = w_right * wheel_radius;
        double v_left = w_left * wheel_radius;

        double v = (v_right + v_left) / 2.0;
        double omega = (v_right - v_left) / wheel_distance;
        return Eigen::Vector3d(v, 0, omega);
//        return Eigen::Vector2d(v, omega);
    }
};

#endif //BHS_GAME_DIFFERENTIALDRIVE_H
