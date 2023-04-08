//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_DIFFERENTIALDRIVE_H
#define BHS_GAME_DIFFERENTIALDRIVE_H

#include "Eigen/Dense"

class DifferentialDrive {
public:
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
    differential_drive_inverse_kinematics(double v, double omega, double wheel_radius, double wheel_distance) {
        // Calculate the linear velocity of each wheel
        double v_right = v + (omega * wheel_distance) / 2;
        double v_left = v - (omega * wheel_distance) / 2;

        // Convert linear velocities to angular velocities
        double w_right = v_right / wheel_radius;
        double w_left = v_left / wheel_radius;

        // Return the result as an Eigen::Vector2d
        return Eigen::Vector2d(w_left, w_right);
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
    static Eigen::Matrix<double, 3, 1>
    differential_drive_forward_kinematics(double w_left, double w_right, double wheel_radius, double wheel_distance) {
        // Calculate the linear velocity of the left and right wheel
        double v_right = w_right * wheel_radius;
        double v_left = w_left * wheel_radius;

        // Calculate the linear and angular velocity of the robot
        double v = (v_right + v_left) / 2.0;
        double omega = (v_right - v_left) / wheel_distance;

        // Return the result as an Eigen::Matrix<double, 3, 1>
        return {v, 0, omega};
    }
};


#endif //BHS_GAME_DIFFERENTIALDRIVE_H
