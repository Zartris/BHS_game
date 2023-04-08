//
// Created by zartris on 4/8/23.
//

#include "bhs_game/agents/AGV.hpp"

namespace bhs_game {
    AAGV::AAGV(int uniqueId, Battery battery, MotorConfig motor_config, float wheel_radius, float wheel_base) : Agent(
            uniqueId) {
        this->battery = battery;
        this->motor_config = motor_config;
        this->wheel_radius = wheel_radius;
        this->wheel_base = wheel_base;
    }

    void AAGV::step(double dt) {
        // Implement the single step logic here



        update(dt);
    }

    Eigen::Vector2d AAGV::inverse_kinematics() {
        return DifferentialDrive::differential_drive_inverse_kinematics(getLinearVelocity(), getAngularVelocity(),
                                                                        wheel_radius, wheel_base);
    }

    Eigen::Matrix<double, 3, 1> AAGV::forward_kinematics() {
        return DifferentialDrive::differential_drive_forward_kinematics(getLeftWheelVelocity(), getRightWheelVelocity(),
                                                                        wheel_radius, wheel_base);
    }

    void AAGV::update(double dt) {
        wheel_speed = wheel_speed.cwiseMax(-motor_config.max_wheel_speed).cwiseMin(motor_config.max_wheel_speed);

        velocity = forward_kinematics();
        update_state(dt);
        wheel_speed = inverse_kinematics();
    }

    void AAGV::update_state(double dt) {
        double orientation = getOrientation();

        Eigen::Matrix<double, 3, 3> A;
        A.setIdentity();

        Eigen::Matrix<double, 3, 2> B;
        B << std::cos(orientation) * dt, 0,
                std::sin(orientation) * dt, 0,
                0, dt;

        Eigen::Matrix<double, 2, 1> vel;
        vel << getLinearVelocity(), getAngularVelocity();
        Eigen::Matrix<double, 3, 1> new_state = A * getState() + B * vel;
        setState(new_state);
    }

    void AAGV::requestReplanning() {
        printf("Requesting replanning\n");
    }

} // bhs_game