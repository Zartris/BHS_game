//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_AGV_H
#define BHS_GAME_AGV_H

#include "Agent.hpp"
#include "bhs_game/robotics/DifferentialDrive.hpp"

namespace bhs_game {
    enum class WorkState {
        IDLE,
        MOVING,
        CHARGING,
        WAITING
    };
    struct Battery {
        double current_battery;
        double max_battery;
    };
    struct MotorConfig {
        double max_speed;
        double max_acceleration;
        double max_deceleration;
        double max_wheel_speed;
    };


    class AAGV : public Agent {
    private:
        // Variables
        int currentTaskId = -1;
        WorkState work_state = WorkState::IDLE;
        bool collided = false;
        Battery battery;
        MotorConfig motor_config;
        double wheel_radius; // Radius of the wheels (m)
        double wheel_base; // Distance between the wheels (m)
        Eigen::Matrix<double, 3, 1> velocity = Eigen::Vector3d::Zero().transpose(); // 0: linear velocity, 2: angular velocity
        Eigen::Matrix<double, 2, 1> wheel_speed = Eigen::Vector2d::Zero().transpose(); // 0: left wheel velocity (rad/s), 1: right wheel velocity (rad/s)
        std::vector<Eigen::Vector2d> path = std::vector<Eigen::Vector2d>();

    public:
        // Methods
        AAGV(int uniqueId, Battery battery, MotorConfig motor_config, float wheel_radius, float wheel_base);

        void step(double dt) override;

        Eigen::Vector2d inverse_kinematics();

        Eigen::Matrix<double, 3, 1> forward_kinematics();

        void update(double dt);

    private:
        //  Methods
        void update_state(double dt);

        void requestReplanning();



// Getter and setters
    public:
        void setPath(std::vector<Eigen::Vector2d> path) {
            this->path = path;
        }

        void setWorkState(WorkState work_state) {
            this->work_state = work_state;
        }

        [[nodiscard]] WorkState getWorkState() const {
            return work_state;
        }

        [[nodiscard]] bool isCollided() const {
            return collided;
        }

        void setCollided(bool collided) {
            this->collided = collided;
        }

        [[nodiscard]] const Battery &getBattery() const {
            return battery;
        }

        [[nodiscard]] const MotorConfig &getMotorConfig() const {
            return motor_config;
        }

        [[nodiscard]] double getWheelRadius() const {
            return wheel_radius;
        }

        [[nodiscard]] double getWheelBase() const {
            return wheel_base;
        }

        [[nodiscard]] const Eigen::Matrix<double, 3, 1> &getVelocity() const {
            return velocity;
        }

        [[nodiscard]] const Eigen::Matrix<double, 2, 1> &getWheelSpeed() const {
            return wheel_speed;
        }

        [[nodiscard]] const std::vector<Eigen::Vector2d> &getPath() const {
            return path;
        }

        double &getLinearVelocity() {
            return velocity[0];
        }

        double &getAngularVelocity() {
            return velocity[2];
        }

        double &getLeftWheelVelocity() {
            return wheel_speed[0];
        }

        double &getRightWheelVelocity() {
            return wheel_speed[1];
        }

    private:

    };
} // bhs_game

#endif //BHS_GAME_AGV_H
