//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_AGV_H
#define BHS_GAME_AGV_H

#include "Agent.hpp"
#include "bhs_game/robotics/DifferentialDrive.hpp"
#include "bhs_game/utils/config.h"
#include "bhs_game/utils/tensor_alias.h"

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

    class MotorConfig {
    public:
        double max_linear_vel;
        double max_acceleration;
        double max_deceleration;
        TScalarDouble max_angular_vel;

//        MotorConfig() = default;

        MotorConfig(double max_linear_vel, double maxAcceleration, double maxDeceleration, double wheel_radius)
                : max_linear_vel(max_linear_vel), max_acceleration(maxAcceleration), max_deceleration(maxDeceleration) {
            // convert from m/s to rad/s
            max_angular_vel = torch::scalar_tensor({max_linear_vel / wheel_radius}, TOptions(torch::kDouble, device));
        }
    };


    class AAGV : public Agent {
    private:
        // Variables
        int currentTaskId = -1;
        WorkState work_state = WorkState::IDLE;
        bool collided = false;
        Battery battery;
        MotorConfig motor_config;
        TScalarDouble wheel_radius; // Radius of the wheels (m)
        TScalarDouble wheel_base; // Distance between the wheels (m)
        TensorXDouble velocity = torch::zeros({3, 1}, torch::TensorOptions().dtype(torch::kDouble).device(
                device)); // 0: linear velocity, 2: angular velocity
        TensorXDouble wheel_speed = torch::zeros({2}, torch::TensorOptions().dtype(torch::kDouble).device(
                device)); // 0: left wheel velocity (rad/s), 1: right wheel velocity (rad/s)
        std::vector<torch::Tensor> path = std::vector<torch::Tensor>();
        const torch::Tensor A = torch::eye(3, TOptions(torch::kDouble, device));
        torch::Tensor B = torch::zeros({3, 2}, TOptions(torch::kDouble, device));

    public:
        // Methods
        AAGV(int uniqueId, Battery battery, MotorConfig motor_config, float wheel_radius, float wheel_base);

        AAGV(int uniqueId, const Config &config);

        ~AAGV() override = default;

        void step(double dt) override;

        torch::Tensor inverse_kinematics();

        torch::Tensor forward_kinematics();

    private:

        //  Methods
        void update_state(double dt);

        void update(double dt); // use step to update

        void requestReplanning();



// Getter and setters
    public:
        void setPath(std::vector<torch::Tensor> path) {
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

        [[nodiscard]] TScalarDouble getWheelRadius() const {
            return wheel_radius;
        }

        [[nodiscard]] TScalarDouble getWheelBase() const {
            return wheel_base;
        }

        [[nodiscard]] const TensorXDouble &getVelocity() const {
            return velocity;
        }

        [[nodiscard]] const TensorXDouble &getWheelSpeed() const {
            return wheel_speed;
        }

        [[nodiscard]] const std::vector<TensorXDouble> &getPath() const {
            return path;
        }

        TScalarDouble getLinearVelocity() {
            return velocity[0];
        }

        TScalarDouble getAngularVelocity() {
            return velocity[2];
        }

        TScalarDouble getLeftWheelVelocity() {
            return wheel_speed[0];
        }

        TScalarDouble getRightWheelVelocity() {
            return wheel_speed[1];
        }

        void setLinearVelocity(const TScalarDouble &linear_velocity) {
            velocity[0].copy_(linear_velocity);
        }

        void setAngularVelocity(const TScalarDouble &angular_velocity) {
            velocity[2].copy_(angular_velocity);
        }

        void overwriteState(TensorXDouble state) {
            this->state = state;
        }

    private:

    };
} // bhs_game

#endif //BHS_GAME_AGV_H
