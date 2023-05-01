//
// Created by zartris on 4/8/23.
//

#ifndef BHS_GAME_AGV_H
#define BHS_GAME_AGV_H

#include "Agent.hpp"
#include "bhs_game/robotics/DifferentialDrive.hpp"
#include "bhs_game/utils/config.h"
#include "bhs_game/robotics/PIDController.h"

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
        double max_angular_vel;

//        MotorConfig() = default;

        MotorConfig(double max_linear_vel, double maxAcceleration, double maxDeceleration, double wheel_radius)
                : max_linear_vel(max_linear_vel), max_acceleration(maxAcceleration), max_deceleration(maxDeceleration) {
            // convert from m/s to rad/s
            max_angular_vel = max_linear_vel / wheel_radius;
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
        double wheel_radius; // Radius of the wheels (m)
        double wheel_base; // Distance between the wheels (m)
        glm::dvec2 velocity = glm::dvec2(0.0); // 0: linear velocity, 2: angular velocity
        glm::dvec2 wheel_speed = glm::dvec2(0.0); // 0: left wheel velocity (rad/s), 1: right wheel velocity (rad/s)
        std::vector<glm::dvec2> path = std::vector<glm::dvec2>(500);
        glm::dvec2 goal = glm::dvec2(0.0);

    public:
        // Methods
        AAGV(int uniqueId, Battery battery, MotorConfig motor_config, float wheel_radius, float wheel_base);

        AAGV(int uniqueId, const Config &config);

        ~AAGV() override = default;

        void step(double dt) override;

        glm::dvec2 inverse_kinematics(bool set_result = false);

        glm::dvec2 forward_kinematics(bool set_result = false);

        bhs_game::GPIDController controller = GPIDController(1., 0., 0.1, 1., 0., 0.1);

    private:

        // Methods
        void update_state(double dt);

        void update(double dt); // use step to update

        void requestReplanning();

        // Getter and setters
    public:
        void setPath(std::vector<glm::dvec2> path) {
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

        [[nodiscard]] const glm::dvec2 &getVelocity() const {
            return velocity;
        }

        [[nodiscard]] const glm::dvec2 &getWheelSpeed() const {
            return wheel_speed;
        }

        [[nodiscard]] const std::vector<glm::dvec2> &getPath() const {
            return path;
        }

        double getLinearVelocity() {
            return velocity[0];
        }

        double getAngularVelocity() {
            return velocity[1];
        }

        double getLeftWheelVelocity() {
            return wheel_speed[0];
        }

        double getRightWheelVelocity() {
            return wheel_speed[1];
        }

        [[nodiscard]] const glm::dvec2 &getGoal() const {
            return goal;
        }

        void setLinearVelocity(double linear_velocity) {
            velocity[0] = linear_velocity;
        }

        void setAngularVelocity(double angular_velocity) {
            velocity[1] = angular_velocity;
        }

        void setGoal(glm::dvec2 goal) {
            this->goal = goal;
        }

    private:

    };
} // bhs_game

#endif //BHS_GAME_AGV_H
