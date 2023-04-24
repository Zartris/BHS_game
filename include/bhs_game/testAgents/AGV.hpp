#ifndef BHS_GAME_EAGV_H
#define BHS_GAME_EAGV_H

#include "bhs_game/agents/AGV.hpp"
#include "bhs_game/agents/Agent.hpp"
#include "bhs_game/testAgents/Agent.hpp"

#include "bhs_game/robotics/DifferentialDrive.hpp"
#include "bhs_game/utils/config.h"
#include "Eigen/Dense"

namespace bhs_game {
    struct EBattery {
        double current_battery;
        double max_battery;
    };

    struct Coordinates {
        double x;
        double y;
    };

    class EMotorConfig {
    public:
        double max_linear_vel;
        double max_acceleration;
        double max_deceleration;
        double max_angular_vel;

        EMotorConfig(double max_linear_vel, double maxAcceleration, double maxDeceleration, double wheel_radius)
                : max_linear_vel(max_linear_vel), max_acceleration(maxAcceleration), max_deceleration(maxDeceleration) {
            // convert from m/s to rad/s
            max_angular_vel = max_linear_vel / wheel_radius;
        }
    };

    class EigenAAGV : public EigenAgent {
    private:
        // Variables
        int currentTaskId = -1;
        WorkState work_state = WorkState::IDLE;
        bool collided = false;
        EBattery battery;
        EMotorConfig motor_config;
        double wheel_radius; // Radius of the wheels (m)
        double wheel_base; // Distance between the wheels (m)
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // 0: linear velocity, 2: angular velocity
        Eigen::Vector2d wheel_speed = Eigen::Vector2d::Zero(); // 0: left wheel velocity (rad/s), 1: right wheel velocity (rad/s)
        std::vector<Coordinates> path = std::vector<Coordinates>(500);
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3, 2> _B = Eigen::Matrix<double, 3, 2>::Zero();

    public:
        // Methods
        EigenAAGV(int uniqueId, EBattery battery, EMotorConfig motor_config, float wheel_radius, float wheel_base);

        EigenAAGV(int uniqueId, const Config &config);

        ~EigenAAGV() override = default;

        void step(double dt) override;

        Eigen::Vector2d inverse_kinematics(bool set_result = false);

        Eigen::Vector3d forward_kinematics(bool set_result = false);

    private:

        //  Methods
        void update_state(double dt);

        void update(double dt); // use step to update

        void requestReplanning();

        // Getter and setters
    public:
        void setPath(std::vector<Coordinates> path) {
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

        [[nodiscard]] const EBattery &getBattery() const {
            return battery;
        }

        [[nodiscard]] const EMotorConfig &getMotorConfig() const {
            return motor_config;
        }

        [[nodiscard]] double getWheelRadius() const {
            return wheel_radius;
        }


        [[nodiscard]] double getWheelBase() const {
            return wheel_base;
        }

        [[nodiscard]] const Eigen::Vector3d &getVelocity() const {
            return velocity;
        }

        [[nodiscard]] const Eigen::Vector2d &getWheelSpeed() const {
            return wheel_speed;
        }

        [[nodiscard]] const std::vector<Coordinates> &getPath() const {
            return path;
        }

        double getLinearVelocity() {
            return velocity[0];
        }

        double getAngularVelocity() {
            return velocity[2];
        }

        double getLeftWheelVelocity() {
            return wheel_speed[0];
        }

        double getRightWheelVelocity() {
            return wheel_speed[1];
        }

        void setLinearVelocity(double linear_velocity) {
            velocity[0] = linear_velocity;
        }

        void setAngularVelocity(double angular_velocity) {
            velocity[2] = angular_velocity;
        }

//        void overwriteState(Eigen::Ref<Eigen::Vector3d> newState) {
//            state = Eigen::Map<Eigen::Vector3d>(newState.data());
//        }

    private:

    };
} // bhs_game

#endif //BHS_GAME_EAGV_H