#pragma once

#include "bhs_game/agents/AGV.hpp"
#include "bhs_game/agents/Agent.hpp"
#include "bhs_game/testAgents/Agent.hpp"

#include "bhs_game/robotics/DifferentialDrive.hpp"
#include "bhs_game/utils/config.h"
#include "Eigen/Dense"
#include "bhs_game/robotics/PIDController.h"

#include <glm/mat3x3.hpp>
#include <glm/mat3x2.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <vector>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/trigonometric.hpp>

namespace bhs_game {
    class TMotorConfig {
    public:
        double max_linear_vel;
        double max_acceleration;
        double max_deceleration;
        TScalarDouble max_angular_vel;

//        MotorConfig() = default;

        TMotorConfig(double max_linear_vel, double maxAcceleration, double maxDeceleration, double wheel_radius)
                : max_linear_vel(max_linear_vel), max_acceleration(maxAcceleration), max_deceleration(maxDeceleration) {
            // convert from m/s to rad/s
            max_angular_vel = torch::scalar_tensor({max_linear_vel / wheel_radius}, TOptions(torch::kDouble, device));
        }
    };


    class TorchAAGV : public TorchAgent {
    private:
        // Variables
        int currentTaskId = -1;
        WorkState work_state = WorkState::IDLE;
        bool collided = false;
        Battery battery;
        TMotorConfig motor_config;
        TScalarDouble wheel_radius; // Radius of the wheels (m)
        TScalarDouble wheel_base; // Distance between the wheels (m)
        TensorXDouble velocity = torch::zeros({3, 1}, torch::TensorOptions().dtype(torch::kDouble).device(
                device)); // 0: linear velocity, 2: angular velocity
        TensorXDouble wheel_speed = torch::zeros({2}, torch::TensorOptions().dtype(torch::kDouble).device(
                device)); // 0: left wheel velocity (rad/s), 1: right wheel velocity (rad/s)
        std::vector<torch::Tensor> path = std::vector<torch::Tensor>();
        const torch::Tensor A = torch::eye(3, TOptions(torch::kDouble, device));
        torch::Tensor _B = torch::zeros({3, 2}, TOptions(torch::kDouble, device));
        torch::Tensor _slice = torch::tensor({{0, 0},
                                              {1, 0},
                                              {2, 1}}, TOptions(torch::kLong, device));
//        auto Ba = _B.packed_accessor<torch::kDouble, 2>();
    public:
        // Methods
        TorchAAGV(int uniqueId, Battery battery, TMotorConfig motor_config, float wheel_radius, float wheel_base);

        TorchAAGV(int uniqueId, const Config &config);

        ~TorchAAGV() override = default;

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

        [[nodiscard]] const TMotorConfig &getMotorConfig() const {
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

    class GLMAAGV : public GLMAgent {
    private:
        // Variables
        int currentTaskId = -1;
        WorkState work_state = WorkState::IDLE;
        bool collided = false;
        EBattery battery;
        EMotorConfig motor_config;
        double wheel_radius; // Radius of the wheels (m)
        double wheel_base; // Distance between the wheels (m)
        glm::dvec2 velocity = glm::dvec2(0.0); // 0: linear velocity, 2: angular velocity
        glm::dvec2 wheel_speed = glm::dvec2(0.0); // 0: left wheel velocity (rad/s), 1: right wheel velocity (rad/s)
        std::vector<Coordinates> path = std::vector<Coordinates>(500);
//        glm::dmat<3, 2, double, glm::defaultp> _B = glm::mat<3, 2, double, glm::defaultp>(0.0);

    public:
        // Methods
        GLMAAGV(int uniqueId, EBattery battery, EMotorConfig motor_config, float wheel_radius, float wheel_base);

        GLMAAGV(int uniqueId, const Config &config);

        ~GLMAAGV() override = default;

        void step(double dt) override;

        glm::dvec2 inverse_kinematics(bool set_result = false);

        glm::dvec2 forward_kinematics(bool set_result = false);

        bhs_game::GPIDController controller = GPIDController(1., 0., 0.1, 1., 0., 0.1);
        glm::dvec2 goal = glm::dvec2(0.0);

    private:

        // Methods
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

        [[nodiscard]] const glm::dvec2 &getVelocity() const {
            return velocity;
        }

        [[nodiscard]] const glm::dvec2 &getWheelSpeed() const {
            return wheel_speed;
        }

        [[nodiscard]] const std::vector<Coordinates> &getPath() const {
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

        void setLinearVelocity(double linear_velocity) {
            velocity[0] = linear_velocity;
        }

        void setAngularVelocity(double angular_velocity) {
            velocity[1] = angular_velocity;
        }

    private:

    };


} // bhs_game
