#include "bhs_game/testAgents/AGV.hpp"
#include <glm/gtx/string_cast.hpp>

namespace bhs_game {
    TorchAAGV::TorchAAGV(int uniqueId, Battery battery, TMotorConfig motor_config, float wheel_radius, float wheel_base)
            :
            TorchAgent(uniqueId, "AAGV", true, false),
            battery(battery), motor_config(motor_config),
            wheel_radius(
                    torch::scalar_tensor(wheel_radius, TOptions(torch::kDouble, device))),
            wheel_base(torch::scalar_tensor(wheel_base, TOptions(torch::kDouble, device))) {
        // Implement the constructor logic here
//        printf("AGV created with id: %d\n", getUniqueId());
    }

    TorchAAGV::TorchAAGV(int uniqueId, const Config &config) :
            TorchAgent(uniqueId, "AAGV", true, false),
            battery{config.agv_params.max_battery, config.agv_params.max_battery},
            motor_config(TMotorConfig(config.agv_params.max_speed, config.agv_params.max_acceleration,
                                      config.agv_params.max_deceleration, config.agv_params.wheel_radius)),
            wheel_radius(torch::scalar_tensor(config.agv_params.wheel_radius,
                                              TOptions(torch::kDouble, device))),
            wheel_base(torch::scalar_tensor(config.agv_params.wheel_distance,
                                            TOptions(torch::kDouble, device))) {
    }

    void TorchAAGV::step(double dt) {
        // Implement the single step logic here
#if defined(VERBOSE) // defined(DEBUG) ||
        printf("AGV %d is stepping\n", getUniqueId());
#endif
        update(dt);
    }

    // TODO:: OPTIMIZE THIS
    torch::Tensor TorchAAGV::inverse_kinematics() {
        return DifferentialDrive::differential_drive_inverse_kinematics(getLinearVelocity(), getAngularVelocity(),
                                                                        wheel_radius, wheel_base, wheel_speed);
    }

    // TODO:: OPTIMIZE THIS
    torch::Tensor TorchAAGV::forward_kinematics() {
        return DifferentialDrive::differential_drive_forward_kinematics(getLeftWheelVelocity(), getRightWheelVelocity(),
                                                                        wheel_radius, wheel_base);
    }

    void TorchAAGV::update(double dt) {
        // Clamp is not what we are looking for as we want to scale it down to the max speed
        auto scale = motor_config.max_angular_vel / torch::abs(wheel_speed).max();
        // Scale is the factor we would have to multiply the wheel speed by to get the max speed
        // If the scale is greater than 1, we don't need to scale it down
        wheel_speed = wheel_speed * torch::min(scale, torch::scalar_tensor(1., TOptions(torch::kDouble, device)));

//        wheel_speed = torch::clamp(wheel_speed, -motor_config.max_angular_vel, motor_config.max_angular_vel);

        velocity = forward_kinematics();
        update_state(dt);
        wheel_speed = inverse_kinematics();
    }

    // TODO:: OPTIMIZE THIS
    void TorchAAGV::update_state(double dt) {
        auto deltaTime = torch::scalar_tensor(dt, TOptions(torch::kDouble, device));
        torch::Tensor orientation = getOrientation();

//        torch::Tensor A = torch::eye(3, torch::TensorOptions().dtype(torch::kDouble).device(device));
        // create slicing on gpu
//        torch::indexing::Slice _slice = torch::indexing::Slice();
        // TODO:: Very slow on gpu, OPTIMIZE THIS
        _B.index_put_({0, 0}, torch::cos(orientation) * deltaTime);
        _B.index_put_({1, 0}, torch::sin(orientation) * deltaTime);
        _B.index_put_({2, 1}, deltaTime);


//        torch::Tensor B = torch::tensor({{torch::cos(orientation) * deltaTime, 0.},
//                                         {torch::sin(orientation) * deltaTime, 0.},
//                                         {0.,                                  deltaTime}},
//                                        TOptions(torch::kDouble, device));
//        double data[] = {torch::cos(orientation).*dt, 0., torch::sin(orientation) * dt, 0., 0., dt};
//        torch::Tensor B = torch::from_blob(data, {3, 2}, torch::kDouble).to(device);


        torch::Tensor vel = torch::stack({getLinearVelocity(), getAngularVelocity()});

        torch::Tensor new_state = A.matmul(getState()) + _B.matmul(vel);
#if defined(VERBOSE)
        std::cout << "Old state: " << getState() << std::endl;
        std::cout << "New state: " << new_state << std::endl;
#endif
        setState(new_state);
    }

    void TorchAAGV::requestReplanning() {
        printf("Requesting replanning\n");
    }

    EigenAAGV::EigenAAGV(int uniqueId, EBattery battery, EMotorConfig motor_config, float wheel_radius,
                         float wheel_base)
            :
            EigenAgent(uniqueId, "AAGV", true, false),
            battery(battery), motor_config(motor_config),
            wheel_radius(wheel_radius),
            wheel_base(wheel_base) {
//        printf("AGV created with id: %d\n", getUniqueId());
    }

    EigenAAGV::EigenAAGV(int uniqueId, const Config &config) :
            EigenAgent(uniqueId, "AAGV", true, false),
            battery{config.agv_params.max_battery, config.agv_params.max_battery},
            motor_config(EMotorConfig(config.agv_params.max_speed, config.agv_params.max_acceleration,
                                      config.agv_params.max_deceleration, config.agv_params.wheel_radius)),
            wheel_radius(config.agv_params.wheel_radius),
            wheel_base(config.agv_params.wheel_distance) {
    }

    void EigenAAGV::step(double dt) {
#if defined(VERBOSE)
        printf("AGV %d is stepping\n", getUniqueId());
#endif
        update(dt);
    }

    Eigen::Vector2d EigenAAGV::inverse_kinematics(bool set_result) {
        Eigen::Vector2d result = DifferentialDrive::differential_drive_inverse_kinematics(getLinearVelocity(),
                                                                                          getAngularVelocity(),
                                                                                          wheel_radius, wheel_base);
        if (set_result) {
            wheel_speed = result;
#ifdef VERBOSE
            printf("AGV %d: left wheel speed: %f, right wheel speed: %f\n", getUniqueId(), result(0), result(1));
#endif
        }
        return result;
    }

    Eigen::Vector3d EigenAAGV::forward_kinematics(bool set_result) {
        Eigen::Vector3d result = DifferentialDrive::differential_drive_forward_kinematics(getLeftWheelVelocity(),
                                                                                          getRightWheelVelocity(),
                                                                                          wheel_radius, wheel_base);
        if (set_result) {
            velocity = result;
        }
        return result;
    }

    void EigenAAGV::update(double dt) {
        auto scale = motor_config.max_angular_vel / wheel_speed.array().abs().maxCoeff();
        wheel_speed = wheel_speed * std::min(scale, 1.0);
#ifdef VERBOSE
        printf("afterclip AGV %d: left wheel speed: %f, right wheel speed: %f\n", getUniqueId(), wheel_speed(0), wheel_speed(1));
#endif
        velocity = forward_kinematics();
        update_state(dt);
        wheel_speed = inverse_kinematics();
    }

    void EigenAAGV::update_state(double dt) {
        double orientation = getOrientation();

        _B(0, 0) = std::cos(orientation) * dt;
        _B(1, 0) = std::sin(orientation) * dt;
        _B(2, 1) = dt;

        Eigen::Vector2d vel(getLinearVelocity(), getAngularVelocity());
        Eigen::Vector3d new_state(A * getState() + _B * vel);

        setState(new_state);
    }

    void EigenAAGV::requestReplanning() {
        printf("Requesting replanning\n");
    }


    GLMAAGV::GLMAAGV(int uniqueId, EBattery battery, EMotorConfig motor_config, float wheel_radius,
                     float wheel_base)
            :
            GLMAgent(uniqueId, "AAGV", true, false),
            battery(battery), motor_config(motor_config),
            wheel_radius(wheel_radius),
            wheel_base(wheel_base) {
    }

    GLMAAGV::GLMAAGV(int uniqueId, const Config &config) :
            GLMAgent(uniqueId, "AAGV", true, false),
            battery{config.agv_params.max_battery, config.agv_params.max_battery},
            motor_config(EMotorConfig(config.agv_params.max_speed, config.agv_params.max_acceleration,
                                      config.agv_params.max_deceleration, config.agv_params.wheel_radius)),
            wheel_radius(config.agv_params.wheel_radius),
            wheel_base(config.agv_params.wheel_distance) {
    }

    void GLMAAGV::step(const double dt) {
        glm::dvec2 control = controller.update(getState(), goal);
        setLinearVelocity(control[0]);
        setAngularVelocity(control[1]);
        inverse_kinematics(true);

        update(dt);
    }

    glm::dvec2 GLMAAGV::inverse_kinematics(bool set_result) {
        glm::dvec2 result = DifferentialDrive::differential_drive_inverse_kinematics_glm(getLinearVelocity(),
                                                                                         getAngularVelocity(),
                                                                                         wheel_radius, wheel_base);
        if (set_result) {
            wheel_speed = result;
        }
        return result;
    }

    glm::dvec2 GLMAAGV::forward_kinematics(bool set_result) {
        glm::dvec2 result = DifferentialDrive::differential_drive_forward_kinematics_glm(getLeftWheelVelocity(),
                                                                                         getRightWheelVelocity(),
                                                                                         wheel_radius, wheel_base);
        if (set_result) {
            velocity = result;
        }
        return result;
    }

    void GLMAAGV::update(const double dt) {
        auto scale = motor_config.max_angular_vel / glm::max(glm::abs(wheel_speed[0]), glm::abs(wheel_speed[1]));
        wheel_speed = wheel_speed * glm::min(scale, 1.0);
        forward_kinematics(true);
        update_state(dt);
        inverse_kinematics(true);
    }

    void GLMAAGV::update_state(const double dt) {
        double orientation = getOrientation();

        glm::dmat3x3 A(1, 0, 0,
                       0, 1, 0,
                       0, 0, 1);

        glm::dmat2x3 B(glm::cos(orientation) * dt, glm::sin(orientation) * dt, 0,
                       0, 0, dt);

        glm::dvec3 new_state = A * getState() + B * getVelocity();

        setState(new_state);
    }

    void GLMAAGV::requestReplanning() {
        printf("Requesting replanning\n");
    }

} // bhs_game
