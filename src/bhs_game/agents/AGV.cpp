//
// Created by zartris on 4/8/23.
//

#include "bhs_game/agents/AGV.hpp"

namespace bhs_game {
    const torch::Tensor t_one = torch::scalar_tensor(1.0, TOptions(torch::kDouble, device));
//    const torch::Tensor A = torch::eye(3, TOptions(torch::kDouble, device));

    AAGV::AAGV(int uniqueId, Battery battery, MotorConfig motor_config, float wheel_radius, float wheel_base) :
            Agent(uniqueId, "AAGV", true, false),
            battery(battery), motor_config(motor_config),
            wheel_radius(
                    torch::scalar_tensor(wheel_radius, TOptions(torch::kDouble, device))),
            wheel_base(torch::scalar_tensor(wheel_base, TOptions(torch::kDouble, device))) {
        // Implement the constructor logic here
//        printf("AGV created with id: %d\n", getUniqueId());
    }

    AAGV::AAGV(int uniqueId, const Config &config) :
            Agent(uniqueId, "AAGV", true, false),
            battery{config.agv_params.max_battery, config.agv_params.max_battery},
            motor_config(MotorConfig(config.agv_params.max_speed, config.agv_params.max_acceleration,
                                     config.agv_params.max_deceleration, config.agv_params.wheel_radius)),
            wheel_radius(torch::scalar_tensor(config.agv_params.wheel_radius,
                                              TOptions(torch::kDouble, device))),
            wheel_base(torch::scalar_tensor(config.agv_params.wheel_distance,
                                            TOptions(torch::kDouble, device))) {
    }

    void AAGV::step(double dt) {
        // Implement the single step logic here
#if defined(VERBOSE) // defined(DEBUG) ||
        printf("AGV %d is stepping\n", getUniqueId());
#endif
        update(dt);
    }

    // TODO:: OPTIMIZE THIS
    torch::Tensor AAGV::inverse_kinematics() {
        return DifferentialDrive::differential_drive_inverse_kinematics(getLinearVelocity(), getAngularVelocity(),
                                                                        wheel_radius, wheel_base, wheel_speed);
    }

    // TODO:: OPTIMIZE THIS
    torch::Tensor AAGV::forward_kinematics() {
        return DifferentialDrive::differential_drive_forward_kinematics(getLeftWheelVelocity(), getRightWheelVelocity(),
                                                                        wheel_radius, wheel_base);
    }

    void AAGV::update(double dt) {
        // Clamp is not what we are looking for as we want to scale it down to the max speed
        auto scale = motor_config.max_angular_vel / torch::abs(wheel_speed).max();
        // Scale is the factor we would have to multiply the wheel speed by to get the max speed
        // If the scale is greater than 1, we don't need to scale it down
        wheel_speed = wheel_speed * torch::min(scale, t_one);

//        wheel_speed = torch::clamp(wheel_speed, -motor_config.max_angular_vel, motor_config.max_angular_vel);

        velocity = forward_kinematics();
        update_state(dt);
        wheel_speed = inverse_kinematics();
    }

    // TODO:: OPTIMIZE THIS
    void AAGV::update_state(double dt) {
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

    void AAGV::requestReplanning() {
        printf("Requesting replanning\n");
    }


} // bhs_game
