#include "bhs_game/testAgents/AGV.hpp"

namespace bhs_game {

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

} // bhs_game
