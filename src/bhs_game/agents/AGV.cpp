//
// Created by zartris on 4/8/23.
//

#include "bhs_game/agents/AGV.hpp"

namespace bhs_game {
    AAGV::AAGV(int uniqueId, Battery battery, MotorConfig motor_config, float wheel_radius,
               float wheel_base)
            :
            Agent(uniqueId, "AAGV", true, false),
            battery(battery), motor_config(motor_config),
            wheel_radius(wheel_radius),
            wheel_base(wheel_base) {
    }

    AAGV::AAGV(int uniqueId, const Config &config) :
            Agent(uniqueId, "AAGV", true, false),
            battery{config.agv_params.max_battery, config.agv_params.max_battery},
            motor_config(MotorConfig(config.agv_params.max_speed, config.agv_params.max_acceleration,
                                     config.agv_params.max_deceleration, config.agv_params.wheel_radius)),
            wheel_radius(config.agv_params.wheel_radius),
            wheel_base(config.agv_params.wheel_distance) {
    }

    void AAGV::step(const double dt) {
        glm::dvec2 control = controller.update(getState(), goal);
        setLinearVelocity(control[0]);
        setAngularVelocity(control[1]);
        inverse_kinematics(true);

        update(dt);
    }

    glm::dvec2 AAGV::inverse_kinematics(bool set_result) {
        glm::dvec2 result = DifferentialDrive::differential_drive_inverse_kinematics_glm(getLinearVelocity(),
                                                                                         getAngularVelocity(),
                                                                                         wheel_radius, wheel_base);
        if (set_result) {
            wheel_speed = result;
        }
        return result;
    }

    glm::dvec2 AAGV::forward_kinematics(bool set_result) {
        glm::dvec2 result = DifferentialDrive::differential_drive_forward_kinematics_glm(getLeftWheelVelocity(),
                                                                                         getRightWheelVelocity(),
                                                                                         wheel_radius, wheel_base);
        if (set_result) {
            velocity = result;
        }
        return result;
    }

    void AAGV::update(const double dt) {
        auto scale = motor_config.max_angular_vel / glm::max(glm::abs(wheel_speed[0]), glm::abs(wheel_speed[1]));
        wheel_speed = wheel_speed * glm::min(scale, 1.0);
        forward_kinematics(true);
        update_state(dt);
        inverse_kinematics(true);
    }

    void AAGV::update_state(const double dt) {
        double orientation = getOrientation();

        glm::dmat3x3 A(1, 0, 0,
                       0, 1, 0,
                       0, 0, 1);

        glm::dmat2x3 B(glm::cos(orientation) * dt, glm::sin(orientation) * dt, 0,
                       0, 0, dt);

        glm::dvec3 new_state = A * getState() + B * getVelocity();

        setState(new_state);
    }

    void AAGV::requestReplanning() {
        printf("Requesting replanning\n");
    }


} // bhs_game
