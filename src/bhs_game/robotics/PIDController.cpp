//
// Created by zartris on 4/11/23.
//

#include "bhs_game/robotics/PIDController.h"

Tensor2Double bhs_game::PIDController::update(const Tensor3Double &currentState, const Tensor2Double &desiredState) {
    // Slice the desired_position tensor to get the first two columns
    torch::Tensor desired_position_slice = desiredState.slice(1, 0, 2); // [:, :2]

    // Slice the current_states tensor to get the first two elements of the first dimension
    torch::Tensor current_states_slice = currentState.squeeze(2).slice(1, 0, 1);

    torch::Tensor position_error = desired_position_slice - current_states_slice;
#ifdef VERBOSE
    std::cout << "Position error:\n" << position_error << std::endl;
#endif

    // This code snippet calculates the norm along dimension 1 (columns) with keepdim set to true, which maintains the original number of dimensions. This will give you a {5, 1} shaped tensor for distance_error.
    torch::Tensor distance_error = torch::norm(position_error, 2, 1, false);
#ifdef VERBOSE
    std::cout << "Distance error:\n" << distance_error << std::endl;
#endif
    v_error_intergral += distance_error;
    torch::Tensor v_error_derivative = distance_error - v_error_prev;
    torch::Tensor desiredVelocity =
            vel_pid.kp * distance_error + vel_pid.ki * v_error_intergral + vel_pid.kd * v_error_derivative;
    v_error_prev = distance_error;


    // Compute the desired_heading
    torch::Tensor desired_heading = torch::atan2(position_error.slice(1, 1, 2), position_error.slice(1, 0, 1));

    // Compute the angular_error
    torch::Tensor current_heading = currentState.squeeze(2).slice(1, 2, 3);
    torch::Tensor angular_error = desired_heading - current_heading;

    // Wrap the angular_error to the range [-pi, pi]
    angular_error = torch::atan2(torch::sin(angular_error), torch::cos(angular_error)).squeeze(1);

    // Print the angular_error tensor
#ifdef VERBOSE
    std::cout << "Angular error:\n" << angular_error << std::endl;
#endif
    a_error_intergral += angular_error;
    torch::Tensor a_error_derivative = angular_error - a_error_prev;
    torch::Tensor desiredAngularVelocity =
            ang_pid.kp * angular_error + ang_pid.ki * a_error_intergral + ang_pid.kd * a_error_derivative;
    a_error_prev = angular_error;
    // Instead of stack which would get us {2, 5} we want {5, 2} so we use unsqueeze to get {5,1} and then cat on last axis
    return torch::cat({desiredVelocity.unsqueeze(1), desiredAngularVelocity.unsqueeze(1)}, 1);
}

TScalarDouble bhs_game::PIDController::getPositionError(Tensor3Double currentState, Tensor2Double desiredState) {
    // Slice the desired_position tensor to get the first two columns
    torch::Tensor desired_position_slice = desiredState.slice(1, 0, 2);

    // Slice the current_states tensor to get the first two elements of the first dimension
    torch::Tensor current_states_slice = currentState.slice(1, 0, 2).slice(2, 0, 1).squeeze(2);

    return desired_position_slice - current_states_slice;
}

TScalarDouble bhs_game::PIDController::getAngleError(Tensor3Double currentState, Tensor2Double desiredState) {
    // Slice the desired_position tensor to get the first two columns
    torch::Tensor desired_position_slice = desiredState.slice(1, 0, 2);

    // Slice the current_states tensor to get the first two elements of the first dimension
    torch::Tensor current_states_slice = currentState.slice(1, 0, 2).slice(2, 0, 1).squeeze(2);

    torch::Tensor position_error = desired_position_slice - current_states_slice;

    // Compute the desired_heading
    torch::Tensor desired_heading = torch::atan2(position_error.slice(1, 1, 2), position_error.slice(1, 0, 1));

    // Compute the angular_error
    torch::Tensor current_heading = currentState.slice(1, 2, 3).slice(2, 0, 1).squeeze(2);
    torch::Tensor angular_error = desired_heading - current_heading;

    // Wrap the angular_error to the range [-pi, pi]
    angular_error = torch::atan2(torch::sin(angular_error), torch::cos(angular_error));


    return angular_error;
}

void bhs_game::PIDController::reset() {

}
