//
// Created by zartris on 4/11/23.
//

#include "bhs_game/robotics/PIDController.h"
#include "glm/geometric.hpp"
#include <cmath>

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
#ifdef VERBOSE
    std::cout << "Angular error:\n" << angular_error << std::endl;
#endif
    // Wrap the angular_error to the range [-pi, pi]
    angular_error = torch::atan2(torch::sin(angular_error), torch::cos(angular_error));


    return angular_error;
}

void bhs_game::PIDController::reset() {

}

//Eigen::MatrixXd bhs_game::EPIDController::update(const Eigen::Matrix<double, 3, Eigen::Dynamic> &currentState,
//                                                 const Eigen::Matrix<double, 2, Eigen::Dynamic> &desiredState) {
//    // Slice the desired_position matrix to get the first two rows
//    Eigen::Matrix<double, 2, Eigen::Dynamic> desired_position_slice = desiredState.topRows<2>();
//
//    // Slice the current_states matrix to get the first two rows
//    Eigen::Matrix<double, 2, Eigen::Dynamic> current_states_slice = currentState.topRows<2>();
//
//    Eigen::MatrixXd position_error = desired_position_slice - current_states_slice;
//
//#ifdef VERBOSE
//    std::cout << "Position error:\n" << position_error << std::endl;
//#endif
//
//    Eigen::VectorXd distance_error = position_error.colwise().norm();
//#ifdef VERBOSE
//    std::cout << "Distance error:\n" << distance_error << std::endl;
//#endif
//
//    v_error_integral += distance_error;
//    Eigen::VectorXd v_error_derivative = distance_error - v_error_prev;
//    Eigen::VectorXd desiredVelocity =
//            vel_pid.kp * distance_error + vel_pid.ki * v_error_integral + vel_pid.kd * v_error_derivative;
//    v_error_prev = distance_error;
//
//    // Compute the desired_heading
//    Eigen::VectorXd desired_heading = (position_error.row(1).array() / position_error.row(0).array()).unaryExpr(
//            [](double ratio) { return atan2(ratio, 1.0); });
//
//
//
////    auto res = cmath::atan2(position_error.row(1), position_error.row(0));
//    // Compute the angular_error
//    Eigen::VectorXd current_heading = currentState.row(2);
//    Eigen::VectorXd angular_error = desired_heading - current_heading;
//
//    // Wrap the angular_error to the range [-pi, pi]
//    angular_error = angular_error.unaryExpr([](double x) { return atan2(sin(x), cos(x)); });
//#ifdef VERBOSE
//    std::cout << "Angular error:\n" << angular_error << std::endl;
//#endif
//    a_error_integral += angular_error;
//    Eigen::VectorXd a_error_derivative = angular_error - a_error_prev;
//    Eigen::VectorXd desiredAngularVelocity =
//            ang_pid.kp * angular_error + ang_pid.ki * a_error_integral + ang_pid.kd * a_error_derivative;
//    a_error_prev = angular_error;
//
//    Eigen::MatrixXd result(currentState.cols(), 2);
//    result << desiredVelocity, desiredAngularVelocity;
//    return result;
//}

Eigen::MatrixXd bhs_game::EPIDController::update(const Eigen::Matrix<double, 3, Eigen::Dynamic> &currentState,
                                                 const Eigen::Matrix<double, 2, Eigen::Dynamic> &desiredState) {
    // Slice the desired_position matrix to get the first two columns
    Eigen::MatrixXd desired_position_slice = desiredState.topRows(2);

    // Slice the current_states matrix to get the first two elements of the first dimension
    Eigen::MatrixXd current_states_slice = currentState.topRows(2);

    Eigen::MatrixXd position_error = desired_position_slice - current_states_slice;
#ifdef VERBOSE
    std::cout << "Position error:\n" << position_error << std::endl;
#endif

    // Compute the distance_error
    Eigen::VectorXd distance_error = position_error.colwise().norm();
#ifdef VERBOSE
    std::cout << "Distance error:\n" << distance_error << std::endl;
#endif
    v_error_integral += distance_error;
    Eigen::VectorXd v_error_derivative = distance_error - v_error_prev;
    Eigen::VectorXd desiredVelocity =
            vel_pid.kp * distance_error + vel_pid.ki * v_error_integral + vel_pid.kd * v_error_derivative;
    v_error_prev = distance_error;

    // Compute the desired_heading
    auto s = position_error.row(1).array();

    Eigen::VectorXd desired_heading = position_error.row(1).array().atan2(position_error.row(0).array());
    // Compute the angular_error
    Eigen::VectorXd current_heading = currentState.row(2);
    Eigen::VectorXd angular_error = desired_heading - current_heading;

    // Wrap the angular_error to the range [-pi, pi]
    angular_error = angular_error.array().unaryExpr(
            [](double angle) { return std::atan2(std::sin(angle), std::cos(angle)); });

#ifdef VERBOSE
    std::cout << "Angular error:\n" << angular_error << std::endl;
#endif
    a_error_integral += angular_error;
    Eigen::VectorXd a_error_derivative = angular_error - a_error_prev;
    Eigen::VectorXd desiredAngularVelocity =
            ang_pid.kp * angular_error + ang_pid.ki * a_error_integral + ang_pid.kd * a_error_derivative;
    a_error_prev = angular_error;

    // Concatenate desiredVelocity and desiredAngularVelocity
    Eigen::MatrixXd result(currentState.cols(), 2);
    result << desiredVelocity, desiredAngularVelocity;
//    return result;
    return result;
}

double bhs_game::EPIDController::getPositionError(Eigen::Matrix<double, 3, Eigen::Dynamic> currentState,
                                                  Eigen::Matrix<double, 2, Eigen::Dynamic> desiredState) {
    Eigen::Matrix<double, 2, Eigen::Dynamic> desired_position_slice = desiredState;
    Eigen::Matrix<double, 2, Eigen::Dynamic> current_states_slice = currentState.topRows<2>();
    return (desired_position_slice - current_states_slice).squaredNorm();
}

double bhs_game::EPIDController::getAngleError(Eigen::Matrix<double, 3, Eigen::Dynamic> currentState,
                                               Eigen::Matrix<double, 2, Eigen::Dynamic> desiredState) {
    Eigen::Matrix<double, 2, Eigen::Dynamic> desired_position_slice = desiredState;
    Eigen::Matrix<double, 2, Eigen::Dynamic> current_states_slice = currentState.topRows<2>();

    Eigen::MatrixXd position_error = desired_position_slice - current_states_slice;
    Eigen::VectorXd desired_heading = (position_error.row(1).array() / position_error.row(0).array()).unaryExpr(
            [](double ratio) { return atan2(ratio, 1.0); });

    Eigen::VectorXd current_heading = currentState.row(2);
    Eigen::VectorXd angular_error = desired_heading - current_heading;
    angular_error = angular_error.unaryExpr([](double x) { return atan2(sin(x), cos(x)); });

    return angular_error.squaredNorm();
}

void bhs_game::EPIDController::reset() {
    int N = v_error_integral.size();
    v_error_integral = Eigen::VectorXd::Zero(N);
    v_error_prev = Eigen::VectorXd::Zero(N);
    a_error_integral = Eigen::VectorXd::Zero(N);
    a_error_prev = Eigen::VectorXd::Zero(N);
}

glm::dvec2 bhs_game::GPIDController::update(const glm::dvec3 &currentState,
                                            const glm::dvec2 &desiredState) {
    glm::dvec2 desired_position_slice = desiredState;
    glm::dvec2 current_states_slice(currentState.x, currentState.y);

    glm::dvec2 position_error = desired_position_slice - current_states_slice;
#ifdef VERBOSE
    std::cout << "Position error: " << position_error.x << ", " << position_error.y << std::endl;
#endif

    double distance_error = glm::length(position_error);
#ifdef VERBOSE
    std::cout << "Distance error: " << distance_error << std::endl;
#endif

    v_error_integral += distance_error;
    double v_error_derivative = distance_error - v_error_prev;
    double desiredVelocity =
            vel_pid.kp * distance_error + vel_pid.ki * v_error_integral + vel_pid.kd * v_error_derivative;
    v_error_prev = distance_error;

    double desired_heading = std::atan2(position_error.y, position_error.x);
    double current_heading = currentState.z;
    double angular_error = desired_heading - current_heading;

    angular_error = std::atan2(std::sin(angular_error), std::cos(angular_error));
#ifdef VERBOSE
    std::cout << "Angular error: " << angular_error << std::endl;
#endif

    a_error_integral += angular_error;
    double a_error_derivative = angular_error - a_error_prev;
    double desiredAngularVelocity =
            ang_pid.kp * angular_error + ang_pid.ki * a_error_integral + ang_pid.kd * a_error_derivative;
    a_error_prev = angular_error;

    return glm::dvec2(desiredVelocity, desiredAngularVelocity);
}

double bhs_game::GPIDController::getPositionError(glm::dvec3 currentState,
                                                  glm::dvec2 desiredState) {
    glm::dvec2 desired_position_slice = desiredState;
    glm::dvec2 current_states_slice(currentState.x, currentState.y);

    glm::dvec2 position_error = desired_position_slice - current_states_slice;

    return glm::length(position_error);
}

double bhs_game::GPIDController::getAngleError(glm::dvec3 currentState,
                                               glm::dvec2 desiredState) {
    glm::dvec2 desired_position_slice = desiredState;
    glm::dvec2 current_states_slice(currentState.x, currentState.y);

    glm::dvec2 position_error = desired_position_slice - current_states_slice;
    double desired_heading = atan2(position_error.y, position_error.x);

    double current_heading = currentState.z;
    double angular_error = desired_heading - current_heading;
    angular_error = atan2(sin(angular_error), cos(angular_error));

    return angular_error * angular_error;
}

void bhs_game::GPIDController::reset() {
    v_error_integral = 0;
    v_error_prev = 0;
    a_error_integral = 0;
    a_error_prev = 0;
}
