#pragma once

#include <Eigen/Dense>
#include <functional>
#include <utility>

#include "software/sensor_fusion/filter/kalman_filter_base.hpp"

/**
 * Extended Kalman filter for discrete-time state estimation.
 *
 * The plain Kalman filter requires the process model to
 * be linear, i.e. a constant state transition matrix F. The extended Kalman
 * filter lifts that restriction by linearizing a nonlinear process model about
 * the current estimate at every step, so instead of F it takes:
 * 1) f(x), which propagates a state forward by one time step
 * 2) its Jacobian F = df/dx, evaluated at whatever state it is handed
 *
 * The measurement side is still linear: measurements are related to the state by
 * the constant matrix H (measurement_model). That is sufficient when every sensor
 * reports a linear combination of state variables. Supporting a nonlinear h(x)
 * would require giving the update step the same function/Jacobian treatment.
 *
 * Because the linearization is only a first-order approximation, the estimate is
 * not optimal in the way the linear Kalman filter's is, and a poor initial
 * estimate can cause the filter to diverge.
 *
 * @tparam DimX The dimension of the state
 * @tparam DimY The dimension of measurement space
 * @tparam DimU The dimension of control space
 */
template <int DimX, int DimY, int DimU>
class ExtendedKalmanFilter : public KalmanFilterBase<DimX, DimY, DimU>
{
   public:
    /**
     * The process model f(x): propagates a state forward by one time step.
     */
    using ProcessModelFunction =
        std::function<Eigen::Vector<double, DimX>(Eigen::Vector<double, DimX>)>;

    /**
     * The Jacobian of the process model (F = df/dx), evaluated at a given state.
     */
    using ProcessModelJacobianFunction =
        std::function<Eigen::Matrix<double, DimX, DimX>(Eigen::Vector<double, DimX>)>;

    /**
     * Creates an extended Kalman filter with all internal matrices and vectors set
     * to zero, and with no process model.
     *
     * Both the process model function and its Jacobian are left empty, so they must
     * be assigned before predict() is called; predicting on a filter constructed
     * this way throws std::bad_function_call.
     */
    ExtendedKalmanFilter();

    /**
     * Creates an extended Kalman filter with the given initial state and model
     * parameters.
     *
     * @param initial_state Initial state estimate (x)
     * @param initial_state_covariance Initial state covariance (P)
     * @param process_model_function The process model f(x), which propagates the
     *                               given state forward by one time step
     * @param process_model_jacobian_function The Jacobian of the process model
     *                                        (F = df/dx) evaluated at the given state
     * @param initial_process_covariance Initial process noise covariance (Q)
     * @param initial_control_model Initial control-to-state transformation (B)
     * @param initial_measurement_model Initial state-to-measurement transformation (H)
     * @param initial_measurement_covariance Initial measurement noise covariance (R)
     */
    ExtendedKalmanFilter(
        Eigen::Vector<double, DimX> initial_state,
        Eigen::Matrix<double, DimX, DimX> initial_state_covariance,
        ProcessModelFunction process_model_function,
        ProcessModelJacobianFunction process_model_jacobian_function,
        Eigen::Matrix<double, DimX, DimX> initial_process_covariance,
        Eigen::Matrix<double, DimX, DimU> initial_control_model,
        Eigen::Matrix<double, DimY, DimX> initial_measurement_model,
        Eigen::Matrix<double, DimY, DimY> initial_measurement_covariance);

    /**
     * Predict the next state estimate by propagating the current estimate and its
     * covariance through the process model:
     *
     *     x = f(x) + B * u
     *     P = F * P * F^T + Q
     *
     * F is the process model Jacobian evaluated at the state estimate as it was
     * *before* f was applied, since that is the point the propagation is
     * linearized about.
     *
     * @param control_input Control input vector
     */
    void predict(Eigen::Vector<double, DimU> control_input) override;

    ProcessModelFunction process_model_function;
    ProcessModelJacobianFunction process_model_jacobian_function;
};

template <int DimX, int DimY, int DimU>
ExtendedKalmanFilter<DimX, DimY, DimU>::ExtendedKalmanFilter()
    : KalmanFilterBase<DimX, DimY, DimU>(),
      process_model_function(),
      process_model_jacobian_function()
{
}

template <int DimX, int DimY, int DimU>
ExtendedKalmanFilter<DimX, DimY, DimU>::ExtendedKalmanFilter(
    Eigen::Vector<double, DimX> initial_state,
    Eigen::Matrix<double, DimX, DimX> initial_state_covariance,
    ProcessModelFunction initial_process_model_function,
    ProcessModelJacobianFunction initial_process_model_jacobian_function,
    Eigen::Matrix<double, DimX, DimX> initial_process_covariance,
    Eigen::Matrix<double, DimX, DimU> initial_control_model,
    Eigen::Matrix<double, DimY, DimX> initial_measurement_model,
    Eigen::Matrix<double, DimY, DimY> initial_measurement_covariance)
    : KalmanFilterBase<DimX, DimY, DimU>(initial_state, initial_state_covariance,
                                         initial_process_covariance,
                                         initial_control_model, initial_measurement_model,
                                         initial_measurement_covariance),
      process_model_function(std::move(initial_process_model_function)),
      process_model_jacobian_function(std::move(initial_process_model_jacobian_function))
{
}

template <int DimX, int DimY, int DimU>
void ExtendedKalmanFilter<DimX, DimY, DimU>::predict(
    Eigen::Vector<double, DimU> control_input)
{
    const Eigen::Matrix<double, DimX, DimX> evaluated_jacobian =
        process_model_jacobian_function(this->state_estimate);
    // Project the current estimate through the process model
    this->state_estimate = process_model_function(this->state_estimate) +
                           this->control_model * control_input;
    this->state_covariance =
        evaluated_jacobian * this->state_covariance * evaluated_jacobian.transpose() +
        this->process_covariance;
}
