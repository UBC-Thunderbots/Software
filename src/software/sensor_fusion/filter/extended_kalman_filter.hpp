#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <utility>

/**
 * Extended Kalman filter for discrete-time state estimation.
 *
 * A Kalman filter combines a process model (how the state evolves over time)
 * with noisy measurements (what sensors report) to produce an estimate of the
 * system state over time. The plain Kalman filter requires the process model to
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
 * It alternates between two steps:
 * 1) predict: propagate state/covariance forward through the process model
 * 2) update: correct that prediction with a new measurement
 *
 * Because the linearization is only a first-order approximation, the estimate is
 * not optimal in the way the linear Kalman filter's is, and a poor initial
 * estimate can cause the filter to diverge.
 *
 * Resources:
 * - https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
 * - https://kalmanfilter.net/
 * - https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 * - https://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf
 *
 * @tparam DimX The dimension of the state
 * @tparam DimY The dimension of measurement space
 * @tparam DimU The dimension of control space
 */
template <int DimX, int DimY, int DimU>
class ExtendedKalmanFilter
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
    void predict(Eigen::Vector<double, DimU> control_input);

    /**
     * Correct the current state estimate with the given measurement.
     *
     * This step is the same as the linear Kalman filter's, because measurements are
     * assumed to relate to the state through the constant matrix H
     * (measurement_model) rather than a nonlinear function.
     *
     * @param measurement Measurement vector
     */
    void update(Eigen::Vector<double, DimY> measurement);

    Eigen::Vector<double, DimX> state_estimate;
    Eigen::Matrix<double, DimX, DimX> state_covariance;
    ProcessModelFunction process_model_function;
    ProcessModelJacobianFunction process_model_jacobian_function;
    Eigen::Matrix<double, DimX, DimX> process_covariance;
    Eigen::Matrix<double, DimX, DimU> control_model;
    Eigen::Matrix<double, DimY, DimX> measurement_model;
    Eigen::Matrix<double, DimY, DimY> measurement_covariance;
};

template <int DimX, int DimY, int DimU>
ExtendedKalmanFilter<DimX, DimY, DimU>::ExtendedKalmanFilter()
    : state_estimate(Eigen::Vector<double, DimX>::Zero()),
      state_covariance(Eigen::Matrix<double, DimX, DimX>::Zero()),
      process_model_function(),
      process_model_jacobian_function(),
      process_covariance(Eigen::Matrix<double, DimX, DimX>::Zero()),
      control_model(Eigen::Matrix<double, DimX, DimU>::Zero()),
      measurement_model(Eigen::Matrix<double, DimY, DimX>::Zero()),
      measurement_covariance(Eigen::Matrix<double, DimY, DimY>::Zero())
{
}

template <int DimX, int DimY, int DimU>
ExtendedKalmanFilter<DimX, DimY, DimU>::ExtendedKalmanFilter(
    Eigen::Vector<double, DimX> initial_state,
    Eigen::Matrix<double, DimX, DimX> initial_state_covariance,
    typename ExtendedKalmanFilter<DimX, DimY, DimU>::ProcessModelFunction
        initial_process_model_function,
    typename ExtendedKalmanFilter<DimX, DimY, DimU>::ProcessModelJacobianFunction
        initial_process_model_jacobian_function,
    Eigen::Matrix<double, DimX, DimX> initial_process_covariance,
    Eigen::Matrix<double, DimX, DimU> initial_control_model,
    Eigen::Matrix<double, DimY, DimX> initial_measurement_model,
    Eigen::Matrix<double, DimY, DimY> initial_measurement_covariance)
    : state_estimate(initial_state),
      state_covariance(initial_state_covariance),
      process_model_function(std::move(initial_process_model_function)),
      process_model_jacobian_function(std::move(initial_process_model_jacobian_function)),
      process_covariance(initial_process_covariance),
      control_model(initial_control_model),
      measurement_model(initial_measurement_model),
      measurement_covariance(initial_measurement_covariance)
{
}

template <int DimX, int DimY, int DimU>
void ExtendedKalmanFilter<DimX, DimY, DimU>::predict(
    Eigen::Vector<double, DimU> control_input)
{
    const Eigen::Matrix<double, DimX, DimX> evaluated_jacobian =
        process_model_jacobian_function(state_estimate);

    // Project the current estimate through the process model
    state_estimate =
        process_model_function(state_estimate) + control_model * control_input;
    state_covariance =
        evaluated_jacobian * state_covariance * evaluated_jacobian.transpose() +
        process_covariance;
}

template <int DimX, int DimY, int DimU>
void ExtendedKalmanFilter<DimX, DimY, DimU>::update(
    Eigen::Vector<double, DimY> measurement)
{
    // Innovation between actual and predicted measurement
    const Eigen::Vector<double, DimY> innovation =
        measurement - measurement_model * state_estimate;

    // Innovation covariance (measurement uncertainty in innovation space)
    const Eigen::Matrix<double, DimY, DimY> innovation_covariance =
        measurement_model * state_covariance * measurement_model.transpose() +
        measurement_covariance;
    const Eigen::Matrix<double, DimY, DimY> regularized_innovation_covariance =
        innovation_covariance.unaryExpr(
            [](double value) { return (std::abs(value) < 1.0e-20) ? 0.0 : value; });

    // Kalman gain defines how much the input measurement will influence the
    // state estimate, i.e., how strongly we trust measurement vs. prediction
    const Eigen::Matrix<double, DimX, DimY> kalman_gain =
        state_covariance *
        (measurement_model.transpose() *
         regularized_innovation_covariance.completeOrthogonalDecomposition()
             .pseudoInverse());

    // Correct state estimate with innovation weighted by Kalman gain
    state_estimate = state_estimate + kalman_gain * innovation;

    // Correct state covariance
    // Joseph form is more numerically stable than P = (I - K*H) * P
    const Eigen::Matrix<double, DimX, DimX> posterior_covariance_factor =
        Eigen::Matrix<double, DimX, DimX>::Identity() - kalman_gain * measurement_model;
    state_covariance = posterior_covariance_factor * state_covariance *
                           posterior_covariance_factor.transpose() +
                       kalman_gain * measurement_covariance * kalman_gain.transpose();
}
