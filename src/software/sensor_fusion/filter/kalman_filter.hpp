#pragma once

#include <Eigen/Dense>
#include <cmath>

/**
 * Linear Kalman filter for discrete-time state estimation.
 *
 * A Kalman filter combines a process model (how the state evolves over time)
 * with noisy measurements (what sensors report) to produce an optimal estimate
 * of the system state over time (assuming system model and noise are Gaussian).
 *
 * It alternates between two steps:
 * 1) predict: propagate state/covariance forward through the process model
 * 2) update: correct that prediction with a new measurement
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
class KalmanFilter
{
   public:
    /**
     * Creates a Kalman filter with all internal matrices and vectors set to zero.
     */
    KalmanFilter();

    /**
     * Creates a Kalman filter with the given initial state and model parameters.
     *
     * @param initial_state Initial state estimate (x)
     * @param initial_state_covariance Initial state covariance (P)
     * @param initial_process_model Initial process/state transition model (F)
     * @param initial_process_covariance Initial process noise covariance (Q)
     * @param initial_control_model Initial control-to-state transformation (B)
     * @param initial_measurement_model Initial state-to-measurement transformation (H)
     * @param initial_measurement_covariance Initial measurement noise covariance (R)
     */
    KalmanFilter(Eigen::Vector<double, DimX> initial_state,
                 Eigen::Matrix<double, DimX, DimX> initial_state_covariance,
                 Eigen::Matrix<double, DimX, DimX> initial_process_model,
                 Eigen::Matrix<double, DimX, DimX> initial_process_covariance,
                 Eigen::Matrix<double, DimX, DimU> initial_control_model,
                 Eigen::Matrix<double, DimY, DimX> initial_measurement_model,
                 Eigen::Matrix<double, DimY, DimY> initial_measurement_covariance);

    /**
     * Predict the next state estimate.
     *
     * @param control_input Control input vector
     */
    void predict(Eigen::Vector<double, DimU> control_input);

    /**
     * Correct the current state estimate with the given measurement.
     *
     * @param measurement Measurement vector
     */
    void update(Eigen::Vector<double, DimY> measurement);

    Eigen::Vector<double, DimX> state_estimate;
    Eigen::Matrix<double, DimX, DimX> state_covariance;
    Eigen::Matrix<double, DimX, DimX> process_model;
    Eigen::Matrix<double, DimX, DimX> process_covariance;
    Eigen::Matrix<double, DimX, DimU> control_model;
    Eigen::Matrix<double, DimY, DimX> measurement_model;
    Eigen::Matrix<double, DimY, DimY> measurement_covariance;
};

template <int DimX, int DimY, int DimU>
KalmanFilter<DimX, DimY, DimU>::KalmanFilter()
    : state_estimate(Eigen::Vector<double, DimX>::Zero()),
      state_covariance(Eigen::Matrix<double, DimX, DimX>::Zero()),
      process_model(Eigen::Matrix<double, DimX, DimX>::Zero()),
      process_covariance(Eigen::Matrix<double, DimX, DimX>::Zero()),
      control_model(Eigen::Matrix<double, DimX, DimU>::Zero()),
      measurement_model(Eigen::Matrix<double, DimY, DimX>::Zero()),
      measurement_covariance(Eigen::Matrix<double, DimY, DimY>::Zero())
{
}

template <int DimX, int DimY, int DimU>
KalmanFilter<DimX, DimY, DimU>::KalmanFilter(
    Eigen::Vector<double, DimX> initial_state,
    Eigen::Matrix<double, DimX, DimX> initial_state_covariance,
    Eigen::Matrix<double, DimX, DimX> initial_process_model,
    Eigen::Matrix<double, DimX, DimX> initial_process_covariance,
    Eigen::Matrix<double, DimX, DimU> initial_control_model,
    Eigen::Matrix<double, DimY, DimX> initial_measurement_model,
    Eigen::Matrix<double, DimY, DimY> initial_measurement_covariance)
    : state_estimate(initial_state),
      state_covariance(initial_state_covariance),
      process_model(initial_process_model),
      process_covariance(initial_process_covariance),
      control_model(initial_control_model),
      measurement_model(initial_measurement_model),
      measurement_covariance(initial_measurement_covariance)
{
}

template <int DimX, int DimY, int DimU>
void KalmanFilter<DimX, DimY, DimU>::predict(Eigen::Vector<double, DimU> control_input)
{
    // Project the current estimate through the process model
    state_estimate = process_model * state_estimate + control_model * control_input;
    state_covariance =
        process_model * state_covariance * process_model.transpose() + process_covariance;
}

template <int DimX, int DimY, int DimU>
void KalmanFilter<DimX, DimY, DimU>::update(Eigen::Vector<double, DimY> measurement)
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
