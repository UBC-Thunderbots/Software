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
