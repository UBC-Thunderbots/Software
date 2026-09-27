#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "software/sensor_fusion/filter/kalman_filter_base.hpp"

/**
 * Linear Kalman filter for discrete-time state estimation.
 *
 * When the process and measurement models are linear and the noise is Gaussian,
 * this gives the optimal state estimate. See KalmanFilterBase for an overview of
 * the predict/update cycle.
 *
 * @tparam DimX The dimension of the state
 * @tparam DimY The dimension of measurement space
 * @tparam DimU The dimension of control space
 */
template <int DimX, int DimY, int DimU>
class KalmanFilter : public KalmanFilterBase<DimX, DimY, DimU>
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
     * Predict the next state estimate:
     *
     *     x = F * x + B * u
     *     P = F * P * F^T + Q
     *
     * @param control_input Control input vector
     */
    void predict(Eigen::Vector<double, DimU> control_input) override;

    Eigen::Matrix<double, DimX, DimX> process_model;
};

template <int DimX, int DimY, int DimU>
KalmanFilter<DimX, DimY, DimU>::KalmanFilter()
    : KalmanFilterBase<DimX, DimY, DimU>(),
      process_model(Eigen::Matrix<double, DimX, DimX>::Zero())
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
    : KalmanFilterBase<DimX, DimY, DimU>(initial_state, initial_state_covariance,
                                         initial_process_covariance,
                                         initial_control_model, initial_measurement_model,
                                         initial_measurement_covariance),
      process_model(initial_process_model)
{
}

template <int DimX, int DimY, int DimU>
void KalmanFilter<DimX, DimY, DimU>::predict(Eigen::Vector<double, DimU> control_input)
{
    // Project the current estimate through the process model
    this->state_estimate =
        process_model * this->state_estimate + this->control_model * control_input;
    this->state_covariance =
        process_model * this->state_covariance * process_model.transpose() +
        this->process_covariance;
}
