#include "software/sensor_fusion/filter/kalman_filter.h"

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
    state_estimate = process_model * state_estimate + control_model * control_input;
    state_covariance =
        process_model * state_covariance * process_model.transpose() + process_covariance;
}

template <int DimX, int DimY, int DimU>
void KalmanFilter<DimX, DimY, DimU>::update(Eigen::Vector<double, DimY> measurement)
{
    const Eigen::Vector<double, DimY> innovation =
        measurement - measurement_model * state_estimate;

    const Eigen::Matrix<double, DimY, DimY> innovation_covariance =
        measurement_model * state_covariance * measurement_model.transpose() +
        measurement_covariance;
    const Eigen::Matrix<double, DimY, DimY> regularized_innovation_covariance =
        innovation_covariance.unaryExpr(
            [](double value) { return (std::abs(value) < 1.0e-20) ? 0.0 : value; });

    const Eigen::Matrix<double, DimX, DimY> kalman_gain =
        state_covariance *
        (measurement_model.transpose() *
         regularized_innovation_covariance.completeOrthogonalDecomposition()
             .pseudoInverse());

    state_estimate = state_estimate + kalman_gain * innovation;

    // Joseph form — more numerically stable than P = (I - K*H) * P
    const Eigen::Matrix<double, DimX, DimX> factor =
        Eigen::Matrix<double, DimX, DimX>::Identity() - kalman_gain * measurement_model;
    state_covariance = factor * state_covariance * factor.transpose() +
                       kalman_gain * measurement_covariance * kalman_gain.transpose();
}

// Explicit instantiation for the ball tracker
template class KalmanFilter<4, 2, 1>;
