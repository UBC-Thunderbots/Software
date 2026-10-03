#pragma once

#include <Eigen/Dense>
#include <cmath>

/**
 * Base class for the Kalman Filter algorithm family
 *
 * A Kalman filter combines a process model (how the state evolves over time)
 * with noisy measurements (what sensors report) to produce an estimate of the
 * system state over time. It alternates between two steps:
 * 1) predict: propagate state/covariance forward through the process model
 * 2) update: correct that prediction with a new measurement
 *
 * Subclasses only implement predict(), since that is where the linear and extended
 * filters differ. update() and mahalanobisDistance() are shared, and assume
 * measurements relate to the state linearly through the constant matrix H
 * (measurement_model).
 *
 * Resources:
 * - https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
 * - https://kalmanfilter.net/
 * - https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 * - https://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf
 *
 *
 * @tparam DimX The dimension of the state
 * @tparam DimY The dimension of measurement space
 * @tparam DimU The dimension of control space
 */

template <int DimX, int DimY, int DimU>
class KalmanFilterBase
{
   public:
    /**
     * Creates a Kalman filter with all internal matrices and vectors set to zero.
     */
    KalmanFilterBase();


    /**
     * Creates a Kalman filter with the given initial state and model parameters.
     *
     * @param initial_state Initial state estimate (x)
     * @param initial_state_covariance Initial state covariance (P)
     * @param initial_process_covariance Initial process noise covariance (Q)
     * @param initial_control_model Initial control-to-state transformation (B)
     * @param initial_measurement_model Initial state-to-measurement transformation (H)
     * @param initial_measurement_covariance Initial measurement noise covariance (R)
     *
     * The base implementation assumes a linear measurement model (z = Hx + v).
     * Variants that require a non-linear measurement model can override update to
     * implement the appropriate measurement function and linearization.
     */
    KalmanFilterBase(Eigen::Vector<double, DimX> initial_state,
                     Eigen::Matrix<double, DimX, DimX> initial_state_covariance,
                     Eigen::Matrix<double, DimX, DimX> initial_process_covariance,
                     Eigen::Matrix<double, DimX, DimU> initial_control_model,
                     Eigen::Matrix<double, DimY, DimX> initial_measurement_model,
                     Eigen::Matrix<double, DimY, DimY> initial_measurement_covariance);

    virtual ~KalmanFilterBase() = default;

    /**
     * Propagate state_estimate and state_covariance forward by
     * one time step through the subclass's process model.
     *
     * The form of the process model depends on the filter implementation. For
     * example, a linear Kalman filter uses a state transition matrix, while an
     * extended Kalman filter uses a nonlinear state transition function and its Jacobian.
     *
     * @param control_input Control input vector
     */
    virtual void predict(Eigen::Vector<double, DimU> control_input) = 0;

    /**
     * Correct the current state estimate with the given measurement.
     *
     * This step is the same in both algorithms, because measurements are
     * assumed to relate to the state through the constant matrix H
     * (measurement_model) rather than a nonlinear function.
     *
     * Also, this is kept virtual in case future measurement model is non-linear
     * which we can use to override
     *
     * @param measurement Measurement vector
     */
    virtual void update(Eigen::Vector<double, DimY> measurement);

    /**
     * Returns the squared Mahalanobis distance between the given measurement and the
     * measurement the current state estimate predicts.
     *
     * Unlike a plain Euclidean distance, this scales the discrepancy by how uncertain
     * the filter currently is, so a measurement that is far away but within a poorly
     * constrained direction is not penalized as heavily as one that contradicts a
     * confident estimate. This makes it a useful gate for rejecting outlier
     * measurements before they are fed to update().
     *
     * @param measurement Measurement vector
     *
     * @return The squared Mahalanobis distance of the measurement
     */
    double mahalanobisDistance(Eigen::Vector<double, DimY> measurement) const;

    /**
     * Returns the squared Mahalanobis distance between the given measurement and the
     * measurement the current state estimate predicts.
     *
     * Unlike a plain Euclidean distance, this scales the discrepancy by how uncertain
     * the filter currently is, so a measurement that is far away but within a poorly
     * constrained direction is not penalized as heavily as one that contradicts a
     * confident estimate. This makes it a useful gate for rejecting outlier
     * measurements before they are fed to update().
     *
     * @param measurement Measurement vector
     *
     * @return The squared Mahalanobis distance of the measurement
     */
    double mahalanobisDistance(Eigen::Vector<double, DimY> measurement) const;

    Eigen::Vector<double, DimX> state_estimate;
    Eigen::Matrix<double, DimX, DimX> state_covariance;
    Eigen::Matrix<double, DimX, DimX> process_covariance;
    Eigen::Matrix<double, DimX, DimU> control_model;
    Eigen::Matrix<double, DimY, DimX> measurement_model;
    Eigen::Matrix<double, DimY, DimY> measurement_covariance;

   private:
    /**
     * Returns the pseudo inverse of the innovation covariance S = H*P*H' + R, which
     * describes the expected spread of the difference between an actual and a predicted
     * measurement.
     *
     * The pseudo-inverse is used instead of a regular inverse so the filter remains
     * numerically stable when S is singular or nearly singular. Since S^{-1} is used
     * in the Kalman gain, very small values are treated as zero before computing the
     * pseudo-inverse to avoid amplifying floating-point noise into very large values.
     *
     * @return The pseudo-inverse of the innovation covariance
     */
    Eigen::Matrix<double, DimY, DimY> innovationCovarianceInverse() const;
};

template <int DimX, int DimY, int DimU>
KalmanFilterBase<DimX, DimY, DimU>::KalmanFilterBase()
    : state_estimate(Eigen::Vector<double, DimX>::Zero()),
      state_covariance(Eigen::Matrix<double, DimX, DimX>::Zero()),
      process_covariance(Eigen::Matrix<double, DimX, DimX>::Zero()),
      control_model(Eigen::Matrix<double, DimX, DimU>::Zero()),
      measurement_model(Eigen::Matrix<double, DimY, DimX>::Zero()),
      measurement_covariance(Eigen::Matrix<double, DimY, DimY>::Zero())
{
}

template <int DimX, int DimY, int DimU>
KalmanFilterBase<DimX, DimY, DimU>::KalmanFilterBase(
    Eigen::Vector<double, DimX> initial_state,
    Eigen::Matrix<double, DimX, DimX> initial_state_covariance,
    Eigen::Matrix<double, DimX, DimX> initial_process_covariance,
    Eigen::Matrix<double, DimX, DimU> initial_control_model,
    Eigen::Matrix<double, DimY, DimX> initial_measurement_model,
    Eigen::Matrix<double, DimY, DimY> initial_measurement_covariance)
    : state_estimate(initial_state),
      state_covariance(initial_state_covariance),
      process_covariance(initial_process_covariance),
      control_model(initial_control_model),
      measurement_model(initial_measurement_model),
      measurement_covariance(initial_measurement_covariance)
{
}

template <int DimX, int DimY, int DimU>
void KalmanFilterBase<DimX, DimY, DimU>::update(Eigen::Vector<double, DimY> measurement)
{
    // Innovation between actual and predicted measurement
    const Eigen::Vector<double, DimY> innovation =
        measurement - measurement_model * state_estimate;
    // Kalman gain defines how much the input measurement will influence the
    // state estimate, i.e., how strongly we trust measurement vs. prediction
    const Eigen::Matrix<double, DimX, DimY> kalman_gain =
        state_covariance *
        (measurement_model.transpose() * innovationCovarianceInverse());
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

template <int DimX, int DimY, int DimU>
double KalmanFilterBase<DimX, DimY, DimU>::mahalanobisDistance(
    Eigen::Vector<double, DimY> measurement) const
{
    // Innovation between actual and predicted measurement
    const Eigen::Vector<double, DimY> innovation =
        measurement - measurement_model * state_estimate;
    return innovation.transpose() * innovationCovarianceInverse() * innovation;
}


template <int DimX, int DimY, int DimU>
Eigen::Matrix<double, DimY, DimY>
KalmanFilterBase<DimX, DimY, DimU>::innovationCovarianceInverse() const
{
    // Innovation covariance (measurement uncertainty in innovation space)
    const Eigen::Matrix<double, DimY, DimY> innovation_covariance =
        measurement_model * state_covariance * measurement_model.transpose() +
        measurement_covariance;
    const Eigen::Matrix<double, DimY, DimY> regularized_innovation_covariance =
        innovation_covariance.unaryExpr(
            [](double value) { return (std::abs(value) < 1.0e-20) ? 0.0 : value; });

    return regularized_innovation_covariance.completeOrthogonalDecomposition()
        .pseudoInverse();
}
