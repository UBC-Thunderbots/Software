#pragma once

#include <Eigen/Dense>

/**
 * A Kalman filter for tracking a 2D object using a constant velocity motion model.
 *
 * The state vector is [x, y, vx, vy]^T. Measurements are 2D positions [x, y]^T.
 * The filter supports predict/update steps as well as Mahalanobis-distance-based
 * gating and a hard reset when the tracked object jumps unexpectedly.
 */
class KalmanFilter{


public:

/**
 * Creates a new KalmanFilter
 *
 * @param X The initial state vector [x, y, vx, vy]^T
 * @param P_i The initial state covariance matrix (4x4)
 * @param Q The process noise covariance matrix (4x4), representing uncertainty
 *          introduced by the motion model at each prediction step
 * @param R The measurement noise covariance matrix (2x2), representing
 *          uncertainty in position measurements
 * @param C The measurement matrix that maps the state space to the measurement
 *          space (2x4)
 * @param damping_term A scalar in [0, 1] applied to the velocity states each
 *                     predict step to model drag/friction
 */
KalmanFilter(
	const Eigen::Matrix<double, 4,1>& X,
	const Eigen::Matrix<double, 4,4>& P_i,
	const Eigen::Matrix<double, 4,4>& Q,
	const Eigen::Matrix<double, 2,2>& R,
	const Eigen::Matrix<double, 2,4>& C,
	double damping_term
	);

/**
 * Propagates the state estimate forward in time using the constant-velocity
 * motion model and adds process noise to the covariance.
 *
 * @param delta_t Time elapsed since the last predict step, in seconds
 */
void predict(const double delta_t);

/**
 * Corrects the state estimate using a new position measurement.
 *
 * @param Z The measurement vector [x, y]^T
 */
void update(const Eigen::Matrix<double,2,1> Z);

/**
 * Resets the filter to the given measurement, zeroing velocities and
 * restoring the initial covariance.
 *
 * @param Z The measurement vector [x, y]^T to reset the state to
 */
void reset(const Eigen::Matrix<double,2,1> Z);

/**
 * Returns the squared Mahalanobis distance between the given measurement and
 * the current predicted measurement. Used for gating outlier detections.
 *
 * @param Z The measurement vector [x, y]^T to evaluate
 * @return The squared Mahalanobis distance
 */
double getMahalanobisDistance(const Eigen::Matrix<double,2,1>& Z) const;

/**
 * Returns the current state estimate vector [x, y, vx, vy]^T.
 *
 * @return The current state vector (4x1)
 */
Eigen::Matrix<double, 4,1> getState();

/**
 * Returns the current state covariance matrix.
 *
 * @return The current covariance matrix (4x4)
 */
Eigen::Matrix<double, 4,4> getCovariance();

private:
Eigen::Matrix<double, 4,1> X; // State
Eigen::Matrix<double, 4,4> P; // State Covariance
Eigen::Matrix<double, 4,4> P_i; // Initial state covariance
Eigen::Matrix<double, 4,4> Q; // Process noise covariance
Eigen::Matrix<double, 2,2> R; // Measurement noise covariance
Eigen::Matrix<double, 2,4> C; // State to measurement matrix
double damping_term;

};
