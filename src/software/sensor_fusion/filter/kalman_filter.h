#pragma once

#include <Eigen/Dense>
#include <iostream>
/**
 * Implementation of a kalman filter.
 *
 * This is probably the best resource on the kalman filter for programmers:
 * https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 *
 * This is a good overview if you are familiar with bayesian maths and state based control theory:
 * https://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf
 *
 * @tparam dim_x The dimension of the state
 * @tparam dim_z The dimension of measurement space
 * @tparam dim_u The dimension of control space
 */
template<int dim_x, int dim_z, int dim_u>
class KalmanFilter {

public:
    /**
     * @param x Initial state vector
     * @param P Initial state covariance matrix
     * @param F Initial process transformation
     * @param Q Initial process covariance
     * @param B Initial control space -> state space transformation
     * @param H Initial state space -> measurement space transformation
     * @param R Initial measurement noise covariance
     */
    KalmanFilter(Eigen::Matrix<double, dim_x, 1> x, Eigen::Matrix<double, dim_x, dim_x> P,
                 Eigen::Matrix<double, dim_x, dim_x> F, Eigen::Matrix<double, dim_x, dim_x> Q,
                 Eigen::Matrix<double, dim_x, dim_u> B, Eigen::Matrix<double, dim_z, dim_x> H,
                 Eigen::Matrix<double, dim_z, dim_z> R);

    /**
     * Creates kalman filter with all matrices and vectors set to zero.
     */
    KalmanFilter();

    /**
     * Uses state model to innovate next state, taking into account expected behaviour from control input.
     * @param u Control inputs
     */
    void predict(Eigen::Matrix<double, dim_u, 1> u);

    /**
     * Incorporates measurement to state.
     * @param z Measurement
     */
    void update(Eigen::Matrix<double, dim_z, 1> z);

    Eigen::Matrix<double, dim_x, 1> x; // State
    Eigen::Matrix<double, dim_x, dim_x> P; // State covariance
    Eigen::Matrix<double, dim_x, dim_x> F; // Process model
    Eigen::Matrix<double, dim_x, dim_x> Q; // Process covariance
    Eigen::Matrix<double, dim_x, dim_u> B; // Control to state
    Eigen::Matrix<double, dim_z, dim_x> H; // Converts state to measurement space
    Eigen::Matrix<double, dim_z, dim_z> R; // Measurement noise covariance

};




