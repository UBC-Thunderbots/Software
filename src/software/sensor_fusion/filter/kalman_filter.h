#pragma once

#include <Eigen/Dense>
/**
 *
 * @tparam dim_x The dimension of the state
 * @tparam dim_z The dimension of measurement space
 * @tparam dim_u The dimension of control space
 */
template<int dim_x, int dim_z, int dim_u>
class KalmanFilter {

public:
    /**
     *
     * @param x
     * @param P
     * @param F
     * @param Q
     * @param B
     * @param H
     * @param R
     */
    KalmanFilter(Eigen::Matrix<double, dim_x, 1> x, Eigen::Matrix<double, dim_x, dim_x> P,
                 Eigen::Matrix<double, dim_x, dim_x> F, Eigen::Matrix<double, dim_x, dim_x> Q,
                 Eigen::Matrix<double, dim_u, dim_x> B, Eigen::Matrix<double, dim_x, dim_z> H,
                 Eigen::Matrix<double, dim_z, dim_z> R):
    x(x),
    P(P),
    F(F),
    Q(Q),
    B(B),
    H(H),
    R(R)
    { }

    /** Uses state model to innovate next state, taking into account expected behaviour from control input
     *
     * @param u Control inputs
     */
    void predict(Eigen::Matrix<double, dim_u, 1> u);

    /** Incorporates measurement to state
     *
     * @param z Measurement
     */
    void update(Eigen::Matrix<double, dim_z, 1> z);

    Eigen::Matrix<double, dim_x, 1> x; // State
    Eigen::Matrix<double, dim_x, dim_x> P; // State covariance
    Eigen::Matrix<double, dim_x, dim_x> F; // Process model
    Eigen::Matrix<double, dim_x, dim_x> Q; // Process covariance
    Eigen::Matrix<double, dim_u, dim_x> B; // Control to state
    Eigen::Matrix<double, dim_x, dim_z> H; // Converts state to measurement space
    Eigen::Matrix<double, dim_z, dim_z> R; // Measurement noise covariance
};


