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
                 Eigen::Matrix<double, dim_z, dim_z> R):
            x(x),
            P(P),
            F(F),
            Q(Q),
            B(B),
            H(H),
            R(R)
    { }

    /**
     * Creates kalman filter with all matrices and vectors set to zero.
     */
    KalmanFilter():
            x(Eigen::Matrix<double, dim_x, 1>().setZero()),
            P(Eigen::Matrix<double, dim_x, dim_x>().setZero()),
            F(Eigen::Matrix<double, dim_x, dim_x>().setZero()),
            Q(Eigen::Matrix<double, dim_x, dim_x>().setZero()),
            B(Eigen::Matrix<double, dim_x, dim_u>().setZero()),
            H(Eigen::Matrix<double, dim_z, dim_x>().setZero()),
            R(Eigen::Matrix<double, dim_z, dim_z>().setZero())
    { }

    /**
     * Uses state model to innovate next state, taking into account expected behaviour from control input.
     * @param u Control inputs
     */
    void predict(Eigen::Matrix<double, dim_u, 1> u) {
        x = F * x + B * u;
        P = F * P * F.transpose() + Q;
    }

    /**
     * Incorporates measurement to state.
     * @param z Measurement
     */
    void update(Eigen::Matrix<double, dim_z, 1> z) {

        Eigen::Matrix<double, dim_z, 1> y = z - H * x; // residual
        Eigen::Matrix<double, dim_z, dim_z> sum = H * P * H.transpose() + R;
        Eigen::Matrix<double, dim_z, dim_z> newSum = sum.unaryExpr([](double l){return (fabs(l)<1.0e-20)?0.:l;});
        Eigen::Matrix<double, dim_x, dim_z> K = P * (H.transpose() * newSum.completeOrthogonalDecomposition().pseudoInverse()); // Kalman gain
        Eigen::Matrix<double, dim_x, 1> newX = x + K * y;
        x = newX;
        // Joseph equation is more stable than  P = (I-KH)P since the latter is susceptible to floating point errors ruining symmetry
        Eigen::Matrix<double, dim_x, dim_x> posteriorCov = Eigen::Matrix<double, dim_x, dim_x>::Identity() - K * H;
        P = posteriorCov * P * posteriorCov.transpose() + K * R * K.transpose();
    }

    Eigen::Matrix<double, dim_x, 1> x; // State
    Eigen::Matrix<double, dim_x, dim_x> P; // State covariance
    Eigen::Matrix<double, dim_x, dim_x> F; // Process model
    Eigen::Matrix<double, dim_x, dim_x> Q; // Process covariance
    Eigen::Matrix<double, dim_x, dim_u> B; // Control to state
    Eigen::Matrix<double, dim_z, dim_x> H; // Converts state to measurement space
    Eigen::Matrix<double, dim_z, dim_z> R; // Measurement noise covariance

};




