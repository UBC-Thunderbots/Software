#pragma once

#include <Eigen/Dense>
#include <iostream>
/**
 * Implementation of a kalman filter.
 *
 * This is probably the best resource on the kalman filter for programmers:
 * https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 *
 * This is a good overview if you are familiar with bayesian maths and state based control
 * theory:
 * https://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf
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
     * @param x Initial state vector
     * @param P Initial state covariance matrix
     * @param F Initial process transformation
     * @param Q Initial process covariance
     * @param B Initial control space -> state space transformation
     * @param H Initial state space -> measurement space transformation
     * @param R Initial measurement noise covariance
     */
    KalmanFilter(Eigen::Matrix<double, DimX, 1> x, Eigen::Matrix<double, DimX, DimX> P,
                 Eigen::Matrix<double, DimX, DimX> F,
                 Eigen::Matrix<double, DimX, DimX> Q,
                 Eigen::Matrix<double, DimX, DimU> B,
                 Eigen::Matrix<double, DimY, DimX> H,
                 Eigen::Matrix<double, DimY, DimY> R)
        : x(x), P(P), F(F), Q(Q), B(B), H(H), R(R)
    {
    }

    /**
     * Creates kalman filter with all matrices and vectors set to zero.
     */
    KalmanFilter()
        : x(Eigen::Matrix<double, DimX, 1>().setZero()),
          P(Eigen::Matrix<double, DimX, DimX>().setZero()),
          F(Eigen::Matrix<double, DimX, DimX>().setZero()),
          Q(Eigen::Matrix<double, DimX, DimX>().setZero()),
          B(Eigen::Matrix<double, DimX, DimU>().setZero()),
          H(Eigen::Matrix<double, DimY, DimX>().setZero()),
          R(Eigen::Matrix<double, DimY, DimY>().setZero())
    {
    }

    /**
     * Uses state model to innovate next state, taking into account expected behaviour
     * from control input.
     * @param u Control inputs
     */
    void predict(Eigen::Matrix<double, DimU, 1> u)
    {
        x = F * x + B * u;
        P = F * P * F.transpose() + Q;
    }

    /**
     * Incorporates measurement to state.
     * @param z Measurement
     */
    void update(Eigen::Matrix<double, DimY, 1> z)
    {
        Eigen::Matrix<double, DimY, 1> y       = z - H * x;  // residual
        Eigen::Matrix<double, DimY, DimY> sum = H * P * H.transpose() + R;
        Eigen::Matrix<double, DimY, DimY> newSum =
            sum.unaryExpr([](double l) { return (fabs(l) < 1.0e-20) ? 0. : l; });
        Eigen::Matrix<double, DimX, DimY> K =
            P *
            (H.transpose() *
             newSum.completeOrthogonalDecomposition().pseudoInverse());  // Kalman gain
        Eigen::Matrix<double, DimX, 1> newX = x + K * y;
        x                                    = newX;
        // Joseph equation is more stable than  P = (I-KH)P since the latter is
        // susceptible to floating point errors ruining symmetry
        Eigen::Matrix<double, DimX, DimX> posteriorCov =
            Eigen::Matrix<double, DimX, DimX>::Identity() - K * H;
        P = posteriorCov * P * posteriorCov.transpose() + K * R * K.transpose();
    }

    Eigen::Matrix<double, DimX, 1> x;      // State
    Eigen::Matrix<double, DimX, DimX> P;  // State covariance
    Eigen::Matrix<double, DimX, DimX> F;  // Process model
    Eigen::Matrix<double, DimX, DimX> Q;  // Process covariance
    Eigen::Matrix<double, DimX, DimU> B;  // Control to state
    Eigen::Matrix<double, DimY, DimX> H;  // Converts state to measurement space
    Eigen::Matrix<double, DimY, DimY> R;  // Measurement noise covariance
};
