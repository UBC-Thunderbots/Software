#include "kalman_filter.h"


template <int dim_x, int dim_z, int dim_u>
void KalmanFilter<dim_x, dim_z, dim_u>::predict(Eigen::Matrix<double, dim_u, 1> u) {
    x = F * x + B * u;
    P = F * P * F.transpose() + Q;
}

template <int dim_x, int dim_z, int dim_u>
void KalmanFilter<dim_x, dim_z, dim_u>::update(Eigen::Matrix<double, dim_z, 1> z) {
    Eigen::Matrix<double, dim_z, 1> y = z - H * x; // residual
    Eigen::Matrix<double, dim_z, dim_z> sum = (H * P * H.transpose() + R);
    Eigen::Matrix<double, dim_z, dim_x> K = P * H.transpose() * sum.inverse(); // Kalman gain
    x = x + K * y;
    // Joseph equation is more stable than  P = (I-KH)P since the latter is susceptible to floating point errors ruining symmetry
    Eigen::Matrix<double, dim_z, dim_z> posteriorCov = Eigen::Matrix<double, dim_z, dim_z>::Identity() - K * H;
    P = posteriorCov * P * posteriorCov.transpose() + K * R * K.transpose();
}
