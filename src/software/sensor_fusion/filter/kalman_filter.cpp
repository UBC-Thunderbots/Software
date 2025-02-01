#include "software/sensor_fusion/filter/kalman_filter.h"


template<int dim_x, int dim_z, int dim_u>
KalmanFilter<dim_x, dim_z, dim_u>::KalmanFilter(Eigen::Matrix<double, dim_x, 1> x,
                                                Eigen::Matrix<double, dim_x, dim_x> P,
                                                Eigen::Matrix<double, dim_x, dim_x> F,
                                                Eigen::Matrix<double, dim_x, dim_x> Q,
                                                Eigen::Matrix<double, dim_x, dim_u> B,
                                                Eigen::Matrix<double, dim_z, dim_x> H,
                                                Eigen::Matrix<double, dim_z, dim_z> R):
        x(x),
        P(P),
        F(F),
        Q(Q),
        B(B),
        H(H),
        R(R)
{ }

template<int dim_x, int dim_z, int dim_u>
KalmanFilter<dim_x, dim_z, dim_u>::KalmanFilter():
        x(Eigen::Matrix<double, dim_x, 1>().setZero()),
        P(Eigen::Matrix<double, dim_x, dim_x>().setZero()),
        F(Eigen::Matrix<double, dim_x, dim_x>().setZero()),
        Q(Eigen::Matrix<double, dim_x, dim_x>().setZero()),
        B(Eigen::Matrix<double, dim_x, dim_u>().setZero()),
        H(Eigen::Matrix<double, dim_z, dim_x>().setZero()),
        R(Eigen::Matrix<double, dim_z, dim_z>().setZero())
{ }

template<int dim_x, int dim_z, int dim_u>
void KalmanFilter<dim_x, dim_z, dim_u>::predict(Eigen::Matrix<double, dim_u, 1> u) {
    x = F * x + B * u;
    P = F * P * F.transpose() + Q;
}

template<int dim_x, int dim_z, int dim_u>
void KalmanFilter<dim_x, dim_z, dim_u>::update(Eigen::Matrix<double, dim_z, 1> z) {

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
