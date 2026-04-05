#pragma once

#include <Eigen/Dense>
/**
 * Implementation of a kalman filter


**/
class KalmanFilter{


public:

KalmanFilter(
	const Eigen::Matrix<double, 4,1>& X,
	const Eigen::Matrix<double, 4,4>& P_i,
	const Eigen::Matrix<double, 2,2>& Q,
	const Eigen::Matrix<double, 4,4>& R,
	const Eigen::Matrix<double, 2,4>& C,
	double damping_term
	);

void predict(const double delta_t);

void update(const Eigen::Matrix<double,2,1> Z);

void reset(const Eigen::Matrix<double,2,1> Z);

double getMahalanobisDistance(const Eigen::Matrix<double,2,1>& Z) const;

Eigen::Matrix<double, 4,1> getState();

Eigen::Matrix<double, 4,4> getCovariance();

private:
Eigen::Matrix<double, 4,1> X; // State
Eigen::Matrix<double, 4,4> P; // State Covariance
Eigen::Matrix<double, 4,4> P_i; // State Covariance
Eigen::Matrix<double, 2,2> Q; // Measurement noise
Eigen::Matrix<double, 4,4> R; // process noise
Eigen::Matrix<double, 2,4> C; // State to measurement
double damping_term;

};

