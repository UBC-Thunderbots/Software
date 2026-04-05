#pragma once

#include <Eigen/Dense>
#include <Software/sensor_fusion/filter/kalman_filter.h>

class KalmanFilter{

	KalmanFilter::KalmanFilter(
	const Eigen::Matrix<double, 4,1>& X,
	const Eigen::Matrix<double, 4,4>& P,
	const Eigen::Matrix<double, 2,2>& Q,
	const Eigen::Matrix<double, 4,4>& R,
	const Eigen::Matrix<double, 2,4>& C
			): 
	X(X),
	P(P),
	Q(Q),
	R(R),
	C(C)
	{

	}

void predict(double delta_t){
	
	const Eigen::Matrix<double, 4,4> A;

	//Initialize motion model with constant velocity model
	A << 1, 0, delta_t, 0,
		 0, 1, 0, delta_t,
		 0, 0, damping_term, 0,
		 0, 0, 0, damping_term;

	X = A*X

	P = A*P*A.transpose() + R
}

void update(Eigen::Matrix<double,2,1> measurement){

}

Eigen::Matrix<double, 4,1> getState(){
	return X;
}

Eigen::Matrix<double, 4,4> getCovariance(){
	return P;
}



