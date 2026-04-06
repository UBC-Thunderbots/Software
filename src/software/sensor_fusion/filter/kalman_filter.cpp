#include <software/sensor_fusion/filter/kalman_filter.h>

KalmanFilter::KalmanFilter(
const Eigen::Matrix<double, 4,1>& X,
const Eigen::Matrix<double, 4,4>& P_i,
const Eigen::Matrix<double, 2,2>& Q,
const Eigen::Matrix<double, 4,4>& R,
const Eigen::Matrix<double, 2,4>& C,
double damping_term,
		): 
X(X),
P(P_i),
P_i(P_i),
Q(Q),
R(R),
C(C),
damping_term(damping_term)
{
}

void KalmanFilter::predict(const double delta_t){
	
	Eigen::Matrix<double, 4,4> A;

	// Using the constant velocity model as motion model
	A << 1, 0, delta_t, 0,
		 0, 1, 0, delta_t,
		 0, 0, damping_term, 0,
		 0, 0, 0, damping_term;

	X = A*X;

	P = A*P*A.transpose() + R;
}

void KalmanFilter::update(const Eigen::Matrix<double,2,1> Z){
	Eigen::Matrix<double, 4,2> Kg;
	Eigen::Matrix<double,2,2> S =C*P*C.transpose()+Q;
	Kg = P*C.transpose() * S.inverse();
	X = X + Kg*(Z-C*X);
	P = (Eigen::Matrix<double,4,4>::Identity()-Kg*C)*P;
}

void KalmanFilter::reset(const Eigen::Matrix<double,2,1> Z){
	X << Z(0), Z(1), 0, 0;
	P = P_i;
}

double KalmanFilter::getMahalanobisDistance(const Eigen::Matrix<double,2,1>& Z){
	Eigen::Matrix<double,2,2> S =C*P*C.transpose()+Q ;
	// Calculate the mahalanobis distance for gating
	double M = (Z - C*X).transpose() * S.inverse() * (Z-C*X);
	return M;
}

Eigen::Matrix<double, 4,1> KalmanFilter::getState(){
	return X;
}

Eigen::Matrix<double, 4,4> KalmanFilter::getCovariance(){
	return P;
}


