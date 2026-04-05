#include "software/sensor_fusion/filter/ball_filter.h"

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

#include "shared/constants.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/math/math_functions.h"
#include "software/sensor_fusion/filter/kalman_filter.h"

namespace {
    const Eigen::Matrix<double,4,1> INITIAL_STATE = Eigen::Matrix<double,4,1>::Zero();
    
    const Eigen::Matrix<double,4,4> INITIAL_COV = Eigen::Matrix<double,4,4>::Identity() * 1000.0;
    
    const Eigen::Matrix<double,2,2> Q = (Eigen::Matrix<double,2,2>() << 
        0.1, 0,
        0, 0.1).finished();
    
    const Eigen::Matrix<double,4,4> R = (Eigen::Matrix<double,4,4>() <<
        0.1, 0, 0, 0,
        0, 0.1, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.01).finished();
    
    const Eigen::Matrix<double,2,4> C = (Eigen::Matrix<double,2,4>() <<
        1, 0, 0, 0,
        0, 1, 0, 0).finished();
}

BallFilter::BallFilter() :
	mahalanobis_count(0),
	kalman_filter(INITIAL_STATE, INITIAL_COV, Q, R, C,0.9)
{
}

std::optional<Ball> BallFilter::estimateBallState(
    const std::vector<BallDetection> &new_ball_detections, const Rectangle &filter_area, const Timestamp& current_time)
{
	BallDetection best_ball_detection = getBestBallDetection(new_ball_detections);	

	if (prev_detection_timestamp){
		double delta_t = (current_time - prev_detection_timestamp).toSeconds();
		kalman_filter.predict(delta_t);
	}
	if (best_ball_detection){
		prev_detection_timestamp = best_ball_detection.timestamp;
		Eigen::Matrix<double,2,1> measurement;
		measurement << best_ball_detection->position.x(), best_ball_detection->position.y();
		double mahalanobis = kalman_filter.getMahalanobisDistance(measurement);
		if (mahalanobis< mahalanobis_threshold){
			kalman_filter.update(measurement);
		}
		else{
			consecutive_outliers++;
		}

		if (consecutive_outliers> mahalanobis_count_threshold){
			kalman_filter.reset(measurement);
			consecutive_outliers=0;
		}
	}


	Eigen::Matrix<double,4,1> kalman_state = kalman_filter.getState();
	Point ball_position = Point(kalman_state(0), kalman_state(1));
	Vector ball_velocity = Vector(kalman_state(2), kalman_state(3));
	double z_height = best_ball_detection->distance_from_ground if best_ball_detection else 0.0;
		
    BallState ball_state(ball_position, ball_velocity, z_height);
    return Ball(ball_state,current_time);
	
}
std::optional<BallDetection> BallFilter::getBestBallDetection(const std::vector<BallDetection> &new_ball_detections){
	if (new_ball_detections.empty()){
		return std::nullopt;
	}
	else{
		return *std::max_element(new_ball_detections.begin(), new_ball_detections.end(),[](const BallDetection& a, const BallDetection& b){
					return a.confidence<b.confidence;
				});
	}
}


