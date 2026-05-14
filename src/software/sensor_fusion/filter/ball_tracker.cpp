#include "software/sensor_fusion/filter/ball_tracker.h"

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

#include "shared/constants.h"
#include "software/constants.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/math/math_functions.h"
#include "software/sensor_fusion/filter/kalman_filter.h"

namespace {
    const Eigen::Vector<double, 4> INITIAL_STATE = Eigen::Vector<double, 4>::Zero();

    const Eigen::Matrix<double,4,4> INITIAL_COV = Eigen::Matrix<double,4,4>::Identity() * 1000.0;

	const Eigen::Matrix<double,4,4> Q = (Eigen::Matrix<double,4,4>() <<
	    2.222e-8, 0,        2.000e-6, 0,
	    0,        2.222e-8, 0,        2.000e-6,
	    2.000e-6, 0,        2.400e-4, 0,
	    0,        2.000e-6, 0,        2.400e-4).finished();

    const Eigen::Matrix<double,2,2> R = (Eigen::Matrix<double,2,2>() <<
        0.0226, 0,
        0, 0.00445).finished();

    const Eigen::Matrix<double,2,4> C = (Eigen::Matrix<double,2,4>() <<
        1, 0, 0, 0,
        0, 1, 0, 0).finished();

    // Empirically measured
	const double DAMPING = 0.9889;

	const double MAHANALOGIS_THRESHOLD = 1;
	const int CONSECUTIVE_OUTLIERS_THRESHOLD = 3;

    const double DRIBBLING_MEASUREMENT_MAX_DISTANCE_METERS = 0.08;
}

BallTracker::BallTracker() :
	consecutive_outliers(0),
	kalman_filter(INITIAL_STATE, INITIAL_COV,
	              Eigen::Matrix<double,4,4>::Identity(),
	              Q,
	              Eigen::Matrix<double,4,1>::Zero(),
	              C, R)
{
}

std::optional<Ball> BallTracker::estimateBallState(
    const std::vector<BallDetection> &new_ball_detections, const Rectangle &filter_area,
    const Timestamp& current_time, std::optional<Robot> dribbling_robot)
{
    Point dribbler_pos;
    if (dribbling_robot.has_value())
    {
        dribbler_pos =
            dribbling_robot->position() +
            Vector::createFromAngle(dribbling_robot->orientation())
                .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                           BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING);
    }

	std::optional<BallDetection> best_ball_detection = getBestBallDetection(new_ball_detections);

    double dt = 0.0;
	if (prev_detection_timestamp){
		dt = (current_time - *prev_detection_timestamp).toSeconds();
		prev_detection_timestamp = current_time;

		Eigen::Matrix<double,4,4> A;
		A << 1, 0, dt, 0,
		     0, 1, 0,  dt,
		     0, 0, DAMPING, 0,
		     0, 0, 0,       DAMPING;
		kalman_filter.process_model = A;
		kalman_filter.predict(Eigen::Vector<double,1>::Zero());
	}

    if (dribbling_robot.has_value())
    {
        bool near_dribbler =
            best_ball_detection.has_value() &&
            (best_ball_detection->position - dribbler_pos).length() <=
                DRIBBLING_MEASUREMENT_MAX_DISTANCE_METERS;

        Eigen::Vector<double, 2> measurement;
        if (near_dribbler)
        {
            measurement << best_ball_detection->position.x(),
                best_ball_detection->position.y();
            kalman_filter.update(measurement);
        }
        else
        {
            measurement << dribbler_pos.x(), dribbler_pos.y();
            kalman_filter.state_estimate << measurement(0), measurement(1), 0, 0;
            kalman_filter.state_covariance = INITIAL_COV;
        }
        prev_detection_timestamp = current_time;
        consecutive_outliers     = 0;
    }
	else if (best_ball_detection){
		Eigen::Vector<double, 2> measurement;
		measurement << best_ball_detection->position.x(), best_ball_detection->position.y();

		Point predicted_pos(kalman_filter.state_estimate(0), kalman_filter.state_estimate(1));
		if (!contains(filter_area, predicted_pos)){
			kalman_filter.state_estimate << measurement(0), measurement(1), 0, 0;
			kalman_filter.state_covariance = INITIAL_COV;
			consecutive_outliers = 0;
		}

		const Eigen::Vector<double, 2> innovation =
		    measurement - kalman_filter.measurement_model * kalman_filter.state_estimate;
		const Eigen::Matrix<double, 2, 2> S =
		    kalman_filter.measurement_model * kalman_filter.state_covariance *
		    kalman_filter.measurement_model.transpose() + kalman_filter.measurement_covariance;
		double mahalanobis = (innovation.transpose() * S.inverse() * innovation)(0, 0);

		if (mahalanobis < MAHANALOGIS_THRESHOLD){
			kalman_filter.update(measurement);
		}
		else{
			consecutive_outliers++;
		}

		if (consecutive_outliers > CONSECUTIVE_OUTLIERS_THRESHOLD){
			kalman_filter.state_estimate << measurement(0), measurement(1), 0, 0;
			kalman_filter.state_covariance = INITIAL_COV;
			consecutive_outliers = 0;
		}
		prev_detection_timestamp = current_time;
	}

	Point ball_position = Point(kalman_filter.state_estimate(0), kalman_filter.state_estimate(1));
	Vector ball_velocity = Vector(kalman_filter.state_estimate(2), kalman_filter.state_estimate(3));
	double z_height = best_ball_detection ? best_ball_detection->distance_from_ground : 0.0;

    BallState ball_state(ball_position, ball_velocity, z_height);
    return Ball(ball_state, current_time);
}

std::optional<BallDetection> BallTracker::getBestBallDetection(const std::vector<BallDetection> &new_ball_detections){
	if (new_ball_detections.empty()){
		return std::nullopt;
	}
	else{
		return *std::max_element(new_ball_detections.begin(), new_ball_detections.end(),[](const BallDetection& a, const BallDetection& b){
					return a.confidence<b.confidence;
				});
	}
}
