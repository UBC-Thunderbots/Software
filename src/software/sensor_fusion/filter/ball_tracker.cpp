#include "software/sensor_fusion/filter/ball_tracker.h"

#include <Eigen/Dense>
#include "software/logger/logger.h"
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

    const Eigen::Matrix<double,4,4> INITIAL_COV = (Eigen::Matrix<double,4,4>() <<
        1000, 0,    0,  0,
        0,    1000, 0,  0,
        0,    0,    1, 0,
        0,    0,    0,  1).finished();

	const Eigen::Matrix<double,4,4> Q = (Eigen::Matrix<double,4,4>() <<
	    2.222e-8, 0,        2.000e-6, 0,
	    0,        2.222e-8, 0,        2.000e-6,
	    2.000e-6, 0,        2.400e-4, 0,
	    0,        2.000e-6, 0,        2.400e-4).finished();

    const Eigen::Matrix<double,2,2> R = (Eigen::Matrix<double,2,2>() <<
        0.0226, 0,
        0, 0.00445).finished();

    const Eigen::Matrix<double,2,2> R_DRIBBLING = (Eigen::Matrix<double,2,2>() <<
        0.0001, 0,
        0, 0.0001).finished();

    const Eigen::Matrix<double,2,4> C = (Eigen::Matrix<double,2,4>() <<
        1, 0, 0, 0,
        0, 1, 0, 0).finished();

    // Empirically measured per-second velocity retention during normal tracking
	const double DAMPING = 0.994;
    // Per-second retention when fully occluded long enough (velocity decays much faster)
    const double DAMPING_OCCLUDED = 0.3;
    // Occlusion time (seconds) before damping starts to increase
    const double OCCLUSION_GRACE_SECONDS = 1.0;
    // Occlusion time (seconds) past the grace period at which damping fully transitions to DAMPING_OCCLUDED
    const double MAX_OCCLUSION_SECONDS = 1.0;

	const double MAHANALOGIS_THRESHOLD = 1;
	const int CONSECUTIVE_OUTLIERS_THRESHOLD = 3;

    const double DRIBBLING_MEASUREMENT_MAX_DISTANCE_METERS = 0.08;

    // Speed gate for the init buffer: reject detections that imply a ball jump
    // beyond this threshold from the last known position.
    // BALL_MAX_SPEED_METERS_PER_SECOND (6.5) + 2.5 buffer for measurement error.
    const double INIT_BUFFER_MAX_SPEED_M_PER_S = BALL_MAX_SPEED_METERS_PER_SECOND + 2.5;
}

BallTracker::BallTracker() :
	consecutive_outliers(0),
	velocity_initialized(false),
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

    if (!velocity_initialized)
    {
        if (best_ball_detection)
            velocity_initialized = tryInitVelocityFromBuffer(
                best_ball_detection->position, current_time);

        const Point pos = best_ball_detection
            ? best_ball_detection->position
            : Point(kalman_filter.state_estimate(0), kalman_filter.state_estimate(1));
        const double z = best_ball_detection ? best_ball_detection->distance_from_ground : 0.0;
        return Ball(BallState(pos, Vector(0, 0), z), current_time);
    }

    double dt = 0.0;
	if (prev_detection_timestamp){
		dt = (current_time - *prev_detection_timestamp).toSeconds();
		prev_detection_timestamp = current_time;

		const double occlusion_seconds = last_measurement_timestamp.has_value()
		    ? (current_time - *last_measurement_timestamp).toSeconds()
		    : 0.0;
		const double alpha = std::min(std::max(occlusion_seconds - OCCLUSION_GRACE_SECONDS, 0.0) / MAX_OCCLUSION_SECONDS, 1.0);
		const double effective_damping = DAMPING * (1.0 - alpha) + DAMPING_OCCLUDED * alpha;

		Eigen::Matrix<double,4,4> A;
		A << 1, 0, dt, 0,
		     0, 1, 0,  dt,
		     0, 0, effective_damping, 0,
		     0, 0, 0,                 effective_damping;
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
        }
        else
        {
            measurement << dribbler_pos.x(), dribbler_pos.y();
        }
        LOG(INFO) << "[BallTracker] dribbling: tight update at ("
                  << measurement(0) << "," << measurement(1) << ")";
        kalman_filter.measurement_covariance = R_DRIBBLING;
        kalman_filter.update(measurement);
        kalman_filter.measurement_covariance = R;
        prev_detection_timestamp    = current_time;
        last_measurement_timestamp  = current_time;
        consecutive_outliers        = 0;
    }
	else if (best_ball_detection){
		Eigen::Vector<double, 2> measurement;
		measurement << best_ball_detection->position.x(), best_ball_detection->position.y();

		Point predicted_pos(kalman_filter.state_estimate(0), kalman_filter.state_estimate(1));
		if (!contains(filter_area, predicted_pos)){
			LOG(INFO) << "[BallTracker] predicted pos out of bounds, reset to measurement";
			kalman_filter.state_estimate << measurement(0), measurement(1), 0, 0;
			kalman_filter.state_covariance = INITIAL_COV;
			consecutive_outliers = 0;
			velocity_initialized = false;
			velocity_init_buffer.clear();
		}

		const Eigen::Vector<double, 2> innovation =
		    measurement - kalman_filter.measurement_model * kalman_filter.state_estimate;
		const Eigen::Matrix<double, 2, 2> S =
		    kalman_filter.measurement_model * kalman_filter.state_covariance *
		    kalman_filter.measurement_model.transpose() + kalman_filter.measurement_covariance;
		double mahalanobis = (innovation.transpose() * S.inverse() * innovation)(0, 0);

		if (mahalanobis < MAHANALOGIS_THRESHOLD){
			kalman_filter.update(measurement);
			last_measurement_timestamp = current_time;
		}
		else{
			LOG(INFO) << "[BallTracker] outlier rejected (mahalanobis=" << mahalanobis << ", consecutive=" << consecutive_outliers + 1 << ")";
			consecutive_outliers++;
		}

		if (consecutive_outliers > CONSECUTIVE_OUTLIERS_THRESHOLD){
			LOG(INFO) << "[BallTracker] too many consecutive outliers, hard reset to measurement";
			kalman_filter.state_estimate << measurement(0), measurement(1), 0, 0;
			kalman_filter.state_covariance = INITIAL_COV;
			last_measurement_timestamp = current_time;
			consecutive_outliers = 0;
			velocity_initialized = false;
			velocity_init_buffer.clear();
		}
		prev_detection_timestamp = current_time;
	}

	Point ball_position = Point(kalman_filter.state_estimate(0), kalman_filter.state_estimate(1));
	Vector ball_velocity = Vector(kalman_filter.state_estimate(2), kalman_filter.state_estimate(3));
	double z_height = best_ball_detection ? best_ball_detection->distance_from_ground : 0.0;

    BallState ball_state(ball_position, ball_velocity, z_height);
    return Ball(ball_state, current_time);
}

bool BallTracker::tryInitVelocityFromBuffer(const Point& position,
                                             const Timestamp& current_time)
{
    // Gate: reject detections that imply the ball teleported from the last
    // known position. Clear the buffer and reanchor rather than poisoning the
    // regression with a noise spike.
    if (!velocity_init_buffer.empty())
    {
        const Point last_pos(kalman_filter.state_estimate(0),
                             kalman_filter.state_estimate(1));
        const double dt_s = current_time.toSeconds() - velocity_init_buffer.back().second;
        if (dt_s > 0 &&
            (position - last_pos).length() / dt_s > INIT_BUFFER_MAX_SPEED_M_PER_S)
        {
            LOG(INFO) << "[BallTracker] init buffer: noisy detection rejected, reanchor";
            velocity_init_buffer.clear();
            kalman_filter.state_estimate(0) = position.x();
            kalman_filter.state_estimate(1) = position.y();
            prev_detection_timestamp        = current_time;
            return false;
        }
    }

    velocity_init_buffer.emplace_back(position, current_time.toSeconds());
    kalman_filter.state_estimate(0) = position.x();
    kalman_filter.state_estimate(1) = position.y();
    prev_detection_timestamp        = current_time;

    if (static_cast<int>(velocity_init_buffer.size()) < VELOCITY_INIT_BUFFER_SIZE)
        return false;

    const int N = static_cast<int>(velocity_init_buffer.size());
    double t_mean = 0.0, x_mean = 0.0, y_mean = 0.0;
    for (const auto& [pos, t] : velocity_init_buffer) {
        t_mean += t; x_mean += pos.x(); y_mean += pos.y();
    }
    t_mean /= N; x_mean /= N; y_mean /= N;

    double Sxx = 0.0, Stx = 0.0, Sty = 0.0;
    for (const auto& [pos, t] : velocity_init_buffer) {
        const double dt_i = t - t_mean;
        Sxx  += dt_i * dt_i;
        Stx  += dt_i * (pos.x() - x_mean);
        Sty  += dt_i * (pos.y() - y_mean);
    }
    if (Sxx > 1e-10)
    {
        kalman_filter.state_estimate(2)      = Stx / Sxx;
        kalman_filter.state_estimate(3)      = Sty / Sxx;
        kalman_filter.state_covariance(2, 2) = R(0, 0) / Sxx;
        kalman_filter.state_covariance(3, 3) = R(1, 1) / Sxx;
    }
    LOG(INFO) << "[BallTracker] velocity initialized from buffer: vx=" << kalman_filter.state_estimate(2) << " vy=" << kalman_filter.state_estimate(3);
    last_measurement_timestamp = current_time;
    consecutive_outliers       = 0;
    velocity_init_buffer.clear();
    return true;
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
