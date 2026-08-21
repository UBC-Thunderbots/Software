#include "software/sensor_fusion/filter/ball_filter.h"

#include <algorithm>
#include <vector>

#include "shared/constants.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/geom_constants.h"

namespace
{
    // The ball starts out unknown, so the initial estimate is given a covariance wide
    // enough to cover anywhere on the field it might be and any speed it might legally be
    // moving at. This makes the filter trust the first detections it sees almost
    // entirely, letting it converge onto the ball within a few frames.
    constexpr double INITIAL_POSITION_UNCERTAINTY_M       = 1.0;
    constexpr double INITIAL_VELOCITY_UNCERTAINTY_M_PER_S = 6.5;
    const Eigen::Vector<double, 4> INITIAL_STATE = Eigen::Vector<double, 4>::Zero();
    const Eigen::Matrix<double, 4, 4> INITIAL_COVARIANCE =
        Eigen::Vector<double, 4>(
            INITIAL_POSITION_UNCERTAINTY_M * INITIAL_POSITION_UNCERTAINTY_M,
            INITIAL_POSITION_UNCERTAINTY_M * INITIAL_POSITION_UNCERTAINTY_M,
            INITIAL_VELOCITY_UNCERTAINTY_M_PER_S * INITIAL_VELOCITY_UNCERTAINTY_M_PER_S,
            INITIAL_VELOCITY_UNCERTAINTY_M_PER_S * INITIAL_VELOCITY_UNCERTAINTY_M_PER_S)
            .asDiagonal();

    // The standard deviation of the acceleration that the constant velocity motion model
    // does not account for: deflections, uneven turf, and the tail of a kick. A kick
    // itself is far larger than this, but it is also abrupt enough that the outlier gates
    // catch it and reset the filter, so this does not need to cover one.
    constexpr double ACCELERATION_NOISE_M_PER_S_SQUARED = 5.0;

    // How noisy we expect SSL Vision's ball position detections to be. Measure this by
    // logging a stationary ball and taking the standard deviation of the detections.
    constexpr double VISION_NOISE_M = 0.01;
    const Eigen::Matrix<double, 2, 2> MEASUREMENT_COVARIANCE =
        Eigen::Matrix<double, 2, 2>::Identity() * (VISION_NOISE_M * VISION_NOISE_M);

    // Vision measures the ball's position but not its velocity
    const Eigen::Matrix<double, 2, 4> MEASUREMENT_MODEL =
        (Eigen::Matrix<double, 2, 4>() << 1, 0, 0, 0, 0, 1, 0, 0).finished();

    // The fraction of its velocity the ball retains each second as it rolls, accounting
    // for friction. Empirically measured.
    constexpr double DAMPING = 0.9889;

    // Detections whose squared Mahalanobis distance from the current estimate exceeds
    // this are treated as outliers and not fed to the filter
    constexpr double MAHALANOBIS_GATE_THRESHOLD = 5;

    // The fastest we will believe the ball could be travelling when deciding whether a
    // detection could plausibly belong to it. This is deliberately well above the 6.5 m/s
    // rule limit; the gate exists to reject detections that are physically impossible,
    // not to enforce the rules on a ball that has been kicked too hard.
    constexpr double MAX_BALL_SPEED_M_PER_S = 15.0;

    // Slack on the max ball speed gate, so that vision noise on a ball that has been
    // sitting still cannot by itself push a detection out of reach of the estimate
    constexpr double MAX_BALL_SPEED_GATE_TOLERANCE_M = 0.05;

    // The fraction of its speed the ball retains when it bounces off a robot
    constexpr double ROBOT_COLLISION_RESTITUTION = 0.6;

    // How many detections in a row may be rejected as outliers before we conclude the
    // estimate itself is wrong and reset onto the newest detection
    constexpr int CONSECUTIVE_OUTLIERS_THRESHOLD = 3;
}  // namespace

BallFilter::BallFilter()
    // The process model and the process covariance both depend on the length of the
    // timestep being predicted over, so they are left zeroed here and built in predict()
    : kalman_filter(INITIAL_STATE, INITIAL_COVARIANCE,
                    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Zero(),
                    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Zero(),
                    Eigen::Matrix<double, STATE_SIZE, CONTROL_SIZE>::Zero(),
                    MEASUREMENT_MODEL, MEASUREMENT_COVARIANCE),
      consecutive_outliers(0)
{
}

std::optional<Ball> BallFilter::estimateBallState(
    const std::vector<BallDetection>& new_ball_detections, const Rectangle& filter_area,
    const std::vector<Robot>& robots, const Timestamp& current_time)
{
    const std::optional<BallDetection> best_ball_detection =
        getBestBallDetection(new_ball_detections, filter_area);

    // Coast the estimate forward to the current time. Doing this before considering the
    // new detection means the filter keeps producing a sensible ball even on frames
    // where the ball is occluded and there is no detection at all.
    //
    // This advances from the last time we predicted rather than from the last accepted
    // detection, so that a run of frames without one coasts the estimate forward by the
    // elapsed time once rather than re-integrating the whole gap on every frame.
    if (last_predict_timestamp)
    {
        predict((current_time - *last_predict_timestamp).toSeconds());
    }
    last_predict_timestamp = current_time;

    // A ball that has run into a robot is not following the motion model any more, so
    // correct for the bounce before comparing the prediction against the new detection
    handleRobotCollisions(robots);

    if (best_ball_detection)
    {
        Measurement measurement(best_ball_detection->position.x(),
                                best_ball_detection->position.y());

        if (isWithinMaxBallSpeed(best_ball_detection->position, current_time) &&
            kalman_filter.mahalanobisDistance(measurement) < MAHALANOBIS_GATE_THRESHOLD)
        {
            kalman_filter.update(measurement);
            consecutive_outliers     = 0;
            prev_measurement         = measurement;
            prev_detection_timestamp = current_time;
        }
        else
        {
            consecutive_outliers++;

            // We have rejected too many detections in a row to still believe our own
            // estimate, so throw it away and start again from what vision is telling us
            if (consecutive_outliers > CONSECUTIVE_OUTLIERS_THRESHOLD)
            {
                reset(measurement, current_time);
            }
        }
    }

    // The filter has never seen a usable detection, so it has nothing to report. Vision
    // has not told us where the ball is even once, and the zeroed initial state is not a
    // real estimate.
    if (!prev_detection_timestamp)
    {
        return std::nullopt;
    }

    const Eigen::Vector<double, STATE_SIZE> state = kalman_filter.state_estimate;
    const Point ball_position(state(0), state(1));
    const Vector ball_velocity(state(2), state(3));
    const double distance_from_ground =
        best_ball_detection ? best_ball_detection->distance_from_ground : 0.0;

    return Ball(BallState(ball_position, ball_velocity, distance_from_ground),
                current_time);
}

std::optional<BallDetection> BallFilter::getBestBallDetection(
    const std::vector<BallDetection>& new_ball_detections, const Rectangle& filter_area)
{
    std::vector<BallDetection> detections_in_filter_area;
    std::copy_if(new_ball_detections.begin(), new_ball_detections.end(),
                 std::back_inserter(detections_in_filter_area),
                 [&filter_area](const BallDetection& detection)
                 { return contains(filter_area, detection.position); });

    if (detections_in_filter_area.empty())
    {
        return std::nullopt;
    }

    return *std::max_element(detections_in_filter_area.begin(),
                             detections_in_filter_area.end(),
                             [](const BallDetection& a, const BallDetection& b)
                             { return a.confidence < b.confidence; });
}

void BallFilter::predict(double delta_t)
{
    // Constant velocity motion model, with the ball's velocity decaying by the damping
    // term as it rolls
    kalman_filter.process_model << 1, 0, delta_t, 0, 0, 1, 0, delta_t, 0, 0, DAMPING, 0,
        0, 0, 0, DAMPING;

    // How much we expect the motion model to be wrong by over this timestep, modelled as
    // white noise on the ball's acceleration. Position error accumulates as delta_t^4 and
    // velocity error as delta_t^2, and the two are correlated because the same
    // acceleration error integrates into both. Rebuilding this every timestep rather than
    // fixing it at one frame rate means a longer gap between frames widens the estimate
    // by the right amount instead of understating how little we know.
    const double acceleration_variance =
        ACCELERATION_NOISE_M_PER_S_SQUARED * ACCELERATION_NOISE_M_PER_S_SQUARED;
    const double delta_t_squared = delta_t * delta_t;
    const double position_noise  = acceleration_variance * delta_t_squared *
                                  delta_t_squared / 4.0;
    const double correlation_noise =
        acceleration_variance * delta_t_squared * delta_t / 2.0;
    const double velocity_noise = acceleration_variance * delta_t_squared;

    kalman_filter.process_covariance << position_noise, 0, correlation_noise, 0, 0,
        position_noise, 0, correlation_noise, correlation_noise, 0, velocity_noise, 0, 0,
        correlation_noise, 0, velocity_noise;

    kalman_filter.predict(Eigen::Vector<double, CONTROL_SIZE>::Zero());
}

void BallFilter::handleRobotCollisions(const std::vector<Robot>& robots)
{
    const Point ball_position(kalman_filter.state_estimate(0),
                              kalman_filter.state_estimate(1));
    const Vector ball_velocity(kalman_filter.state_estimate(2),
                               kalman_filter.state_estimate(3));

    // BALL_MAX_RADIUS_METERS is not constexpr, so neither is this
    const double collision_distance = ROBOT_MAX_RADIUS_METERS + BALL_MAX_RADIUS_METERS;

    for (const Robot& robot : robots)
    {
        const Vector robot_to_ball = ball_position - robot.position();

        // The ball is not touching this robot, or it is sitting exactly on top of the
        // robot's centre and there is no direction to bounce it in
        if (robot_to_ball.length() > collision_distance ||
            robot_to_ball.length() < FIXED_EPSILON)
        {
            continue;
        }

        // The surface we are bouncing off points from the robot out towards the ball
        const Vector collision_normal = robot_to_ball.normalize();
        const double approach_speed   = ball_velocity.dot(collision_normal);

        // The ball is already moving away from the robot, so it has either bounced
        // already or is rolling out of the collision under its own momentum
        if (approach_speed >= 0)
        {
            continue;
        }

        const Vector reflected_velocity =
            (ball_velocity - collision_normal * (2 * approach_speed)) *
            ROBOT_COLLISION_RESTITUTION;

        kalman_filter.state_estimate(2) = reflected_velocity.x();
        kalman_filter.state_estimate(3) = reflected_velocity.y();

        // How the ball really came off the robot depends on the spin it had and where on
        // the hull it hit, neither of which we know, so widen the estimate back out and
        // let the next few detections pin it down again
        kalman_filter.state_covariance = INITIAL_COVARIANCE;

        // One bounce per frame. A ball wedged between two robots would otherwise have its
        // velocity reflected twice and come back out pointing the way it came in.
        break;
    }
}

bool BallFilter::isWithinMaxBallSpeed(const Point& detection_position,
                                      const Timestamp& current_time) const
{
    // Without a previous detection there is no interval to reason over, so we have no
    // grounds to call this one impossible
    if (!prev_detection_timestamp)
    {
        return true;
    }

    const double delta_t = (current_time - *prev_detection_timestamp).toSeconds();
    const Point predicted_position(kalman_filter.state_estimate(0),
                                   kalman_filter.state_estimate(1));
    const double reachable_distance = MAX_BALL_SPEED_M_PER_S * std::max(delta_t, 0.0) +
                                      MAX_BALL_SPEED_GATE_TOLERANCE_M;

    return (detection_position - predicted_position).length() <= reachable_distance;
}

void BallFilter::reset(const Measurement& measurement, const Timestamp& current_time)
{
    // If we have a previous measurement to compare against we can seed a velocity from
    // it, otherwise we can only say where the ball is and not where it is going
    Vector velocity(0, 0);
    if (prev_measurement && prev_detection_timestamp)
    {
        const double delta_t = (current_time - *prev_detection_timestamp).toSeconds();
        if (delta_t > 0)
        {
            velocity = Vector((measurement(0) - (*prev_measurement)(0)) / delta_t,
                              (measurement(1) - (*prev_measurement)(1)) / delta_t);
        }
    }

    kalman_filter.state_estimate << measurement(0), measurement(1), velocity.x(),
        velocity.y();
    kalman_filter.state_covariance = INITIAL_COVARIANCE;

    consecutive_outliers = 0;
    // The reset measurement is now what the estimate is built on, so it becomes the
    // reference for the next timestep. Leaving the old timestamp here would make the
    // next predict() jump forward by the whole rejection streak.
    prev_measurement         = measurement;
    prev_detection_timestamp = current_time;
    last_predict_timestamp   = current_time;
}
