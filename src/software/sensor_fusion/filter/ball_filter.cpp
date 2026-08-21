#include "software/sensor_fusion/filter/ball_filter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include "shared/constants.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/circle.h"
#include "software/geom/geom_constants.h"
#include "software/geom/segment.h"

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
    // this are treated as outliers and not fed to the filter. The measurement is 2D, so
    // this is a chi-square quantile with 2 degrees of freedom: 9.21 keeps 99% of
    // correctly predicted detections, 13.82 keeps 99.9%. Tightening it much below this
    // starts throwing away good data whenever the motion model is briefly wrong, which
    // it always is for a frame or two after a kick.
    constexpr double MAHALANOBIS_GATE_THRESHOLD = 9.21;

    // The fastest we will believe the ball could be travelling when deciding whether a
    // detection could plausibly belong to it. This is deliberately well above the 6.5 m/s
    // rule limit; the gate exists to reject detections that are physically impossible,
    // not to enforce the rules on a ball that has been kicked too hard.
    constexpr double MAX_BALL_SPEED_M_PER_S = 15.0;

    // Slack on the max ball speed gate, so that vision noise on a ball that has been
    // sitting still cannot by itself push a detection out of reach of the estimate
    constexpr double MAX_BALL_SPEED_GATE_TOLERANCE_M = 0.05;

    // The fraction of its speed the ball retains when it bounces off something
    constexpr double COLLISION_RESTITUTION = 0.6;

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
    const std::vector<BallDetection>& new_ball_detections, const Field& field,
    const std::vector<Robot>& robots, const Timestamp& current_time)
{
    const std::optional<BallDetection> best_ball_detection =
        getBestBallDetection(new_ball_detections, field.fieldBoundary());

	// We record position before prediction, to compute segment travelled within a frame. This is used in collision handling
    const Point position_before_predict(kalman_filter.state_estimate(0),
                                        kalman_filter.state_estimate(1));
    if (last_predict_timestamp)
    {
        predict((current_time - *last_predict_timestamp).toSeconds());
    }
    last_predict_timestamp = current_time;
    handleCollisions(position_before_predict, robots, field);

	// We use the detection if there is any
    if (best_ball_detection)
    {
        Measurement measurement(best_ball_detection->position.x(),
                                best_ball_detection->position.y());

		// Two gates determining whether we take the detection:
		// 1. Whether it is physically possible to arrive the new destination
		// 2. Statistical gating using mahalanobis
        if (isWithinMaxBallSpeed(best_ball_detection->position, current_time) &&
            kalman_filter.mahalanobisDistance(measurement) < MAHALANOBIS_GATE_THRESHOLD)
        {
            kalman_filter.update(measurement);
            consecutive_outliers     = 0;
            prev_measurement         = measurement;
            prev_detection_timestamp = current_time;
        }
		// If rejected, accumulate outliers. Once a threshold is reached we reset to adapt to new position
        else
        {
            consecutive_outliers++;

            if (consecutive_outliers > CONSECUTIVE_OUTLIERS_THRESHOLD)
            {
                reset(measurement, current_time);
            }
        }
    }

	// if there isn't a detection we report nothing
	// This is handled here because the code above might reject the incoming detection
    if (!prev_detection_timestamp)
    {
        return std::nullopt;
    }

	// Returns the ball
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
    const double velocity_retained = std::pow(DAMPING, delta_t);

    kalman_filter.process_model << 1, 0, delta_t, 0, 0, 1, 0, delta_t, 0, 0,
        velocity_retained, 0, 0, 0, 0, velocity_retained;

   // We compute the process covariance with the Discrete White Noise Acceleration model.
   // It depends on delta_t, so we compute it dynamically based on time passed since last prediction
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

	// Actual prediction step
    kalman_filter.predict(Eigen::Vector<double, CONTROL_SIZE>::Zero());
}

void BallFilter::handleCollisions(const Point& previous_position,
                                  const std::vector<Robot>& robots, const Field& field)
{
    const Point ball_position(kalman_filter.state_estimate(0),
                              kalman_filter.state_estimate(1));
    const Vector ball_velocity(kalman_filter.state_estimate(2),
                               kalman_filter.state_estimate(3));

	// Using the position before and after the model prediction step, we construct a segment
    const Segment ball_path(previous_position, ball_position);

	// Outwatd normal of object in contact, if there is any. We need this for rebouncing velocity
    std::optional<Vector> contact_normal;

    const double robot_collision_distance =
        ROBOT_MAX_RADIUS_METERS + BALL_MAX_RADIUS_METERS;

    // Robots are the one obstacle we treat as round, so the surface normal points
    // straight out from the robot's centre through the point of contact
    for (const Robot& robot : robots)
    {
        if (!intersects(ball_path, Circle(robot.position(), robot_collision_distance)))
        {
            continue;
        }

        const Point contact_point  = closestPoint(robot.position(), ball_path);
        const Vector robot_to_ball = contact_point - robot.position();

        // The ball passed exactly over the robot's centre, so there is no direction to
        // bounce it in
        if (robot_to_ball.length() < FIXED_EPSILON)
        {
            continue;
        }

        contact_normal = robot_to_ball.normalize();
        break;
    }

	// If we still haven't found a contact, we check the goals
	// This only checks the net, and two posts
    if (!contact_normal)
    {
        const std::array<std::pair<Rectangle, double>, 2> goals = {
            std::pair(field.friendlyGoal(), field.friendlyGoal().xMin()),
            std::pair(field.enemyGoal(), field.enemyGoal().xMax())};

        const std::vector<Segment>& walls = field.fieldBoundary().getSegments();

        std::vector<Segment> barriers;
        barriers.reserve(goals.size() * 3 + walls.size());

        for (const auto& [goal, back_x] : goals)
        {
            barriers.emplace_back(Point(back_x, goal.yMin()), Point(back_x, goal.yMax()));
            barriers.emplace_back(Point(goal.xMin(), goal.yMax()),
                                  Point(goal.xMax(), goal.yMax()));
            barriers.emplace_back(Point(goal.xMin(), goal.yMin()),
                                  Point(goal.xMax(), goal.yMin()));
        }

        barriers.insert(barriers.end(), walls.begin(), walls.end());

        for (const Segment& barrier : barriers)
        {
            // Either the ball crossed the barrier this frame, or it is sitting against it
            // with too little speed for the path to reach across
            if (!intersects(ball_path, barrier) &&
                distance(ball_path.getEnd(), barrier) > BALL_MAX_RADIUS_METERS)
            {
                continue;
            }

            const Vector barrier_direction = barrier.getEnd() - barrier.getStart();

            if (barrier_direction.length() < FIXED_EPSILON)
            {
                continue;
            }

            Vector normal = barrier_direction.perpendicular().normalize();

            // A segment has two perpendiculars; we want the one pointing back towards the
            // side the ball approached from
            const Vector barrier_to_ball =
                ball_path.getStart() - closestPoint(ball_path.getStart(), barrier);

            if (normal.dot(barrier_to_ball) < 0)
            {
                normal = -normal;
            }

            contact_normal = normal;
            break;
        }
    }

    // The ball is in free flight, so the motion model still describes it and there is
    // nothing to correct
    if (!contact_normal)
    {
        return;
    }

	// If the function hasn't returned by now, the ball is in contact we something. We widen 
	// the covaiance as we can't trust the physics model anymore; We must trust the measurement 
	// as the ball is being moved by an external entirty
    kalman_filter.state_covariance = INITIAL_COVARIANCE;

    const double approach_speed = ball_velocity.dot(*contact_normal);

    // The ball is already moving away from the surface, so it has either bounced already
    // or is rolling out of the contact under its own momentum. There is nothing to
    // reflect, but the widened covariance above still stands.
    if (approach_speed >= 0)
    {
        return;
    }

    const Vector reflected_velocity =
        (ball_velocity - *contact_normal * (2 * approach_speed)) * COLLISION_RESTITUTION;

    kalman_filter.state_estimate(2) = reflected_velocity.x();
    kalman_filter.state_estimate(3) = reflected_velocity.y();
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
    // Start the estimate at rest. Differencing two measurements to seed a velocity
    // divides vision noise by a very short timestep, and the pair either side of a
    // rejection streak is the least trustworthy pair to difference. The wide covariance
    // below lets the next few detections pull the velocity in on their own.
    kalman_filter.state_estimate << measurement(0), measurement(1), 0, 0;
    kalman_filter.state_covariance = INITIAL_COVARIANCE;

    consecutive_outliers = 0;
    // The reset measurement is now what the estimate is built on, so it becomes the
    // reference for the next timestep. Leaving the old timestamp here would make the
    // next predict() jump forward by the whole rejection streak.
    prev_measurement         = measurement;
    prev_detection_timestamp = current_time;
    last_predict_timestamp   = current_time;
}
