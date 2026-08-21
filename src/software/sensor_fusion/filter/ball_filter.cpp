#include "software/sensor_fusion/filter/ball_filter.h"

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

#include "shared/constants.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/circle.h"
#include "software/geom/geom_constants.h"
#include "software/logger/logger.h"

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

    // Coast the estimate forward to the current time. Doing this before considering the
    // new detection means the filter keeps producing a sensible ball even on frames
    // where the ball is occluded and there is no detection at all.
    //
    // This advances from the last time we predicted rather than from the last accepted
    // detection, so that a run of frames without one coasts the estimate forward by the
    // elapsed time once rather than re-integrating the whole gap on every frame.
    const Point position_before_predict(kalman_filter.state_estimate(0),
                                        kalman_filter.state_estimate(1));

    if (last_predict_timestamp)
    {
        predict((current_time - *last_predict_timestamp).toSeconds());
    }
    last_predict_timestamp = current_time;

    // A ball touching anything is not following the motion model any more, so correct for
    // the contact before comparing the prediction against the new detection
    handleCollisions(position_before_predict, robots, field);

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

void BallFilter::handleCollisions(const Point& previous_position,
                                  const std::vector<Robot>& robots, const Field& field)
{
    const Point ball_position(kalman_filter.state_estimate(0),
                              kalman_filter.state_estimate(1));
    const Vector ball_velocity(kalman_filter.state_estimate(2),
                               kalman_filter.state_estimate(3));

    const std::optional<Collision> collision =
        getCollision(Segment(previous_position, ball_position), robots, field);

    if (!collision)
    {
        return;
    }

    // The ball is touching something, so the motion model no longer describes what it is
    // about to do and the covariance we have built up is not justified. Widen it back out
    // and let the next few detections pin the ball down again. This happens on any
    // contact, including the ones we cannot usefully reflect off below.
    kalman_filter.state_covariance = INITIAL_COVARIANCE;

    const double approach_speed = ball_velocity.dot(collision->normal);

    // The ball is already moving away from the surface, so it has either bounced already
    // or is rolling out of the contact under its own momentum. There is nothing to
    // reflect, but the widened covariance above still stands.
    if (approach_speed >= 0)
    {
        return;
    }

    const Vector reflected_velocity =
        (ball_velocity - collision->normal * (2 * approach_speed)) *
        COLLISION_RESTITUTION;

    kalman_filter.state_estimate(2) = reflected_velocity.x();
    kalman_filter.state_estimate(3) = reflected_velocity.y();

}

std::vector<std::pair<Segment, std::string_view>> BallFilter::getBarriers(
    const Field& field)
{
    std::vector<std::pair<Segment, std::string_view>> barriers;

    // A goal is a frame, not a box. Its mouth is an opening the ball travels through, so
    // only the back of the net and the two posts can be bounced off; treating the mouth
    // as a surface would reflect every shot straight back out of the goal it just entered.
    const auto add_goal = [&barriers](const Rectangle& goal, double back_x,
                                      std::string_view name)
    {
        barriers.emplace_back(
            Segment(Point(back_x, goal.yMin()), Point(back_x, goal.yMax())), name);
        barriers.emplace_back(
            Segment(Point(goal.xMin(), goal.yMax()), Point(goal.xMax(), goal.yMax())),
            name);
        barriers.emplace_back(
            Segment(Point(goal.xMin(), goal.yMin()), Point(goal.xMax(), goal.yMin())),
            name);
    };

    // Each goal sits outside the field lines, so the back of the net is the edge further
    // from the centre of the field
    add_goal(field.friendlyGoal(), field.friendlyGoal().xMin(), "the friendly goal");
    add_goal(field.enemyGoal(), field.enemyGoal().xMax(), "the enemy goal");

    for (const Segment& wall : field.fieldBoundary().getSegments())
    {
        barriers.emplace_back(wall, "the field boundary");
    }

    return barriers;
}

std::optional<BallFilter::Collision> BallFilter::getCollision(
    const Segment& ball_path, const std::vector<Robot>& robots, const Field& field)
{
    // BALL_MAX_RADIUS_METERS is not constexpr, so neither is this
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

        const Point contact_point   = closestPoint(robot.position(), ball_path);
        const Vector robot_to_ball  = contact_point - robot.position();

        // The ball passed exactly over the robot's centre, so there is no direction to
        // bounce it in
        if (robot_to_ball.length() < FIXED_EPSILON)
        {
            continue;
        }

        return Collision{.normal = robot_to_ball.normalize(), .object = "a robot"};
    }

    for (const auto& [barrier, name] : getBarriers(field))
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

        return Collision{.normal = normal, .object = name};
    }

    return std::nullopt;
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
