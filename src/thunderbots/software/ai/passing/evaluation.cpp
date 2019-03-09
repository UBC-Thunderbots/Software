/**
 * Implementation of evaluation functions for passing
 */

#include "ai/passing/evaluation.h"

#include <numeric>

#include "../shared/constants.h"
#include "geom/util.h"
#include "util/parameter/dynamic_parameters.h"

using namespace AI::Passing;

double AI::Passing::ratePassShootScore(Field field, Team enemy_team,
                                       AI::Passing::Pass pass)
{
    double ideal_shoot_angle_degrees =
        Util::DynamicParameters::AI::Passing::ideal_min_shoot_angle_degrees.value();
    double ideal_min_rotation_to_shoot_degrees =
        Util::DynamicParameters::AI::Passing::ideal_min_rotation_to_shoot_degrees.value();

    std::vector<Point> obstacles;
    for (const Robot& robot : enemy_team.getAllRobots())
    {
        obstacles.emplace_back(robot.position());
    }

    // Figure out the range of angles for which we have an open shot to the goal after
    // receiving the pass
    Angle open_angle_to_goal =
        angleSweepCircles(pass.receiverPoint(), field.enemyGoalpostNeg(),
                          field.enemyGoalpostPos(), obstacles, ROBOT_MAX_RADIUS_METERS)
            .second;

    // Create the shoot score by creating a sigmoid that goes to a large value as
    // we get to the ideal shoot angle.
    double shot_openness_score =
        sigmoid(open_angle_to_goal.toDegrees(), 0.5 * ideal_shoot_angle_degrees,
                ideal_shoot_angle_degrees);

    // Prefer angles where the robot does not have to turn much after receiving the
    // pass to take the shot
    Angle pass_orientation = (pass.passerPoint() - pass.receiverPoint()).orientation();
    Angle post0_diff       = pass_orientation.minDiff(
        (field.enemyGoalpostNeg() - pass.receiverPoint()).orientation());
    Angle post1_diff = pass_orientation.minDiff(
        (field.enemyGoalpostPos() - pass.receiverPoint()).orientation());
    Angle min_rotation_to_shot_after_pass   = std::min(post0_diff, post1_diff);
    double required_rotation_for_shot_score = sigmoid(
        min_rotation_to_shot_after_pass.toDegrees(),
        0.5 * ideal_min_rotation_to_shoot_degrees, ideal_min_rotation_to_shoot_degrees);

    return shot_openness_score * required_rotation_for_shot_score;
}

double AI::Passing::ratePassEnemyRisk(const Team& enemy_team, const Pass& pass)
{
    double enemy_proximity_importance =
        Util::DynamicParameters::AI::Passing::enemy_proximity_importance.value();

    // Calculate a risk score based on the distance of the enemy robots from the receive
    // point, based on an exponential function of the distance of each robot from the
    // receiver point
    auto enemy_robots                    = enemy_team.getAllRobots();
    double enemy_receiver_proximity_risk = 1;
    for (const Robot& enemy : enemy_team.getAllRobots())
    {
        double dist = (pass.receiverPoint() - enemy.position()).len();
        enemy_receiver_proximity_risk *=
            enemy_proximity_importance * std::exp(-dist * dist);
    }
    if (enemy_robots.empty())
    {
        enemy_receiver_proximity_risk = 0;
    }

    double intercept_risk = calculateInterceptRisk(enemy_team, pass);

    // We want to rate a pass more highly if it is lower risk, so subtract from 1
    return 1 - std::max(intercept_risk, enemy_receiver_proximity_risk);
}

double AI::Passing::calculateInterceptRisk(const Team& enemy_team, const Pass& pass)
{
    // Return the highest risk for all the enemy robots, if there are any
    auto enemy_robots = enemy_team.getAllRobots();
    if (enemy_robots.empty())
    {
        return 0;
    }
    std::vector<double> enemy_intercept_risks;
    std::transform(enemy_robots.begin(), enemy_robots.end(),
                   std::back_inserter(enemy_intercept_risks),
                   [&](Robot robot) { return calculateInterceptRisk(robot, pass); });
    return *std::max_element(enemy_intercept_risks.begin(), enemy_intercept_risks.end());
}

double AI::Passing::calculateInterceptRisk(Robot enemy_robot, const Pass& pass)
{
    // We estimate the intercept by the risk that the robot will get to the closest
    // point on the pass before the ball, and by the risk that the robot will get to
    // the reception point before the ball. We take the greater of these two risks.
    // We assume that the enemy continues moving at it's current velocity until the
    // pass starts

    // We force any negative duration to 0 here
    Duration time_until_pass = pass.startTime() - enemy_robot.lastUpdateTimestamp();

    // Estimate where the enemy will be when we start the pass
    enemy_robot.updateStateToPredictedState(time_until_pass);

    // If the enemy cannot intercept the pass at BOTH the closest point on the pass and
    // the the receiver point for the pass, then it is guaranteed that it will not be
    // able to intercept the pass anywhere.

    // Figure out how long the enemy robot and ball will take to reach the closest
    // point on the pass to the enemy's current position
    Point closest_point_on_pass_to_robot = closestPointOnSeg(
        enemy_robot.position(), pass.passerPoint(), pass.receiverPoint());
    Duration enemy_robot_time_to_closest_pass_point =
        getTimeToPositionForRobot(enemy_robot, closest_point_on_pass_to_robot,
                                  ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
                                  ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Duration ball_time_to_closest_pass_point = Duration::fromSeconds(
        (closest_point_on_pass_to_robot - pass.passerPoint()).len() / pass.speed());

    // Figure out how long the enemy robot and ball will take to reach the receive point
    // for the pass.
    Duration enemy_robot_time_to_pass_receive_position = getTimeToPositionForRobot(
        enemy_robot, pass.receiverPoint(), ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Duration ball_time_to_pass_receive_position = pass.estimatePassDuration();

    double robot_ball_time_diff_at_closest_pass_point =
        (enemy_robot_time_to_closest_pass_point - ball_time_to_closest_pass_point)
            .getSeconds();
    double robot_ball_time_diff_at_pass_receive_point =
        (enemy_robot_time_to_pass_receive_position - ball_time_to_pass_receive_position)
            .getSeconds();

    // We take a smooth "max" of these two values using a log-sum-exp function
    // https://en.wikipedia.org/wiki/LogSumExp (NOTE: we use the more computationally
    // stable version mentioned towards the bottom of the wiki page)
    double max_time_diff_unsmooth = std::max(robot_ball_time_diff_at_closest_pass_point,
                                             robot_ball_time_diff_at_pass_receive_point);
    double max_time_diff_smooth =
        max_time_diff_unsmooth +
        std::log(std::exp(robot_ball_time_diff_at_closest_pass_point -
                          max_time_diff_unsmooth) +
                 std::exp(robot_ball_time_diff_at_pass_receive_point -
                          max_time_diff_unsmooth));

    // Whether or not the enemy will be able to intercept the pass can be determined
    // by whether or not they will be able to reach the pass receive position before
    // the pass does. As such, we place the time difference between the robot and ball
    // on a sigmoid that is centered at 0, and goes to 1 at positive values, 0 at
    // negative values. We then subtract this from 1 to essentially invert it, getting
    // a sigmoid that goes to 1 at negative values, and 0 at positive values.
    return 1 - sigmoid(max_time_diff_smooth, 0, 1);
}

double AI::Passing::ratePassFriendlyCapability(const Team& friendly_team,
                                               const Pass& pass)
{
    // We need at least one robot to pass to
    if (friendly_team.getAllRobots().empty())
    {
        return 0;
    }

    // Get the robot that is closest to where the pass would be received
    Robot best_receiver = friendly_team.getAllRobots()[0];
    for (Robot& robot : friendly_team.getAllRobots())
    {
        double distance = (robot.position() - pass.receiverPoint()).len();
        double curr_best_distance =
            (best_receiver.position() - pass.receiverPoint()).len();
        if (distance < curr_best_distance)
        {
            best_receiver = robot;
        }
    }

    // Figure out what time the robot would have to receive the ball at
    Duration ball_travel_time = Duration::fromSeconds(
        (pass.receiverPoint() - pass.passerPoint()).len() / pass.speed());
    Timestamp receive_time = pass.startTime() + ball_travel_time;

    // Figure out how long it would take our robot to get there
    Duration min_robot_travel_time = getTimeToPositionForRobot(
        best_receiver, pass.receiverPoint(), ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Timestamp earliest_time_to_receive_point =
        best_receiver.lastUpdateTimestamp() + min_robot_travel_time;

    // Figure out what angle the robot would have to be at to receive the ball
    Angle receive_angle = (best_receiver.position() - pass.passerPoint()).orientation();
    Duration time_to_receive_angle = getTimeToOrientationForRobot(
        best_receiver, receive_angle, ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND,
        ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED);
    Timestamp earliest_time_to_receive_angle =
        best_receiver.lastUpdateTimestamp() + time_to_receive_angle;

    // Figure out if rotation or moving will take us longer
    Timestamp latest_time_to_reciever_state =
        std::max(earliest_time_to_receive_angle, earliest_time_to_receive_point);

    // Create a sigmoid that goes to 0 as the time required to get to the reception
    // point exceeds the time we would need to get there by
    return sigmoid(receive_time.getSeconds(),
                   latest_time_to_reciever_state.getSeconds() + 0.5, 1);
}

Duration AI::Passing::getTimeToOrientationForRobot(const Robot& robot,
                                                   const Angle& desired_orientation,
                                                   const double& max_velocity,
                                                   const double& max_acceleration)
{
    // We assume a linear acceleration profile:
    // (1) velocity = MAX_ACCELERATION*time
    // we integrate (1) to get:
    // (2) displacement = MAX_ACCELERATION/2 * time^2
    // we rearrange to get:
    // (3) time = sqrt(2 * displacement / MAX_ACCELERATION)
    // we sub. (3) into (1) to get:
    // (4) velocity = MAX_ACCELERATION*sqrt(2 * displacement / MAX_ACCELERATION)
    // and rearrange to get:
    // (5) displacement = (velocity / MAX_ACCELERATION)^2 * MAX_ACCELERATION/2
    // We re-arrange (3) to get:
    // (6) displacement = time^2 * MAX_ACCELERATION/2

    double dist = robot.orientation().minDiff(desired_orientation).toRadians();

    // Calculate the distance required to reach max possible velocity of the robot
    // using (5)
    double dist_to_max_possible_vel =
        std::pow(max_velocity / max_acceleration, 2) * max_acceleration / 2;

    // Calculate how long we'll accelerate for using (3), taking into account that we
    // might not actually reach the max velocity if it will take too much distance
    double acceleration_time =
        std::sqrt(2 * std::min(dist / 2, dist_to_max_possible_vel) / max_acceleration);

    // Calculate how long we'll be at the max possible velocity (if any time at all)
    double time_at_max_velocity =
        std::max(0.0, dist - 2 * dist_to_max_possible_vel) / max_velocity;

    // The time taken to get to the target angle is:
    // time to accelerate + time at the max velocity + time to de-accelerate
    // Note that the acceleration time is the same as a de-acceleration time
    double travel_time = 2 * acceleration_time + time_at_max_velocity;

    return Duration::fromSeconds(travel_time);
}

Duration AI::Passing::getTimeToPositionForRobot(const Robot& robot, const Point& dest,
                                                const double& max_velocity,
                                                const double& max_acceleration)
{
    // We assume a linear acceleration profile:
    // (1) velocity = MAX_ACCELERATION*time
    // we integrate (1) to get:
    // (2) displacement = MAX_ACCELERATION/2 * time^2
    // we rearrange to get:
    // (3) time = sqrt(2 * displacement / MAX_ACCELERATION)
    // we sub. (3) into (1) to get:
    // (4) velocity = MAX_ACCELERATION*sqrt(2 * displacement / MAX_ACCELERATION)
    // and rearrange to get:
    // (5) displacement = (velocity / MAX_ACCELERATION)^2 * MAX_ACCELERATION/2
    // We re-arrange (3) to get:
    // (6) displacement = time^2 * MAX_ACCELERATION/2

    double dist = (robot.position() - dest).len();

    // Calculate the distance required to reach max possible velocity of the robot
    // using (5)
    double dist_to_max_possible_vel =
        std::pow(max_velocity / max_acceleration, 2) * max_acceleration / 2;

    // Calculate how long we'll accelerate for using (3), taking into account that we
    // might not actually reach the max velocity if it will take too much distance
    double acceleration_time =
        std::sqrt(2 * std::min(dist / 2, dist_to_max_possible_vel) / max_acceleration);

    // Calculate how long we'll be at the max possible velocity (if any time at all)
    double time_at_max_velocity =
        std::max(0.0, dist - 2 * dist_to_max_possible_vel) / max_velocity;

    // The time taken to get to the receiver point is:
    // time to accelerate + time at the max velocity + time to de-accelerate
    // Note that the acceleration time is the same as a de-acceleration time
    double travel_time = 2 * acceleration_time + time_at_max_velocity;

    return Duration::fromSeconds(travel_time);
}

double AI::Passing::getStaticPositionQuality(const Field& field, const Point& position)
{
    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sig_width = 0.1;

    // The offset from the sides of the field for the center of the sigmoid functions
    double x_offset =
        Util::DynamicParameters::AI::Passing::static_field_position_quality_x_offset
            .value();
    double y_offset =
        Util::DynamicParameters::AI::Passing::static_field_position_quality_y_offset
            .value();
    double friendly_goal_weight =
        Util::DynamicParameters::AI::Passing::
            static_field_position_quality_friendly_goal_distance_weight.value();

    // Make a slightly smaller field, and positive weight values in this reduced field
    double half_field_length = field.length() / 2;
    double half_field_width  = field.width() / 2;
    Rectangle reduced_size_field(
        Point(-half_field_length + x_offset, -half_field_width + y_offset),
        Point(half_field_length - x_offset, half_field_width - y_offset));
    double on_field_quality = rectangleSigmoid(reduced_size_field, position, sig_width);

    // Add a negative weight for positions closer to our goal
    Vector vec_to_friendly_goal      = Vector(field.friendlyGoal().x() - position.x(),
                                         field.friendlyGoal().y() - position.y());
    double distance_to_friendly_goal = vec_to_friendly_goal.len();
    double near_friendly_goal_quality =
        (1 -
         std::exp(-friendly_goal_weight * (std::pow(5, -2 + distance_to_friendly_goal))));

    // Add a strong negative weight for positions within the enemy defense area, as we
    // cannot pass there
    double in_enemy_defense_area_quality =
        1 - rectangleSigmoid(field.enemyDefenseArea(), position, sig_width);

    return on_field_quality * near_friendly_goal_quality * in_enemy_defense_area_quality;
}

double AI::Passing::rectangleSigmoid(const Rectangle& rect, const Point& point,
                                     const double& sig_width)
{
    double x_offset = rect.centre().x();
    double y_offset = rect.centre().y();
    double x_size   = rect.width() / 2;
    double y_size   = rect.height() / 2;
    double x        = point.x();
    double y        = point.y();

    // For both x and y here we use two sigmoid functions centered at the positive and
    // negative edge of the rectangle respectively

    double x_val = std::min(sigmoid(x, x_offset + x_size, -sig_width),
                            sigmoid(x, x_offset - x_size, sig_width));

    double y_val = std::min(sigmoid(y, y_offset + y_size, -sig_width),
                            sigmoid(y, y_offset - y_size, sig_width));

    return x_val * y_val;
}

double AI::Passing::circleSigmoid(const Circle& circle, const Point& point,
                                  const double& sig_width)
{
    // Calculate how far the point is from the circle center
    double distance_from_circle_center = (point - circle.getOrigin()).len();

    return sigmoid(distance_from_circle_center, circle.getRadius(), -sig_width);
}

double AI::Passing::sigmoid(const double& v, const double& offset,
                            const double& sig_width)
{
    // This is factor that changes how quickly the sigmoid goes from 0 to 1 it. We divide
    // 8 by it because that is the distance a sigmoid function centered about 0 takes to
    // go from 0.018 to 0.982 (and that is what the `sig_width` is, as per the javadoc
    // comment for this function)
    double sig_change_factor = 8 / sig_width;

    return 1 / (1 + std::exp(sig_change_factor * (offset - v)));
}
