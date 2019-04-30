/**
 * Implementation of evaluation functions for passing
 */


#include "ai/passing/evaluation.h"

#include <numeric>

#include "../shared/constants.h"
#include "ai/evaluation/pass.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "geom/util.h"
#include "util/parameter/dynamic_parameters.h"

using namespace AI::Passing;
using namespace AI::Evaluation;

double AI::Passing::ratePass(const World& world, const AI::Passing::Pass& pass,
                             const std::optional<Rectangle>& target_region)
{
    double static_pass_quality =
        getStaticPositionQuality(world.field(), pass.receiverPoint());

    double friendly_pass_rating = ratePassFriendlyCapability(world.friendlyTeam(), pass);

    double enemy_pass_rating = ratePassEnemyRisk(world.enemyTeam(), pass);

    double shoot_pass_rating = ratePassShootScore(world.field(), world.enemyTeam(), pass);

    // Rate all passes outside our target region as 0 if we have one
    double in_region_quality = 1;
    if (target_region)
    {
        in_region_quality = rectangleSigmoid(*target_region, pass.receiverPoint(), 0.1);
    }

    // Place strict limits on pass start time
    double min_pass_time_offset =
        Util::DynamicParameters::AI::Passing::min_time_offset_for_pass_seconds.value();
    double max_pass_time_offset =
        Util::DynamicParameters::AI::Passing::max_time_offset_for_pass_seconds.value();
    // TODO (Issue #423): We should use the timestamp from the world instead of the ball
    double pass_time_offset_quality =
        sigmoid(pass.startTime().getSeconds(),
                min_pass_time_offset + world.ball().lastUpdateTimestamp().getSeconds(),
                0.5) *
        (1 -
         sigmoid(pass.startTime().getSeconds(),
                 max_pass_time_offset + world.ball().lastUpdateTimestamp().getSeconds(),
                 0.5));

    // Place strict limits on the ball speed
    double min_pass_speed =
        Util::DynamicParameters::AI::Passing::min_pass_speed_m_per_s.value();
    double max_pass_speed =
        Util::DynamicParameters::AI::Passing::max_pass_speed_m_per_s.value();
    double pass_speed_quality = sigmoid(pass.speed(), min_pass_speed, 0.2) *
                                (1 - sigmoid(pass.speed(), max_pass_speed, 0.2));

    double pass_quality = static_pass_quality * friendly_pass_rating * enemy_pass_rating *
                          shoot_pass_rating * in_region_quality *
                          pass_time_offset_quality * pass_speed_quality;
    return pass_quality;
}

double AI::Passing::ratePassShootScore(const Field& field, const Team& enemy_team,
                                       const AI::Passing::Pass& pass)
{
    double ideal_shoot_angle_degrees =
        Util::DynamicParameters::AI::Passing::ideal_min_shoot_angle_degrees.value();
    double ideal_max_rotation_to_shoot_degrees =
        Util::DynamicParameters::AI::Passing::ideal_max_rotation_to_shoot_degrees.value();

    std::vector<Point> obstacles;
    for (const Robot& robot : enemy_team.getAllRobots())
    {
        obstacles.emplace_back(robot.position());
    }

    // Figure out the range of angles for which we have an open shot to the goal after
    // receiving the pass
    auto shot_opt =
        angleSweepCircles(pass.receiverPoint(), field.enemyGoalpostNeg(),
                          field.enemyGoalpostPos(), obstacles, ROBOT_MAX_RADIUS_METERS);
    Angle open_angle_to_goal = Angle::zero();
    Point shot_target        = field.enemyGoal();
    if (shot_opt)
    {
        open_angle_to_goal = shot_opt->second;
    }

    // Figure out what the maximum open angle of the goal could be from the receiver pos.
    Angle goal_angle = acuteVertexAngle(field.enemyGoalpostNeg(), pass.receiverPoint(),
                                        field.enemyGoalpostPos())
                           .abs();
    double net_percent_open = 0;
    if (goal_angle > Angle::zero())
    {
        net_percent_open = open_angle_to_goal.toDegrees() / goal_angle.toDegrees();
    }

    // Create the shoot score by creating a sigmoid that goes to a large value as
    // the section of net we're shooting on approaches 100% (ie. completely open)
    double shot_openness_score = sigmoid(net_percent_open, 0.5, 0.95);

    // Prefer angles where the robot does not have to turn much after receiving the
    // pass to take the shot (or equivalently the shot deflection angle)
    Angle rotation_to_shot_target_after_pass = pass.receiverOrientation().minDiff(
        (shot_target - pass.receiverPoint()).orientation());
    double required_rotation_for_shot_score =
        1 - sigmoid(rotation_to_shot_target_after_pass.abs().toDegrees(),
                    ideal_max_rotation_to_shoot_degrees, 4);

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
    std::vector<double> enemy_intercept_risks(enemy_robots.size());
    std::transform(enemy_robots.begin(), enemy_robots.end(),
                   enemy_intercept_risks.begin(),
                   [&](Robot robot) { return calculateInterceptRisk(robot, pass); });
    return *std::max_element(enemy_intercept_risks.begin(), enemy_intercept_risks.end());
}

double AI::Passing::calculateInterceptRisk(Robot enemy_robot, const Pass& pass)
{
    // We estimate the intercept by the risk that the robot will get to the closest
    // point on the pass before the ball, and by the risk that the robot will get to
    // the reception point before the ball. We take the greater of these two risks.

    // If the enemy cannot intercept the pass at BOTH the closest point on the pass and
    // the the receiver point for the pass, then it is guaranteed that it will not be
    // able to intercept the pass anywhere.

    // Figure out how long the enemy robot and ball will take to reach the closest
    // point on the pass to the enemy's current position
    Point closest_point_on_pass_to_robot = closestPointOnSeg(
        enemy_robot.position(), pass.passerPoint(), pass.receiverPoint());
    Duration enemy_robot_time_to_closest_pass_point = getTimeToPositionForRobot(
        enemy_robot, closest_point_on_pass_to_robot,
        ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, ROBOT_MAX_RADIUS_METERS);
    Duration ball_time_to_closest_pass_point = Duration::fromSeconds(
        (closest_point_on_pass_to_robot - pass.passerPoint()).len() / pass.speed());

    // Figure out how long the enemy robot and ball will take to reach the receive point
    // for the pass.
    Duration enemy_robot_time_to_pass_receive_position = getTimeToPositionForRobot(
        enemy_robot, pass.receiverPoint(), ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, ROBOT_MAX_RADIUS_METERS);
    Duration ball_time_to_pass_receive_position = pass.estimatePassDuration();

    Duration time_until_pass = pass.startTime() - enemy_robot.lastUpdateTimestamp();

    double robot_ball_time_diff_at_closest_pass_point =
        (enemy_robot_time_to_closest_pass_point -
         (ball_time_to_closest_pass_point + time_until_pass))
            .getSeconds();
    double robot_ball_time_diff_at_pass_receive_point =
        (enemy_robot_time_to_pass_receive_position -
         (ball_time_to_pass_receive_position + time_until_pass))
            .getSeconds();

    double min_time_diff = std::min(robot_ball_time_diff_at_closest_pass_point,
                                    robot_ball_time_diff_at_pass_receive_point);

    // Whether or not the enemy will be able to intercept the pass can be determined
    // by whether or not they will be able to reach the pass receive position before
    // the pass does. As such, we place the time difference between the robot and ball
    // on a sigmoid that is centered at 0, and goes to 1 at positive values, 0 at
    // negative values.
    return 1 - sigmoid(min_time_diff, 0, 1);
}

double AI::Passing::ratePassFriendlyCapability(const Team& friendly_team,
                                               const Pass& pass)
{
    // We need at least one robot to pass to
    if (friendly_team.getAllRobots().empty())
    {
        return 0;
    }

    // Special case where pass speed is 0
    if (pass.speed() == 0)
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
    Angle receive_angle = (pass.passerPoint() - best_receiver.position()).orientation();
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
                   latest_time_to_reciever_state.getSeconds() + 0.25, 0.5);
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
