#include "software/ai/passing/cost_function.h"

#include <numeric>

#include "software/../shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/pass.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"

double ratePass(const World& world, const Pass& pass)
{
    double static_pass_quality =
        getStaticPositionQuality(world.field(), pass.receiverPoint());

    double friendly_pass_rating =
        ratePassFriendlyCapability(world.ball(), world.friendlyTeam(), pass);

    double enemy_pass_rating = ratePassEnemyRisk(world.ball(), world.enemyTeam(), pass);

    double shoot_pass_rating = ratePassShootScore(world.ball(), world.field(), world.enemyTeam(), pass);

    // Place strict limits on the ball speed
    double min_pass_speed = DynamicParameters->getAiConfig()
                                ->getPassingConfig()
                                ->getMinPassSpeedMPerS()
                                ->value();
    double max_pass_speed = DynamicParameters->getAiConfig()
                                ->getPassingConfig()
                                ->getMaxPassSpeedMPerS()
                                ->value();
    double pass_speed_quality = sigmoid(pass.speed(), min_pass_speed, 0.2) *
                                (1 - sigmoid(pass.speed(), max_pass_speed, 0.2));

    return static_pass_quality * friendly_pass_rating * enemy_pass_rating *
           shoot_pass_rating * pass_speed_quality;
}

double ratePassShootScore(const Ball& ball, const Field& field, const Team& enemy_team, const Pass& pass)
{
    double ideal_max_rotation_to_shoot_degrees = DynamicParameters->getAiConfig()
                                                     ->getPassingConfig()
                                                     ->getIdealMaxRotationToShootDegrees()
                                                     ->value();

    // Figure out the range of angles for which we have an open shot to the goal after
    // receiving the pass
    auto shot_opt =
        calcBestShotOnGoal(Segment(field.enemyGoalpostNeg(), field.enemyGoalpostPos()),
                           pass.receiverPoint(), enemy_team.getAllRobots());

    Angle open_angle_to_goal = Angle::zero();
    Point shot_target        = field.enemyGoalCenter();
    if (shot_opt && shot_opt->getOpenAngle().abs() > Angle::fromDegrees(0))
    {
        open_angle_to_goal = shot_opt->getOpenAngle();
    }

    // Figure out what the maximum open angle of the goal could be from the receiver pos.
    Angle goal_angle = acuteAngle(field.enemyGoalpostNeg(), pass.receiverPoint(),
                                  field.enemyGoalpostPos())
                           .abs();
    double net_percent_open = 0;
    if (goal_angle > Angle::zero())
    {
        net_percent_open = open_angle_to_goal.toDegrees() / goal_angle.toDegrees();
    }

    // Create the shoot score by creating a sigmoid that goes to a large value as
    // the section of net we're shooting on approaches 100% (ie. completely open)
    //
    // TODO (ticket here) This sigmoid is partially responsible for the vanishing
    // gradient. When we don't have a shot on net, we should still be able to look
    // for passes because not all passes need to have a shot on net. But this sigmoid
    // zeros out the score when the net is blocked off.
    //
    // For now, this will be a "leaky sigmoid" where even when there is
    // NO shot on net, the shot_openness_score > 0.
    //
    // This should be fixed w/ a better activation function or potentially changing
    // how we combine the scores together (or both).
    double shot_openness_score = sigmoid(net_percent_open, 0.2, 0.95);

    // Prefer angles where the robot does not have to turn much after receiving the
    // pass to take the shot (or equivalently the shot deflection angle)
    
    auto receiver_orientation = (ball.position() - pass.receiverPoint()).orientation();
    Angle rotation_to_shot_target_after_pass = receiver_orientation.minDiff(
        (shot_target - pass.receiverPoint()).orientation());
    double required_rotation_for_shot_score =
        1 - sigmoid(rotation_to_shot_target_after_pass.abs().toDegrees(),
                    ideal_max_rotation_to_shoot_degrees, 4);

    return shot_openness_score * required_rotation_for_shot_score;
}

double ratePassEnemyRisk(const Ball& ball, const Team& enemy_team, const Pass& pass)
{
    double enemy_proximity_importance = DynamicParameters->getAiConfig()
                                            ->getPassingConfig()
                                            ->getEnemyProximityImportance()
                                            ->value();

    // Calculate a risk score based on the distance of the enemy robots from the receive
    // point, based on an exponential function of the distance of each robot from the
    // receiver point
    auto enemy_robots                    = enemy_team.getAllRobots();
    double enemy_receiver_proximity_risk = 1;
    for (const Robot& enemy : enemy_team.getAllRobots())
    {
        double dist = (pass.receiverPoint() - enemy.position()).length();
        enemy_receiver_proximity_risk *=
            enemy_proximity_importance * std::exp(-dist * dist);
    }
    if (enemy_robots.empty())
    {
        enemy_receiver_proximity_risk = 0;
    }

    double intercept_risk = calculateInterceptRisk(ball, enemy_team, pass);

    // We want to rate a pass more highly if it is lower risk, so subtract from 1
    return 1 - std::max(intercept_risk, enemy_receiver_proximity_risk);
}

double calculateInterceptRisk(const Ball& ball, const Team& enemy_team, const Pass& pass)
{
    // Return the highest risk for all the enemy robots, if there are any
    const std::vector<Robot>& enemy_robots = enemy_team.getAllRobots();
    if (enemy_robots.empty())
    {
        return 0;
    }
    std::vector<double> enemy_intercept_risks(enemy_robots.size());
    std::transform(enemy_robots.begin(), enemy_robots.end(),
                   enemy_intercept_risks.begin(),
                   [&](Robot robot) { return calculateInterceptRisk(ball, robot, pass); });
    return *std::max_element(enemy_intercept_risks.begin(), enemy_intercept_risks.end());
}

double calculateInterceptRisk(const Ball& ball, const Robot& enemy_robot, const Pass& pass)
{
    // We estimate the intercept by the risk that the robot will get to the closest
    // point on the pass before the ball, and by the risk that the robot will get to
    // the reception point before the ball. We take the greater of these two risks.

    // If the enemy cannot intercept the pass at BOTH the closest point on the pass and
    // the receiver point for the pass, then it is guaranteed that it will not be
    // able to intercept the pass anywhere.

    // Figure out how long the enemy robot and ball will take to reach the closest
    // point on the pass to the enemy's current position
    Point closest_point_on_pass_to_robot = closestPoint(
        enemy_robot.position(), Segment(ball.position(), pass.receiverPoint()));
    Duration enemy_robot_time_to_closest_pass_point = getTimeToPositionForRobot(
        enemy_robot.position(), closest_point_on_pass_to_robot,
        ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, ROBOT_MAX_RADIUS_METERS);
    Duration ball_time_to_closest_pass_point = Duration::fromSeconds(
        (closest_point_on_pass_to_robot - ball.position()).length() / pass.speed());

    // Check for division by 0
    if (pass.speed() == 0)
    {
        ball_time_to_closest_pass_point =
            Duration::fromSeconds(std::numeric_limits<int>::max());
    }

    // Figure out how long the enemy robot and ball will take to reach the receive point
    // for the pass.
    Duration enemy_robot_time_to_pass_receive_position = getTimeToPositionForRobot(
        enemy_robot.position(), pass.receiverPoint(),
        ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, ROBOT_MAX_RADIUS_METERS);

    Duration ball_time_to_pass_receive_position = Duration::fromSeconds(
        (pass.receiverPoint() - ball.position()).length() / pass.speed());

    Duration enemy_reaction_time = Duration::fromSeconds(DynamicParameters->getAiConfig()
                                                             ->getPassingConfig()
                                                             ->getEnemyReactionTime()
                                                             ->value());

    double robot_ball_time_diff_at_closest_pass_point =
        ((enemy_robot_time_to_closest_pass_point + enemy_reaction_time) -
         (ball_time_to_closest_pass_point))
            .toSeconds();
    double robot_ball_time_diff_at_pass_receive_point =
        ((enemy_robot_time_to_pass_receive_position + enemy_reaction_time) -
         (ball_time_to_pass_receive_position))
            .toSeconds();

    double min_time_diff = std::min(robot_ball_time_diff_at_closest_pass_point,
                                    robot_ball_time_diff_at_pass_receive_point);

    // Whether or not the enemy will be able to intercept the pass can be determined
    // by whether or not they will be able to reach the pass receive position before
    // the pass does. As such, we place the time difference between the robot and ball
    // on a sigmoid that is centered at 0, and goes to 1 at positive values, 0 at
    // negative values.
    return 1 - sigmoid(min_time_diff, 0, 1);
}

double ratePassFriendlyCapability(const Ball& ball, Team friendly_team, const Pass& pass)
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
    for (const Robot& robot : friendly_team.getAllRobots())
    {
        double distance = (robot.position() - pass.receiverPoint()).length();
        double curr_best_distance =
            (best_receiver.position() - pass.receiverPoint()).length();
        if (distance < curr_best_distance)
        {
            best_receiver = robot;
        }
    }

    // Figure out what time the robot would have to receive the ball at
    Duration ball_travel_time = Duration::fromSeconds(
        (pass.receiverPoint() - ball.position()).length() / pass.speed());
    Timestamp receive_time = ball.timestamp() + ball_travel_time;

    // Figure out how long it would take our robot to get there
    Duration min_robot_travel_time = getTimeToPositionForRobot(
        best_receiver.position(), pass.receiverPoint(), ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Timestamp earliest_time_to_receive_point =
        best_receiver.timestamp() + min_robot_travel_time;

    // Figure out what angle the robot would have to be at to receive the ball
    Angle receive_angle = (ball.position() - best_receiver.position()).orientation();
    Duration time_to_receive_angle = getTimeToOrientationForRobot(
        best_receiver.orientation(), receive_angle, ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND,
        ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED);
    Timestamp earliest_time_to_receive_angle =
        best_receiver.timestamp() + time_to_receive_angle;

    // Figure out if rotation or moving will take us longer
    Timestamp latest_time_to_reciever_state =
        std::max(earliest_time_to_receive_angle, earliest_time_to_receive_point);

    // Create a sigmoid that goes to 0 as the time required to get to the reception
    // point exceeds the time we would need to get there by
    return sigmoid(receive_time.toSeconds(),
                   latest_time_to_reciever_state.toSeconds() + 0.25, 0.4);
}

double getStaticPositionQuality(const Field& field, const Point& position)
{
    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sig_width = 0.1;

    // The offset from the sides of the field for the center of the sigmoid functions
    double x_offset = DynamicParameters->getAiConfig()
                          ->getPassingConfig()
                          ->getStaticFieldPositionQualityXOffset()
                          ->value();
    double y_offset = DynamicParameters->getAiConfig()
                          ->getPassingConfig()
                          ->getStaticFieldPositionQualityYOffset()
                          ->value();
    double friendly_goal_weight =
        DynamicParameters->getAiConfig()
            ->getPassingConfig()
            ->getStaticFieldPositionQualityFriendlyGoalDistanceWeight()
            ->value();

    // Make a slightly smaller field, and positive weight values in this reduced field
    double half_field_length = field.xLength() / 2;
    double half_field_width  = field.yLength() / 2;
    Rectangle reduced_size_field(
        Point(-half_field_length + x_offset, -half_field_width + y_offset),
        Point(half_field_length - x_offset, half_field_width - y_offset));
    double on_field_quality = rectangleSigmoid(reduced_size_field, position, sig_width);

    // Add a negative weight for positions closer to our goal
    Vector vec_to_friendly_goal = Vector(field.friendlyGoalCenter().x() - position.x(),
                                         field.friendlyGoalCenter().y() - position.y());
    double distance_to_friendly_goal = vec_to_friendly_goal.length();
    double near_friendly_goal_quality =
        (1 -
         std::exp(-friendly_goal_weight * (std::pow(5, -2 + distance_to_friendly_goal))));

    // Add a strong negative weight for positions within the enemy defense area, as we
    // cannot pass there
    double in_enemy_defense_area_quality =
        1 - rectangleSigmoid(field.enemyDefenseArea(), position, sig_width);

    return on_field_quality * near_friendly_goal_quality * in_enemy_defense_area_quality;
}
