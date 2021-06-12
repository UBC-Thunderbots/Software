#include "software/ai/passing/cost_function.h"

#include <numeric>
#include <random>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/../shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/pass.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

double ratePass(const World& world, const Pass& pass, const Rectangle& zone,
                std::shared_ptr<const PassingConfig> passing_config)
{
    double static_pass_quality =
        getStaticPositionQuality(world.field(), pass.receiverPoint(), passing_config);

    double friendly_kick_pass_rating =
        rateKickPassFriendlyCapability(world.friendlyTeam(), pass, passing_config);
    double friendly_chip_pass_rating =
        rateChipPassFriendlyCapability(world.friendlyTeam(), pass, passing_config);

    double enemy_kick_pass_rating =
        rateKickPassEnemyRisk(world.enemyTeam(), pass,
			     Duration::fromSeconds(passing_config->getEnemyReactionTime()->value()),
        		     passing_config->getEnemyProximityImportance()->value());

    double enemy_chip_pass_rating =
        rateChipPassEnemyRisk(world.enemyTeam(), pass, passing_config);

    double chip_pass_rating = friendly_chip_pass_rating * enemy_chip_pass_rating;
    double kick_pass_rating = friendly_kick_pass_rating * enemy_kick_pass_rating;

    double in_region_quality = rectangleSigmoid(zone, pass.receiverPoint(), 0.2);

    double shoot_pass_rating =
        ratePassShootScore(world.field(), world.enemyTeam(), pass, passing_config);

    // Place strict limits on the ball speed
    double min_pass_speed     = passing_config->getMinPassSpeedMPerS()->value();
    double max_pass_speed     = passing_config->getMaxPassSpeedMPerS()->value();
    double pass_speed_quality = sigmoid(pass.speed(), min_pass_speed, 0.2) *
                                (1 - sigmoid(pass.speed(), max_pass_speed, 0.2));

    return static_pass_quality * kick_pass_rating * chip_pass_rating  * shoot_pass_rating *
           pass_speed_quality * in_region_quality;
}

double rateZone(const World& world, const Rectangle& zone, const Point& receive_position,
                std::shared_ptr<const PassingConfig> passing_config)
{
    // We sample random points in the zone to check what the pass scores would
    // be, ignoring friendly capability. This will give us a pretty good idea
    // of what our pass scores would be if we sent a robot there.
    const size_t X_POINTS_TO_SAMPLE = 5;
    const size_t Y_POINTS_TO_SAMPLE = 5;

    double zone_rating = 0.0;
    double x_step      = zone.xLength() / X_POINTS_TO_SAMPLE;
    double y_step      = zone.yLength() / Y_POINTS_TO_SAMPLE;

    for (double x = zone.xMin(); x < zone.xMax(); x += x_step)
    {
        for (double y = zone.yMin(); y < zone.yMax(); y += y_step)
        {
            Pass pass = Pass(Point(x, y), receive_position,
                             BALL_MAX_SPEED_METERS_PER_SECOND);
            zone_rating += rateKickPassEnemyRisk(world.enemyTeam(), pass,
			     Duration::fromSeconds(passing_config->getEnemyReactionTime()->value()),
        		     passing_config->getEnemyProximityImportance()->value())
                + rateChipPassEnemyRisk(world.enemyTeam(), pass, passing_config);
        }
    }

    zone_rating /= (X_POINTS_TO_SAMPLE * Y_POINTS_TO_SAMPLE);

    double pass_up_field_rating =
        (zone.centre().x() + world.field().totalXLength()/2) / world.field().totalXLength();

    // These zones are meant for receiving the ball, if they are ridiculously close
    // to the ball or worse in the zone that the ball is in, then the passes will be very
    // low quality.
    // 
    // We don't enforce a max pass length, that is driven by the other cost functions,
    // but we create a cost "force field" around the ball to discourage selecting the
    // zone the ball is in (and maybe the neighbouring zone if its on the edge).
    double zone_ball_proximity_rating =
        sigmoid((receive_position - zone.centre()).length(), zone.xLength(), zone.xLength());

    double zone_quality = getStaticPositionQuality(world.field(), zone.centre(), passing_config);

    return zone_rating * pass_up_field_rating * zone_ball_proximity_rating * zone_quality;
}

double ratePassShootScore(const Field& field, const Team& enemy_team, const Pass& pass,
                          std::shared_ptr<const PassingConfig> passing_config)
{
    double ideal_max_rotation_to_shoot_degrees =
        passing_config->getIdealMaxRotationToShootDegrees()->value();

    // Figure out the range of angles for which we have an open shot to the goal after
    // receiving the pass
    auto shot_opt = calcBestShotOnGoal(
        Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()), pass.receiverPoint(),
        enemy_team.getAllRobots(), TeamType::ENEMY);

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
    double shot_openness_score = sigmoid(net_percent_open, 0.45, 0.95);

    Angle rotation_to_shot_target_after_pass = pass.receiverOrientation().minDiff(
        (shot_target - pass.receiverPoint()).orientation());
    double required_rotation_for_shot_score =
        1 - sigmoid(rotation_to_shot_target_after_pass.abs().toDegrees(),
                    ideal_max_rotation_to_shoot_degrees, 4);

    return (shot_openness_score + required_rotation_for_shot_score)/2;
}

double rateKickPassEnemyRisk(const Team& enemy_team, const Pass& pass,
                         const Duration& enemy_reaction_time,
                         double enemy_proximity_importance)
{
    double enemy_receiver_proximity_risk = calculateProximityRisk(
        pass.receiverPoint(), enemy_team, enemy_proximity_importance);
    double intercept_risk = calculateKickInterceptRisk(enemy_team, pass, enemy_reaction_time);

    // We want to rate a pass more highly if it is lower risk, so subtract from 1
    return 1 - std::max(intercept_risk, enemy_receiver_proximity_risk);
}

double rateChipPassEnemyRisk(const Team& enemy_team, const Pass& pass,
                             std::shared_ptr<const PassingConfig> passing_config)
{
    // Calculate a risk score based on the distance of the enemy robots from the receive
    // point, based on an exponential function of the distance of each robot from the
    // receiver point
    // Get the robot that is closest to where the pass would be received
    Robot closest_enemy = enemy_team.getAllRobots()[0];
    auto enemy_robots   = enemy_team.getAllRobots();

    for (const Robot& robot : enemy_robots)
    {
        double distance = (robot.position() - pass.receiverPoint()).length();
        double curr_best_distance =
            (closest_enemy.position() - pass.receiverPoint()).length();
        if (distance < curr_best_distance)
        {
            closest_enemy = robot;
        }
    }
    if (enemy_robots.empty())
    {
        return 0;
    }
    else
    {
        return sigmoid((closest_enemy.position() - pass.receiverPoint()).length(), 0.5, 2.0);
    }
}

double calculateKickInterceptRisk(const Team& enemy_team, const Pass& pass,
                              const Duration& enemy_reaction_time)
{
    // Return the highest risk for all the enemy robots, if there are any
    const std::vector<Robot>& enemy_robots = enemy_team.getAllRobots();
    if (enemy_robots.empty())
    {
        return 0;
    }
    std::vector<double> enemy_intercept_risks(enemy_robots.size());
    std::transform(enemy_robots.begin(), enemy_robots.end(),
                   enemy_intercept_risks.begin(), [&](Robot robot) {
                       return calculateKickInterceptRisk(robot, pass, enemy_reaction_time);
                   });
    return *std::max_element(enemy_intercept_risks.begin(), enemy_intercept_risks.end());
}

double calculateKickInterceptRisk(const Robot& enemy_robot, const Pass& pass,
                              const Duration& enemy_reaction_time)
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
        enemy_robot.position(), Segment(pass.passerPoint(), pass.receiverPoint()));
    Duration enemy_robot_time_to_closest_pass_point = getTimeToPositionForRobot(
        enemy_robot.position(), closest_point_on_pass_to_robot,
        ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, ROBOT_MAX_RADIUS_METERS);
    Duration ball_time_to_closest_pass_point = Duration::fromSeconds(
        (closest_point_on_pass_to_robot - pass.passerPoint()).length() / pass.speed());

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
    Duration ball_time_to_pass_receive_position = pass.estimatePassDuration();

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

double rateKickPassFriendlyCapability(Team friendly_team, const Pass& pass,
                                      std::shared_ptr<const PassingConfig> passing_config)
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
        (pass.receiverPoint() - pass.passerPoint()).length() / pass.speed());
    Timestamp receive_time = best_receiver.timestamp() + ball_travel_time;

    // Figure out how long it would take our robot to get there
    Duration min_robot_travel_time = getTimeToPositionForRobot(
        best_receiver.position(), pass.receiverPoint(), ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Timestamp earliest_time_to_receive_point =
        best_receiver.timestamp() + min_robot_travel_time;

    // Figure out what angle the robot would have to be at to receive the ball
    Angle receive_angle = (pass.passerPoint() - best_receiver.position()).orientation();
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
    double sigmoid_width                  = 0.4;
    double time_to_receiver_state_slack_s = 0.25;

    return sigmoid(
        receive_time.toSeconds(),
        latest_time_to_reciever_state.toSeconds() + time_to_receiver_state_slack_s,
        sigmoid_width);
}

double rateChipPassFriendlyCapability(Team friendly_team, const Pass& pass,
                                      std::shared_ptr<const PassingConfig> passing_config)
{
    // We need at least one robot to pass to
    if (friendly_team.getAllRobots().empty())
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

    // When we chip the ball, we are more conservative with how much we allow
    // the robot to move to intercept the ball. This is because our current
    // chipping interface takes a chip target distance and doesn't give us control over
    // the speed of the chip, making it harder for us to predict _when_ the ball will
    // land.
    //
    // Running the CalibrationPlay shows that our hangtime is usually greater than 1
    // second.
    Duration ball_travel_time = Duration::fromSeconds(1.00);
    Timestamp receive_time    = best_receiver.timestamp() + ball_travel_time;

    // Figure out how long it would take our robot to get there
    Duration min_robot_travel_time = getTimeToPositionForRobot(
        best_receiver.position(), pass.receiverPoint(), ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Timestamp earliest_time_to_receive_point =
        best_receiver.timestamp() + min_robot_travel_time;

    // Figure out what angle the robot would have to be at to receive the ball
    Angle receive_angle = (pass.passerPoint() - best_receiver.position()).orientation();
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
    double sigmoid_width                  = 0.4;
    double time_to_receiver_state_slack_s = 0.25;

    return sigmoid(
        receive_time.toSeconds(),
        latest_time_to_reciever_state.toSeconds() + time_to_receiver_state_slack_s,
        sigmoid_width);
}

double getStaticPositionQuality(const Field& field, const Point& position,
                                std::shared_ptr<const PassingConfig> passing_config)
{
    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sig_width = 0.1;

    // The offset from the sides of the field for the center of the sigmoid functions
    double x_offset = passing_config->getStaticFieldPositionQualityXOffset()->value();
    double y_offset = passing_config->getStaticFieldPositionQualityYOffset()->value();
    double friendly_goal_weight =
        passing_config->getStaticFieldPositionQualityFriendlyGoalDistanceWeight()
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

double calculateProximityRisk(const Point& point, const Team& enemy_team,
                              double enemy_proximity_importance)
{
    // Calculate a risk score based on the distance of the enemy robots from the receive
    // point, based on an exponential function of the distance of each robot from the
    // receiver point
    auto enemy_robots                 = enemy_team.getAllRobots();
    double point_enemy_proximity_risk = 1;
    for (const Robot& enemy : enemy_team.getAllRobots())
    {
        double dist = (point - enemy.position()).length();
        point_enemy_proximity_risk *= enemy_proximity_importance * std::exp(-dist * dist);
    }
    if (enemy_robots.empty())
    {
        point_enemy_proximity_risk = 0;
    }
    return point_enemy_proximity_risk;
}
