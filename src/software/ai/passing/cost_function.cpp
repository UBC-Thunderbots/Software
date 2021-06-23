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
#include "software/geom/algorithms/intersects.h"
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

    double enemy_kick_pass_rating = rateKickPassEnemyRisk(
        world.enemyTeam(), pass,
        Duration::fromSeconds(passing_config->getEnemyReactionTime()->value()),
        passing_config->getEnemyProximityImportance()->value());
    double enemy_chip_pass_rating = rateChipPassEnemyRisk(
        world.enemyTeam(), pass,
        Duration::fromSeconds(passing_config->getEnemyReactionTime()->value()));

    double chip_pass_rating = friendly_chip_pass_rating * enemy_chip_pass_rating;
    double kick_pass_rating = friendly_kick_pass_rating * enemy_kick_pass_rating;
    double pass_rating      = std::max(kick_pass_rating , chip_pass_rating);

    double in_region_quality = rectangleSigmoid(zone, pass.receiverPoint(), 0.2);

    double shoot_pass_rating =
        ratePassShootScore(world.field(), world.enemyTeam(), pass, passing_config);

    // Place strict limits on the ball speed
    double min_pass_speed     = passing_config->getMinPassSpeedMPerS()->value();
    double max_pass_speed     = passing_config->getMaxPassSpeedMPerS()->value();
    double pass_speed_quality = sigmoid(pass.speed(), min_pass_speed, 0.2) *
                                (1 - sigmoid(pass.speed(), max_pass_speed, 0.2));

    double pass_up_field_rating = (zone.centre().x() + world.field().totalXLength() / 2) /
                                  world.field().totalXLength();

    return static_pass_quality * pass_rating * pass_up_field_rating * shoot_pass_rating *
           pass_speed_quality * in_region_quality;
}

double rateZone(const World& world, const Rectangle& zone, const Point& receive_position,
                std::shared_ptr<const PassingConfig> passing_config)
{
    // We sample random points in the zone to check what the pass scores would
    // be, ignoring friendly capability. This will give us a pretty good idea
    // of what our pass scores would be if we sent a robot there.
    const size_t X_POINTS_TO_SAMPLE         = 5;
    const size_t Y_POINTS_TO_SAMPLE         = 5;
    const size_t NUM_COST_FUNCTIONS_SAMPLED = 3;

    double zone_rating = 0.0;
    double x_step      = zone.xLength() / X_POINTS_TO_SAMPLE;
    double y_step      = zone.yLength() / Y_POINTS_TO_SAMPLE;

    for (double x = zone.xMin(); x < zone.xMax(); x += x_step)
    {
        for (double y = zone.yMin(); y < zone.yMax(); y += y_step)
        {
            Pass pass =
                Pass(Point(x, y), receive_position, BALL_MAX_SPEED_METERS_PER_SECOND);

            zone_rating += ratePassShootScore(world.field(), world.enemyTeam(), pass,
                                              passing_config);
            zone_rating += rateChipPassEnemyRisk(
                    world.enemyTeam(), pass,
                    Duration::fromSeconds(passing_config->getEnemyReactionTime()->value()));
            zone_rating += rateKickPassEnemyRisk(
                    world.enemyTeam(), pass,
                    Duration::fromSeconds(passing_config->getEnemyReactionTime()->value()),
                    passing_config->getEnemyProximityImportance()->value());
        }
    }

    // average the cost to bring it to be between 0 and 1
    zone_rating /= (NUM_COST_FUNCTIONS_SAMPLED * X_POINTS_TO_SAMPLE * Y_POINTS_TO_SAMPLE);

    double pass_up_field_rating =
        (zone.centre().x() + (world.field().totalXLength() / 2)) /
        world.field().totalXLength();

    // These zones are meant for receiving the ball, if they are ridiculously close
    // to the ball or worse in the zone that the ball is in, then the passes will be very
    // low quality.
    //
    // We don't enforce a max pass length, that is driven by the other cost functions,
    // but we create a cost "force field" around the ball to discourage selecting the
    // zone the ball is in (and maybe the neighbouring zone if its on the edge).
    double zone_ball_proximity_rating = sigmoid(
        (receive_position - zone.centre()).length(), zone.xLength(), zone.xLength());

    double zone_quality =
        getStaticPositionQuality(world.field(), zone.centre(), passing_config);

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

    // Prefer angles where the robot does not have to turn much after receiving the
    // pass to take the shot (or equivalently the shot deflection angle)
    Angle rotation_to_shot_target_after_pass = pass.receiverOrientation().minDiff(
        (shot_target - pass.receiverPoint()).orientation());

    double width = (Angle::half()-Angle::fromDegrees(ideal_max_rotation_to_shoot_degrees)).toDegrees();
    double required_rotation_for_shot_score =
        1 - sigmoid(rotation_to_shot_target_after_pass.abs().toDegrees(),
                Angle::fromDegrees(ideal_max_rotation_to_shoot_degrees).toDegrees() + width/2.0, width);

    return shot_openness_score * required_rotation_for_shot_score;
}

double rateKickPassEnemyRisk(const Team& enemy_team, const Pass& pass,
                             const Duration& enemy_reaction_time,
                             double enemy_proximity_importance)
{
    double enemy_receiver_proximity_risk = calculateProximityRisk(
        pass.receiverPoint(), enemy_team, enemy_proximity_importance);
    double intercept_risk =
        calculateKickInterceptRisk(enemy_team, pass, enemy_reaction_time);

    // We want to rate a pass more highly if it is lower risk, so subtract from 1
    return 1 - std::max(intercept_risk, enemy_receiver_proximity_risk);
}

double rateChipPassEnemyRisk(const Team& enemy_team, const Pass& pass,
                             const Duration& enemy_reaction_time)
{
    /**
     *  We assume the chip trajectory will clear robots for the initial portion
     *  of the chip, and then roll for the remainder.
     *
     *
     *                            .---.                    x = chip landing point
     *                           /     \
     *                          /       \
     *                (passer) /         \x_______ (receiver)
     *                            CHIP      ROLL
     *
     *  So, we only need to calculate the risk of the enemy robot closest to the passer
     *  point, and then the risk of enemies intercepting the pass after it starts rolling.
     *
     *  We also assume that the ROLL distance is proportional to the CHIP distance by
     *  CHIP_PASS_TARGET_DISTANCE_TO_ROLL_RATIO. The further we chip, the further we
     *  roll.
     *
     *  So, to figure out the risk of the enemy interfering, we just make sure
     *
     */
    auto closest_enemy_to_passer   = enemy_team.getNearestRobot(pass.passerPoint());
    auto closest_enemy_to_receiver = enemy_team.getNearestRobot(pass.receiverPoint());

    if (!closest_enemy_to_passer.has_value() || !closest_enemy_to_receiver.has_value())
    {
        return 0;
    }

    // The sigmoid with of 2.0 meters is somewhat arbitrary but it discourages really
    // short chips that can be easily intercepted, as the ball doesn't roll as fast after
    // landing.
    double sigmoid_width = 2.0;

    // This should figure out if the pass will clear the enemy.
    double enemy_risk_near_passer_point =
        sigmoid((closest_enemy_to_passer->position() - pass.passerPoint()).length(),
                ROBOT_MIN_CHIP_CLEAR_DISTANCE_METERS, sigmoid_width);

    // Now we just need to figure out where we have landed, since we don't have chip speed
    // as mentioned above, we use CHIP_PASS_TARGET_DISTANCE_TO_ROLL_RATIO to get an
    // estimate of where we might have landed.
    auto pass_vector = pass.receiverPoint() - pass.passerPoint();

    // NOTE: The robot performing the pass should also use
    // CHIP_PASS_TARGET_DISTANCE_TO_ROLL_RATIO if it would like to chip.
    auto chip_landing_point =
        pass.passerPoint() +
        pass_vector.normalize(CHIP_PASS_TARGET_DISTANCE_TO_ROLL_RATIO *
                              pass_vector.length());

    // Create a "virtual" kick pass from the chip landing point and the receive point
    // and evaluate the likely hood of an enemy intercepting that pass.
    double enemy_risk_near_receiver_point = calculateKickInterceptRisk(
        enemy_team,
        Pass(chip_landing_point, pass.receiverPoint(),
             pass_vector.length() * CHIP_PASS_TARGET_DISTANCE_TO_SPEED_RATIO),
        enemy_reaction_time);

    return 1.0 - enemy_risk_near_passer_point * enemy_risk_near_receiver_point;
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
                       return calculateKickInterceptRisk(robot, pass,
                                                         enemy_reaction_time);
                   });
    return *std::max_element(enemy_intercept_risks.begin(), enemy_intercept_risks.end());
}

double calculateKickInterceptRisk(const Robot& enemy_robot, const Pass& pass,
                                  const Duration& enemy_reaction_time)
{
    auto pass_segment = Segment(pass.passerPoint(), pass.receiverPoint());

    // We estimate the intercept by the risk that the robot will get to the closest
    // point on the pass before the ball, and by the risk that the robot will get to the
    // reception point before the ball. We take the greater of these two risks. If the
    // enemy cannot intercept the pass at BOTH the closest point on the pass and the
    // receiver point for the pass, then it is guaranteed that it will not be able to
    // intercept the pass anywhere.

    // Figure out how long the enemy robot and ball will take to reach the closest
    // point on the pass to the enemy's current position
    Point closest_point_on_pass_to_robot =
        closestPoint(enemy_robot.position(), pass_segment);
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

    // FUTURE TODO (#2167): IMPORTANT!!!
    // Previously we were always just adding the enemy reaction time to the estimated time to destination. This meant
    // that even if there was a robot right in front of the passer, we would think we could get the ball past them
    // before they could intercept (which is obviously wrong). REMOVE THIS FIX, MAKE A TEST FAIL, AND ADD A PROPER REGRESSION TEST
    const double REACTION_TIME_SCALING_FACTOR = 3.0;
    double scaled_enemy_reaction_time_closest_pass_point = std::clamp(enemy_robot_time_to_closest_pass_point.toSeconds() /
                        REACTION_TIME_SCALING_FACTOR, 0.0, enemy_reaction_time.toSeconds());

    double robot_ball_time_diff_at_closest_pass_point =
        ((enemy_robot_time_to_closest_pass_point + Duration::fromSeconds(scaled_enemy_reaction_time_closest_pass_point)) -
         (ball_time_to_closest_pass_point))
        .toSeconds();

    double scaled_enemy_reaction_time_receive_point = std::clamp(enemy_robot_time_to_pass_receive_position.toSeconds() /
            REACTION_TIME_SCALING_FACTOR, 0.0, enemy_reaction_time.toSeconds());
    double robot_ball_time_diff_at_pass_receive_point =
        ((enemy_robot_time_to_pass_receive_position + Duration::fromSeconds(scaled_enemy_reaction_time_receive_point)) -
         (ball_time_to_pass_receive_position))
            .toSeconds();

    double min_time_diff = std::min(robot_ball_time_diff_at_closest_pass_point,
                                    robot_ball_time_diff_at_pass_receive_point);

    // Whether or not the enemy will be able to intercept the pass can be determined
    // by whether or not they will be able to reach the pass receive position before
    // the pass does. As such, we place the time difference between the robot and ball
    // on a sigmoid that is centered at 0, and goes to 1 at positive values, 0 at
    // negative values.
    return 1 - sigmoid(min_time_diff, 0.4, 1);
}

double rateKickPassFriendlyCapability(Team friendly_team, const Pass& pass,
                                      std::shared_ptr<const PassingConfig> passing_config)
{
    // Get the robot that is closest to where the pass would be received
    auto best_receiver = friendly_team.getNearestRobot(pass.receiverPoint());
    auto best_passer   = friendly_team.getNearestRobot(pass.passerPoint());

    // If we don't have a receiver, we can't pass
    if (!best_receiver.has_value() || !best_receiver.has_value())
    {
        return 0;
    }

    if (best_receiver.value() == best_passer.value())
    {
        return 0;
    }

    // We need at least one robot to pass to and the pass should have a speed,
    // and the past must be evaluated for now or the future, not in the past.
    //
    if (pass.speed() == 0)
    {
        return 0;
    }

    // Figure out what time the robot would have to receive the ball at
    Timestamp receive_time = best_receiver->timestamp() + pass.estimatePassDuration();
    Timestamp latest_time_to_reciever_state =
        calculateEarliestTimeRobotCanReceive(best_receiver.value(), pass);

    // Create a sigmoid that goes to 0 as the time required to get to the reception
    // point exceeds the time we would need to get there by
    double sigmoid_width                  = 0.5;
    double time_to_receiver_state_slack_s = 0.25;

    return sigmoid(
        receive_time.toSeconds(),
        latest_time_to_reciever_state.toSeconds() + time_to_receiver_state_slack_s,
        sigmoid_width);
}

double rateChipPassFriendlyCapability(Team friendly_team, const Pass& pass,
                                      std::shared_ptr<const PassingConfig> passing_config)
{
    auto best_receiver = friendly_team.getNearestRobot(pass.receiverPoint());
    auto best_passer   = friendly_team.getNearestRobot(pass.passerPoint());

    // If we don't have a receiver or a passer, we can't pass
    if (!best_receiver.has_value() || !best_receiver.has_value())
    {
        return 0;
    }

    if (best_receiver.value() == best_passer.value())
    {
        return 0;
    }

    // When we chip the ball, we are more conservative with how much we allow
    // the robot to move to intercept the ball. This is because our current
    // chipping interface takes a chip target distance and doesn't give us control over
    // the speed of the chip, making it harder for us to predict _when_ the ball will
    // land.
    //
    // TODO (#2167) robocup hack, we know the kick angle is fixed, and we get perfect
    // speed through ssl_simulator_robot.cpp, so we can compute the ball hang time
    // with some basic kinematics
    Angle chip_angle = Angle::fromDegrees(ROBOT_CHIP_ANGLE_DEGREES);
    double range = (pass.receiverPoint() - pass.passerPoint()).length() *
                                        CHIP_PASS_TARGET_DISTANCE_TO_ROLL_RATIO;
    double numerator = range * ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED;
    double denominator = 2.0 * (chip_angle * 2.0).sin();
    double chip_speed  = std::sqrt(numerator / denominator);
    double hang_time = (2 * Vector::createFromAngle(chip_angle).normalize(chip_speed).y())/
        ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED;

    const Timestamp receive_time = best_receiver->timestamp() + Duration::fromSeconds(hang_time);
    Timestamp latest_time_to_reciever_state =
        calculateEarliestTimeRobotCanReceive(best_receiver.value(), pass);

    // Create a sigmoid that goes to 0 as the time required to get to the reception
    // point exceeds the time we would need to get there by
    double sigmoid_width                  = 0.4;
    double time_to_receiver_state_slack_s = 0.05;

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

Timestamp calculateEarliestTimeRobotCanReceive(const Robot& best_receiver,
                                               const Pass& pass)
{
    // Figure out how long it would take our robot to get there
    Duration min_robot_travel_time = getTimeToPositionForRobot(
        best_receiver.position(), pass.receiverPoint(),
        best_receiver.robotConstants().robot_max_speed_m_per_s,
        best_receiver.robotConstants().robot_max_acceleration_m_per_s_2);
    Timestamp earliest_time_to_receive_point =
        best_receiver.timestamp() + min_robot_travel_time;

    // Figure out what angle the robot would have to be at to receive the ball
    Angle receive_angle = (pass.passerPoint() - best_receiver.position()).orientation();
    Duration time_to_receive_angle = getTimeToOrientationForRobot(
        best_receiver.orientation(), receive_angle,
        best_receiver.robotConstants().robot_max_ang_speed_rad_per_s,
        best_receiver.robotConstants().robot_max_ang_acceleration_rad_per_s_2);
    Timestamp earliest_time_to_receive_angle =
        best_receiver.timestamp() + time_to_receive_angle;

    return std::max(earliest_time_to_receive_angle, earliest_time_to_receive_point);
}
